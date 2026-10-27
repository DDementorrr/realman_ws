#!/usr/bin/env python3
"""Real adapter/services with a MOCK Gazebo transport world; sends no motion.

The world accepts plugin loading but drops its initial attached event, matching
the observed one-shot discovery race. All later events are either delivered or
deliberately dropped. Neither outcome is a real Gazebo execution test.
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from std_srvs.srv import SetBool


WORLD = r'''
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <thread>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity_plugin_v.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
int main(int argc, char**) {
  gz::transport::Node node;
  auto state = node.Advertise<gz::msgs::StringMsg>("/pick_place/gz_attachment_state");
  auto poses = node.Advertise<gz::msgs::Pose_V>("/world/empty/pose/info");
  std::atomic<bool> loaded{false}, attached{false};
  const bool drop_all = argc > 1;
  auto publish = [&](const char* value) {
    if (!drop_all) { gz::msgs::StringMsg msg; msg.set_data(value); state.Publish(msg); }
  };
  std::function<bool(const gz::msgs::EntityPlugin_V&, gz::msgs::Boolean&)> load =
    [&](const auto&, auto& reply) {
      loaded = true; attached = true; reply.set_data(true);
      std::cout << "loaded: initial attached event deliberately dropped" << std::endl;
      return true;
    };
  node.Advertise("/world/empty/entity/system/add", load);
  std::function<void(const gz::msgs::Empty&)> detach = [&](const auto&) {
    if (loaded && attached.exchange(false)) { publish("detached"); std::cout << "detached" << std::endl; }
  };
  std::function<void(const gz::msgs::Empty&)> attach = [&](const auto&) {
    if (loaded && !attached.exchange(true)) { publish("attached"); std::cout << "attached" << std::endl; }
  };
  node.Subscribe("/pick_place/gz_detach", detach);
  node.Subscribe("/pick_place/gz_attach", attach);
  for (;;) {
    gz::msgs::Pose_V msg; msg.mutable_header()->mutable_stamp()->set_sec(1);
    auto* pose = msg.add_pose(); pose->set_id(42); pose->set_name("rm65b");
    poses.Publish(msg); std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
'''


def run_case(binary, root, drop_all):
    env = os.environ.copy()
    env['GZ_PARTITION'] = f'stage6_handshake_{os.getpid()}_{int(drop_all)}'
    adapter = Path(get_package_prefix('pick_place_sim'))/'lib/pick_place_sim/attachment_transport'
    processes = []
    with (root/f'world-{drop_all}.log').open('w') as world_log, (root/f'adapter-{drop_all}.log').open('w') as adapter_log:
        try:
            processes.append(subprocess.Popen([str(binary)]+(['drop_all'] if drop_all else []), env=env, stdout=world_log, stderr=subprocess.STDOUT))
            processes.append(subprocess.Popen([str(adapter)], env=env, stdout=adapter_log, stderr=subprocess.STDOUT))
            rclpy.init()
            node = rclpy.create_node('test_stage6_attachment_handshake')
            client = node.create_client(SetBool, '/pick_place/sim/set_attachment')
            if not client.wait_for_service(timeout_sec=10.):
                raise RuntimeError('production adapter service unavailable')
            time.sleep(1.)  # Allow the mock entity pose and transport discovery.
            future = client.call_async(SetBool.Request(data=True))
            rclpy.spin_until_future_complete(node, future, timeout_sec=8.)
            if not future.done() or future.result() is None:
                raise RuntimeError('attachment service did not return')
            response = future.result()
            if drop_all:
                assert not response.success and 'detached handshake was not confirmed' in response.message, response
            else:
                assert response.success and response.message == 'Gazebo confirmed attached', response
            node.destroy_node()
            rclpy.shutdown()
        finally:
            for process in processes:
                process.terminate()
            for process in processes:
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
            if rclpy.ok():
                rclpy.shutdown()
    adapter_text = (root/f'adapter-{drop_all}.log').read_text()
    world_text = (root/f'world-{drop_all}.log').read_text()
    assert 'requiring a fresh detached handshake' in adapter_text
    assert 'detached' in world_text
    assert ('\nattached\n' in world_text) != drop_all
    print('PASS all acknowledgements lost: refuses attachment' if drop_all else
          'PASS initial event lost: fresh detached then attached acknowledgements required', flush=True)


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='stage6-attachment-test-') as temporary:
        root = Path(temporary)
        source, binary = root/'mock_world.cpp', root/'mock_world'
        source.write_text(WORLD)
        flags = subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gz-transport13', 'gz-msgs10'], text=True)
        subprocess.run(['g++', '-std=c++17', str(source), '-o', str(binary), *shlex.split(flags)], check=True)
        for drop_all in (False, True):
            run_case(binary, root, drop_all)
