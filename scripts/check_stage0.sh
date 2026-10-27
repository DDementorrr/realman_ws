#!/usr/bin/env bash
# Reproducible public-package build and isolation acceptance. No simulation launch.
set -eo pipefail
stage0_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" != --clean-environment ]]; then
  stage0_evidence="$(mktemp -d /tmp/realman-stage0-acceptance.XXXXXX)"
  exec env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 \
    STAGE0_EVIDENCE_DIR="$stage0_evidence" \
    bash --noprofile --norc "$0" --clean-environment
fi
exec > >(tee "$STAGE0_EVIDENCE_DIR/acceptance.log") 2>&1
echo "Evidence: $STAGE0_EVIDENCE_DIR"
source /opt/ros/humble/setup.bash
stage0_mtc="$stage0_root/.third_party/mtc/debroot/opt/ros/humble"
export AMENT_PREFIX_PATH="$stage0_mtc:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$stage0_mtc:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$stage0_mtc/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$stage0_mtc/local/lib/python3.10/dist-packages:$stage0_mtc/lib/python3.10/site-packages:${PYTHONPATH:-}"
export PYTHONNOUSERSITE=1 CMAKE_BUILD_PARALLEL_LEVEL=2 ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="$((20 + $$ % 200))"
cd "$stage0_root"
# Four public packages, plus three model asset packages used only as external fixtures.
# No workspace install/setup, Gazebo/control overlay, demo or simulation package.
colcon --log-base "$STAGE0_EVIDENCE_DIR/log" build --parallel-workers 2 \
  --base-paths src/ros2_adapters/pick_place_perception src/ros2_adapters/pick_place_scene \
    src/ros2_adapters/pick_place_runtime src/ros2_adapters/pick_place_mtc \
    src/realman_rm65/rm_description src/realman_rm65/robot_realman_moveit_config \
    src/crt_ctm2f110_gripper_visualization/robot_changingtek_description \
  --build-base "$STAGE0_EVIDENCE_DIR/build" --install-base "$STAGE0_EVIDENCE_DIR/install" \
  --cmake-args -DPICK_PLACE_REQUIRE_MTC=ON -DBUILD_TESTING=OFF
source "$STAGE0_EVIDENCE_DIR/install/local_setup.bash"
env | sort > "$STAGE0_EVIDENCE_DIR/environment.txt"
python3 scripts/verify_mtc_isolation.py
python3 scripts/verify_vision_scene_separation.py
python3 scripts/test_execution_protocol.py -v
python3 scripts/test_execution_scene_revision.py -v
python3 scripts/test_rgbd_contract.py -v
python3 scripts/test_perception_config.py -v
python3 scripts/test_scene_sync.py -v
export PYTHONPATH="$stage0_root/src/ros2_adapters/pick_place_sim/scripts:${PYTHONPATH:-}"
python3 scripts/test_ros_action_backend.py -v
python3 scripts/test_execution_fault_latch.py -v
python3 scripts/test_ros_action_backend_rclpy.py
python3 scripts/test_mtc_config.py \
  "$STAGE0_EVIDENCE_DIR/install/pick_place_mtc/lib/pick_place_mtc/plan_pick_place" \
  --fixture-out "$STAGE0_EVIDENCE_DIR"
python3 scripts/test_public_launch_isolation.py
bash scripts/check_joint_motion_score.sh
bash scripts/check_candidate_revision_contract.sh
git diff --check HEAD --
echo 'PASS stage 0 public build, configuration, launch isolation and backend protocol'
