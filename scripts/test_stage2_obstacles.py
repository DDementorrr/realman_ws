#!/usr/bin/env python3
"""Offline generator/world-conversion regression; no model APIs or simulator."""

from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
for package in ('pick_place_scene', 'pick_place_sim'):
    sys.path.insert(0, str(ROOT / 'src/ros2_adapters' / package))
from pick_place_sim.obstacle_scene import config_from, generate_scene, gap, rect
from pick_place_sim.workbench_world import generate_world
from pick_place_scene.static_geometry import load_geometry

ADAPTERS = ROOT / 'src/ros2_adapters'
CONFIG = ADAPTERS / 'pick_place_sim/config/obstacles.yaml'
BENCH = ADAPTERS / 'pick_place_scene/config/u_workbench.yaml'
TASK = ADAPTERS / 'pick_place_demo/config/known_scene.yaml'
GAZEBO = ROOT / 'src/realman_rm65/robot_gazebo'
MOUNT = GAZEBO / 'urdf/rm65b_ctm2f110_gz.urdf.xacro'


class ObstaclesTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def generate(self, name='scene', **overrides):
        return generate_scene(BENCH, TASK, MOUNT, config_from(CONFIG, **overrides), self.root / name)

    def test_reproducible_seed_and_full_archive(self):
        for seed in (42, 77):
            a, am = self.generate(f'{seed}a', obstacle_mode='random', obstacle_seed=seed)
            b, bm = self.generate(f'{seed}b', obstacle_mode='random', obstacle_seed=seed)
            self.assertEqual(a.read_bytes(), b.read_bytes())
            self.assertEqual(am, bm)
            self.assertEqual(am['config']['obstacle_seed'], seed)
            self.assertEqual(am['geometry'], yaml.safe_load(a.read_text()))
            self.assertTrue(am['generator_version'])
            self.assertEqual((a.parent / 'task.yaml').read_bytes(), TASK.read_bytes())
        self.assertNotEqual((self.root/'42a/geometry.yaml').read_bytes(),
                            (self.root/'77a/geometry.yaml').read_bytes())

    def test_many_seeds_structural_filters(self):
        for seed in range(40):
            path, m = self.generate(str(seed), obstacle_mode='random', obstacle_seed=seed)
            _, boxes = load_geometry(path)
            self.assertEqual(len(boxes), 8)
            for i, box in enumerate(boxes[3:]):
                self.assertTrue(any(all(lo >= sl+.005-1e-9 and hi <= sh-.005+1e-9
                    for (lo, hi), (sl, sh) in zip(rect(box), rect(s))) for s in boxes[:3]))
                self.assertTrue(all(gap(rect(box), z) > 0 for z in m['protected_zones_xy']))
                self.assertGreaterEqual(gap(rect(box), ((0, 0), (0, 0))), .32)
                for other in boxes[3:3+i]:
                    self.assertGreaterEqual(gap(rect(box), rect(other)), .04-1e-9)

    def test_fixed_benchmark_catalogue(self):
        for name in config_from(CONFIG)['fixtures']:
            path, m = self.generate(name, obstacle_mode='benchmark', benchmark=name)
            if name.startswith('blocked_'):
                self.assertEqual(m['expected'], 'blocked')
                self.assertIn('occupied_pick_or_place_zone', m['reason'])
            else:
                self.assertEqual(m['reason'], [])
                self.assertEqual(m['expected'], 'candidate')
        with self.assertRaisesRegex(ValueError, 'negative fixtures'):
            self.generate('bad-negative', obstacle_mode='fixed', benchmark='blocked_pick')

    def test_invalid_configuration(self):
        for key, value in [('obstacle_seed', True), ('obstacle_count', -1),
                           ('obstacle_clearance', float('nan')), ('max_attempts', 0),
                           ('obstacle_min_height', 1.), ('obstacle_mode', 'typo'),
                           ('benchmark', 'typo'), ('footprints', {'wall': [0., .1]})]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                config_from(CONFIG, **{key: value})
        with self.assertRaises(ValueError):
            config_from(CONFIG, obstacle_cout=5)

    def test_exhaustion_does_not_write_partial_scene(self):
        with self.assertRaisesRegex(ValueError, 'exhausted'):
            self.generate(obstacle_mode='random', obstacle_count=100, max_attempts=1)
        self.assertFalse((self.root/'scene').exists())

    def test_invalid_fixed_layouts_do_not_become_candidates(self):
        cases = {
            'outside_support': [['cube', 2., 0., .12]],
            'occupied_pick_or_place_zone': [['cube', .43, -.18, .12]],
            'overlap_or_insufficient_clearance': [['cube', .65, .4, .12], ['cube', .65, .41, .12]],
            'robot_base_keepout': [['cube', .1, 0., .12]],
        }
        for reason, fixtures in cases.items():
            config = config_from(CONFIG)
            config['fixtures']['single_wall'] = fixtures
            with self.subTest(reason=reason), self.assertRaisesRegex(ValueError, reason):
                generate_scene(BENCH, TASK, MOUNT, config, self.root/reason)
            self.assertFalse((self.root/reason).exists())

    def test_no_overwrite(self):
        path, _ = self.generate()
        before = path.read_bytes()
        with self.assertRaises(FileExistsError):
            self.generate(obstacle_mode='random')
        self.assertEqual(path.read_bytes(), before)

    def test_task_must_rest_on_configured_support(self):
        data = yaml.safe_load(TASK.read_text())
        data['known_pick_place_scene']['ros__parameters']['place_pose'][2] += .1
        task = self.root/'unsupported-task.yaml'
        task.write_text(yaml.safe_dump(data))
        with self.assertRaisesRegex(ValueError, 'not supported'):
            generate_scene(BENCH, task, MOUNT, config_from(CONFIG), self.root/'scene')

    def test_both_adapters_consume_identical_snapshot(self):
        from pick_place_scene.static_geometry import collision_object
        for mode, name in [('fixed', 'single_wall'), ('random', 'single_wall'),
                           ('benchmark', 'blocked_place')]:
            path, _ = self.generate(mode, obstacle_mode=mode, benchmark=name)
            frame, boxes = load_geometry(path)
            output = self.root / f'{mode}.sdf'
            generate_world(GAZEBO/'worlds/rm65b_pick_place_fixed.sdf', path, MOUNT, output)
            world = ET.parse(output).find('world')
            for box in boxes:
                model = world.find(f"model[@name='{box.id}']")
                pose = list(map(float, model.findtext('pose').split()))
                self.assertAlmostEqual(pose[2], box.pose[2]+.15)
                for kind in ('visual', 'collision'):
                    self.assertEqual(list(map(float, model.findtext(f'link/{kind}/geometry/box/size').split())), list(box.size))
                obj = collision_object(frame, box)
                self.assertEqual(list(obj.primitives[0].dimensions), list(box.size))
            self.assertIsNotNone(world.find("model[@name='pick_object']"))

    def test_cannot_replace_robot_or_target_template_entities(self):
        path, _ = self.generate()
        data = yaml.safe_load(path.read_text())
        data['boxes'].append({'id': 'pick_object', 'size': [.1]*3, 'pose': [0.]*6})
        path.write_text(yaml.safe_dump(data))
        with self.assertRaisesRegex(ValueError, 'must not replace'):
            generate_world(GAZEBO/'worlds/rm65b_pick_place_fixed.sdf', path, MOUNT, self.root/'bad.sdf')


if __name__ == '__main__':
    unittest.main()
