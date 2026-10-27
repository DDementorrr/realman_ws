#!/usr/bin/env bash
# Requires an already running simulation in this terminal's ROS domain.
# Runs native GUI Plan/Execute for a small simulated joint1 move; no hardware use.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/source_pick_place_env.sh
panel_output="${1:-$PWD/log/operator-panel-$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$panel_output"
ros2 param dump /move_group --output-dir "$panel_output" >/dev/null
python3 - "$panel_output" <<'PY'
import sys
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
out = Path(sys.argv[1])
params = next(iter(yaml.safe_load((out/'move_group.yaml').read_text()).values()))['ros__parameters']
share = Path(get_package_share_directory('robot_realman_moveit_config'))
# Copy only source configuration, not optional parameters dynamically declared by MoveIt.
# Dumped runtime type placeholders cannot be replayed as RViz overrides.
p = {k: params[k] for k in ('robot_description', 'robot_description_semantic', 'use_sim_time')}
if 'ctm2f110_tool0' not in p['robot_description']:
    raise SystemExit('此完整探针需要 with_gripper:=true 的已知物体仿真')
p['robot_description_kinematics'] = yaml.safe_load((share/'config/rm65b_kinematics.yaml').read_text())
p['robot_description_planning'] = yaml.safe_load((share/'config/rm65b_ctm2f110_joint_limits.yaml').read_text())
p['planning_pipelines'] = ['ompl']
p['ompl'] = yaml.safe_load((share/'config/rm65b_ctm2f110_ompl_planning.yaml').read_text())
yaml.safe_dump({'/**': {'ros__parameters': p}}, (out/'rviz.yaml').open('w'))
PY
panel_config="$(ros2 pkg prefix robot_realman_moveit_config)/share/robot_realman_moveit_config/rviz/rm65b.rviz"
ros2 run rm65b_operator_panel native_panel_probe "$panel_config" "$panel_output/probe" \
  --ros-args --params-file "$panel_output/rviz.yaml" > "$panel_output/rviz-probe.log" 2>&1
python3 - "$panel_output/probe.json" <<'PY'
import json,sys
v=json.load(open(sys.argv[1])); print(json.dumps(v, ensure_ascii=False, indent=2)); assert v['accepted']
PY
