"""Consume the planner's actual full-task archive; no simulator dependencies.

The wire messages are retained as CDR and compared semantically after decoding.
This avoids assuming that CDR padding bytes from two serializers are identical.
"""
import hashlib
import math
from pathlib import Path


def decode_report(text):
    import yaml
    report = yaml.safe_load(text)
    if not isinstance(report, dict) or report.get('schema') != 'pick_place_candidates/v1':
        raise ValueError('candidate_binding_missing_or_unsupported')
    chosen = report['candidates'][report['selected_index']]
    if (chosen.get('accepted') is not True or chosen['candidate_id'] != report['selected_id']
            or not math.isfinite(float(chosen['score']))):
        raise ValueError('candidate_selection_not_accepted')
    binding = report['binding']
    if hashlib.sha256(binding['solution_cdr_hex'].encode()).hexdigest() != binding['solution_sha256']:
        raise ValueError('candidate_solution_digest_mismatch')
    return report


def decoded(hexadecimal, kind):
    from rclpy.serialization import deserialize_message
    return deserialize_message(bytes.fromhex(hexadecimal), kind)


def verify_payload(report, solution, description):
    from moveit_task_constructor_msgs.msg import Solution, TaskDescription
    binding = report['binding']
    if solution != decoded(binding['solution_cdr_hex'], Solution):
        raise ValueError('candidate_solution_payload_mismatch')
    if description != decoded(binding['description_cdr_hex'], TaskDescription):
        raise ValueError('candidate_description_payload_mismatch')


def verify_files(report):
    for path, expected in report['binding'].get('files', {}).items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected:
            raise ValueError('candidate_configuration_or_calibration_file_changed:'+path)


def verify_start(report, positions, age):
    binding = report['binding']
    tolerance = float(binding['start_tolerance_rad'])
    if not 0. < tolerance <= .01 or not 0. <= age <= 1.:
        raise ValueError('candidate_start_feedback_stale_or_invalid')
    for joint, expected in binding['start_positions'].items():
        actual = positions.get(joint, float('nan'))
        if not math.isfinite(actual) or abs(actual-float(expected)) > tolerance:
            raise ValueError('candidate_start_state_mismatch:'+joint)


def pose_signature(message):
    # Acquisition stamps change while the independently known target is static.
    p, q = message.pose.position, message.pose.orientation
    return message.header.frame_id, tuple(round(x, 8) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))


def verify_targets(report, target, place):
    from geometry_msgs.msg import PoseStamped
    for field, actual in (('target_cdr_hex', target), ('place_cdr_hex', place)):
        if pose_signature(actual) != pose_signature(decoded(report['binding'][field], PoseStamped)):
            raise ValueError('candidate_target_or_place_changed')


def scene_signature(scene):
    from rosidl_runtime_py.convert import message_to_ordereddict
    def normalize(value):
        if isinstance(value, dict):
            return {k: normalize(v) for k,v in value.items() if k != 'stamp'}
        if isinstance(value, (list, tuple)):
            return [normalize(v) for v in value]
        if isinstance(value, float):
            if not math.isfinite(value):
                raise ValueError('candidate_scene_nonfinite')
            return round(value, 8)
        return value
    value = normalize(message_to_ordereddict(scene))
    # Robot feedback is checked separately with bounded age and raw joint deltas.
    state = value.pop('robot_state')
    value.pop('name', None)
    value.pop('is_diff', None)
    value.pop('object_colors', None)
    value['attached_objects'] = sorted(state['attached_collision_objects'], key=lambda x: x['object']['id'])
    value['world']['collision_objects'].sort(key=lambda x: x['id'])
    value['fixed_frame_transforms'].sort(key=lambda x: x['child_frame_id'])
    value['link_padding'].sort(key=lambda x: x['link_name'])
    value['link_scale'].sort(key=lambda x: x['link_name'])
    # Preserve ACM entries including explicit false and defaults, independent of
    # serialization row order. Never expand the matrix to permit new contact.
    acm = value['allowed_collision_matrix']
    entries = {}
    for i, name in enumerate(acm['entry_names']):
        for j, other in enumerate(acm['entry_names']):
            entries[(name,other)] = acm['entry_values'][i]['enabled'][j]
    value['allowed_collision_matrix'] = (
        sorted(entries.items()), sorted(zip(acm['default_entry_names'], acm['default_entry_values'])))
    return value


def verify_scene(report, current):
    from moveit_msgs.msg import PlanningScene
    expected = decoded(report['binding']['scene_cdr_hex'], PlanningScene)
    if scene_signature(expected) != scene_signature(current):
        raise ValueError('candidate_planning_scene_changed')


def expected_parameters(report, field):
    from rcl_interfaces.msg import Parameter
    return {name: decoded(value, Parameter).value
            for name,value in report['binding'][field].items()}
