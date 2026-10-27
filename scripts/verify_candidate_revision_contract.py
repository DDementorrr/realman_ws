#!/usr/bin/env python3
"""Static contract check for MTC candidate scene-revision binding.

Reads the MTC planner, the Gazebo execution adapter and the pick_place_sim
manifest, then exercises the pure-Python runtime helper directly so the
legacy-revision compatibility path is verified by behaviour, not by string
matching.
"""

from pathlib import Path
import ast
import re
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PLANNER = ROOT / "src/ros2_adapters/pick_place_mtc/src/plan_pick_place.cpp"
EXECUTOR = ROOT / "src/ros2_adapters/pick_place_sim/scripts/execute_pick_place.py"
SIM_MANIFEST = ROOT / "src/ros2_adapters/pick_place_sim/package.xml"
RUNTIME_PACKAGE = ROOT / "src/ros2_adapters/pick_place_runtime"

SCENE_REVISION_TOPIC = "/perception/scene_revision"
SCENE_READY_TOPIC = "/perception/scene_ready"
CANDIDATE_TOPIC = "/pick_place/candidate_scene_revision"

DEPENDENCY_TAGS = {
    "depend",
    "build_depend",
    "build_export_depend",
    "buildtool_depend",
    "exec_depend",
    "run_depend",
    "test_depend",
    "doc_depend",
}


def fail(message: str) -> None:
    print(f"FAIL {message}")
    raise SystemExit(1)


def read(path: Path, label: str) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as error:
        fail(f"cannot read {label} ({path.relative_to(ROOT).as_posix()}): {error}")
    return ""


def matching_brace(source: str, open_index: int) -> int:
    """Return the index of the brace closing the '(' at ``open_index``."""
    depth = 0
    for index in range(open_index, len(source)):
        char = source[index]
        if char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
            if depth == 0:
                return index
    fail("unbalanced parentheses while reading C++ source")


def matching_brace_block(source: str, open_index: int) -> int:
    """Return the index of the brace closing the '{' at ``open_index``."""
    depth = 0
    for index in range(open_index, len(source)):
        char = source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return index
    fail("unbalanced braces while reading C++ source")


def cpp_function_body(source: str, name: str) -> str:
    """Extract the brace-delimited body of the method named ``name``."""
    match = re.search(r"\b" + re.escape(name) + r"\s*\(", source)
    if match is None:
        fail(f"cannot find function {name}() in MTC planner")
    params_end = matching_brace(source, match.end() - 1)
    body_start = source.find("{", params_end)
    if body_start < 0:
        fail(f"function {name}() has no body in MTC planner")
    body_end = matching_brace_block(source, body_start)
    return source[body_start + 1:body_end]


def cpp_statement(source: str, start: int) -> str:
    end = source.find(";", start)
    if end < 0:
        fail("unterminated C++ statement")
    return source[start:end]


def get_calls(tree: ast.AST, method: str) -> list:
    """Return ``self.<method>(...)`` call nodes."""
    calls = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if (isinstance(func, ast.Attribute) and func.attr == method
                and isinstance(func.value, ast.Name) and func.value.id == "self"):
            calls.append(node)
    return calls


def positional_topics(call: ast.Call) -> set:
    """Topics passed positionally or as ``topic=`` to a create_subscription call."""
    topics = set()
    for arg in call.args:
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
            topics.add(arg.value)
    for kw in call.keywords:
        if kw.arg == "topic" and isinstance(kw.value, ast.Constant):
            topics.add(kw.value.value)
    return topics


def find_method(tree: ast.Module, name: str):
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
            return node
    return None


# --------------------------------------------------------------------------
# 1. MTC planner: subscribes to revision/ready, publishes candidate revision.
# --------------------------------------------------------------------------
planner_source = read(PLANNER, "MTC planner")
relative_planner = PLANNER.relative_to(ROOT).as_posix()

subscriptions = re.findall(r"create_subscription<[^;]*?;", planner_source, re.DOTALL)
if not subscriptions:
    fail(f"{relative_planner} creates no topic subscriptions")
for topic, label in (
        (SCENE_REVISION_TOPIC, "scene revision subscription"),
        (SCENE_READY_TOPIC, "scene ready subscription")):
    if not any(f'"{topic}"' in statement for statement in subscriptions):
        fail(f"MTC planner does not create a {label} on {topic}")

publisher_match = re.search(
    r"(\w+)\s*=\s*[\w.>-]*create_publisher<std_msgs::msg::UInt64>"
    r"\s*\(\s*\"([^\"]+)\"", planner_source)
if publisher_match is None:
    fail("MTC planner does not create a UInt64 publisher for the candidate revision")
publisher_name, publisher_topic = publisher_match.group(1), publisher_match.group(2)
if publisher_topic != CANDIDATE_TOPIC:
    fail(f"MTC candidate UInt64 publisher is bound to {publisher_topic}, "
         f"expected {CANDIDATE_TOPIC}")

run_body = cpp_function_body(planner_source, "run")

publish_index = run_body.find(f"{publisher_name}->publish(")
if publish_index < 0:
    fail(f"MTC planner run() never publishes the candidate revision via {publisher_name}")

plan_index = run_body.find("working.plan(")
selected_guard = re.search(r"if\s*\(\s*!selected\s*\)\s*\{", run_body)
if plan_index < 0 or selected_guard is None:
    fail("MTC planner run() does not plan and select a candidate solution")
if plan_index >= selected_guard.start():
    fail("MTC selected-candidate guard must follow the planning call")
selected_guard_open = selected_guard.end() - 1
selected_guard_close = matching_brace_block(run_body, selected_guard_open)
if publish_index <= selected_guard_close:
    fail("MTC candidate revision must be published after the no-selected-solution failure guard")

for required, label in (
        ("revision_bound", "revision-stream gate"),
        ("scene_revision_.load()", "revision snapshot"),
        ("candidate_revision", "captured candidate revision"),
        ("scene_ready_.load()", "scene-ready gate")):
    if required not in run_body:
        fail(f"MTC planner run() is missing the {label} ({required})")

publish_statement = cpp_statement(run_body, publish_index)
if "candidate_revision" not in publish_statement:
    fail("MTC planner must publish the snapshotted candidate revision")
if "scene_revision_.load()" in publish_statement:
    fail("MTC planner must publish the revision snapshot, not a re-read value")

binding_gate = re.search(r"if\s*\(\s*revision_bound\s*\)\s*\{", run_body)
if binding_gate is None:
    fail("MTC planner must guard the candidate publication on the captured revision")
gate_open = binding_gate.end() - 1
gate_body = run_body[gate_open + 1:matching_brace_block(run_body, gate_open)]
if publisher_name not in gate_body:
    fail("MTC candidate revision publication must be inside the revision-bound guard")

if re.search(r"!scene_revision_seen_\.load\(\)\s*\|\|\s*scene_ready_\.load\(\)",
             run_body) is None:
    fail("MTC planner readiness must require scene_ready when the revision stream exists")

print("PASS MTC planner subscribes to scene revision/ready and binds the planned "
      "candidate to the snapshotted revision after planning success")

# --------------------------------------------------------------------------
# 2. Execution adapter: subscriptions, helper import and gated ready().
# --------------------------------------------------------------------------
relative_executor = EXECUTOR.relative_to(ROOT).as_posix()
executor_source = read(EXECUTOR, "pick place execution adapter")
try:
    executor_tree = ast.parse(executor_source, filename=str(EXECUTOR))
except SyntaxError as error:
    fail(f"cannot parse {relative_executor}: {error}")

imported_names = set()
for node in ast.walk(executor_tree):
    if isinstance(node, ast.ImportFrom) and node.module == "pick_place_runtime":
        imported_names.update(alias.name for alias in node.names)
if "scene_revision_status" not in imported_names:
    fail(f"{relative_executor} must import scene_revision_status from pick_place_runtime")

subscribed_topics = set()
for call in get_calls(executor_tree, "create_subscription"):
    subscribed_topics |= positional_topics(call)
for topic in (SCENE_REVISION_TOPIC, SCENE_READY_TOPIC, CANDIDATE_TOPIC):
    if topic not in subscribed_topics:
        fail(f"execution adapter does not subscribe to {topic}")

helper_calls = [
    node for node in ast.walk(executor_tree)
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    and node.func.id == "scene_revision_status"
]
if not helper_calls:
    fail("execution adapter never calls scene_revision_status")

ready_fn = find_method(executor_tree, "ready")
if ready_fn is None:
    fail("execution adapter has no ready() gate")

ready_helper_calls = [
    node for node in ast.walk(ready_fn)
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    and node.func.id == "scene_revision_status"
]
if not ready_helper_calls:
    fail("ready() must consult scene_revision_status")

returns = [node for node in ast.walk(ready_fn) if isinstance(node, ast.Return) and node.value]
if len(returns) != 1:
    fail("ready() must return a single decision expression")
decision = returns[0].value

unpacked_scene_names = set()
for node in ast.walk(ready_fn):
    if (isinstance(node, ast.Assign) and len(node.targets) == 1
            and isinstance(node.targets[0], (ast.Tuple, ast.List))
            and len(node.targets[0].elts) >= 1
            and isinstance(node.targets[0].elts[0], ast.Name)
            and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Name)
            and node.value.func.id == "scene_revision_status"):
        unpacked_scene_names.add(node.targets[0].elts[0].id)
if not unpacked_scene_names:
    fail("ready() must unpack the scene_ok result of scene_revision_status")
if not isinstance(decision, ast.BoolOp) or not isinstance(decision.op, ast.And):
    fail("ready() decision must be a positive conjunction so no readiness gate can bypass scene state")
if not any(isinstance(value, ast.Name) and value.id in unpacked_scene_names
           for value in decision.values):
    fail("ready() must include scene_ok as a direct positive condition in its readiness conjunction")

print("PASS execution adapter subscribes to revision/ready/candidate topics, imports "
      "and calls scene_revision_status, and gates ready() on its result")

# --------------------------------------------------------------------------
# 3. pick_place_sim manifest declares the runtime dependency.
# --------------------------------------------------------------------------
relative_manifest = SIM_MANIFEST.relative_to(ROOT).as_posix()
try:
    manifest_root = ET.parse(SIM_MANIFEST).getroot()
except (OSError, ET.ParseError) as error:
    fail(f"cannot parse {relative_manifest}: {error}")
if (manifest_root.findtext("name") or "").strip() != "pick_place_sim":
    fail(f"{relative_manifest} is not the pick_place_sim manifest")
declared = {
    element.text.strip()
    for element in manifest_root
    if element.tag in DEPENDENCY_TAGS and element.text and element.text.strip()
}
if "pick_place_runtime" not in declared:
    fail(f"{relative_manifest} does not declare the pick_place_runtime dependency")
print("PASS pick_place_sim manifest declares the pick_place_runtime dependency")

# --------------------------------------------------------------------------
# 4. Behaviour of the pure-Python helper (real calls, not string search).
# --------------------------------------------------------------------------
runtime_path = str(RUNTIME_PACKAGE)
if runtime_path not in sys.path:
    sys.path.insert(0, runtime_path)

try:
    from pick_place_runtime.candidate_binding import scene_revision_status
except Exception as error:
    fail(f"cannot import pick_place_runtime.candidate_binding: {error}")


def expect(args, allowed, code, label):
    try:
        result = scene_revision_status(*args)
    except Exception as error:
        fail(f"scene_revision_status raised for {label}: {error}")
    if not isinstance(result, tuple) or len(result) != 2:
        fail(f"scene_revision_status must return a (allowed, code) tuple for {label}")
    ok, reason = result
    if bool(ok) is not allowed:
        fail(f"{label}: expected allowed={allowed}, got {result!r}")
    if reason != code:
        fail(f"{label}: expected reason {code!r}, got {reason!r}")


# Legacy path: no revision stream at all still permits a known-scene candidate.
expect((None, None, False, False), True, "scene_revision_unavailable_legacy_mode",
       "legacy mode without a revision stream")
expect((None, 3, True, False), True, "scene_revision_unavailable_legacy_mode",
       "legacy mode while stream is still unseen")

# Once a revision stream exists, every incomplete condition is rejected.
expect((None, 4, True, True), False, "candidate_has_no_scene_revision",
       "candidate without revision while stream exists")
expect((4, 4, False, True), False, "scene_not_ready",
       "candidate while scene not ready")
expect((4, 5, True, True), False, "stale_scene_revision",
       "candidate revision behind current scene")
expect((5, 4, True, True), False, "stale_scene_revision",
       "candidate revision ahead of current scene")
expect((4, None, True, True), False, "stale_scene_revision",
       "candidate while current revision is unknown")

# Fully matching candidate is allowed.
expect((4, 4, True, True), True, "scene_revision_confirmed",
       "matching candidate revision")
expect((0, 0, True, True), True, "scene_revision_confirmed",
       "matching zero revision")

print("PASS runtime helper keeps legacy compatibility and enforces candidate "
      "revision/ready/match rejection by actual invocation")
print("candidate-revision-contract-check: PASS")
