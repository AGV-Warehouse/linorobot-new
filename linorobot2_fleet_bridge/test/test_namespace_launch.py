"""ROS-environment launch planning tests; does not start robots or Gazebo."""
import importlib.util
from pathlib import Path
import pytest
pytest.importorskip("launch_ros")
import yaml
from launch import LaunchContext

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("fleet_launch", PACKAGE / "launch/fleet.launch.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

@pytest.mark.parametrize("namespace", ["", "lab01", "lab02"])
def test_parameter_root_applied_exactly_once(namespace):
    context = LaunchContext()
    raw, nav, bridge = module.parameter_files(PACKAGE, namespace, str(PACKAGE / "config/bridge.yaml"))
    raw_data = yaml.safe_load(Path(raw.perform(context)).read_text())
    nav_data = yaml.safe_load(Path(nav.perform(context)).read_text())
    bridge_data = yaml.safe_load(Path(bridge.perform(context)).read_text())
    assert "amcl" in raw_data and "controller_server" in raw_data
    if namespace:
        assert set(nav_data) == {namespace}
        assert set(bridge_data) == {namespace}
        nav_data, bridge_data = nav_data[namespace], bridge_data[namespace]
    assert nav_data["controller_server"]["ros__parameters"]["controller_frequency"] == 10.0
    assert nav_data["local_costmap"]["local_costmap"]["ros__parameters"]["voxel_layer"]["scan"]["topic"] == "scan"
    assert bridge_data["linorobot2_fleet_bridge"]["ros__parameters"]["commissioned"] is False
    assert bridge_data["linorobot2_fleet_bridge"]["ros__parameters"]["initial_pose_covariance_xy"] == 0.25
    assert bridge_data["linorobot2_fleet_bridge"]["ros__parameters"]["initial_pose_covariance_yaw"] == 0.0685



def test_simulation_nav_override_preserves_localization_and_navigation_settings(tmp_path):
    config = yaml.safe_load((PACKAGE / "config/fleet_navigation.yaml").read_text())
    amcl = config["amcl"]["ros__parameters"]
    amcl.update(set_initial_pose=True, initial_pose={"x": -4.0, "y": 0.0, "z": 0.0, "yaw": 0.0}, scan_topic="sim_scan")
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    local.pop("footprint", None)
    local["robot_radius"] = 0.275
    local["voxel_layer"]["scan"]["topic"] = "sim_scan"
    config["controller_server"]["ros__parameters"]["general_goal_checker"]["xy_goal_tolerance"] = 0.08
    override = tmp_path / "simulation.yaml"
    override.write_text(yaml.safe_dump(config))
    context = LaunchContext()
    raw, nav, bridge = module.parameter_files(PACKAGE, "lab_01", str(PACKAGE / "config/bridge.yaml"), str(override))
    localization = yaml.safe_load(Path(raw.perform(context)).read_text())
    navigation = yaml.safe_load(Path(nav.perform(context)).read_text())["lab_01"]
    assert localization["amcl"]["ros__parameters"]["initial_pose"]["x"] == -4.0
    assert localization["amcl"]["ros__parameters"]["scan_topic"] == "sim_scan"
    params = navigation["local_costmap"]["local_costmap"]["ros__parameters"]
    assert "footprint" not in params and params["robot_radius"] == 0.275
    assert params["voxel_layer"]["scan"]["topic"] == "sim_scan"
    assert navigation["controller_server"]["ros__parameters"]["general_goal_checker"]["xy_goal_tolerance"] == 0.08

def test_no_hardware_package_resolution_when_disabled(tmp_path, monkeypatch):
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: unused.pgm")
    context = LaunchContext()
    context.launch_configurations.update(map=str(map_file), namespace="lab01",
        bridge_config=str(PACKAGE / "config/bridge.yaml"), start_hardware="false", sim="true")
    calls = []
    def resolve(name):
        calls.append(name)
        if name == "linorobot2_bringup":
            raise AssertionError("Hardware package must not be resolved")
        return str(PACKAGE)
    monkeypatch.setattr(module, "get_package_share_directory", resolve)
    assert len(module.start(context)) == 1
    assert "linorobot2_bringup" not in calls



def test_upstream_localization_noncomposition_conditions_evaluate(tmp_path, monkeypatch):
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription, GroupAction
    from launch_ros.actions import Node, LoadComposableNodes
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: unused.pgm")
    context = LaunchContext()
    context.launch_configurations.update(map=str(map_file), namespace="lab01",
        bridge_config=str(PACKAGE / "config/bridge.yaml"), start_hardware="false", sim="true")
    nav = Path(get_package_share_directory("nav2_bringup"))
    monkeypatch.setattr(module, "get_package_share_directory",
        lambda name: str(PACKAGE if name == "linorobot2_fleet_bridge" else nav))
    group = module.start(context)[0]
    include = next(action for action in group.get_sub_entities()
                   if isinstance(action, IncludeLaunchDescription))
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    for name, value in include.launch_arguments:
        context.launch_configurations[name] = perform_substitutions(
            context, normalize_to_list_of_substitutions(value))
    spec = importlib.util.spec_from_file_location("upstream_localization", nav / "launch/localization_launch.py")
    upstream = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(upstream)
    description = upstream.generate_launch_description()
    conditional_nodes = []
    standalone_groups = []
    composed_actions = []
    unconditional_amcl = []
    def visit(entity, ancestor_enabled=True):
        condition = getattr(entity, "condition", None)
        own_enabled = condition.evaluate(context) if condition is not None else True
        effective = ancestor_enabled and own_enabled
        if isinstance(entity, GroupAction) and condition is not None:
            if any(isinstance(child, Node) for child in entity.get_sub_entities()):
                standalone_groups.append(effective)
        if isinstance(entity, LoadComposableNodes):
            composed_actions.append(effective)
        if isinstance(entity, Node):
            executable = perform_substitutions(
                context, normalize_to_list_of_substitutions(entity.node_executable))
            if condition is not None:
                conditional_nodes.append(effective)
            if executable == "amcl":
                unconditional_amcl.append(condition is None and effective)
        # A child may have a true condition inside a disabled ancestor group.
        if hasattr(entity, "get_sub_entities"):
            for child in entity.get_sub_entities():
                visit(child, effective)
    for entity in description.entities:
        visit(entity)
    assert standalone_groups and all(standalone_groups)
    assert composed_actions and not any(composed_actions)
    assert any(conditional_nodes)  # One of the two map-server branches is selected.
    assert unconditional_amcl == [True]


@pytest.mark.parametrize("enabled", ["true", "false"])
def test_bridge_can_be_supervised_without_changing_nav2_launch(tmp_path, monkeypatch, enabled):
    from launch_ros.actions import Node
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: unused.pgm")
    context = LaunchContext()
    context.launch_configurations.update(map=str(map_file), namespace="lab_01",
        bridge_config=str(PACKAGE / "config/bridge.yaml"), start_hardware="false",
        start_bridge=enabled, sim="true")
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(PACKAGE))
    actions = module.start(context)[0].get_sub_entities()
    nodes = {perform_substitutions(context, normalize_to_list_of_substitutions(action.node_executable)): action
             for action in actions if isinstance(action, Node)}
    assert nodes["fleet_bridge"].condition.evaluate(context) == (enabled == "true")
    for executable in ("planner_server", "controller_server", "velocity_smoother", "collision_monitor"):
        assert nodes[executable].condition is None
