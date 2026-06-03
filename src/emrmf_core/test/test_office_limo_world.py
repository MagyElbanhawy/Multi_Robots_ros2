from pathlib import Path
import xml.etree.ElementTree as ET

WORLD_PATH = Path(__file__).parents[1] / 'worlds' / 'emrmf_office_indoor.world'
LAUNCH_PATH = Path(__file__).parents[1] / 'launch' / 'emrmf_limo_scalability.launch.py'
PLACEHOLDER_PATH = Path(__file__).parents[1] / 'models' / 'limo_placeholder' / 'limo_placeholder.urdf.xacro'


def test_office_world_is_feature_rich_and_lightweight():
    root = ET.parse(WORLD_PATH).getroot()
    world = root.find('world')
    assert world is not None
    model_names = {model.attrib['name'] for model in world.findall('model')}

    assert world.find("plugin[@filename='libgazebo_ros_init.so']") is not None
    assert world.find("plugin[@filename='libgazebo_ros_factory.so']") is not None
    assert len(world.findall('light')) >= 3
    assert 'ground_plane' in model_names
    assert len([name for name in model_names if 'wall' in name]) >= 12
    assert len([name for name in model_names if 'office_desk' in name]) >= 6
    assert 'meeting_table' in model_names
    assert len([name for name in model_names if 'chair' in name]) >= 8
    assert len([name for name in model_names if 'shelf_cabinet' in name]) >= 4
    assert len([name for name in model_names if 'static_box' in name]) >= 6
    assert len([name for name in model_names if 'round_column' in name]) >= 4


def test_office_world_has_safe_limo_spawn_markers_and_large_floor():
    root = ET.parse(WORLD_PATH).getroot()
    model_names = {model.attrib['name'] for model in root.findall('world/model')}
    for robot_index in range(1, 6):
        assert f'limo_{robot_index}_spawn_marker' in model_names

    floor_size = root.find("world/model[@name='ground_plane']/link/collision/geometry/box/size")
    assert floor_size is not None
    width, length, _ = [float(value) for value in floor_size.text.split()]
    assert width >= 12.0
    assert length >= 10.0


def test_limo_launch_defaults_to_office_world_and_supports_screenshot_counts():
    launch_text = LAUNCH_PATH.read_text()
    assert 'emrmf_office_indoor.world' in launch_text
    assert "VALID_ROBOT_COUNTS = {0, 2, 3, 4, 5}" in launch_text
    assert "limo_" in launch_text
    assert "gazebo_emrmf_bridge_node" in launch_text
    assert "lora_network_simulator" in launch_text
    assert "trust_factor_node" in launch_text
    assert "global_fusion_node" in launch_text


def test_limo_placeholder_exposes_required_gazebo_topics():
    text = PLACEHOLDER_PATH.read_text()
    assert 'libgazebo_ros_diff_drive.so' in text
    assert 'libgazebo_ros_ray_sensor.so' in text
    assert '<remapping>cmd_vel:=cmd_vel</remapping>' in text
    assert '<remapping>odom:=odom</remapping>' in text
    assert '<remapping>~/out:=scan</remapping>' in text
    assert '<namespace>$(arg robot_namespace)</namespace>' in text
