import xml.etree.ElementTree as ET
from pathlib import Path


WORLD_PATH = Path(__file__).parents[1] / 'worlds' / 'emrmf_indoor.world'


def test_indoor_world_has_required_gazebo_classic_elements():
    root = ET.parse(WORLD_PATH).getroot()
    world = root.find('world')
    assert world is not None
    model_names = {model.attrib['name'] for model in world.findall('model')}
    assert 'ground_plane' in model_names
    assert {'wall_north', 'wall_south', 'wall_east', 'wall_west'} <= model_names
    assert {'rack_west', 'rack_east', 'crate_stack_1', 'pillar_center'} <= model_names
    assert {f'robot_spawn_{index}' for index in range(1, 6)} <= model_names
    assert world.find("plugin[@filename='libgazebo_ros_init.so']") is not None
    assert world.find("plugin[@filename='libgazebo_ros_factory.so']") is not None
    assert len(world.findall('light')) >= 2


def test_indoor_world_is_approximately_15_by_10_meters():
    root = ET.parse(WORLD_PATH).getroot()
    floor_size = root.find("world/model[@name='ground_plane']/link/collision/geometry/box/size")
    assert floor_size is not None
    assert floor_size.text.split() == ['15', '10', '0.05']
