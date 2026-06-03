from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).parents[3]
PACKAGE = ROOT / 'src' / 'emrmf_experiments'


def test_package_metadata_and_launch_files_exist():
    ET.parse(PACKAGE / 'package.xml')
    assert (PACKAGE / 'setup.py').exists()
    assert (PACKAGE / 'config' / 'emrmf_experiment_params.yaml').exists()
    assert (PACKAGE / 'launch' / 'emrmf_experiment_validation.launch.py').exists()
    assert (PACKAGE / 'launch' / 'emrmf_experiment_orchestrator.launch.py').exists()


def test_proxy_defaults_document_required_topics():
    config = (PACKAGE / 'config' / 'emrmf_experiment_params.yaml').read_text(encoding='utf-8')
    for topic in [
        '/robot1/local_map',
        '/robot2/local_map',
        '/robot1/pose_update',
        '/robot2/pose_update',
        '/map_fusion/inter_robot_constraints',
        '/proxy/robot1/local_map',
        '/proxy/robot2/local_map',
        '/proxy/map_fusion/inter_robot_constraints',
    ]:
        assert topic in config
