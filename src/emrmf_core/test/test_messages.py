from emrmf_core.messages import clamp, decode, encode, merge_landmarks, weighted_pose_average


def test_json_roundtrip():
    payload = {'robot_id': 'robot_1', 'theta': 0.5}
    assert decode(encode(payload)) == payload


def test_clamp_bounds():
    assert clamp(-1.0) == 0.0
    assert clamp(2.0) == 1.0


def test_weighted_fusion_helpers():
    items = [
        {'pose': [0.0, 0.0, 0.0], 'theta': 1.0, 'landmarks': [{'id': 'A', 'x': 0.0, 'y': 0.0}]},
        {'pose': [2.0, 0.0, 0.0], 'theta': 1.0, 'landmarks': [{'id': 'A', 'x': 2.0, 'y': 0.0}]},
    ]
    assert weighted_pose_average(items)[0] == 1.0
    assert merge_landmarks(items)[0]['x'] == 1.0
