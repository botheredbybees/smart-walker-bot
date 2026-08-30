from walker_companion_app.occupancy_grid_json import grid_to_json


def test_grid_to_json_basic_fields():
    result = grid_to_json(width=4, height=3, resolution=0.05, origin_x=-1.0, origin_y=-1.0, data=[0, 100, -1, 50])
    assert result == {
        'width': 4,
        'height': 3,
        'resolution': 0.05,
        'origin_x': -1.0,
        'origin_y': -1.0,
        'data': [0, 100, -1, 50],
    }


def test_grid_to_json_converts_data_to_plain_list():
    result = grid_to_json(width=2, height=1, resolution=0.1, origin_x=0.0, origin_y=0.0, data=(1, 2))
    assert result['data'] == [1, 2]
    assert isinstance(result['data'], list)


def test_grid_to_json_empty_data():
    result = grid_to_json(width=0, height=0, resolution=0.1, origin_x=0.0, origin_y=0.0, data=[])
    assert result['data'] == []
