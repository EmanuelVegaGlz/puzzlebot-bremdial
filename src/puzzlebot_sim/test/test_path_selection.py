from pathlib import Path

import pytest

from puzzlebot_sim.path_selection import load_path_points
from puzzlebot_sim.path_selection import normalize_path_points
from puzzlebot_sim.path_selection import resolve_path_selection


WORKSPACE = Path(__file__).resolve().parents[3]
PATH_PARAMS = WORKSPACE / 'src/puzzlebot_sim/config/path_params.yaml'


def test_current_path_points_are_loaded_as_indexed_pairs():
    points = load_path_points(PATH_PARAMS)

    assert points == [
        (0.36, -0.27),
        (1.7, -2.04),
        (2.18, -0.3),
        (2.72, -2.72),
        (0.36, -2.72),
        (0.36, -0.94),
    ]


def test_segment_selection_uses_initial_point_coordinates_and_goal_index():
    initial_x, initial_y, goal_point = resolve_path_selection(
        PATH_PARAMS,
        initial_point='0',
        goal_point='1',
        legacy_initial_x='9.0',
        legacy_initial_y='8.0',
    )

    assert (initial_x, initial_y) == (0.36, -0.27)
    assert goal_point == 1


def test_minus_one_preserves_legacy_initial_pose_and_goal_behavior():
    selection = resolve_path_selection(
        PATH_PARAMS,
        initial_point='-1',
        goal_point='-1',
        legacy_initial_x='0.3',
        legacy_initial_y='-0.3',
    )

    assert selection == (0.3, -0.3, -1)


@pytest.mark.parametrize(
    ('initial_point', 'goal_point', 'expected_message'),
    [
        ('6', '1', 'initial_point=6 is out of range'),
        ('0', '6', 'goal_point=6 is out of range'),
        ('0.0', '1', 'initial_point must be an integer'),
    ],
)
def test_invalid_selectors_are_rejected(
    initial_point,
    goal_point,
    expected_message,
):
    with pytest.raises(ValueError, match=expected_message):
        resolve_path_selection(
            PATH_PARAMS,
            initial_point=initial_point,
            goal_point=goal_point,
            legacy_initial_x='0.3',
            legacy_initial_y='-0.3',
        )


def test_path_points_require_complete_numeric_pairs():
    with pytest.raises(ValueError, match='even number'):
        normalize_path_points([0.0, 1.0, 2.0])

    with pytest.raises(ValueError, match='numeric'):
        normalize_path_points([0.0, 'bad'])
