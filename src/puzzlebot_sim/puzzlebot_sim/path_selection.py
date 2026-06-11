"""Helpers for selecting indexed points from a path parameter file."""

from collections.abc import Sequence
import re

import yaml


def normalize_path_points(raw_points):
    """Convert a flat x/y parameter list into validated point pairs."""
    if (
        isinstance(raw_points, (str, bytes))
        or not isinstance(raw_points, Sequence)
    ):
        raise ValueError('path_points must be a flat list of x,y values')
    if not raw_points:
        raise ValueError('path_points must contain at least one x,y pair')
    if len(raw_points) % 2 != 0:
        raise ValueError(
            'path_points must have an even number of elements (x,y pairs)'
        )

    try:
        values = [float(value) for value in raw_points]
    except (TypeError, ValueError) as exc:
        raise ValueError('path_points values must be numeric') from exc

    return [
        (values[index], values[index + 1])
        for index in range(0, len(values), 2)
    ]


def load_path_points(parameter_file):
    """Load path_generator.path_points from a ROS parameter YAML file."""
    with open(parameter_file, encoding='utf-8') as stream:
        parameter_data = yaml.safe_load(stream) or {}

    for node_name, node_data in parameter_data.items():
        if not isinstance(node_data, dict):
            continue
        if (
            str(node_name).rstrip('/').endswith('path_generator')
            or node_name == '/**'
        ):
            parameters = node_data.get('ros__parameters', {})
            if 'path_points' in parameters:
                return normalize_path_points(parameters['path_points'])

    raise ValueError(
        f'{parameter_file} does not define path_generator.path_points'
    )


def validate_point_index(index, point_count, parameter_name):
    """Validate a point selector, where -1 means legacy behavior."""
    if isinstance(index, bool):
        raise ValueError(f'{parameter_name} must be an integer')

    if isinstance(index, int):
        selected_index = index
    elif isinstance(index, str) and re.fullmatch(r'[+-]?\d+', index.strip()):
        selected_index = int(index)
    else:
        raise ValueError(f'{parameter_name} must be an integer')
    if selected_index < -1:
        raise ValueError(f'{parameter_name} must be -1 or greater')
    if point_count is not None and selected_index >= point_count:
        raise ValueError(
            f'{parameter_name}={selected_index} is out of range for '
            f'{point_count} path points'
        )
    return selected_index


def resolve_path_selection(
    parameter_file,
    initial_point,
    goal_point,
    legacy_initial_x,
    legacy_initial_y,
):
    """Resolve launch selectors to an initial pose and goal point index."""
    initial_point = validate_point_index(
        initial_point,
        None,
        'initial_point',
    )
    goal_point = validate_point_index(
        goal_point,
        None,
        'goal_point',
    )

    if initial_point == -1 and goal_point == -1:
        return float(legacy_initial_x), float(legacy_initial_y), goal_point

    points = load_path_points(parameter_file)
    initial_point = validate_point_index(
        initial_point,
        len(points),
        'initial_point',
    )
    goal_point = validate_point_index(
        goal_point,
        len(points),
        'goal_point',
    )

    if initial_point == -1:
        return float(legacy_initial_x), float(legacy_initial_y), goal_point

    initial_x, initial_y = points[initial_point]
    return initial_x, initial_y, goal_point
