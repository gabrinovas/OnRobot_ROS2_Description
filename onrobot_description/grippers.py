# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The OnRobot grippers known to this workspace: one YAML file per gripper in config/grippers."""

import os

from ament_index_python.packages import get_package_share_directory

from onrobot_description.config_loader import ConfigError, load_config

PACKAGE = 'onrobot_description'
GRIPPERS_DIRECTORY = os.path.join('config', 'grippers')


def grippers_directory():
    """Return the installed directory that holds one file per gripper."""
    return os.path.join(get_package_share_directory(PACKAGE), GRIPPERS_DIRECTORY)


def gripper_names():
    """Return the names of the available grippers (the file names without extension), sorted."""
    files = [name for name in os.listdir(grippers_directory()) if name.endswith('.yaml')]
    names = sorted(name[:-len('.yaml')] for name in files)
    if not names:
        raise ConfigError(f"[Fail-Fast] No gripper file found in '{grippers_directory()}'")
    return names


def load_gripper(name):
    """Load the description of one gripper; an unknown name is an error listing the valid ones."""
    path = os.path.join(grippers_directory(), f'{name}.yaml')
    if not os.path.isfile(path):
        available = ', '.join(gripper_names())
        raise ConfigError(f"[Fail-Fast] Unknown gripper '{name}'. Available grippers: {available}")
    return load_config(path)
