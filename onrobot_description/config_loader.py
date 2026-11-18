# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Load YAML configuration files and read their values, failing at once when something is missing.

This is the single implementation used by the OnRobot and UR + OnRobot packages: no value has a
fallback written in code, so a missing file or key stops the launch with a message naming both.
"""

import os

from ament_index_python.packages import get_package_share_directory
import yaml


class ConfigError(RuntimeError):
    """A configuration file or one of its keys is missing or malformed."""


def package_file(package, *relative):
    """Return the absolute path of a file installed in the share directory of a package."""
    path = os.path.join(get_package_share_directory(package), *relative)
    if not os.path.isfile(path):
        raise ConfigError(f"[Fail-Fast] File '{path}' of package '{package}' does not exist")
    return path


def load_config(path):
    """Load a YAML file whose top level is a mapping."""
    if not os.path.isfile(path):
        raise ConfigError(f"[Fail-Fast] Configuration file '{path}' does not exist")
    with open(path, 'r') as stream:
        content = yaml.safe_load(stream)
    if not isinstance(content, dict):
        raise ConfigError(f"[Fail-Fast] Configuration file '{path}' must contain a mapping")
    content['__file__'] = path
    return content


def require(config, *keys):
    """Return config[key1][key2]...; raise naming the missing key and the file it should be in."""
    source = config.get('__file__', '<unknown file>')
    value = config
    for depth, key in enumerate(keys):
        if not isinstance(value, dict) or key not in value:
            missing = '.'.join(keys[:depth + 1])
            raise ConfigError(f"[Fail-Fast] Missing key '{missing}' in '{source}'")
        value = value[key]
    return value
