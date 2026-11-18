# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Declare launch arguments from a table of a configuration file, not from values in code."""

from launch.actions import DeclareLaunchArgument

from onrobot_description.config_loader import ConfigError, require


def as_launch_text(value):
    """Return a configuration value as launch argument text (booleans in lower case)."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def declare_arguments(config, table, choices=None):
    """
    Return one DeclareLaunchArgument per entry of config[table].

    Every entry needs 'description' and either 'default' or 'required: true' (an argument the
    caller must give); 'choices' is optional. `choices` maps argument names to lists computed by
    the caller (for example the available grippers) and takes precedence over the file.
    """
    computed = choices or {}
    unknown = set(computed) - set(require(config, table))
    if unknown:
        raise ConfigError(
            f"[Fail-Fast] Choices given for arguments that '{table}' does not declare: "
            f'{sorted(unknown)}')
    declared = []
    for name in require(config, table):
        options = computed.get(name, require(config, table, name).get('choices'))
        details = {
            'description': as_launch_text(require(config, table, name, 'description')),
            'choices': [as_launch_text(option) for option in options] if options else None,
        }
        if not require(config, table, name).get('required', False):
            details['default_value'] = as_launch_text(require(config, table, name, 'default'))
        declared.append(DeclareLaunchArgument(name, **details))
    return declared
