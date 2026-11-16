# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Check the gripper list and the fail-fast behaviour of the configuration loader."""

from onrobot_description.config_loader import ConfigError, load_config, package_file, require
from onrobot_description.grippers import gripper_names, load_gripper
from onrobot_description.launch_arguments import declare_arguments
import pytest


def test_the_three_grippers_are_listed():
    assert gripper_names() == ['2fg7', '3fg15', 'vgc10']


@pytest.mark.parametrize('gripper', gripper_names())
def test_every_gripper_file_has_what_its_readers_need(gripper):
    description = load_gripper(gripper)
    assert require(description, 'kind') in ('finger', 'vacuum')
    package_file('onrobot_description', require(description, 'geometry'))
    assert require(description, 'tcp_offset') > 0.0
    for joint in require(description, 'joints'):
        for limit in ('lower', 'upper', 'velocity', 'effort'):
            require(description, 'joints', joint, limit)


def test_an_unknown_gripper_names_the_valid_ones():
    with pytest.raises(ConfigError, match=r'\[Fail-Fast\].*2fg7, 3fg15, vgc10'):
        load_gripper('no_such_gripper')


def test_a_missing_key_names_the_key_and_the_file():
    description = load_gripper('2fg7')
    expected = r"\[Fail-Fast\] Missing key 'joints.finger_width.no_such_limit'.*2fg7.yaml"
    with pytest.raises(ConfigError, match=expected):
        require(description, 'joints', 'finger_width', 'no_such_limit')


def test_a_missing_file_is_an_error():
    with pytest.raises(ConfigError, match=r'\[Fail-Fast\]'):
        load_config('/no/such/file.yaml')


def test_launch_arguments_come_from_the_configuration_file():
    config = load_config(package_file('onrobot_description', 'config', 'onrobot_description.yaml'))
    declared = declare_arguments(
        config, 'view_launch_arguments', choices={'onrobot_type': gripper_names()})
    assert [argument.name for argument in declared] == ['onrobot_type', 'prefix', 'ns', 'gui']
    with pytest.raises(ConfigError, match=r'\[Fail-Fast\]'):
        declare_arguments(config, 'view_launch_arguments', choices={'no_such_argument': ['a']})
