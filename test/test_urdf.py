# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Check that xacro generates a valid URDF for every gripper file."""

import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from onrobot_description.config_loader import package_file
from onrobot_description.grippers import gripper_names, load_gripper
import pytest


def generate(gripper, *arguments):
    """Run xacro on the standalone gripper description and return the finished process."""
    description = package_file('onrobot_description', 'urdf', 'onrobot.urdf.xacro')
    return subprocess.run(['xacro', description, f'onrobot_type:={gripper}', *arguments],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


@pytest.mark.parametrize('gripper', gripper_names())
def test_every_gripper_generates_a_valid_urdf(gripper):
    result = generate(gripper, 'prefix:=test_')
    assert result.returncode == 0, f'xacro failed for {gripper}:\n{result.stderr}'
    root = ET.fromstring(result.stdout)
    assert root.tag == 'robot'

    links = {element.attrib.get('name') for element in root.findall('link')}
    assert {'test_onrobot_base_link', 'test_gripper_tcp'} <= links

    joints = {element.attrib.get('name') for element in root.findall('joint')}
    for joint in load_gripper(gripper)['joints']:
        assert f'test_{joint}' in joints, f'{gripper}: joint {joint} is not in the URDF'

    check_urdf = shutil.which('check_urdf')
    if check_urdf:
        with tempfile.NamedTemporaryFile(suffix='.urdf', mode='w', delete=False) as stream:
            stream.write(result.stdout)
        try:
            checked = subprocess.run([check_urdf, stream.name], stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True)
            assert checked.returncode == 0, (
                f'check_urdf failed for {gripper}:\n{checked.stderr}\n{checked.stdout}')
        finally:
            os.remove(stream.name)


def test_the_gripper_is_required():
    description = package_file('onrobot_description', 'urdf', 'onrobot.urdf.xacro')
    missing = subprocess.run(['xacro', description],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert missing.returncode != 0
    assert 'onrobot_type' in missing.stderr


def test_the_prefix_is_empty_unless_given():
    root = ET.fromstring(generate(gripper_names()[0]).stdout)
    assert 'onrobot_base_link' in {element.attrib.get('name') for element in root.findall('link')}


def test_an_unknown_gripper_is_an_error():
    unknown = generate('no_such_gripper')
    assert unknown.returncode != 0
    assert 'no_such_gripper.yaml' in unknown.stderr
