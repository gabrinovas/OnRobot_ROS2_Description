# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Tests for validating OnRobot XACRO and URDF generation."""

import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import pytest
from ament_index_python.packages import get_package_share_directory


@pytest.fixture(scope="module")
def package_share():
    """Return share directory of the package."""
    try:
        return get_package_share_directory("onrobot_description")
    except Exception:
        return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


@pytest.mark.parametrize("gripper_type", ["2fg7", "3fg15", "vgc10"])
def test_xacro_generation_all_grippers(package_share, gripper_type):
    """Verify that xacro generates valid URDF XML for each gripper model."""
    urdf_xacro = os.path.join(package_share, "urdf", "onrobot.urdf.xacro")
    assert os.path.exists(urdf_xacro), f"Missing {urdf_xacro}"

    cmd = [
        "xacro",
        urdf_xacro,
        f"onrobot_type:={gripper_type}",
        "prefix:=test_",
        "use_mock_hardware:=true",
    ]

    result = subprocess.run(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    assert result.returncode == 0, (
        f"xacro failed for {gripper_type}:\n{result.stderr}"
    )

    xml_content = result.stdout
    assert "<robot" in xml_content

    # Validate XML syntax
    root = ET.fromstring(xml_content)
    assert root.tag == "robot"

    # Verify base link and tcp link exist
    link_names = [elem.attrib.get("name") for elem in root.findall("link")]
    assert "test_onrobot_base_link" in link_names, (
        f"Base link missing in {gripper_type}"
    )
    assert "test_gripper_tcp" in link_names, (
        f"TCP link missing in {gripper_type}"
    )

    # Verify check_urdf if available
    check_urdf_path = shutil.which("check_urdf")
    if check_urdf_path:
        with tempfile.NamedTemporaryFile(suffix=".urdf", mode="w", delete=False) as f:
            f.write(xml_content)
            temp_urdf_path = f.name
        try:
            check_result = subprocess.run(
                [check_urdf_path, temp_urdf_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            assert check_result.returncode == 0, (
                f"check_urdf failed for {gripper_type}:\n"
                f"{check_result.stderr}\n{check_result.stdout}"
            )
        finally:
            if os.path.exists(temp_urdf_path):
                os.remove(temp_urdf_path)
