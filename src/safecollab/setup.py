import os
from glob import glob

from setuptools import find_packages, setup

package_name = "safecollab"

setup(
    name=package_name,
    version="0.7.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # urdf/xacro files -- required so FindPackageShare('safecollab') resolves
        # cell.xacro (which pulls in the UR5e macro) after `colcon build`
        # urdf/*.sdf  -- operator.sdf (yellow visual body) must also be installed
        # so that cell.launch.py's `ros_gz_sim create -file` can resolve the path
        # via FindPackageShare('safecollab')/urdf/operator.sdf after colcon build.
        (
            os.path.join("share", package_name, "urdf"),
            glob("urdf/*.xacro") + glob("urdf/*.sdf"),
        ),
        # config files (controllers/risk/safety yaml + view.rviz) -- same reason.
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml") + glob("config/*.rviz"),
        ),
        # SRDF (MoveIt semantic model of the cell) -- read by planner_node's
        # MoveItConfigsBuilder from the installed share (kinematics/pilz/joint-limit
        # yamls live in config/ above so MoveItConfigsBuilder's defaults find them).
        (
            os.path.join("share", package_name, "srdf"),
            glob("srdf/*.xacro"),
        ),
        # launch files -- required so FindPackageShare('safecollab') resolves them
        # after `colcon build`.
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # world files -- cell.sdf must be installed so cell.launch.py can resolve
        # FindPackageShare('safecollab')/worlds/cell.sdf. cell.sdf adds the gz
        # Sensors system that the stock empty.sdf omits (without it the camera
        # never renders); see worlds/cell.sdf header and README.md.
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nicola Guerra",
    maintainer_email="nicola.ng2k@gmail.com",
    description="SafeCollab: a perception-driven, human-aware collaborative kitting cell "
    "implementing ISO/TS 15066 Speed-and-Separation Monitoring.",
    license="TODO",
    tests_require=["pytest", "pytest-cov"],
    entry_points={
        "console_scripts": [
            "planner_node = safecollab.planner_node:main",
            "human_node = safecollab.human_node:main",
            "perception_node = safecollab.perception_node:main",
            "safety_monitor = safecollab.safety_monitor:main",
            "motion_node = safecollab.motion_node:main",
            "hud_node = safecollab.hud_node:main",
        ],
    },
)
