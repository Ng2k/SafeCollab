import os
from glob import glob

from setuptools import find_packages, setup

package_name = "safecollab"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # launch files — required so FindPackageShare('safecollab') resolves them
        # after `colcon build`.  Added by Stream G (cell.launch.py ownership, §6).
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nicola",
    maintainer_email="TODO@example.com",
    description="SafeCollab: a perception-driven, human-aware collaborative kitting cell "
    "implementing ISO/TS 15066 Speed-and-Separation Monitoring.",
    license="TODO",
    tests_require=["pytest", "pytest-cov"],
    entry_points={
        "console_scripts": [
            "task_node = safecollab.task_node:main",
            "human_node = safecollab.human_node:main",
            "perception_node = safecollab.perception_node:main",
            "safety_monitor = safecollab.safety_monitor:main",
            "motion_node = safecollab.motion_node:main",
        ],
    },
)
