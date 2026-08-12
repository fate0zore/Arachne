from glob import glob

from setuptools import find_packages, setup

package_name = "viewpoint_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Arachne Maintainers",
    maintainer_email="maintainer@example.com",
    description="Viewpoint planning around a target (fibonacci sphere sampling + IK reachability filter).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "viewpoint_planner = viewpoint_planner.viewpoint_planner_node:main",
        ],
    },
)
