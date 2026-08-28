from glob import glob

from setuptools import find_packages, setup

package_name = "arachne_mcp"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "mcp>=1.0,<2"],
    zip_safe=True,
    maintainer="Arachne Maintainers",
    maintainer_email="maintainer@example.com",
    description="MCP exposure for Arachne viewpoint planning and acquisition chain.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mcp_servers = arachne_mcp.mcp_servers:main",
            "viewpoint_planner_mcp = arachne_mcp.viewpoint_planner_mcp:main",
            "viewpoint_acquire_mcp = arachne_mcp.execution_mcp:main",
        ],
    },
)
