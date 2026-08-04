from glob import glob

from setuptools import find_packages, setup

package_name = "arachne_operator"

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
    description="Lightweight operator status panel for Arachne.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "action_chunk_translator = arachne_operator.action_chunk_translator:main",
            "arachne = arachne_operator.cli:main",
            "apriltag_hand_eye_calibrator = arachne_operator.apriltag_hand_eye_calibrator:main",
            "apriltag_nav_initializer = arachne_operator.apriltag_nav_initializer:main",
            "aubo_readonly_probe = arachne_operator.aubo_readonly_probe:main",
            "demo_orchestrator = arachne_operator.demo_orchestrator:main",
            "grasp_task_server = arachne_operator.grasp_task_server:main",
            "grasp_preview_pipeline = arachne_operator.grasp_preview_pipeline:main",
            "hand_eye_calibration_check = arachne_operator.hand_eye_calibration_check:main",
            "intrinsic_cal_solve = arachne_operator.intrinsic_cal_solve:main",
            "operator_panel = arachne_operator.operator_panel:main",
            "raw_image_viewer = arachne_operator.raw_image_viewer:main",
            "real_hardware_acceptance_test = arachne_operator.real_hardware_acceptance_test:main",
            "road_cleanup_task_server = arachne_operator.road_cleanup_task_server:main",
            "sequence_executor = arachne_operator.sequence_executor:main",
            "step_cleanup_demo = arachne_operator.step_cleanup_demo:main",
            "teach_panel = arachne_operator.teach_panel:main",
            (
                "teach_visualization_joint_states = "
                "arachne_operator.teach_visualization_joint_states:main"
            ),
        ],
    },
)
