from setuptools import setup

package_name = "drone_control_framework"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rafael",
    maintainer_email="rafael@email.com",
    description="Drone Control Framework",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "offboard_controller = drone_control_framework.offboard_controller:main",
        ],
    },
)