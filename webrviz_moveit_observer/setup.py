from setuptools import find_packages, setup

package_name = "webrviz_moveit_observer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/observer.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="Observe MoveIt 2 actions and republish stable WebRviz event topics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "observer_node = webrviz_moveit_observer.observer_node:main",
        ],
    },
)
