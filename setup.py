from glob import glob

from setuptools import setup

package_name = "ur3_llm_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/prompts", glob("prompts/*.txt")),
        ("share/" + package_name + "/worlds", glob("worlds/*.world")),
    ],
    install_requires=["setuptools", "requests", "PyYAML"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="Điều khiển UR3/UR3e bằng câu lệnh tự nhiên: LLM (Ollama hoặc 9Router) -> skill plan -> MoveIt 2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "llm_commander = ur3_llm_control.commander_node:main",
        ],
    },
)
