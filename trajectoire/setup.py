import os
from glob import glob
from setuptools import find_packages, setup

package_name = "trajectoire"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="paul",
    maintainer_email="paul@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "trajectoire = trajectoire.trajectoire:main",
            "init_service_decollage = trajectoire.init_service_decollage:main",
        ],
    },
)
