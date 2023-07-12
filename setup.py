# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
import os

try:
    from catkin_pkg.python_setup import generate_distutils_setup

    # fetch values from package.xml
    setup_args = generate_distutils_setup(
        packages=["ros_examples"], package_dir={"": "script"}
    )

    setup(**setup_args)

    # if private dependencies changed and pyproject version wasn't changed add --force-reinstall to upgrade
    # os.system(
    #     "pip install --upgrade --extra-index-url  https://rospypi.github.io/simple/ . --force-reinstall"
    # )

except ImportError:
    setup()
