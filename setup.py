from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['scripts/get_state_world'],
    packages=['dynamical_planner'],
    package_dir={'': 'src'}
)

setup(**d)