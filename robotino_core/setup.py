## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name='robotino_core',
    version='1.0.0',
    url='https://github.com/MatthiasDR96/robotino_core.git',
    license='BSD',
    author='Matthias De Ryck',
    author_email='matthias.deryck@kuleuven.be',
    description='Control software for the Robotino AGVs at the Ultimate Factory in the KU Leuven Campus in Bruges',
    packages=['robotino_core', 'robotino_core.agv', 'robotino_core.solvers'],
    package_data={'robotino_core': ['params/*.yaml', 'params/*.pgm']},
    package_dir={'': 'src'},
    scripts=['scripts/main.py'],
    install_requires=['numpy', 'pandas', 'mysql-connector-python', 'matplotlib']
)

setup(**setup_args)
