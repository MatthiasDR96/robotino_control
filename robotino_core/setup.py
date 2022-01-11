from distutils.core import setup

setup(
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
    scripts=['scripts/main.py', 'scripts/main_ta_agent.py', 'scripts/main_ro_agent.py', 'scripts/main_rm_agent.py'],
    install_requires=['yaml', 'numpy', 'pandas', 'mysql-connector-python', 'matplotlib']
)
