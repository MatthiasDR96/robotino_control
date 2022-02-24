from distutils.core import setup

setup(
    name='robotino_core',
    version='1.0.0',
    url='https://github.com/MatthiasDR96/robotino_demo.git',
    license='BSD',
    author='Matthias De Ryck',
    author_email='matthias.deryck@kuleuven.be',
    description='Demo software for the Robotino AGVs at the Ultimate Factory in the KU Leuven Campus in Bruges',
    packages=['robotino_demo'],
    package_data={'robotino_demo': ['params/*.yaml', 'locations/*.yaml']},
    package_dir={'': 'src'},
    install_requires=['yaml', 'numpy']
)
