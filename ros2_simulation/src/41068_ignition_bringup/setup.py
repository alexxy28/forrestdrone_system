from setuptools import find_packages, setup

package_name = '41068_ignition_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_dir={'': '.'},
    install_requires=['setuptools', 'rqt_gui', 'rqt_gui_py', 'python_qt_binding', 'numpy'],
    entry_points={
        'rqt_gui.plugins': [
            'TerrainMap = rqt_terrain_plugin.terrain_plugin:TerrainMapPlugin',
        ],
    },
)
