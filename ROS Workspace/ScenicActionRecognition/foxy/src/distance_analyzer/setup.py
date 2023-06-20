from setuptools import setup

package_name = 'distance_analyzer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        "proxemics_analizer",
        "proxemics_analizer.actor_position",
        "proxemics_analizer.signal_trend_classification",
        "proxemics_analizer.movement_direction",
        "proxemics_analizer.proxemic_zone"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lorenzo',
    maintainer_email='lorenzo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'dist_an = distance_analyzer.distance_analyzer_single:main',
        ],
    },
)
