from setuptools import find_packages, setup

package_name = 'kfupm_f1tenth_ultra_instinct_v2'

setup(
    name=package_name,
    version='2.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Ultra Instinct 2.1',
    maintainer_email='raian@kfupm.edu.sa',
    description='Team Ultra Instinct 2.1 - Universal Map-to-Raceline Momentum Tracker (Stage 2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'controller = kfupm_f1tenth_ultra_instinct_v2.ultra_instinct_v2_controller:main',
        ],
    },
)
