from setuptools import find_packages, setup

package_name = 'moka_plan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/plan.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tong',
    maintainer_email='tong166159@163.com',
    description='Motion planning module for ROS2 and MoveIt2',
    license='Apace-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'launch_plan_node = moka_plan.scripts.launch_plan:main', 
        ],
    },
)
