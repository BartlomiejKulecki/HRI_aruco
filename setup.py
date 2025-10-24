from setuptools import find_packages, setup

package_name = 'hri_aruco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bartlomiej Kulecki',
    maintainer_email='b.kulecki@gmail.com',
    description='Simple Human-Robot Interaction using ArUco markers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hri_node = hri_aruco.hri_node:main',
            'hri_node_goal_pub = hri_aruco.hri_node_goal_pub:main'
        ],
    },
)
