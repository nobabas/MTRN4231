from setuptools import find_packages, setup

package_name = 'ur5_moveit_rqt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Minh Thang Pham',
    maintainer_email='z5423085@ad.unsw.edu.au',
    description='rqt plugin to control UR5 MoveIt services',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur5_moveit_rqt = ur5_moveit_rqt.ur5_moveit_plugin:main'
        ],
        'rqt_gui_py.plugin': [
            'ur5_moveit_rqt = ur5_moveit_rqt.ur5_moveit_plugin:UR5MoveItPlugin'
        ],
    },
)

