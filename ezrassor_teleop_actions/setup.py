from setuptools import setup

package_name = 'ezrassor_teleop_actions'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andyp',
    maintainer_email='andy.ponce@outlook.com',
    description='ROS2 version of ezrassor_teleop_actions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'send_goal_keyboard_cancel = ezrassor_teleop_actions.send_goal_keyboard_cancel',
                    'send_goal_list = ezrassor_teleop_actions.send_goal_list',
                    'send_goal_no_cancel = ezrassor_teleop_actions.send_goal_no_cancel',
                    'send_goal_timer_cancel = ezrassor_teleop_actions.send_goal_timer_cancel',
                    'teleop_action_server = ezrassor_teleop_actions.teleop_action_server:main'
        ],
    },
)
