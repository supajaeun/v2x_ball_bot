from setuptools import setup

package_name = 'v2x_ball_bot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ss802',
    maintainer_email='YOUR_EMAIL_HERE',
    description='Ball detector node using YOLOv8 and Orbbec depth camera',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_detector_node = v2x_ball_bot_control.ball_detector_node:main',
        ],
    },
)
