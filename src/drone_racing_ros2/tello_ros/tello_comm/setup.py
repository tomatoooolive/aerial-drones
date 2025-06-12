from setuptools import setup

package_name = 'tello_comm'

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
    maintainer='ayesha',
    maintainer_email='mst.ayesha1702@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leader_cmd_pub = tello_comm.leader_cmd_pub:main',
            'follower_cmd_listener = tello_comm.follower_cmd_listener:main',
            'leader_handshake = tello_comm.leader_handshake:main',
            'follower_handshake = tello_comm.follower_handshake:main',
        ],
    },
)
