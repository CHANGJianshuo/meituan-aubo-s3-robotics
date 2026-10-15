from setuptools import find_packages, setup

package_name = 'mtc_motion_execution'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meituan_challenge team',
    maintainer_email='team@example.invalid',
    description='Motion execution abstraction layer with mock backend (offline default).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_executor = mtc_motion_execution.motion_executor_node:main',
        ],
    },
)
