from setuptools import find_packages, setup

package_name = 'mtc_aubo_bridge'

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
    description='AUBO S3 SDK bridge (disabled by default, offline stub only).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aubo_bridge = mtc_aubo_bridge.bridge_node:main',
        ],
    },
)
