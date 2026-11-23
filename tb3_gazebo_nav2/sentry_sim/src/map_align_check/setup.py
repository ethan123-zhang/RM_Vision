from setuptools import setup

package_name = 'map_align_check'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ethan',
    maintainer_email='ethan@example.com',
    description='Odin1 map frame vs field/CAD frame offset check (S2c)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_align_check = map_align_check.cli:main',
        ],
    },
)