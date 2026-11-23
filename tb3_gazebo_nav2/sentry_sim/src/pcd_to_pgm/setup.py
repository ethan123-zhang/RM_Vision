from setuptools import setup

package_name = 'pcd_to_pgm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ethan',
    maintainer_email='ethan@example.com',
    description='PointCloud2 + TF -> Nav2 pgm/yaml map, with truth-map compare.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pcd_to_pgm = pcd_to_pgm.build:main',
            'compare_maps = pcd_to_pgm.compare_cli:main',
        ],
    },
)