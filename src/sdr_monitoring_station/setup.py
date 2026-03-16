from setuptools import find_packages, setup

package_name = 'sdr_monitoring_station'

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
    maintainer='ubuntu',
    maintainer_email='study.iru@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 파일명이 gui_node.py이고 그 안에 def main()이 있어야 합니다.
            'gui_node = sdr_monitoring_station.gui_node:main', 
        ],
    },
)
