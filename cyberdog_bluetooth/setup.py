from setuptools import setup

package_name = 'cyberdog_bluetooth'

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
    maintainer='wenlinfeng',
    maintainer_email='wenlinfeng@xiaomi.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cyberdog_bluetooth_main = cyberdog_bluetooth.cyberdog_bluetooth_main:main'
        ],
    },
)
