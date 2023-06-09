from setuptools import setup

package_name = 'cyberdog_wifi'

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
    author='dukun',
    author_email='dukun1@xiaomi.com',
    maintainer='wenlinfeng',
    maintainer_email='wenlinfeng@xiaomi.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Wifi ROS2 interface for Cyberdog.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cyberdog_wifi = cyberdog_wifi.cyberdog_wifi:main'
        ],
    },
)
