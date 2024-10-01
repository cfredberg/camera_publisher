from setuptools import find_packages, setup


package_name = 'camera_publisher'

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
    maintainer='charlie',
    maintainer_email='charliefredberg@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_camera_publisher = camera_publisher.usb_publisher:main',
            'webcam_publisher = camera_publisher.webcam_publisher:main',
        ],
    },
)
