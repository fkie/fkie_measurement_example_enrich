from setuptools import find_packages, setup

package_name = 'fkie_measurement_example_enrich'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/with_odom.xml', 'launch/with_tf.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Tiderko',
    maintainer_email='alexander.tiderko@fkie.fraunhofer.de',
    description='A simple example how to create a measurement publisher.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'measurement_publisher = fkie_measurement_example_enrich.measurement_publisher:main'
        ],
    },
)
