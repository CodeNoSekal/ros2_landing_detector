from setuptools import setup
from glob import glob

package_name = 'landing_zone_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=['landing_zone_detector_py'],  # только имя папки с кодом
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # если будут launch-файлы
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Landing zone detection from NPY point clouds.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = landing_zone_detector_py.publisher:main',
            'subscriber = landing_zone_detector_py.subscriber:main',
        ],
    },
)
