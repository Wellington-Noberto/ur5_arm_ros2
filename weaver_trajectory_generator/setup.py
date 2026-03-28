from setuptools import setup

package_name = 'weaver_trajectory_generator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/weaver_trajectory_generator/config', ['config/weaver_settings.config']),

    ],
    # ...
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Python package for generating robot trajectory from images',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'weaver_trajectory_server = weaver_trajectory_generator.weaver_trajectory_generator:main',
        ],
    },
)
