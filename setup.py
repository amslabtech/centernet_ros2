from setuptools import find_packages
from setuptools import setup

package_name = 'centernet_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    # py_modules=[
    #     'scripts.node_test'
    # ],
    data_files=[],
    install_requires=['setuptools'],
    zip_safe=True,
    author='amsl',
    author_email='',
    maintainer='',
    maintainer_email='',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
    ),
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_test = scripts.node_test:main'
        ],
    },
)
