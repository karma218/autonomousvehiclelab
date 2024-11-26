from setuptools import setup, find_packages

package_name = 'data_logger_py' 

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=["test"]), 
    data_files=[
        ('share/ament_index/resource_index/packages',  
                ['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']), 
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    description='My awesome package.',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'data_logger_py_node = data_logger_py.data_logger_py_node:main'
        ],
    },
)
