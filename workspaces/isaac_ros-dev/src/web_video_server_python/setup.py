from setuptools import setup

package_name = 'web_video_server_python'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'aiohttp',
        'websockets',
        'opencv-python',
        'numpy',
        'cv_bridge'
    ],
    entry_points={
        'console_scripts': [
            'web_video_server_python = web_video_server_python.server:main',
        ],
    },
)
