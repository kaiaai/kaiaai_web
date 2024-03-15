from setuptools import find_packages, setup

package_name = 'kaiaai_python'

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
    maintainer='Ilia O.',
    maintainer_email='iliao@kaia.ai',
    description='Kaia.ai robot sensing, decision making',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = kaiaai_python.server:main',
        ],
    },
    scripts=[
        'scripts/run_ssl.sh',
    ],
)
