from setuptools import setup

package_name = 'bot_bestie'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hongyi',
    maintainer_email='hongyilin.mail@gmail.com',
    description='Bot Bestie package for navigation and control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = src.my_node:main',
        ],
    },
)
