from setuptools import setup

package_name = 'bot_bestie'

setup(
    name=package_name,
    version='0.0.1',
    packages=['bot_bestie'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hongyi',
    maintainer_email='hongyilin.mail@gmail.com',
    description='Bot Bestie Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_controller = nodes.global_controller:main',
        ],
    },
)
