from setuptools import find_packages, setup

package_name = 'knowledge_graph_server'

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
    maintainer='rod',
    maintainer_email='rodrigo.perez@urjc.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'server = knowledge_graph_server.kg_server:main',
        'client = knowledge_graph_server.kg_sample_client:main',
        ],
    },
)
