from setuptools import find_packages, setup

package_name = 'ur3_llm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chuc',
    maintainer_email='chuc@todo.todo',
    description='UR3e LLM skill based control',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_skills = ur3_llm_control.robot_skills:main',
        ],
    },
)
