from setuptools import setup, find_packages

setup(
    name="sim2real",
    version="0.0.1",
    description="Sim2Real experiment code",
    packages=find_packages(),
    include_package_data=True,  # si añades package_data
    install_requires=[
        "numpy",
        "torch",
        "pyyaml",
        "typing_extensions",
        "git+https://github.com/UniversalRobots/RTDE_Python_Client_Library.git@main"
    ],
    entry_points={
        "console_scripts": [
            "sim2real-ur5 = sim2realimp.ur5:main",
        ],
    },
)