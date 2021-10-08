from setuptools import setup, find_packages

setup(
    name="crazyflie-suite",
    packages=find_packages(),
    install_requires=[
        "cflib",
        "cfclient",
        "numpy",
        "pandas",
        "matplotlib",
        "jupyterlab",
        "pyyaml",
        "pre-commit",
    ],
)
