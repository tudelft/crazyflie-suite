from setuptools import setup, find_packages

setup(
    name="crazyflie-suite",
    packages=find_packages(),
    install_requires=["cflib", "numpy", "pyyaml", "pre-commit"],
)
