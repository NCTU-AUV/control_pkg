from setuptools import setup
from os.path import join, dirname

__version__ = '1.1.0'
__author__ = 'Bobson Lin'

with open(join(dirname(__file__), 'requirements.txt'), 'r') as f:
    install_requires = f.read().split("\n")

setup(
    name='rabboni-python-sdk',
    version=__version__,
    license='MIT',
    author=__author__,
    install_requires=install_requires,
    packages=['rabboni'],
    data_files=[('', ['hidapi.dll', 'hidapi.lib', 'requirements.txt', 'generate-udev-rules.sh']),
                ('rabboni', ['rabboni/muls.pkl'])],
    zip_safe=False,
    keywords='rabboni sdk'
)
