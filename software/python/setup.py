from setuptools import setup, find_packages

setup(
    name='RobustCoDesign',
    author='Federico Girlanda',
    version='3.0.0',
    url="https://github.com/FedericoGirlanda/RobustCodesign",
    packages=find_packages(),
    install_requires=[
        # general
        'numpy',
        'matplotlib',
        'scipy',
        'ipykernel',
        'pyyaml',
        'pandas',
        'argparse',
        'sympy',
        'lxml',
        'tikzplotlib',
        'jinja2',

        # optimal control
        #'drake==1.5.0',
        'filterpy',
        'cma'
    ],
    classifiers=[
          'Development Status :: 5 - Stable',
          'Environment :: Console',
          'Intended Audience :: Academic Usage',
          'Programming Language :: Python',
          ],
)