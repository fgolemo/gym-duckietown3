from setuptools import setup

setup(name='gym_duckietown3',
      version='1.0',
      install_requires=[ # I'm not sure these versions are actually required
          'gym>=0.9.4', # but this is what I had installed at the time
          'pybullet>=1.7.6',
          'numpy>=1.13.3',
          'matplotlib>=2.1.0'
      ]
)
