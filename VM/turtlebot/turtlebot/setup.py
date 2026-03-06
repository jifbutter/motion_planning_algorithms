from setuptools import find_packages, setup
from glob import glob

package_name = 'turtlebot'

# Create a mapping of other files to be copied in the src -> install
# build.  This is a list of tuples.  The first entry in the tuple is
# the install folder into which to place things.  The second entry is
# a list of files to place into that folder.
otherfiles = [
    ('share/' + package_name + '/launch', glob('launch/*')),
    ('share/' + package_name + '/rviz',   glob('rviz/*')),
    ('share/' + package_name + '/maps',   glob('maps/*')),
    ('share/' + package_name + '/urdf',   glob('urdf/*')),
    ('share/' + package_name + '/models', glob('models/*')),
]

# Also add all bag files (which are folders and their contents).
for bag in glob('bags/*'):
    otherfiles.append(('share/' + package_name + '/' + bag, glob(bag + '/*')))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]+otherfiles,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133b TurtleBot/Mapping Code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'buildmap          = turtlebot.buildmap:main',
            'slowbuildmap      = turtlebot.slowbuildmap:main',
            'teleop            = turtlebot.teleop:main',
            'noisylocalization = turtlebot.noisylocalization:main',
        ],
    },
)
