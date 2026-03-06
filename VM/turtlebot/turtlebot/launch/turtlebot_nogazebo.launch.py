"""Launch the TurtleBot3 (Waffle) WITHOUT Gazebo

   In the last section, please COMMENT/UNCOMMENT the pieces that you need!
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import AppendEnvironmentVariable
from launch.actions                    import ExecuteProcess
from launch.actions                    import IncludeLaunchDescription
from launch.actions                    import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    pkgfolder = pkgdir('turtlebot')

    # # WORLD: Locate the Gazebo world.  Regular house or alternate world.
    # worldfile = os.path.join(
    #     pkgdir('turtlebot3_gazebo'), 'worlds', 'turtlebot3_house.world')
    # #   pkgdir('turtlebot3_gazebo'), 'worlds', 'turtlebot3_dqn_stage4.world')

    # # BRIDGE: Define the Gazebo-ROS bridge parameters.
    # bridgefile = os.path.join(
    #     pkgdir('turtlebot3_gazebo'), 'params', 'turtlebot3_waffle_bridge.yaml')

    # # MODEL: Locate the Gazebo TurtleBot model: With clean or noisy lidar.
    # # The original is: turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
    # modelfile  = os.path.join(pkgfolder, 'models', 'turtlebot.sdf')
    # withcamera = os.path.join(pkgfolder, 'models', 'turtlebot_full.sdf')
    # cleanlidar = os.path.join(pkgfolder, 'models', 'turtlebot_cleanlidar.sdf')
    # noisylidar = os.path.join(pkgfolder, 'models', 'turtlebot_noisylidar.sdf')

    # URDF: Locate and load the TurtleBot URDF, colored orange to stand out.
    # The original is: turtlebot3_description/urdf/turtlebot3_waffle.urdf
    urdffile = os.path.join(pkgfolder, 'urdf', 'turtlebot.urdf')
    with open(urdffile, 'r') as file:
        robot_description = file.read()

    # MAP: Locate the good map
    mapfile = os.path.join(pkgfolder, 'maps', 'goodmap.yaml')

    # RVIZ: Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgfolder, 'rviz', 'viewturtlebot.rviz')

    # BAGS: Locate the BAG folder.  The first five are in the main house.
    # The alternate recordings use the alternate world (see above).
    #bagfolder = os.path.join(pkgfolder, 'bags', 'singleroom_cleanlaser')
    bagfolder = os.path.join(pkgfolder, 'bags', 'leftside_cleanlaser')
    #bagfolder = os.path.join(pkgfolder, 'bags', 'rightside_cleanlaser')
    #bagfolder = os.path.join(pkgfolder, 'bags', 'singleroom_noisylaser')
    #bagfolder = os.path.join(pkgfolder, 'bags', 'leftside_noisylaser')

    #bagfolder = os.path.join(pkgfolder, 'bags', 'alternate_cleanlaser')
    #bagfolder = os.path.join(pkgfolder, 'bags', 'alternate_noisylaser')


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    ### SIMULATION.
    # # Gazebo Server.  Use the standard launch description.
    # incl_gzserver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkgdir('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args':          ['-r -s -v2 ', worldfile],
    #                       'on_exit_shutdown': 'true'}.items())

    # # Gazebo Client (Window).  Optional.  Use the standard launch description.
    # incl_gzclient = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkgdir('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args':          '-g -v2 ',
    #                       'on_exit_shutdown': 'true'}.items())

    # # Gazebo Path.  Use if either server or client are started.
    # set_gazebo_path = AppendEnvironmentVariable(
    #     'GZ_SIM_RESOURCE_PATH',
    #     os.path.join(pkgdir('turtlebot3_gazebo'), 'models'))

    # # Gazebo-ROS Bridge/Image-Bridge.  Use to connect the Gazebo to ROS.
    # node_bridge = Node(
    #     name       = 'ros_gz_bridge',
    #     package    = 'ros_gz_bridge',
    #     executable = 'parameter_bridge',
    #     output     = 'screen',
    #     arguments  = ['--ros-args', '-p', f'config_file:={bridgefile}'])

    # node_image = Node(
    #     name       = 'ros_gz_image',
    #     package    = 'ros_gz_image',
    #     executable = 'image_bridge',
    #     output     = 'screen',
    #     arguments  = ['/camera/image_raw'])

    # # Spawn the TurtleBot.  This could either be the original, a version
    # # with a perfectly clean lidar scanner, or a version with a noisy lidar.
    # node_spawn_turtlebot_plain = Node(
    #     name       = 'spawn_turtlenbot',
    #     package    = 'ros_gz_sim',
    #     executable = 'create',
    #     output     = 'screen',
    #     arguments  = ['-name', 'waffle', '-file', modelfile,
    #                   '-x', '-2.0', '-y', '1.0', '-z', '0.01'])

    # node_spawn_turtlebot_withcamera = Node(
    #     name       = 'spawn_turtlenbot',
    #     package    = 'ros_gz_sim',
    #     executable = 'create',
    #     output     = 'screen',
    #     arguments  = ['-name', 'waffle', '-file', withcamera,
    #                   '-x', '-2.0', '-y', '1.0', '-z', '0.01'])

    # node_spawn_turtlebot_cleanlidar = Node(
    #     name       = 'spawn_turtlenbot',
    #     package    = 'ros_gz_sim',
    #     executable = 'create',
    #     output     = 'screen',
    #     arguments  = ['-name', 'waffle', '-file', cleanlidar,
    #                   '-x', '-2.0', '-y', '1.0', '-z', '0.01'])

    # node_spawn_turtlebot_noisylidar = Node(
    #     name       = 'spawn_turtlenbot',
    #     package    = 'ros_gz_sim',
    #     executable = 'create',
    #     output     = 'screen',
    #     arguments  = ['-name', 'waffle', '-file', noisylidar,
    #                   '-x', '-2.0', '-y', '1.0', '-z', '0.01'])

    ### SIMULATION ALTERNATIVE
    # ROS Bag Playback.
    cmd_playback = ExecuteProcess(
            cmd    = ['ros2', 'bag', 'play', '--clock', '10', bagfolder],
            output = 'screen')


    ### RVIZ
    # This is configured to show the robot, laser scan, and the map.
    node_rviz = Node(
        name       = 'rviz',
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        parameters = [{'use_sim_time': True}],
        on_exit    = Shutdown())

    ### URDF PROCESSING
    # Robot State Publisher, to create the TF frames and publish the model.
    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher',
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description},
                      {'use_sim_time':      True}])


    ### LOCALIZATION
    # Select one of the following.  Either assume perfect odometry
    # (which makes the odom frame match the map frame), or assume
    # noisy odometry (which drifts the two frames apart).
    node_perfectlocalization = Node(
        name       = 'localization',
        package    = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments  = ['--frame-id', 'map', '--child-frame-id', 'odom',
                      '--x', '-2.0', '--y', '1.0'])

    node_noisylocalization = Node(
        name       = 'localization',
        package    = 'turtlebot',
        executable = 'noisylocalization',
        output     = 'screen',
        parameters = [{'use_sim_time': True},
                      {'x':            -2.0},
                      {'y':             1.0}])


    ### MAPPING
    # The map server publishes an existing map.  It is part of the
    # navigation stack, which requires a lifecycle manager to start.
    node_map_server = Node(
        name       = 'map_server',
        package    = 'nav2_map_server',
        executable = 'map_server',
        output     = 'screen',
        parameters = [{'use_sim_time':  True},
                      {'yaml_filename': mapfile},
                      {'topic_name':    "map"},
                      {'frame_id':      "map"}])

    node_lifecycle = Node(
        name       = 'lifecycle_manager_localization',
        package    = 'nav2_lifecycle_manager',
        executable = 'lifecycle_manager',
        output     = 'screen',
        parameters = [{'use_sim_time': True},
                      {'autostart':    True},
                      {'node_names':   ['map_server']}])

    ### OUR CODE
    # Build the map ourselves.
    node_buildmap = Node(
        name       = 'buildmap',
        package    = 'turtlebot',
        executable = 'buildmap',
        output     = 'screen',
        parameters = [{'use_sim_time': True}])


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST

    # Return the description, built as a python list.
    return LaunchDescription([

        # SIMULATION: Start the simulation. If you use Gazebo, you
        # have to start the server, define the path (so it knows where
        # to grab the models), and create "bridges" to ROS.
        #incl_gzserver,
        #set_gazebo_path,
        #node_bridge,
        #node_image,

        # ROBOT: To simulate, you need to spawn ONE of the robots.
        # Select the model with the clean or noisy lidar.
        #node_spawn_turtlebot_plain,
        #node_spawn_turtlebot_withcamera,
        #node_spawn_turtlebot_cleanlidar,
        #node_spawn_turtlebot_noisylidar,

        # PLAYBACK: To play a bag file, do not start the simulation or
        # spawn a robot above, nor the Gazebo client below.  Instead
        # start the playback (select the BAG folder at the top).
        cmd_playback,

        # VIEWER: Select a viewer.  RVIZ or Gazebo client.
        #incl_gzclient,
        node_rviz,

        # FRAMES: For anything outside Gazebo, you will need the robot
        # state publisher (to read the URDF and place the frames on
        # the robot), as well as ONE of the localizations (to locate
        # the robot in the map): perfect or noisy.
        node_robot_state_publisher,
        node_perfectlocalization,
        #node_noisylocalization,

        # MAP: Use either the first two lines (together, to show the
        # perfect map).  OR the last line to start your code...
        node_lifecycle,
        node_map_server,
        #node_buildmap,

    ])
