from inspect import Parameter
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    package_name = 'basic_mobile_robot'
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name)
    default_model_path = os.path.join(pkg_share, 'urdf/nav2_tutorial_description_v2.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_tutorial_v1.rviz')
    default_ekf_config_path = os.path.join(pkg_share, 'config/nav2_tutorial_ekf_v1.yaml')

    # We need 
    # 1. robot_state_publisher (Node)
    # 2. joint_state_publisher / joint_state_publisher_gui (Node)
    # 3. rviz (Node)

    # /opt/ros/galactic/lib/robot_state_publisher/robot_state_publisher --ros-args --params-file /tmp/launch_params_6czh98rt

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
        condition=launch.conditions.LaunchConfigurationEquals('add_joint_state_publisher', '1')
    )
    # /opt/ros/galactic/lib/joint_state_publisher_gui/joint_state_publisher_gui --ros-args -r __node:=joint_state_publisher_gui
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        # condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
        condition=launch.conditions.LaunchConfigurationEquals('add_joint_state_publisher', '2')
        #
        # OR IfCondition(PythonExpression(["'", LaunchConfiguration('bag_version'), "' == 'v2'"]))
        
    )
    # /opt/ros/galactic/lib/rviz2/rviz2 -d /home/smith/dev_ws/install/sam_bot_description2/share/sam_bot_description2/rviz/urdf_config.rviz --ros-args -r __node:=rviz2'
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'nav2_tutorial_bot_v2', '-topic', 'robot_description'],
        output='screen'
    )
    gazebo = launch.actions.ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], 
        output='screen',
        condition=launch.conditions.LaunchConfigurationEquals('add_joint_state_publisher', '0')
    )
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, default_ekf_config_path), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
)
    # Create the launch description and populate
    ld = launch.LaunchDescription()
    # DeclareLaunchArgument(name='gui', default_value='True', description='......')
    #   ros2 launch <pkg> <launch.py> gui:=True
    # 
    # launch_ros.actions.Node
    #   parameters=[
    #       'aaaa',
    #       'bbbb',
    #       {'a':'1', 'b':'2'}
    # ]
    # arguments=['-d', rviz_config_file]
    # 
    # IncludeLaunchDescription 
    #   PythonLaunchDescriptionSource(os.path.join(<PATH>, 'gzserver.launch.py')),
    #   launch_arguments={'world': world, 'verbose':'true'}.items()
    #       mapping to child's DeclareLaunchArgument

    # 0: no, 1: yes, 2: yes+gui
    ld.add_action(
        launch.actions.DeclareLaunchArgument(name='add_joint_state_publisher', default_value='0',
                                            description='Flag to enable joint_state_publisher')
                                            )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file')
                                            )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file')
                                            )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time')
                                            )
    ld.add_action(joint_state_publisher_node)
    ld.add_action(gazebo)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity)
    ld.add_action(robot_localization_node)
    ld.add_action(rviz_node)
    return ld
    