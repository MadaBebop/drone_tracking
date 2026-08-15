from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_tracking',
            executable='detector_node',
            name='detector_node',
            output='screen'
        ),
        Node(
            package='drone_tracking',
            executable='tracker_node',
            name='tracker_node',
            output='screen'
        ),
        Node(
            package='drone_tracking',
            executable='jammer_node',
            name='jammer_node',
            output='screen'
        ),
        Node(
            package='drone_tracking',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
        Node(
            package='drone_tracking',
            executable='mission_node',
            name='mission_node',
            output='screen'
        ),
        Node(
            package='drone_tracking',
            executable='target_mover_node',
            name='target_mover_node',
            output='screen'
        ),
        ExecuteProcess(
      	  cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
       		 '/drone/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
    	  output='screen'
	),
    ])
