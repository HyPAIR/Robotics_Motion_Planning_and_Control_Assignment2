import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')


def load_waypoints(yaml_file_path):
    if not os.path.exists(yaml_file_path):
        raise FileNotFoundError(f"YAML file not found: {yaml_file_path}")

    with open(yaml_file_path, 'r') as file:
        waypoints = yaml.safe_load(file)

    return waypoints


def generate_model_file(context, model_file_path, waypoint_file_path):
    initial_pose_x = context.perform_substitution(LaunchConfiguration('initial_pose_x'))
    initial_pose_y = context.perform_substitution(LaunchConfiguration('initial_pose_y'))
    initial_pose_z = context.perform_substitution(LaunchConfiguration('initial_pose_z'))
    initial_pose_yaw = context.perform_substitution(LaunchConfiguration('initial_pose_yaw'))

    waypoints = load_waypoints(waypoint_file_path)

    model_dir = os.path.dirname(model_file_path)
    if not os.path.exists(model_dir):
        os.makedirs(model_dir, exist_ok=True)

    scaler = 3
    obstacle_regions = [
        [(25 / scaler, 35 / scaler), (45 / scaler, 56 / scaler)],
        [(67 / scaler, 89 / scaler), (57 / scaler, 76 / scaler)],
        [(50 / scaler, 55 / scaler), (80 / scaler, 89 / scaler)],
        [(20 / scaler, 60 / scaler), (25 / scaler, 35 / scaler)]
    ]

    obstacles = []
    for idx, ((y_min, y_max), (x_min, x_max)) in enumerate(obstacle_regions):
        x_center = (x_min + x_max) / 2
        y_center = (y_min + y_max) / 2
        z = 0.5  
        x_size = x_max - x_min
        y_size = y_max - y_min
        obstacles.append(f"<model name='obstacle_{idx}'>\n      <pose>{x_center} {y_center} {z} 0 0 0</pose>\n      <static>true</static>\n      <link name='link'>\n        <visual name='visual'>\n          <geometry>\n            <box>\n              <size>{x_size} {y_size} 1</size>\n            </box>\n          </geometry>\n          <material>\n            <ambient>1 0 0 1</ambient>\n            <diffuse>1 0 0 1</diffuse>\n          </material>\n        </visual>\n      </link>\n    </model>")

    waypoint_models = []
    for idx, waypoint in enumerate(waypoints.get('trajectory', [])):
        x, y, z = waypoint['y'], waypoint['x'], waypoint.get('z', 0.0)
        waypoint_models.append(f"""
        <model name="waypoint_{idx}">
          <pose>{x} {y} {z} 0 0 0</pose>
          <static>true</static>
          <link name="link">
            <visual name="visual">
              <geometry>
                <sphere>
                  <radius>0.2</radius>
                </sphere>
              </geometry>
              <material>
                <ambient>1 0.5 0 1</ambient>
                <diffuse>1 0.5 0 1</diffuse>
              </material>
            </visual>
          </link>
        </model>
        """)

    model_content = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <pose>{initial_pose_x} {initial_pose_y} {initial_pose_z} 0.0 0.0 {initial_pose_yaw}</pose>
      <uri>model://turtlebot3_{TURTLEBOT3_MODEL}</uri>
    </include>
    {''.join(obstacles)}
    {''.join(waypoint_models)}
  </world>
</sdf>
"""
    with open(model_file_path, 'w') as model_file:
        model_file.write(model_content)
    print(f"Model file generated at: {model_file_path}")


def generate_launch_description():
    initial_pose_x = LaunchConfiguration('initial_pose_x', default='1.0')
    initial_pose_y = LaunchConfiguration('initial_pose_y', default='2.0')
    initial_pose_z = LaunchConfiguration('initial_pose_z', default='0.01')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw', default='1.57')

    base_dir = os.path.dirname(os.path.dirname(__file__))
    waypoint_file_path = os.path.abspath(os.path.join(base_dir, 'launch', 'solution.yaml'))
    model_file_path = os.path.abspath(os.path.join(base_dir, 'turtlebot3', 'worlds', 'waffle_dynamic_with_obstacles_and_waypoints.model'))
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        DeclareLaunchArgument('initial_pose_x', default_value='1.0', description='initial X position'),
        DeclareLaunchArgument('initial_pose_y', default_value='2.0', description='initial Y position'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.01', description='initial Z position'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='1.57', description='initial Yaw angle'),

        OpaqueFunction(function=lambda context: generate_model_file(context, model_file_path, waypoint_file_path)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': model_file_path}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ),
    ])