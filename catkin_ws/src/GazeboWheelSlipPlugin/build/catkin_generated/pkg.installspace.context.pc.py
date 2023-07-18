# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "message_runtime;gazebo_msgs;roscpp;rospy;nodelet;angles;std_srvs;geometry_msgs;sensor_msgs;nav_msgs;urdf;tf;tf2_ros;dynamic_reconfigure;rosgraph_msgs;trajectory_msgs;image_transport;rosconsole;std_msgs;visualization_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lgazebo_ros_utils;-lgazebo_ros_template;-lgazebo_ros_force;-lgazebo_ros_joint_state_publisher;-lgazebo_ros_joint_pose_trajectory;-lgazebo_ros_diff_drive;-lgazebo_ros_tricycle_drive;-lgazebo_ros_skid_steer_drive;-lgazebo_ros_planar_move;-lgazebo_ros_range".split(';') if "-lgazebo_ros_utils;-lgazebo_ros_template;-lgazebo_ros_force;-lgazebo_ros_joint_state_publisher;-lgazebo_ros_joint_pose_trajectory;-lgazebo_ros_diff_drive;-lgazebo_ros_tricycle_drive;-lgazebo_ros_skid_steer_drive;-lgazebo_ros_planar_move;-lgazebo_ros_range" != "" else []
PROJECT_NAME = "gazebo_wheel_slip_plugin"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "2.9.2"
