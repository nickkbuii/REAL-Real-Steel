# To be sourced when setting up for Turtlebot work

# Stop here if ROS2 isn't actually installed
[ -f /opt/ros/turtlebot3_ws/install/setup.bash ] || return 0

# if in Distrobox, use the host system's hostname instead of the container's
if [ -f /run/host/etc/hostname ]; then
	read hostname < /run/host/etc/hostname
else
	hostname="$HOSTNAME"
fi

source /opt/ros/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

# Set ROS_DOMAIN_ID to (last quad of $privnet_ip + 20):
#     c105-1 -> 192.168.1.1 -> ROS_DOMAIN_ID=21
#     c105-2 -> 192.168.1.2 -> ROS_DOMAIN_ID=22
#     [...]
privnet_ip="$(getent hosts "$hostname".local | awk '{ print $1 }')"
if [ -n "$privnet_ip" ]; then
	export ROS_DOMAIN_ID=$(( "${privnet_ip##*.}" + 20 ))
fi

# Prevent 95-select_robot.sh from running again
export EE106_ENV_SETUP=turtlebot_ros2
