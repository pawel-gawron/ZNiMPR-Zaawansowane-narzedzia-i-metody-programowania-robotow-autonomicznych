rocker --network host --privileged --nvidia --x11 --user --name znimpra_container \
  	--env="USER" \
	--env="ROS_DOMAIN_ID=0" \
	--env="ROS_LOCALHOST_ONLY=1" \
	--env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
	--volume /dev:/dev \
	--volume "$PWD:$HOME/cam_3d_ws" \
	-- osrf/ros:humble-desktop

