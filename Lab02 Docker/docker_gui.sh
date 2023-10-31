xhost + local:root

docker run -it \
--runtime=nvidia \
--env="DISPLAY" \
--env="NVIDIA_VISIBLE_DEVICES=all" \
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" \
--privileged \
ros_znimpra:znimpra_noetic