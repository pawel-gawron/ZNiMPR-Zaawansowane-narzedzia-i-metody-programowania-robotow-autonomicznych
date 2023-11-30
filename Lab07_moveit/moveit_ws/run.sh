rocker --network host --privileged --nvidia --x11 --user --name ros_container \
    --env "USER" \
    --env "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    --env "ROS_DOMAIN_ID=95" \
    --volume "${PWD}:${HOME}/${PWD##*/}" \
    --volume /dev/:/dev/ \
    -- amadeuszsz/moveit:humble 
