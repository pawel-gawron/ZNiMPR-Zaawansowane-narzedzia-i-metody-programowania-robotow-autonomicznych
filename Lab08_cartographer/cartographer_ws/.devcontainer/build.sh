DOCKER_BUILDKIT=1 docker build --network=host \
    --build-arg WORKSPACE=cartographer_ws \
    -t amadeuszsz/cartographer:humble .
