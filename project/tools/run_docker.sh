# Source code directory
readonly project_dir="$(dirname "$0")/.."

# Build directory
mkdir -p "$(dirname "$0")/../../build"
readonly build_dir="$(dirname "$0")/../../build"

# Allow X11 connections
xhost +local:docker

# Enter Docker container
docker run  \
  --network bridge \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${project_dir}":"/home/ubuntu/project" \
  -v "${build_dir}":"/home/ubuntu/build" \
  --device=/dev/video0:/dev/video0 \
  --group-add video \
  -e HOME_DIR="/home/ubuntu/" \
  -it --rm perlin-container
