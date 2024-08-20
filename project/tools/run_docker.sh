readonly project_dir="$(dirname "$0")/.."
readonly build_dir="$(dirname "$0")/build"

docker run  \
  -v "${project_dir}":"/app/project" \
  -v "${build_dir}":"/app/build" \
  -e APP_DIR="/app/" \
  -it --rm perlin-test-docker
