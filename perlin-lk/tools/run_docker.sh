readonly src_dir="$(dirname "$0")/project"
readonly build_dir="$(dirname "$0")/build"

docker run  \
  -v "${src_dir}":"/app/project" \
  -v "${build_dir}":"/app/build" \
  -e HI="/app/project" \
  -it --rm perlin-test-docker /bin/bash -c 'echo $PROJECT_DIR && ls $PROJECT_DIR'
