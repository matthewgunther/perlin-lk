readonly src_dir="$(dirname "$0")/src"
readonly build_dir="$(dirname "$0")/build"

docker run  \
  -v "${src_dir}":"/app/src" \
  -v "${build_dir}":"/app/build" \
  -it --rm perlin-test-docker
