# Build Docker image in same directory
docker build -t perlin-container "$(dirname "$0")"
