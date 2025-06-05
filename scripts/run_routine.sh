#!/usr/bin/env bash
set -xe

# Optional: build if you pass --build
if [[ "$1" == "--build" ]]; then
  docker build --tag learning_machines .
  shift
fi

# Make sure results folder exists
mkdir -p ./results

# Run the Docker container
docker run -it --rm \
  --net=host \
  -v "$(pwd)/results:/root/results" \
  learning_machines "$@"

# Fix ownership of results
#sudo chown "$USER":"$USER" ./results -R

