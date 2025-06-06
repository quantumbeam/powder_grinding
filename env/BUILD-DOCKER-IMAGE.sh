#!/bin/bash

# Set the Docker container name from a <user> input argument (first argument).
# If no <user> argument is given, use the current user name as docker project name.
DOCKER_PROJECT=$1
if [ -z "${DOCKER_PROJECT}" ]; then
  DOCKER_PROJECT=${USER}
fi
DOCKER_CONTAINER="${DOCKER_PROJECT}_powder_grinding_ros_1"
echo "$0: USER=${DOCKER_PROJECT}"
echo "$0: DOCKER_CONTAINER=${DOCKER_CONTAINER}"

# Check if the UBUNTU_PRO_TOKEN environment variable is set.
if [ -z "${UBUNTU_PRO_TOKEN}" ]; then
  echo "Error: UBUNTU_PRO_TOKEN environment variable is not set." >&2
  echo "Please set UBUNTU_PRO_TOKEN before running this script. For example:" >&2
  echo "  export UBUNTU_PRO_TOKEN=\"YOUR_TOKEN_HERE\"" >&2
  exit 1
fi
echo "UBUNTU_PRO_TOKEN is set. Proceeding with Docker build."

# Stop and remove the Docker container.
EXISTING_DOCKER_CONTAINER_ID=`docker ps -aq -f name=${DOCKER_CONTAINER}`
if [ ! -z "${EXISTING_DOCKER_CONTAINER_ID}" ]; then
  echo "Stop the container ${DOCKER_CONTAINER} with ID: ${EXISTING_DOCKER_CONTAINER_ID}."
  docker stop ${EXISTING_DOCKER_CONTAINER_ID}
  echo "Remove the container ${DOCKER_CONTAINER} with ID: ${EXISTING_DOCKER_CONTAINER_ID}."
  docker rm ${EXISTING_DOCKER_CONTAINER_ID}
fi

# Build docker image
docker compose -p ${DOCKER_PROJECT} -f docker/compose.yaml build

# Initialize environments in the container
echo "Finished building docker image."
echo "You should run RUN-DOCKER-CONTAINER.sh at home directory and INITIAL_SETUP_ROS_ENVIROMENTS.sh in docker container at first."
echo "After that, you can run ROS commands and use packages in docker container."