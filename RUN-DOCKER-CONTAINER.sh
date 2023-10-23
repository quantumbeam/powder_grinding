#!/bin/bash

################################################################################

# Set the Docker container name from a project name (first argument).
# If no argument is given, use the current user name as the project name.
PROJECT=$1
if [ -z "${PROJECT}" ]; then
  PROJECT=${USER}
fi
CONTAINER="${PROJECT}_onolab_ros_1"
echo "$0: PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"

# Run the Docker container in the background.
# Any changes made to './docker/docker-compose.yml' will recreate and overwrite the container.
docker-compose -p ${PROJECT} -f ./docker/docker-compose.yml up -d 


# Display GUI through X Server by granting full access to any external client.
xhost +local:


# Enter the Docker container with a Bash shell
# case "$3" and "$2" are used to run a command from teerminator config.
case "$3" in
  ( "" )
  case "$2" in
    ( "" )
    docker exec -it ${CONTAINER} bash -c "cd /home/ubuntu/onolab ; bash"
  esac
  ;;
  ( *".launch")
  docker exec -it ${CONTAINER} bash -c "cd /home/ubuntu/onolab && ./terminator/run-roslaunch-repeatedly.sh $2 $3"
  ;;
  ( * )
  docker exec -it ${CONTAINER} bash -c "cd /home/ubuntu/onolab && ./terminator/run-command-repeatedly.sh $2 $3"
  ;;
esac