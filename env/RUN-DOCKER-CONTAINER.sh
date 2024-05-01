#!/bin/bash

################################################################################

# Set the Docker container name from a project name (first argument).
# If no argument is given, use the current user name as the project name.
PROJECT=$1
if [ -z "${PROJECT}" ]; then
  PROJECT=${USER}
fi
CONTAINER="${PROJECT}-powder-grinding-ros-1"
echo "$0: PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"
echo $(dirname "$0")/docker/compose.yaml


# Run the Docker container in the background.
# Any changes made to './docker/compose.yaml' will recreate and overwrite the container.

docker compose -p ${PROJECT} -f $(dirname "$0")/docker/compose.yaml up -d 


# Display GUI through X Server by granting full access to any external client.
xhost +local:


# Enter the Docker container with a Bash shell
# case "$3" and "$2" are used to run a command from teerminator config.

# Command Explanation
# Docker exec option
# -i, --interactive=false    Keep STDIN open even if not attached
# -t, --tty=false            Allocate a pseudo-TTY
# -w, --workdir=""           Working directory inside the container

# Bash options
# -i        If the -i option is provided, the shell operates in interactive mode, reading ~/.bashrc upon launch.
# -c        When the -c option is provided, commands are read from a string.
# A shell launched with the -c option is not an interactive shell, and hence it does not read ~/.bashrc.
case "$3" in
  ( "" )
  case "$2" in
    ( "" )
    docker exec -it -w /root/catkin_ws ${CONTAINER} bash -i
  esac
  ;;
  ( *".launch")
  docker exec -it -w /root/catkin_ws ${CONTAINER} bash -i -c "./src/powder_grinding/env/terminator/run-roslaunch-repeatedly.sh $2 $3"
  ;;
  ( * )
  docker exec -it -w /root/catkin_ws ${CONTAINER} bash -i -c "./src/powder_grinding/env/terminator/run-command-repeatedly.sh $2 $3"
  ;;
esac