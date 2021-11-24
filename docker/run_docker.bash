#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
WORKSPACE=$(readlink -f "${SCRIPT_DIR}")
WORKING_DIR="${HOME}/teleoperated_driving/wsp" #default
DOCKER_IMAGE="tod-operator-base:latest" #default
CONTAINER_NAME="tod-operator"#default

# Check if GPU and nvidia-container are available -> "--gpus all" option required
lspci | grep NVIDIA >/dev/null 2>&1
has_gpu=$?
which nvidia-container-toolkit >/dev/null 2>&1
has_nvidia_docker=$?

if [ $has_gpu -ne 0 ] || [ $has_nvidia_docker -ne 0 ]; then
  if [[ $has_gpu -eq 0 && $has_nvidia_docker -ne 0 ]]; then
    echo "ERROR: Check if nvidia-docker is installed!"
  fi
  RUN_CMD="docker run"
else
  RUN_CMD="docker run --gpus all"
fi

while [[ $# -gt 0 ]]; do
  key="$1"
  if [ -z ${POSITIONAL+x} ]; then
    case $key in
      -n)
      CONTAINER_NAME="$2"
      shift
      shift
      ;;
      -w)
      WORKING_DIR="$2"
      shift
      shift
      ;;
      -i)
      DOCKER_IMAGE="$2"
      shift
      shift
      ;;
    esac
  else
    POSITIONAL+=("$1")
    shift
  fi
done

# create container args
DOCKER_ARGS=(
  -v $WORKING_DIR:$WORKING_DIR
  -v $WORKSPACE:$WORKSPACE
  -v $HOME:$HOME
  -v /tmp/.X11-unix:/tmp/.X11-unix
  -v /etc/passwd:/etc/passwd:ro
  -v /dev/snd:/dev/snd
  -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native
  -e DISPLAY=$DISPLAY
  -e ROS_HOSTNAME=localhost
  -e USER="$USER"
  -e MESA_GL_VERSION_OVERRIDE="3.3"
  -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native
  --network=host
  --ulimit core=-1
  --privileged
  --user $(id -u):$(id -g)
  --group-add sudo
  --group-add dialout
  --entrypoint "${SCRIPT_DIR}/entrypoint.sh"
  -d --rm # in detached mode and remove when exited
  -w ${WORKING_DIR}
)

DOCKER_ARGS+=(--name "$CONTAINER_NAME" -e CONTAINER_NAME="$CONTAINER_NAME")
DOCKER_ARGS+=($DOCKER_IMAGE)

#start container
$RUN_CMD ${DOCKER_ARGS[@]} || exit 1
echo "$IMAGE_NAME with name $CONTAINER_NAME created" 
