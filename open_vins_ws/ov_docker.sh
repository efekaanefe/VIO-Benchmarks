#!/bin/bash
# Helper for the OpenVINS ROS1 (20.04 / noetic) docker workspace.
#
#   ./ov_docker.sh            open a shell in the container (starts it if needed)
#   ./ov_docker.sh image      (re)build the docker image
#   ./ov_docker.sh build ...  catkin build inside the container (extra args are passed on;
#                             JOBS=n sets parallel jobs, default 4 since each one needs ~1.2GB RAM)
#   ./ov_docker.sh stop       stop and remove the container
#   ./ov_docker.sh <cmd...>   run any command inside the container (ROS env sourced)
#
# Container layout:
#   /catkin_ws                 <- this directory (build/ devel/ logs/ live on the host)
#   /catkin_ws/src/open_vins   <- the open_vins repo (lives in src/ of this directory)
#   /datasets (and ~/datasets) <- $DATASETS_DIR
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$WS_DIR/src/open_vins"
DATASETS_DIR="${DATASETS_DIR:-/media/efe/Data/datasets}"
IMAGE="ov_ros1_20_04"
NAME="open_vins"
CHOME="/home/$(id -un)"


start() {
  if [ -n "$(docker ps -q -f name="^${NAME}$")" ]; then return; fi
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  mkdir -p "$WS_DIR/.home"
  local xauth=()
  if [ -n "$XAUTHORITY" ] && [ -f "$XAUTHORITY" ]; then
    xauth=(-e XAUTHORITY=/tmp/.docker.xauth -v "$XAUTHORITY:/tmp/.docker.xauth:ro")
  fi
  # run as the host user so build products in the workspace are not root-owned
  docker run -d --name "$NAME" \
    --net=host --ipc=host \
    --user "$(id -u):$(getent passwd "$(id -u)" | cut -d: -f4)" \
    -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro \
    -e HOME="$CHOME" -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw "${xauth[@]}" \
    -v "$WS_DIR:/catkin_ws" \
    -v "$WS_DIR/.home:$CHOME" \
    -v "$DATASETS_DIR:/datasets" \
    -w /catkin_ws \
    "$IMAGE" sleep infinity >/dev/null
}

case "$1" in
  image)
    docker build -t "$IMAGE" -f "$REPO_DIR/Dockerfile_ros1_20_04" "$REPO_DIR"
    ;;
  stop)
    docker rm -f "$NAME"
    ;;
  "")
    start
    docker exec -it -e DISPLAY="$DISPLAY" "$NAME" bash
    ;;
  *)
    start
    [ "$1" = build ] && { shift; set -- catkin build -j"${JOBS:-4}" "$@" --cmake-args -DCMAKE_BUILD_TYPE=Release; }
    tty=(); [ -t 0 ] && tty=(-it)
    docker exec "${tty[@]}" -e DISPLAY="$DISPLAY" "$NAME" bash -c \
      'source /opt/ros/noetic/setup.bash; [ -f devel/setup.bash ] && source devel/setup.bash; exec "$@"' -- "$@"
    ;;
esac
