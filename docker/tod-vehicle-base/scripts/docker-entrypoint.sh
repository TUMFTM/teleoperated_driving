#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash
export GPG_TTY=$(tty)
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

exec "$@"