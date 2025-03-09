include .main.env
export $(shell sed 's/=.*//' .main.env)
current_dir = $(shell pwd)
THIS_FILE := $(lastword $(MAKEFILE_LIST))

build:
	docker build -t unitree_go -f Dockerfile .

_start_container_if_not_running:
	@if ! docker ps -a | grep -w unitree_container; then $(MAKE) run; fi

run:
	docker stop unitree_container || true && docker rm unitree_container || true
	xhost +local:root
	docker run \
	--rm \
	--net=host \
	--privileged \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="${current_dir}:/media" \
	--volume="${HOME}/.Xauthority:/root/.Xauthority:rw" \
	--name unitree_container \
	--device /dev/dri \
	--group-add video \
	-t unitree_go

run-simulation: _start_container_if_not_running
	docker exec -t unitree_container bash -c "source devel/setup.bash && roslaunch unitree_gazebo normal.launch rname:=go2 wname:=earth"	 & \

run-debug: _start_container_if_not_running
	docker exec -it unitree_container bash

stop:
	docker kill unitree_container