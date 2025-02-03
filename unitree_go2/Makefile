include .main.env
export $(shell sed 's/=.*//' .main.env)
current_dir = $(shell pwd)
THIS_FILE := $(lastword $(MAKEFILE_LIST))

build-sim:
	docker build -t unitree_go -f dockerfiles/Dockerfile.sim .

build-dog:
	docker buildx build --platform=linux/arm64 -t unitree_go2 -f dockerfiles/Dockerfile.dog .

_start_container_if_not_running:
	@if ! docker ps -a | grep -w unitree_container; then $(MAKE) run; fi

run:
	docker stop unitree_container || true && docker rm unitree_container || true
	docker run \
	--rm \
	--net=host \
	--privileged \
	--name unitree_container \
	-t unitree_go2

run-simulation: _start_container_if_not_running
	docker exec -t unitree_container bash -c "source devel/setup.bash && roslaunch unitree_gazebo normal.launch rname:=go2 wname:=earth"	 & \

run-debug: _start_container_if_not_running
	docker exec -it unitree_container bash

stop:
	docker kill unitree_container