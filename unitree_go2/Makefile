include .main.env
export $(shell sed 's/=.*//' .main.env)
current_dir = $(shell pwd)
THIS_FILE := $(lastword $(MAKEFILE_LIST))
DISPLAY_ENV = $(DISPLAY)
HOME_ENV = "/home/unitree/"

build-sim:
	docker build -t unitree_go -f dockerfiles/Dockerfile.sim .

build-dog:
	# docker build -t unitree_go2 -f dockerfiles/Dockerfile.dog.fromros2jetson .
	docker buildx build --platform=linux/arm64 -t unitree_go2 -f dockerfiles/Dockerfile.dog.sdk .

build-lidar:
	docker buildx build --platform=linux/arm64 -t lidar_processor -f dockerfiles/Dockerfile.dog.lidar  .

_start_container_if_not_running:
	@if ! docker ps -a | grep -w unitree_container; then $(MAKE) run; fi

_start_lidar_container_if_not_running:
	@if ! docker ps -a | grep -w lidar_processor; then $(MAKE) run; fi

run:
	docker stop unitree_container || true && docker rm unitree_container || true
	docker run \
	-d \
	--rm \
	--net=host \
	--privileged \
	--name unitree_container \
	unitree_go2 tail -f /dev/null

run-lidar:
	docker stop lidar_processor || true && docker rm lidar_processor || true
	docker run \
	-e DISPLAY=localhost:10.0 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v $(HOME_ENV)/.Xauthority:/root/.Xauthority:rw \
    	--env XAUTHORITY=/root/.Xauthority \
	--rm \
	--net=host \
	--privileged \
	--name lidar_processor \
	-v "$(current_dir):/root/data_log/" \
	-t lidar_processor 

run-lidar-debug: _start_lidar_container_if_not_running
	docker exec -it lidar_processor bash


run-simulation: _start_container_if_not_running
	docker exec -t unitree_container bash -c "source devel/setup.bash && roslaunch unitree_gazebo normal.launch rname:=go2 wname:=earth"	 & \

run-debug: _start_container_if_not_running
	docker exec -it unitree_container bash

stop:
	docker kill unitree_container
