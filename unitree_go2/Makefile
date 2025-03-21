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
	docker buildx build --platform=linux/arm64 -t lidar_processor -f dockerfiles/Dockerfile.dog.lidar_processor .

_start_container_if_not_running:
	@if ! docker ps -a | grep -w unitree_container; then $(MAKE) run; fi

_start_lidar_container_if_not_running:
	@if ! docker ps -a | grep -w lidar_container; then $(MAKE) run-lidar; fi

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
	docker stop lidar_container || true && docker rm lidar_container || true
	docker run \
	-e DISPLAY=localhost:11.0 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v $(HOME_ENV)/.Xauthority:/root/.Xauthority:rw \
    	--env XAUTHORITY=/root/.Xauthority \
	--rm \
	--net=host \
	--privileged \
	--name lidar_container \
	-t lidar_processor

run-lidar-debug: _start_lidar_container_if_not_running
	docker exec -it lidar_container bash

run-simulation: _start_container_if_not_running
	docker exec -t unitree_container bash -c "source devel/setup.bash && roslaunch unitree_gazebo normal.launch rname:=go2 wname:=earth"	 & \

run-debug: _start_container_if_not_running
	docker exec -it unitree_container bash

run-interface: _start_container_if_not_running
	docker exec -t unitree_container bash -c "source /root/ros2_ws/src/unitree_ros2/setup.sh && ros2 run dog_interface dog_interface"

run-stand: _start_container_if_not_running
	docker exec -t unitree_container bash -c "source /root/ros2_ws/src/unitree_ros2/setup.sh && ros2 run dog_interface dog_stand"

run-crouch: _start_container_if_not_running
	docker exec -t unitree_container bash -c "source /root/ros2_ws/src/unitree_ros2/setup.sh && ros2 run dog_interface dog_crouch"

run-sit: _start_container_if_not_running
	docker exec -t unitree_container bash -c "source /root/ros2_ws/src/unitree_ros2/setup.sh && ros2 run dog_interface dog_sit"

stop:
	docker kill unitree_container

stop-lidar:
	docker kill lidar_container
