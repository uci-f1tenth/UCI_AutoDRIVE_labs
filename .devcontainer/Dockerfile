FROM docker.io/ros:kilted-ros-base

RUN sudo apt-get update
RUN sudo apt-get install -y ros-$ROS_DISTRO-foxglove-bridge ros-$ROS_DISTRO-slam-toolbox ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-cv-bridge python3-pip
RUN pip3 install "greenlet>=3.0.0" "gevent>=23.9.0" "eventlet>=0.35.0" Flask==1.1.1 Flask-SocketIO==4.1.0 python-socketio==4.2.0 python-engineio==3.13.0 gevent-websocket==0.10.1 itsdangerous==2.0.1 werkzeug==2.0.3 transforms3d pillow --break-system-packages