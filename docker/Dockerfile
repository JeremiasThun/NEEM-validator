FROM ubuntu:bionic
SHELL ["/bin/bash", "-c"]

### Installing ROS

RUN apt-get update && apt-get install -y tzdata

RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-get install curl -y && \
    apt-get install gnupg -y && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update

RUN apt-get install ros-melodic-ros-base -y
# reminder: because of Docker, /opt/ros/melodic/setup.bash needs to be sourced again every time it is needed.

RUN apt-get install python-rosdep \
                    python-rosinstall \
                    python-rosinstall-generator \
                    python-wstool build-essential -y
RUN rosdep init && rosdep update

### Installing KnowRob-Dependencies

RUN apt-get install software-properties-common \
                    wget -y

# Prolog
RUN add-apt-repository ppa:swi-prolog/stable &&\
    apt-get update
RUN apt-get install swi-prolog -y

# MongoDB
RUN wget -qO - https://www.mongodb.org/static/pgp/server-5.0.asc | apt-key add - && \
    echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu bionic/mongodb-org/5.0 multiverse" | tee /etc/apt/sources.list.d/mongodb-org-5.0.list && \
    apt-get update
RUN apt-get install mongodb-org -y
RUN apt-get install libmongoc-1.0-0 -y

### Installing KnowRob

RUN apt install python-wstool \
                python-catkin-tools -y

RUN mkdir -p /home/nv/catkin_ws/src
WORKDIR /home/nv/catkin_ws/src
RUN source /opt/ros/melodic/setup.bash &&\
    wstool init &&\
    wstool merge https://raw.github.com/knowrob/knowrob/master/rosinstall/knowrob-base.rosinstall &&\
    wstool update &&\
    yes | rosdep install --ignore-src --from-paths .

ENV SWI_HOME_DIR=/usr/lib/swi-prolog

### Installing NEEM-Validator

RUN python -m pip install pymongo

WORKDIR /home/nv/catkin_ws/src
RUN apt-get install git -y
RUN git clone https://gitlab.informatik.uni-bremen.de/bachelor-thesis/neem_validator.git
WORKDIR /home/nv/catkin_ws
RUN source /opt/ros/melodic/setup.bash &&\
    catkin_make

RUN mkdir /home/nv/neem

# In Order to execute the neem-validator, run the following:
#
# docker build -t nv_img .
# docker run -dit --name neem_validator nv_img
# docker cp <path_to_your_neem> neem_validator:/home/nv/neem/
#
