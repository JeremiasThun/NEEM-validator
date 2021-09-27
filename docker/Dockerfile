FROM ros:melodic
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install software-properties-common \
                    wget curl -y

# Prolog
RUN add-apt-repository ppa:swi-prolog/stable &&\
    apt-get update
RUN apt-get install swi-prolog -y

# Pip
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
RUN python get-pip.py


### Installing KnowRob
RUN mkdir -p /home/validator/catkin_ws/src

WORKDIR /home/validator/catkin_ws/src

RUN apt install python-catkin-tools -y

RUN source /ros_entrypoint.sh &&\
    wstool init &&\
    wstool merge https://raw.github.com/knowrob/knowrob/master/rosinstall/knowrob-base.rosinstall &&\
    wstool update &&\
    yes | rosdep install --ignore-src --from-paths .

ENV SWI_HOME_DIR=/usr/lib/swi-prolog


### Changing to modified KnowRob
WORKDIR /home/validator/catkin_ws/src
RUN rm -rf knowrob
RUN git clone --branch mongo_interface https://github.com/JeremiasThun/knowrob.git

### Installing NEEM-Validator

RUN python -m pip install pymongo rdflib

WORKDIR /home/validator/catkin_ws/src
RUN apt install git -y
RUN git clone https://gitlab.informatik.uni-bremen.de/bachelor-thesis/neem_validator.git
WORKDIR /home/validator/catkin_ws
RUN source /ros_entrypoint.sh && catkin build
WORKDIR /home/validator/catkin_ws/src/neem_validator

COPY docker/source_ros.sh /source_ros.sh
ENTRYPOINT ["/source_ros.sh"]


#1 In Order to execute the neem-validator, run the following:
#
# docker build -t nv_img .
# docker run -dit --name neem_validator nv_img
# docker cp <path_to_your_neem> neem_validator:/home/nv/neem/
#
