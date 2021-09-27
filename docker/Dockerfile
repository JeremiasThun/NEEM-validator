FROM ros:melodic
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install software-properties-common \
                    wget -y

# Prolog
RUN add-apt-repository ppa:swi-prolog/stable &&\
    apt-get update
RUN apt-get install swi-prolog -y


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

### Installing NEEM-Validator

#RUN python -m pip install pymongo

WORKDIR /home/validator/catkin_ws/src
RUN apt install git -y
RUN git clone https://gitlab.informatik.uni-bremen.de/bachelor-thesis/neem_validator.git
WORKDIR /home/validator/catkin_ws
RUN source /ros_entrypoint.sh && catkin build


#1 In Order to execute the neem-validator, run the following:
#
# docker build -t nv_img .
# docker run -dit --name neem_validator nv_img
# docker cp <path_to_your_neem> neem_validator:/home/nv/neem/
#
