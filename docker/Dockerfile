FROM sasjonge/knowrob_cloud
SHELL ["/bin/bash", "-c"]

### Installing NEEM-Validator
USER root

# Making git builds accessible for ros
RUN mkdir /builds
RUN chown ros:ros /builds

# DVC
RUN wget https://dvc.org/download/linux-deb/dvc-2.7.4
RUN mv dvc-2.7.4 dvc.deb
RUN apt install ./dvc.deb

USER ros

# Pip for Python 2.7
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
RUN python get-pip.py

# Python dependencies for Validator
RUN python -m pip install pymongo rdflib ansi2html

WORKDIR /home/ros/src
RUN git clone https://gitlab.informatik.uni-bremen.de/bachelor-thesis/neem_validator.git

COPY docker/source_ros.sh /source_ros.sh

ENTRYPOINT []


#1 In Order to execute the neem-validator, run the following:
#
# docker build -t nv_img .
# docker run -dit --name neem_validator nv_img
# docker cp <path_to_your_neem> neem_validator:/home/nv/neem/
#
