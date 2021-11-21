# NEEM validator

The NEEM validator is a ROS package to validate NEEMs that have been recorded by following the specifications in the NEEM-Handbook (https://ease-crc.github.io/soma/owl/current/NEEM-Handbook.pdf) by the project EASE (https://ease-crc.org). 

## Prerequisites

* ROS melodic
* KnowRob (https://github.com/knowrob/knowrob)

Python 2.7 libraries:
* pymongo
* rdflib
* ansi2html

## Installation on a local machine

Pull this repository into the `src` directory of KnowRob's catkin workspace (e.g `~/catkin_ws/src`) and source the workspace:

```
source ~/catkin/devel/setup.bash
```

To start the NEEM validator, you can run roslaunch with the following arguments:
* `path_to_neem` (required): _absolute_ path to the NEEM that has to be validated
* `terminate` (default: False): whether KnowRob should continue running after this validator has died
* `workflow_file` (default: settings/pipeline_workflow.yaml): _relative_ path to the workflow definition
* `logging` (default: False): whether timestamps for tthe different checks should be recorded and displayed

To start the default configurations for a NEEM, located at `/home/me/neems/mynewneem`, you can run
```
roslaunch neem_validator validator.launch path_to_neem:=/home/me/neems/mynewneem
```

## Results

* The validation results are printed to rosout and stored in `logs/validation_results`.
* In case logging was activated, the timestamps are stored in `logs/time`.
* The OWLReasonerCheck runs SOMA's "uglify" script on the OWL files. The result is stored in `logs/uglify.owl`.

## Customizing the workflow

The default workflow file can be found here: https://github.com/JeremiasThun/NEEM-validator/blob/master/settings/pipeline_workflow.yaml

The workflow file defines the checks that are called aswell as their configuration. The python docstring in the checks located at `src/checks/` tells the user, which attributes are available and which ones are required per check.

Every check can be called multiple times with different configurations. 

The `Stages` entry may occur only once and determines the order in which the checks are called.
