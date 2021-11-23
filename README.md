# NEEM validator

## Running the NEEM validator on a local machine

The NEEM validator is a ROS package to validate NEEMs that have been recorded by following the specifications in the NEEM-Handbook (https://ease-crc.github.io/soma/owl/current/NEEM-Handbook.pdf) by the project EASE (https://ease-crc.org). 

### Prerequisites

* ROS melodic
* KnowRob (https://github.com/knowrob/knowrob)

Python 2.7 libraries:
* pymongo
* rdflib
* ansi2html

### Installation on a local machine

Pull this repository into the `src` directory of KnowRob's catkin workspace (e.g `~/catkin_ws/src`) and source the workspace:

```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
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

**Careful:** The default configuration of the NEEM validator removes all contents from the Mongo database linked in your KnowRob instance and your environment variables (normally, this is the `roslog` database). It then uploads the NEEM data to this Mongo database. If you want to validate the NEEM data that are already stored in your MongoDB without removing them, just remove the following two lines from the `Stages` list in the `settings/pipeline_workflow.yaml`:

```
  - clear_mongo_collections
  - mongorestore
```

### Results

* The validation results are printed to rosout and stored in `logs/validation_results`.
* In case logging was activated, the timestamps are stored in `logs/time`.
* The OWLReasonerCheck runs SOMA's "uglify" script on the OWL files. The result is stored in `logs/uglify.owl`.

### Customizing the workflow

The default workflow file can be found in [`settings/pipeline_workflow.yaml`](https://github.com/JeremiasThun/NEEM-validator/blob/master/settings/pipeline_workflow.yaml).

The workflow file defines the checks that are called aswell as their configuration. The python docstring in the checks located at [`src/checks/`](https://github.com/JeremiasThun/NEEM-validator/tree/master/src/checks) tells the user, which attributes are available and which ones are required per check.

Every check can be called multiple times with different configurations. 

The `Stages` entry may occur only once and determines the order in which the checks are called.

## Running the NEEM validator in a local Docker image

1. Clone this repository.
2. Navigate to the `docker` directory
3. Create a `neem` folder and copy the NEEM you want to validate there.
4. Run `docker-compose up`
5. The `logs` directory contains your validation results

## Running the NEEM validator as a GitLab CI/CD pipeline

1. A GitLab instance with a [GitLab runner](https://docs.gitlab.com/runner/) that runs Docker container has to be installed.
2. Create a customized version of the `gitlab-ci.yml` (or use the default, located at [`ci-pipeline/gitlab-ci.yml`](https://github.com/JeremiasThun/NEEM-validator/blob/master/ci-pipeline/gitlab-ci.yml)).
3. If you use a customized version of the NEEM validator, you should also adjust the `docker/Dockerfile`, build it, upload it, and link the uploaded docker image in your `gitlab-ci.yml`.
4. Open the GitLab admin area and navigate to settings -> CI/CD: `https://YURGITLAB.org/admin/application_settings/ci_cd`. Under "Continuous Integration and Deployment" navigate to "Default CI/CD configuration file" and insert a link to your version of the `gitlab-ci.yml`.

