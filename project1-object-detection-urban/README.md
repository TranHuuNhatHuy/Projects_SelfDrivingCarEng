# Object detection in an urban environment

In this project, you will learn how to train an object detection model using the [Tensorflow Object Detection API](https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/index.html) and [AWS Sagemaker](https://aws.amazon.com/sagemaker/). At the end of this project, you will be able to generate videos such as the one below. 

<p align="center">
    <img src="data/animation.gif" alt="drawing" width="600"/>
</p>

## Installation

Refer to the **Setup Instructions** page in the classroom to setup the Sagemaker Notebook instance required for this project.

>Note: The `conda_tensorflow2_p310` kernel contains most of the required packages for this project. The notebooks contain lines for manual installation when required.

## Usage

This repository contains two notebooks:
* [1_train_model](1_model_training/1_train_model.ipynb): this notebook is used to launch a training job and create tensorboard visualizations. 
* [2_deploy_model](2_run_inference/2_deploy_model.ipynb): this notebook is used to deploy your model, run inference on test data and create a gif similar to the one above.

First, run `1_train_model.ipynb` to train your model. Once the training job is complete, run `2_deploy_model.ipynb` to deploy your model and generate the animation.

Each notebook contains the instructions for running the code, as well the requirements for the writeup. 
>Note: Only the first notebook requires a write up. 


# My result

Result was achieved using pre-trained [`EfficientDet D1 640x640`](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md#:~:text=EfficientDet%20D1%20640x640), with configs:

- Training configs:
    + Training steps: 2000
    + Batch size: 8
    + Data augmentation method: random horizontal flip & random scale cropping and padding to square
    + Optimizer: momentum optimizer
- Evaluation configs:
    + Metric: COCO detection
    + Batch size: 1
- Notebook computation instance type: ml.g5.4xlarge

Result video can be seen at [./2_run_inference/output.avi](https://github.com/TranHuuNhatHuy/SelfDrivingCarEngineer/blob/master/project1-object-detection-urban/2_run_inference/output.avi)

<img width="624" alt="image" src="https://github.com/user-attachments/assets/6753f001-06f3-4f35-aeed-f4f2f7a51943">
