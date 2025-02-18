# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Detect 3D objects in lidar point clouds using deep learning
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# general package imports
import numpy as np
import torch
from easydict import EasyDict as edict

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

# model-related
from tools.objdet_models.resnet.models import fpn_resnet
from tools.objdet_models.resnet.utils.evaluation_utils import decode, post_processing 
from tools.objdet_models.resnet.utils.torch_utils import _sigmoid

from tools.objdet_models.darknet.models.darknet2pytorch import Darknet as darknet
from tools.objdet_models.darknet.utils.evaluation_utils import post_processing_v2


# load model-related parameters into an edict
def load_configs_model(model_name='darknet', configs=None):

    # init config file, if none has been passed
    if configs==None:
        configs = edict()  

    # get parent directory of this file to enable relative paths
    curr_path = os.path.dirname(os.path.realpath(__file__))
    parent_path = configs.model_path = os.path.abspath(os.path.join(curr_path, os.pardir))    
    
    # set parameters according to model type
    if model_name == "darknet":
        configs.model_path = os.path.join(parent_path, 'tools', 'objdet_models', 'darknet')
        configs.pretrained_filename = os.path.join(configs.model_path, 'pretrained', 'complex_yolov4_mse_loss.pth')
        configs.arch = 'darknet'
        configs.batch_size = 4
        configs.cfgfile = os.path.join(configs.model_path, 'config', 'complex_yolov4.cfg')
        configs.conf_thresh = 0.5
        configs.min_iou = 0.5
        configs.distributed = False
        configs.img_size = 608
        configs.nms_thresh = 0.4
        configs.num_samples = None
        configs.num_workers = 4
        configs.pin_memory = True
        configs.use_giou_loss = False

    elif model_name == 'fpn_resnet':

        # ======================================= ID_S3_EX1-3 START ======================================= #

        configs.model_path = os.path.join(parent_path, 'tools', 'objdet_models', 'resnet')
        configs.pretrained_filename = os.path.join(configs.model_path, 'pretrained', 'fpn_resnet_18_epoch_300.pth')
        configs.min_iou = 0.5
        configs.saved_fn = "fpn_resnet_18"                  # The name using for saving logs, models,...
        configs.arch = "fpn_resnet"                         # The name of the model architecture
        configs.K = 50                                      # The number of top K
        configs.no_cuda = False                             # If true, cuda is not used
        configs.gpu_idx = 0                                 # GPU index to use
        configs.output_width = 608                          # Input image size
        configs.batch_size = 1                              # Mini-batch size
        configs.conf_thresh = 0.5                           # Confidence threshold
        configs.nms_thresh = 0.4                            # NMS threshold
        configs.num_samples = None                          # Take a subset of the dataset to run and debug
        configs.num_workers = 1                             # Number of threads for data loader
        configs.num_layers = 18                             # The number of layers in backbone
        configs.peak_thresh = 0.2                           # Threshold for peak detection
        configs.save_test_output = False                    # Save test output
        configs.output_format = "image"                     # The type of the test output (support image or video)
        configs.output_video_fn = "out_fpn_resnet"          # The video filename if the output format is video
        configs.output_width = 608                          # the width of showing output, the height maybe vary
        configs.pin_memory = True                           # Pin memory in data loader
        configs.distributed = False                         # For testing on 1 GPU only
        
        configs.input_size = (608, 608)                     # The input size of the model
        configs.hm_size = (152, 152)                        # The size of the output heatmap
        configs.down_ratio = 4                              # The down ratio of the model
        configs.max_objects = 50                            # The maximum number of objects to detect
        configs.imagenet_pretrained = False                 # Use imagenet pretrained model
        configs.head_conv = 64                              # The number of head convolutional channels
        configs.num_classes = 3                             # The number of classes to detect
        configs.num_center_offset = 2                       # The number of center offset
        configs.num_z = 1                                   # The number of depth
        configs.num_dim = 3                                 # The number of dimensions
        configs.num_direction = 2                           # The number of orientation

        configs.heads = {
            "hm_cen": configs.num_classes,
            "cen_offset": configs.num_center_offset,
            "direction": configs.num_direction,
            "z_coor": configs.num_z,
            "dim": configs.num_dim
        }
        configs.num_input_features = 4

        # ================================================================================================= #


    else:
        raise ValueError("Error: Invalid model name")

    # GPU vs. CPU
    configs.no_cuda = True # if true, cuda is not used
    configs.gpu_idx = 0  # GPU index to use.
    configs.device = torch.device('cpu' if configs.no_cuda else 'cuda:{}'.format(configs.gpu_idx))

    return configs


# load all object-detection parameters into an edict
def load_configs(model_name='fpn_resnet', configs=None):

    # init config file, if none has been passed
    if configs==None:
        configs = edict()    

    # birds-eye view (bev) parameters
    configs.lim_x = [0, 50] # detection range in m
    configs.lim_y = [-25, 25]
    configs.lim_z = [-1, 3]
    configs.lim_r = [0, 1.0] # reflected lidar intensity
    configs.bev_width = 608  # pixel resolution of bev image
    configs.bev_height = 608 

    # add model-dependent parameters
    configs = load_configs_model(model_name, configs)

    # visualization parameters
    configs.output_width = 608 # width of result image (height may vary)
    configs.obj_colors = [[0, 255, 255], [0, 0, 255], [255, 0, 0]] # 'Pedestrian': 0, 'Car': 1, 'Cyclist': 2

    return configs


# create model according to selected model type
def create_model(configs):

    # check for availability of model file
    assert os.path.isfile(configs.pretrained_filename), "No file at {}".format(configs.pretrained_filename)

    # create model depending on architecture name
    if (configs.arch == 'darknet') and (configs.cfgfile is not None):
        print('using darknet')
        model = darknet(cfgfile=configs.cfgfile, use_giou_loss=configs.use_giou_loss)    
    
    elif 'fpn_resnet' in configs.arch:
        print('using ResNet architecture with feature pyramid')
        
        # ======================================= ID_S3_EX1-4 START ======================================= #

        model = fpn_resnet.get_pose_net(
            heads = configs.heads,
            head_conv = configs.head_conv,
            num_layers = configs.num_layers,
            imagenet_pretrained = configs.imagenet_pretrained
        )
        
        # ================================================================================================= #
    
    else:
        assert False, 'Undefined model backbone'

    # load model weights
    model.load_state_dict(torch.load(configs.pretrained_filename, map_location='cpu'))
    print('Loaded weights from {}\n'.format(configs.pretrained_filename))

    # set model to evaluation state
    configs.device = torch.device('cpu' if configs.no_cuda else 'cuda:{}'.format(configs.gpu_idx))
    model = model.to(device=configs.device)  # load model to either cpu or gpu
    model.eval()          

    return model


# detect trained objects in birds-eye view
def detect_objects(input_bev_maps, model, configs):

    # deactivate autograd engine during test to reduce memory usage and speed up computations
    with torch.no_grad():  

        # perform inference
        outputs = model(input_bev_maps)

        # decode model output into target object format
        if 'darknet' in configs.arch:

            # perform post-processing
            output_post = post_processing_v2(outputs, conf_thresh=configs.conf_thresh, nms_thresh=configs.nms_thresh) 
            detections = []
            for sample_i in range(len(output_post)):
                if output_post[sample_i] is None:
                    continue
                detection = output_post[sample_i]
                for obj in detection:
                    x, y, w, l, im, re, _, _, _ = obj
                    yaw = np.arctan2(im, re)
                    detections.append([1, x, y, 0.0, 1.50, w, l, yaw])    

        elif 'fpn_resnet' in configs.arch:
            # decode output and perform post-processing
            
            # ======================================= ID_S3_EX1-5 START ======================================= #

            outputs["hm_cen"] = _sigmoid(outputs["hm_cen"])
            outputs["cen_offset"] = _sigmoid(outputs["cen_offset"])
            detections = decode(
                hm_cen = outputs["hm_cen"],
                cen_offset = outputs["cen_offset"],
                direction = outputs["direction"],
                z_coor = outputs["z_coor"],
                dim = outputs["dim"],
                K = configs.K,
            ).cpu().numpy().astype(np.float32)
            detections = post_processing(detections, configs)[0][1]
            
            # ================================================================================================= #


    # ======================================= ID_S3_EX2 START ======================================= #
    
    # Extract 3d bounding boxes from model response
    objects = [] 

    # Step 1 : check whether there are any detections
    if len(detections) == 0:
        return objects

    # Step 2 : loop over all detections
    for det in detections:
        this_id, this_x, this_y, this_z, this_h, this_w, this_l, this_yaw = det

        # Step 3 : perform the conversion using the limits for x, y and z set in the configs structure
        deltalim_x = configs.lim_x[1] - configs.lim_x[0]
        deltalim_y = configs.lim_y[1] - configs.lim_y[0]
        x = this_y * deltalim_x / configs.bev_height
        y = this_x * deltalim_y / configs.bev_width - deltalim_y / 2
        w = this_w * deltalim_y / configs.bev_width
        l = this_l * deltalim_x / configs.bev_height
        z = this_z
        yaw = this_yaw
        h = this_h
        
        # Step 4 : append the current object to the 'objects' array
        if (
            (x >= configs.lim_x[0]) and (x <= configs.lim_x[1]) and
            (y >= configs.lim_y[0]) and (y <= configs.lim_y[1]) and
            (z >= configs.lim_z[0]) and (z <= configs.lim_z[1])
        ):
            objects.append([1, x, y, z, h, w, l, yaw]
        )
        
    # =============================================================================================== #
    
    return objects    

