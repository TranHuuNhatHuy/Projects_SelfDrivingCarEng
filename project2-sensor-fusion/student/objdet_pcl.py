# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Process the point-cloud and prepare it for object detection
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# General pkgs
import cv2
import numpy as np
import torch
import open3d
import zlib

# Project dir to Python path for relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

# Waymo Open Dataset reader
from tools.waymo_reader.simple_waymo_open_dataset_reader import utils as waymo_utils
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2, label_pb2

# ObjDet tools and helper functions
import misc.objdet_tools as tools

# Misc params
name_window = "Lidar Point-Cloud"
name_BEV_inten_window = "BEV Intensity Map (normalized)"
name_BEV_height_window = "BEV Height Map (normalized)"


# Visualize lidar point-cloud
def show_pcl(pcl):

    # ======================================= ID_S1_EX2 START ======================================= #   
    
    # Step 1 : initialize open3d with key callback and create window
    visualizer = open3d.visualization.VisualizerWithKeyCallback()
    visualizer.create_window(
        window_name = name_window,
        width = 1920, height = 1080,
    )
    visualizer.register_key_callback(262, lambda vis: vis.destroy_window())
    
    # Step 2 : create instance of open3d point-cloud class
    pcd = open3d.geometry.PointCloud()

    # Step 3 : set points in pcd instance by converting the point-cloud into 3d vectors
    # (using open3d function Vector3dVector)
    pcd.points = open3d.utility.Vector3dVector(pcl[:, :3])

    # Step 4 : for the first frame, add the pcd instance to visualization using add_geometry
    # For all other frames, use update_geometry instead
    visualizer.add_geometry(pcd)
    
    # Step 5 : visualize point cloud & keep window open till right-arrow is pressed (key-code 262)
    visualizer.run()

    # =============================================================================================== #     
       

# Visualize range image
def show_range_image(frame: dataset_pb2.Frame, lidar_name: int):

    # ======================================= ID_S1_EX1 START ======================================= #     

    # Step 1 : extract lidar data and range image for the roof-mounted lidar
    laser_data = [
        obj for obj in frame.lasers
        if obj.name == lidar_name
    ][0]
    range_img = []
    if len(laser_data.ri_return1.range_image_compressed) > 0:
        range_img = dataset_pb2.MatrixFloat()
        range_img.ParseFromString(
            zlib.decompress(
                laser_data.ri_return1.range_image_compressed
            )
        )
        range_img = np.array(range_img.data).reshape(range_img.shape.dims)

    # Crop range image to +/- 90 deg around x-axis
    if lidar_name is dataset_pb2.LaserName.TOP:
        width = range_img.shape[1]
        mid = width // 2 
        crop_width = width // 4  # Half 180-deg range (90 deg left and right)
        range_img = range_img[:, mid - crop_width: mid + crop_width, :]
    
    # Step 2 : extract the range and the intensity channel from the range image
    range_channel = range_img[:, :, 0]
    inten_channel = range_img[:, :, 1]
    
    # Step 3 : set values <0 to zero
    # Clip according to paper
    if lidar_name is dataset_pb2.LaserName.TOP:
        max_range = 75
    else:
        max_range = 20
    range_channel = np.clip(
        range_channel,
        0, max_range
    )
    inten_channel[inten_channel < 0] = 0.0

    # Step 4 : map the range channel onto an 8-bit scale
    # Make sure that the full range of values is appropriately considered
    range_channel = (range_channel / (np.amax(range_channel) - np.amin(range_channel)) * 255.0).astype(np.uint8)
    
    # Step 5 : map the intensity channel onto an 8-bit scale
    # Normalize with the difference between the 1- and 99-percentile for outlier mitigation
    inten_min = np.percentile(inten_channel, 1)
    inten_max = np.percentile(inten_channel, 99)
    inten_channel = np.clip(
        inten_channel,
        inten_min, 
        inten_max
    )
    inten_channel = ((inten_channel - inten_min) / (inten_max - inten_min) * 255.0).astype(np.uint8)
    
    # Step 6 : stack the range and intensity image vertically
    # Convert the result to an unsigned 8-bit integer
    img_range_intensity = np.vstack((range_channel, inten_channel)).astype(np.uint8)
    
    return img_range_intensity

    # =============================================================================================== #


# Create birds-eye view of lidar data
def bev_from_pcl(lidar_pcl, configs, visualize = True):

    # Remove lidar points outside detection area and with too low reflectivity
    range_pcl = lidar_pcl[:, 0]
    inten_pcl = lidar_pcl[:, 1]
    elong_pcl = lidar_pcl[:, 2]
    mask = np.where((range_pcl >= configs.lim_x[0]) & (range_pcl <= configs.lim_x[1]) &
                    (inten_pcl >= configs.lim_y[0]) & (inten_pcl <= configs.lim_y[1]) &
                    (elong_pcl >= configs.lim_z[0]) & (elong_pcl <= configs.lim_z[1]))
    lidar_pcl = lidar_pcl[mask]
    
    # Shift level of ground plane to avoid flipping from 0 to 255 for neighboring pixels
    lidar_pcl[:, 2] = lidar_pcl[:, 2] - configs.lim_z[0]

    # Convert sensor coordinates to bev-map coordinates (center is bottom-middle)

    # ======================================= ID_S2_EX1 START ======================================= #  

    # Step 1 :  compute bev-map discretization by dividing x-range by the bev-image height (see configs)
    bev_interval = (configs.lim_x[1] - configs.lim_x[0]) / configs.bev_height

    # Step 2 : create a copy of the lidar pcl and transform all metrix x-coordinates into bev-image coordinates
    lidar_pcl_cpy = np.copy(lidar_pcl)
    lidar_pcl_cpy[:, 0] = (np.floor((lidar_pcl_cpy[:, 0]) / bev_interval)).astype(int)

    # Step 3 : perform the same operation as in step 2 for the y-coordinates but make sure that no negative bev-coordinates occur
    bev_offset = (configs.bev_width + 1) / 2
    lidar_pcl_cpy[:, 1] = (np.floor((lidar_pcl_cpy[:, 1] / bev_interval) + bev_offset)).astype(int)
    lidar_pcl_cpy[lidar_pcl_cpy < 0] = 0

    # Step 4 : visualize point-cloud using the function show_pcl from a previous task
    if visualize:
        show_pcl(lidar_pcl_cpy)

    # =============================================================================================== #    
    
    
    # Compute intensity layer of the BEV map

    # ======================================= ID_S2_EX2 START ======================================= #

    # Step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map
    intensity_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    lidar_pcl_cpy[lidar_pcl_cpy[:, 3] > 1.0, 3] = 1.0

    # Step 2 : re-arrange elements in lidar_pcl_cpy by sorting first by x, then y, then -z (use numpy.lexsort)
    lidar_pcl_cpy = lidar_pcl_cpy[
        np.lexsort(keys = (
            -lidar_pcl_cpy[:, 2], 
            lidar_pcl_cpy[:, 1], 
            lidar_pcl_cpy[:, 0]
        ))
    ]

    # Step 3 : extract all points with identical x and y such that only the top-most z-coordinate is kept (use numpy.unique)
    # Also, store the number of points per x,y-cell in a variable named "counts" for use in the next task
    _, lidar_pcl_unique_index, counts = np.unique(
        lidar_pcl_cpy[ : , 0 : 2], 
        axis = 0, 
        return_index = True, 
        return_counts = True
    )
    lidar_pcl_top = lidar_pcl_cpy[lidar_pcl_unique_index]

    # Step 4 : assign the intensity value of each unique entry in lidar_pcl_top to the intensity map 
    # Make sure that the intensity is scaled in such a way that objects of interest (e.g. vehicles) are clearly visible    
    # Also, make sure that the influence of outliers is mitigated by normalizing intensity on the difference between the max. and min. value within the point cloud
    lidar_pcl_intensity = lidar_pcl_top[:, 3]
    intensity_map[
        np.int_(lidar_pcl_top[:, 0]), 
        np.int_(lidar_pcl_top[:, 1])
    ] = np.clip(
        lidar_pcl_intensity / (np.amax(lidar_pcl_intensity) - np.amin(lidar_pcl_intensity)),
        0, 1
    )

    # Step 5 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background
    if visualize:
        inten_img = (intensity_map * 256.0).astype(np.uint8)
        cv2.imshow(name_BEV_inten_window, inten_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # =============================================================================================== #


    # Compute height layer of the BEV map

    # ======================================= ID_S2_EX3 START ======================================= #

    # Step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map
    height_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))

    # Step 2 : assign the height value of each unique entry in lidar_pcl_top to the height map 
    # Make sure that each entry is normalized on the difference between the upper and lower height defined in the config file
    # Use the lidar_pcl_top data structure from the previous task to access the pixels of the height_map
    height_map[
        np.int_(lidar_pcl_top[:, 0]), 
        np.int_(lidar_pcl_top[:, 1])
    ] = lidar_pcl_top[:, 2] / float(np.abs(configs.lim_z[1] - configs.lim_z[0]))

    # Step 3 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background
    if visualize:
        height_img = (height_map * 256).astype(np.uint8)
        cv2.imshow(name_BEV_height_window, height_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # =============================================================================================== #


    # TODO remove after implementing all of the above steps
    # lidar_pcl_cpy = []
    # lidar_pcl_top = []
    # height_map = []
    # intensity_map = []

    # Compute density layer of the BEV map
    density_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    _, _, counts = np.unique(lidar_pcl_cpy[:, 0:2], axis=0, return_index=True, return_counts=True)
    normalizedCounts = np.minimum(1.0, np.log(counts + 1) / np.log(64)) 
    density_map[np.int_(lidar_pcl_top[:, 0]), np.int_(lidar_pcl_top[:, 1])] = normalizedCounts
        
    # assemble 3-channel bev-map from individual maps
    bev_map = np.zeros((3, configs.bev_height, configs.bev_width))
    bev_map[2, :, :] = density_map[:configs.bev_height, :configs.bev_width]  # r_map
    bev_map[1, :, :] = height_map[:configs.bev_height, :configs.bev_width]  # g_map
    bev_map[0, :, :] = intensity_map[:configs.bev_height, :configs.bev_width]  # b_map

    # expand dimension of bev_map before converting into a tensor
    s1, s2, s3 = bev_map.shape
    bev_maps = np.zeros((1, s1, s2, s3))
    bev_maps[0] = bev_map

    bev_maps = torch.from_numpy(bev_maps)  # create tensor from birds-eye view
    input_bev_maps = bev_maps.to(configs.device, non_blocking=True).float()
    return input_bev_maps


