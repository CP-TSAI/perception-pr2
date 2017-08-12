#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

import pcl


''' Gets surface normals '''
def get_normals(cloud):
  get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
  return get_normals_prox(cloud).cluster


''' Creates a yaml friendly dictionary from ROS messages '''
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
  yaml_dict = {}
  yaml_dict["test_scene_num"] = test_scene_num.data
  yaml_dict["arm_name"]  = arm_name.data
  yaml_dict["object_name"] = object_name.data
  yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
  yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
  return yaml_dict


''' Outputs dictionary to yaml file '''
def send_to_yaml(yaml_filename, dict_list):
  data_dict = {"object_list": dict_list}
  with open(yaml_filename, 'w') as outfile:
    yaml.dump(data_dict, outfile, default_flow_style=False)


''' Returns Downsampled version of a point cloud
    The bigger the leaf size the less information retained '''
def do_voxel_grid_filter(point_cloud, LEAF_SIZE = 0.01):
  voxel_filter = point_cloud.make_voxel_grid_filter()
  voxel_filter.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE) 
  return voxel_filter.filter()


''' Returns only the point cloud information at a specific range of a specific axis '''
def do_passthrough_filter(point_cloud, name_axis = 'z', min_axis = 0.6, max_axis = 1.1):
  pass_filter = point_cloud.make_passthrough_filter()
  pass_filter.set_filter_field_name(name_axis);
  pass_filter.set_filter_limits(min_axis, max_axis)
  return pass_filter.filter()


''' Uses RANSAC planse segmentation to separate plane and not plane points
    Returns inliers (plane) and outliers (not plane) '''
def do_ransac_plane_segmentation(point_cloud, max_distance = 0.01):

  segmenter = point_cloud.make_segmenter()

  segmenter.set_model_type(pcl.SACMODEL_PLANE)
  segmenter.set_method_type(pcl.SAC_RANSAC)
  segmenter.set_distance_threshold(max_distance)

  # obtain inlier indices and model coefficients
  inlier_indices, coefficients = segmenter.segment()

  inliers = point_cloud.extract(inlier_indices, negative = False)
  outliers = point_cloud.extract(inlier_indices, negative = True)

  return inliers, outliers


def filter_statistical_outliers(point_cloud, mean = 50, stdev = 0.5):
  outlier_filter = point_cloud.make_statistical_outlier_filter()
  outlier_filter.set_mean_k(mean)
  outlier_filter.set_std_dev_mul_thresh(stdev)
  return outlier_filter.filter()


''' This pipeline performs three pass through filter in each cartesian axis '''
def get_region_of_interest(cloud):

  filtered_cloud_z = do_passthrough_filter(point_cloud = cloud, 
                                         name_axis = 'z', min_axis = 0.6, max_axis = 1.3)

  filtered_cloud_zx = do_passthrough_filter(point_cloud = filtered_cloud_z, 
                                         name_axis = 'x', min_axis = 0.3, max_axis = 1.0)

  filtered_cloud_zxy = do_passthrough_filter(point_cloud = filtered_cloud_zx, 
                                         name_axis = 'y', min_axis = -0.5, max_axis = 0.5)

  return filtered_cloud_zxy


''' This pipeline separates the objects in the table from the given scene '''
def split_cloud(cloud):
  
  # Reduce noise by taking out statistical outliers
  reduced_noise_cloud = filter_statistical_outliers(point_cloud = cloud, mean = 20, stdev = 0.5)

  # Downsample the cloud as high resolution which comes with a computation cost
  downsampled_cloud = do_voxel_grid_filter(point_cloud = reduced_noise_cloud, LEAF_SIZE = 0.005)

  # Get only information in our region of interest as we don't care about the other parts
  roi_cloud = get_region_of_interest(downsampled_cloud)

  # Separate the table from everything else
  table_cloud, objects_cloud = do_ransac_plane_segmentation(point_cloud = roi_cloud, max_distance = 0.01)

  return objects_cloud, table_cloud


''' This pipeline returns groups of indices for each cluster of points
    Each cluster of indices is grouped as belonging to the same object
    This uses DBSCAN Algorithm Density-Based Spatial Clustering of Applications with noise 
    Aka Euclidian clustering to group points '''
def get_clusters(cloud, tolerance, min_size, max_size):

  tree = cloud.make_kdtree()
  extraction_object = cloud.make_EuclideanClusterExtraction()

  extraction_object.set_ClusterTolerance(tolerance)
  extraction_object.set_MinClusterSize(min_size)
  extraction_object.set_MaxClusterSize(max_size)
  extraction_object.set_SearchMethod(tree)

  # Get clusters of indices for each cluster of points, each clusterbelongs to the same object
  # 'clusters' is effectively a list of lists, with each list containing indices of the cloud
  clusters = extraction_object.Extract()
  return clusters
  

''' clusters is a list of lists each list containing indices of the cloud
    cloud is an array with each cell having three numbers corresponding to x, y, z position
    Returns list of [x, y, z, color] '''
def get_colored_clusters(clusters, cloud):
  
  # Get a random unique colors for each object
  number_of_clusters = len(clusters)
  colors = get_color_list(number_of_clusters)

  colored_points = []

  # Assign a color for each point
  # Points with the same color belong to the same cluster
  for cluster_id, cluster in enumerate(clusters):
    for c, i in enumerate(cluster):
      x, y, z = cloud[i][0], cloud[i][1], cloud[i][2]
      color = rgb_to_float(colors[cluster_id])
      colored_points.append([x, y, z, color])
  
  return colored_points


''' Callback function for your Point Cloud Subscriber '''
def pcl_callback(pcl_msg):

  # Convert ROS msg to PCL data
  cloud = ros_to_pcl(pcl_msg) 

  # Extract objects and table from the scene
  objects_cloud, table_cloud = split_cloud(cloud) 

  # Get a point cloud of only the position information without color information
  colorless_cloud = XYZRGB_to_XYZ(objects_cloud)
  
  # Get groups of indices for each cluster of points
  # Each group of points belongs to the same object
  # This is effectively a list of lists, with each list containing indices of the cloud
  clusters = get_clusters(colorless_cloud, tolerance = 0.01, min_size = 100, max_size = 15000)

  print "Number of clusters:", len(clusters)
 
  # Assign a unique color float for each point (x, y, z)
  # Points with the same color belong to the same cluster
  colored_points = get_colored_clusters(clusters, colorless_cloud)

  # Create a cloud with each cluster of points having the same color
  clusters_cloud = pcl.PointCloud_PointXYZRGB()
  clusters_cloud.from_list(colored_points)
  
  # ---------------------------
  # CLASSIFY THE CLUSTERS 
  # ---------------------------

  detected_objects_labels = []
  detected_objects = []

  for i, indices in enumerate(clusters):
    
    cluster = objects_cloud.extract(indices)
    
    # Convert point cloud cluster to ros message
    cluster_msg = pcl_to_ros(cluster)
    
    # Get features
    color_hist = compute_color_histograms(cluster_msg, using_hsv = True)
    normal_hist = compute_normal_histograms(get_normals(cluster_msg))
    features = np.concatenate((color_hist, normal_hist))    
    
    # Predict and get label
    prediction = classifier.predict(scaler.transform(features.reshape(1, -1)))
    label = encoder.inverse_transform(prediction)[0]
    detected_objects_labels.append(label)

    # Get label position near object and publish in RViz
    label_position = list(colorless_cloud[indices[0]])
    label_position[2] += 0.3
    object_markers_publisher.publish(make_label(label, label_position, i))

    # Add detection to list of detected objects
    detectedObject = DetectedObject()
    detectedObject.label = label
    detectedObject.cloud = pcl_to_ros(clusters_cloud)
    detected_objects.append(detectedObject)
 
  # Convert pcl data to ros messages
  objects_msg = pcl_to_ros(objects_cloud)
  table_msg = pcl_to_ros(table_cloud)
  clusters_msg = pcl_to_ros(clusters_cloud)

  # Publish ROS messages
  objects_publisher.publish(objects_msg)
  table_publisher.publish(table_msg)
  clusters_publisher.publish(clusters_msg)

  # Publish the list of detected objects
  rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
  detected_objects_publisher.publish(detected_objects)

'''
  try:
    pr2_mover(detected_objects_list)
  except rospy.ROSInterruptException:
    pass
'''


'''
# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables

    # TODO: Get/Read parameters

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list

        # TODO: Get the PointCloud for a given object and obtain it's centroid

        # TODO: Create 'place_pose' for the object

        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
'''


if __name__ == '__main__':

  # ROS node initialization
  rospy.init_node('clustering', anonymous = True)

  # Create Subscribers
  subscriber = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size = 1)
    
  # Create Publishers
  objects_publisher = rospy.Publisher("/pcl_objects", PointCloud2, queue_size = 1)
  table_publisher = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)
  clusters_publisher = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)
  object_markers_publisher = rospy.Publisher("/object_markers", Marker, queue_size = 1)
  detected_objects_publisher = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size = 1)
  
  # Load Model From disk
  model = pickle.load(open('model.sav', 'rb'))
  classifier = model['classifier']
  encoder = LabelEncoder()
  encoder.classes_ = model['classes']
  scaler = model['scaler']

  # Initialize color_list
  get_color_list.color_list = []

  # Spin while node is not shutdown
  while not rospy.is_shutdown():
    rospy.spin()
