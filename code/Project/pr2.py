#!/usr/bin/env python

# Import modules
import numpy as np
from sklearn import *
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
from time import sleep
import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    # TODO: Statistical Outlier Filtering

    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = .003  #changing the leaf size determines the size of the vox output cloud
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
  
    # TODO: Make Statistical outlier filter
    #Create statistical outlier filter object
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()

    #Set the number of nerighboring points to analyze for any given point
    outlier_filter.set_mean_k(50)
    
    #set threshold factor
    x = 1.0
    
    #Any point with a mean distance larger than global (mean distanc+x*std_dev)
    #will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)
    
    #Call the filter function
    cloud_filtered = outlier_filter.filter()    
    
    # TODO: PassThrough Filter along the z axis to eliminate table
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = .6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

     # TODO: PassThrough Filter along the y-axis to eliminate drop boxes
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -.5
    axis_max = .5
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()    

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = .01
    seg.set_distance_threshold(max_distance)

    # TODO: Extract inliers and outliers
    inliers, coeffecients = seg.segment()
    extracted_inliers = cloud_filtered.extract(inliers, negative = False)
    extracted_outliers = cloud_filtered.extract(inliers, negative = True)
    
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_outliers) 
    #Create k-d tree from converted point cloud    
    tree = white_cloud.make_kdtree()
    #Create euclidean cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    #set euclidian cluster settings to cluster by    
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(20000)
    #Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    #Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # to loop through a array and assign values to r g b and list item  
    color_cluster_point_list = []

    #get colors for color list
    cluster_color = get_color_list(len(cluster_indices))
    
    
    #iterate thought the cluster indices building a nx4 array the length of the color
    #list with color label     
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #create new cloud that contains all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    
    
    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages need to update these publishers
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

 #Exercise-3 TODOs:

    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud objects)
        #these points  #are in the form of XYZRGB
        pcl_cluster = extracted_outliers.extract(pts_list)
        print('index')
        print(index)
        #print('pts_list')
        #print(pts_list)      
        #convert cluster from pcl to ros for transmission through subscriber
        ros_cloud = pcl_to_ros(pcl_cluster)        
                
        # Compute the associated feature vector
        # Extract histogram features
        chists = compute_color_histograms(ros_cloud, using_hsv=True)
        normals = get_normals(ros_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
     
        # Make the prediction
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
          
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] +=.4
        object_markers_pub.publish(make_label(label,label_pos,index))                        
                    
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cloud
        detected_objects.append(do)
        detected_objects_pub.publish(detected_objects) 
	
        detected_objects_pub.publish(detected_objects)
    rospy.loginfo('Detected {} object: {}'.format(len(detected_objects_labels), detected_objects_labels))
   
    # Get parameters from ROS for the objects in the list
    object_list_param = rospy.get_param('/object_list')

    # Create empty list to append object pick list
    object_name_list = []
    object_pick_group = []
    
    # Loop through object_list_param and assign name and group to list
    for j in range(0, len(object_list_param)):
        object_name_list.append(object_list_param[j]['name'])
        object_pick_group.append(object_list_param[j]['group'])

    # Create unique list with set to prevent duplicates
    object_name_list = list(set(object_name_list))
    object_pick_group = list(set(object_pick_group))
    # Set mover flag to false before each check of detected objects
    mover_flag = 'FALSE'
    # Check if detected objects from Ros match the pick list from the yaml file
    for k in range(len(object_name_list)):
        for r in range(len(detected_objects_labels)) :      
            if(object_name_list[k] == detected_objects_labels[r]):
            # Set the mover flag to true if obejects are detected that match the pick list
                mover_flag = 'TRUE'

    # If mover flag is set to true call pr2 mover to pass ouptu yaml
    if mover_flag == 'TRUE':        
        try:
            pr2_mover(detected_objects)
        except rospy.ROSInterruptException:
            pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    print('pr2_mover called')
    labels = []
    centroids = []
    object_name = String()
    object_group = String()    
    pick_pose = Pose()
    place_pose = Pose()
    arm_name = String()
    dict_list = []
    # Assign object list to variable
    object_list_param = rospy.get_param('/object_list')
    object_box_param = rospy.get_param('/dropbox')
    test_scene_num = Int32()
    test_num = 3
    test_scene_num.data = test_num
    output_filename = "output_3.yaml"
    
    # Assign name and group name to variables from the pick list
    for j in range(0, len(object_list_param)):
        object_name = object_list_param[j]['name']
        object_group = object_list_param[j]['group']
               
        # Check obejct in pick list exists in detected objects
        # If it does obatin pick and place position
        for obj, obj_val in enumerate(object_list):
            if object_name == (obj_val.label):
                print('match')
                #print(object_name)
                #$print(obj_val.label)
                #print('need centroid info')
                points_arr = ros_to_pcl(obj_val.cloud).to_array()
                centroid = np.mean(points_arr, axis = 0)[:3]
                pick_pose.position.x = np.asscalar(centroid[0])
                pick_pose.position.y = np.asscalar(centroid[1])
                pick_pose.position.z = np.asscalar(centroid[2])
                if object_group == 'green':
                    arm_name.data = 'right'
                    place_pose.position.x = object_box_param[0]['position'][0]
                    place_pose.position.y = object_box_param[0]['position'][1]
                    place_pose.position.z = object_box_param[0]['position'][2]
                   # print('green drop box')                
                   # print(place_pose)
                    yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
                    dict_list.append(yaml_dict)
                if object_group == 'red':
                    arm_name.data = 'left'
                    place_pose.position.x = object_box_param[1]['position'][0]
                    place_pose.position.y = object_box_param[1]['position'][1]
                    place_pose.position.z = object_box_param[1]['position'][2]
                    #print('red drop box')                
                    #print(place_pose)
                    yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
                    dict_list.append(yaml_dict)
            if object_name == (obj_val.label): 
                print("no match do nothing")
    # Send ouput to yaml file        
    send_to_yaml(output_filename, dict_list)



if __name__ == '__main__':

    #ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    #Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)
    
    #Create Publishers create publisher objects with the following charestics and values
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
    detected_objects_pub= rospy.Publisher('/detected_objects', DetectedObjectsArray , queue_size=1)
    
    # TODO: Load Model From disk
    model = pickle.load(open('model_pr2_200.sav','rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']    

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        #delay to send messages every 10ms
        rospy.spin()    

   
