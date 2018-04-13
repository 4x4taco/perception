#!/usr/bin/env python

# Import modules
from pcl_helper import *

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert Ros message to point cloud message
    cloud = ros_to_pcl(pcl_msg)

    #create a VoxelGrid filter object for our input point cloud
    vox = cloud.make_voxel_grid_filter()

    #choose a voxel (also known as leaf) size
    #Note:  this (1) is a poor choise of leaf size
    #Leaf size repesents the syz size of the volume element	
    #experiment and find the appropriate size!
    LEAF_SIZE = .01  #changing the leaf size determines resolution of the point cloud

    #set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE )

    #call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
    #filename = 'voxel_downsampled.pcd'
    #pcl.save(cloud_filtered, filename)
    

    # PassThrough filter
    #create a passthough filter object
    passthrough = cloud_filtered.make_passthrough_filter()

    #assign axis and range to the passthrough filter object.    
    #Assigns static filter to clip all data in the z direction set by axis min and max
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = .7
    axis_max = 1.2
    passthrough.set_filter_limits(axis_min, axis_max)

    #finally use the filter funciton to obtain the filtered point cloud
    #Apply passthough filter and save file to the filename
    cloud_filtered = passthrough.filter()
    #filename = 'pass_through_filtered.pcd'
    #pcl.save(cloud_filtered, filename)

    # RANSAC plane segmentation
    #create the segmentation object
    seg = cloud_filtered.make_segmenter()

    #set the model you wish to fit
    #apply setting to the segmentation object that define type of object RANSAC is segmenting for
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    #max distance for a point to be considered fitting the model
    #experiment with different values for max_distance
    #for segmenting the table
    max_distance =.01
    seg.set_distance_threshold(max_distance)

    #call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    extracted_inliers = cloud_filtered.extract(inliers, negative = False)
    #filename = 'extracted_inliers.pcd'
    #pcl.save(extracted_inliers, filename)

    # Extract inliers
    #extracted the inliers by setting the negative value to true and saving under a different name
    extracted_outliers = cloud_filtered.extract(inliers, negative = True)
    #filename = 'extracted_outliers.pcd'
    #pcl.save(extracted_outliers, filename)

    #Create euclidian clusting algorithm 
    #Convert XYZRGB to just XYZ the k-d tree make function only uses spaital information
    
    white_cloud = XYZRGB_to_XYZ(extracted_outliers) 
    
    #Crete k-d tree from converted point cloud    
    tree = white_cloud.make_kdtree()
   
    #  Create euclidean cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    
    #set values of euclidean cluster object
    #set tolearances for distance threshold as well as minimum and maximum cluster size (in points)
    #Note original values provided in the file are poor choices
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(2000)
    
    #Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    
    #Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    #Assign a color corresponding to each segmented object in the scene by using nested for loops
    # to loop through a array and assign values to r g b and list item
    #create empty list    
    color_cluster_point_list = []

    #get colors for color list
    cluster_color = get_color_list(len(cluster_indices))
    print(cluster_color)
    
    #iterate thought the cluster indices building a nx4 array the length of the color list with color labe    
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #create new cloud that contains all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    #convert PCL data back to PointCloud2 format for Rviz and Gazebo within ROS
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    
    #Publish ROS messages one for objects on table the table and the cluster cloud for Euclidean clustering
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)



if __name__ == '__main__':

    #ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    #Create Subscribers
    pcl_sub = rospy.Subscriber('/sensor_stick/point_cloud', pc2.PointCloud2, pcl_callback, queue_size=1)
    
    #create rate to send out pointcloud data on publishers
    rate = rospy.Rate(10) #10hz
    
    #Create Publishers create publisher objects with the following charestics and values
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size=1)
    
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        #delay to send messages every 10ms
        rospy.spin()
