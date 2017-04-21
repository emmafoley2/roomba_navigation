#!/usr/bin/python
# to produce pointcloud topic run: roslaunch openni_launch openni.launch
##########################################################################
#   Program to capture a point icloud from ROS or read one from a 
#   file and process it to produce a 2D grid representation of the area
#   in view. 
##########################################################################

import pcl
import sys
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
import rospy
from roslib import message
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

##########################################################################

gridsize=40
unknownval=0
cellsize=0.125

##########################################################################

# Load point cloud from file

#print"Loading point cloud", sys.argv[1]
#cloud = pcl.load(sys.argv[1])
#print "loaded, cloud size:", cloud.size,"NaN removed: ", cloud.is_dense
def callback(data):
    global roscloud
    pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    pc_list = []
    for p in pc:
        pc_list.append( [p[0],p[1],p[2]] )

    p = pcl.PointCloud()
    p.from_list(pc_list)
    roscloud=p
    #rospy.loginfo(roscloud)
rospy.init_node("mapper", anonymous=True)
subscriber=rospy.Subscriber("/camera/depth_registered/points",PointCloud2, callback)

roscloud=pcl.PointCloud()

# Remove points with NaN values from cloud
sleep(4)
cloud=roscloud
processing=1
filtered=pcl.PointCloud()
arr=cloud.to_array()
arr= arr[~np.isnan(arr).any(1)]
filtered.from_array(arr)
cloud=filtered

print"filtered NaN, cloud size:", cloud.size,"NaN removed: ", cloud.is_dense

# Apply a VoxelGrid filter to reduce cloud size

sor = cloud.make_voxel_grid_filter()
sor.set_leaf_size(0.02,0.02,0.02)
filtered=sor.filter()
cloud=filtered

print"Voxel grid, cloud size:", cloud.size,"NaN removed: ", cloud.is_dense

# angle limits for floor plane identification
upper=30*np.pi/180
lower=0

incloud=cloud
orig_size=cloud.size

camheight=0
cam_correction=0

while incloud.size > 0.1*orig_size: #while 10% of the original cloud remains  
    #extract plane from input cloud
    seg=incloud.make_segmenter()
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.05)
    inliers, coefficients=seg.segment()
    
    if len(inliers) == 0: #no more planes found
        break
    
    a,b,c,d = coefficients
    angle=np.arccos((-1*b)/np.sqrt(a*a+b*b+c*c))
    if(a < 0.1 and angle < upper and angle > lower and d > camheight):
    #if floor is within the right angle range and is the furthest away plane within this range
        cam_correction=-angle
        camheight=d
            
    incloud = incloud.extract(inliers, negative=True) #filter the cloud to remove the inliers of the current plane

if cam_correction ==0 and cam_height==0:
    print "Unable to identify floor plane, using default values"
    cam_correction=-25*np.pi/180
    camheight=1.4    

else: 
    print "Floor plane identified at ", -cam_correction*180/np.pi,"degrees, ",camheight, "m below"



tf_x=np.identity(3)
tf_x[1][1] = np.cos(cam_correction)
tf_x[1][2] = np.sin(cam_correction)
tf_x[2][1] = -np.sin(cam_correction)
tf_x[2][2] = np.cos(cam_correction)

tf_y=np.identity(3)
tf_z=np.identity(3)

tfmatrix=np.dot(tf_x, np.dot(tf_y, tf_z))  #note: for rotation only in x axis this = tf_x
print tfmatrix[1][2], tfmatrix[2][1]

print"Building grid"
grid=np.ones((gridsize, gridsize))*unknownval

for point in cloud:
    x,z,y = np.dot(point, tfmatrix)
    z=camheight-z
    
    xgrid=int(x/cellsize) + gridsize/2;
    ygrid=int(y/cellsize)
    
    if ygrid < gridsize and xgrid < gridsize :
        if z > 0.2:
            try:
                grid[xgrid][ygrid]=1
            except IndexError as e:
                print e, x, xgrid, y, ygrid, gridsize, z
        #elif grid[xgrid][ygrid]!=1:
        #    grid[xgrid][ygrid]=0
    
processing=0
plt.imshow(grid, cmap='gray_r', interpolation='none', origin='lower')
plt.show()    


