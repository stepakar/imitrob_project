#!/usr/bin/env python

##################################################
## Author: Karla Stepanova
## Email: karla.stepanova@cvut.cz
## Date of creation: 15.6.2019
##################################################

import rospy
from geometry_msgs.msg import Pose, PoseStamped
#from std_msgs.msg import Float64MultiArray
import geometry_msgs
import tf
import tf2_ros
import tf2_geometry_msgs
from tf import transformations as t
from marker_msgs.msg import MarkerDetection, Marker
import collections
import support_functions.quaternion_averaging as quaternion_averaging
import numpy as np
import json

from rospy_message_converter import json_message_converter
from object_detection_aruco.msg import MarkersAvgList, MarkersAvg

tf_buffer = None
tf_listener = None
global markers
markers = {}
markers_avg = {}
pub = None

def callback(data):
    # trans2 = tf_buffer.lookup_transform('r2_link_0',
    #                                     'camera_link',
    #                                     rospy.Time(0),
    #                                     rospy.Duration(1.0))
    trans2 = tf_buffer.lookup_transform('r2_link_0',
                                        'camera_color_optical_frame',
                                        rospy.Time(0),
                                        rospy.Duration(1.0))
    if len(data.markers)>0:
        for i in range(0,len(data.markers)):
            poseS = PoseStamped ()
            poseS.pose = data.markers[i].pose
            poseS.header = data.header
            pose_transformed_obj = tf2_geometry_msgs.do_transform_pose(poseS, trans2)
            #rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose_transformed_obj)

            pose_transformed_obj.header = poseS.header
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose_transformed_obj)
            if data.markers[i].ids[0] in markers.keys():
                #append new marker detection to collection
                markers[data.markers[i].ids[0]].append(pose_transformed_obj)
            else:
                #if the marker was not detected so far
                #markers will be stored in the collection with buffer of 20 detections (approx.1 second)
                #TODO check proper size of buffer, this will slow down the real-time process
                markers[data.markers[i].ids[0]] = collections.deque(maxlen = 20)
                markers[data.markers[i].ids[0]].append(pose_transformed_obj)
    else:
        print('No markers detected')

    dataM = []
    for key, item in markers.items():
        duration = (item[-1].header.stamp-item[0].header.stamp).to_sec()
        duration2 = (rospy.Time.now()-item[-1].header.stamp).to_sec()
        distance = ((100 * (item[-1].pose.position.x - item[0].pose.position.x)) ** 2 + (
                    100 * (item[-1].pose.position.y - item[0].pose.position.y)) ** 2 + (
                                100 * (item[-1].pose.position.z - item[0].pose.position.z)) ** 2)

        # print('key: {}, number of poses: {}, duration last to first pose: {}'.format(key, len(item), duration))
        # print('duration {}'.format(duration2))
        #TODO add duration and distance from current time (historical data we want to throw away)
        #TODO add constraint on distance (not moving objects counted)
        #if number of items for given marker is 20 within last N seconds and not too old data, save its average
        if (((len(item) == 20 and duration < 10) and (duration2<10)) and distance < 1):
            marker_position = []
            marker_orientation = np.array([])
            for poses in item:
                #orientation has to be via np.stack as a correct input to quaternion_averaging - needs correct shape
                if (np.shape(marker_orientation)[0] > 0):
                    marker_orientation = np.vstack(
                        (marker_orientation, [poses.pose.orientation.x, poses.pose.orientation.y,
                                              poses.pose.orientation.z, poses.pose.orientation.w]))
                else:
                    marker_orientation = np.array([poses.pose.orientation.x, poses.pose.orientation.y,
                                              poses.pose.orientation.z, poses.pose.orientation.w])
                marker_position.append([poses.pose.position.x,
                            poses.pose.position.y, poses.pose.position.z])
            #print('quaternion  avg {}'.format(quaternion_averaging.averageQuaternions(marker_orientation)))
            #print('position avg {}'.format(sum(np.array(marker_position)) / np.size(np.array(marker_position))))
            markers_avg[key] = {}
            markers_avg[key]['orientation'] = quaternion_averaging.averageQuaternions(marker_orientation).tolist()
            div = np.size(np.array(marker_position))
            markers_avg[key]['position'] =  sum(np.array(marker_position),0) / np.size(np.array(marker_position),0)
            dataM1 = MarkersAvg()
            dataM1.id = key
            dataM1.avg_pose.position.x = markers_avg[key]['position'][0]
            dataM1.avg_pose.position.y = markers_avg[key]['position'][1]
            dataM1.avg_pose.position.z = markers_avg[key]['position'][2]
            dataM1.avg_pose.orientation.x = markers_avg[key]['orientation'][0]
            dataM1.avg_pose.orientation.y = markers_avg[key]['orientation'][1]
            dataM1.avg_pose.orientation.z = markers_avg[key]['orientation'][2]
            dataM1.avg_pose.orientation.w = markers_avg[key]['orientation'][3]
            if dataM1.avg_pose.position.x < 0.4:
                print('hallo')
            dataM.append(dataM1)
    print('markers average {}'.format(markers_avg))

    global pub
    if pub is None:
        pub = rospy.Publisher('/averaged_markers', MarkersAvgList, queue_size=10)

    msg = MarkersAvgList ()
    # msg.header.stamp = rospy.Time.now()
    if pose_transformed_obj:
        msg.header = pose_transformed_obj.header
    else:
        msg.header.frame_id = trans2.header.frame_id  # pose_transformed_obj.header
        msg.header.stamp = rospy.Time.now()
    msg.markers = dataM
    #rospy.init_node('averaged_markers', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    if len(dataM) == 0:
        return
    pub.publish(msg)

def averaging_aruco_pose_node():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('averaging_aruco_pose_node', anonymous=False)

    # translation:
    #   x: 0.909480344525
    #   y: -0.266042768948
    #   z: 1.29520475995
    # rotation:
    #   x: -0.718862965166
    #   y: -0.672505426905
    #   z: 0.127747039845
    #   w: 0.121050327991

    # translation:
    #   x: 0.906409291874
    #   y: -0.245099493827
    #   z: 1.29990429444
    # rotation:
    #   x: -0.701589685131
    #   y: -0.692603050646
    #   z: 0.132803067756
    #   w: 0.102158079214


    global tf_buffer
    tf_buffer = tf2_ros.Buffer(rospy.Duration(50.0))  # tf buffer length
    global tf_listener
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(1.0)

    rospy.Subscriber("/markersAruco", MarkerDetection, callback)




if __name__ == '__main__':
    averaging_aruco_pose_node()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
