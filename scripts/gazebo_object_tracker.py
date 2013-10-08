#!/usr/bin/env python
import roslib; roslib.load_manifest('gazebo_object_tracker')
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates, ModelState
from ar_pose.msg import ARMarkers, ARMarker
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from visualization_msgs.msg import Marker
import tf

_frame_id = "/base_link"
_pub_freq = 25.0

"""
COCA-COLA-CAN-250ML Marker 1
IKEA-CUP-SOLBRAEND-BLU Marker 5

"""

class GazeboObjectTracker:
    def __init__(self):
        rospy.init_node('gazebo_object_tracker')
        
        self.objs = {}
        self.objDict = {'COCA-COLA-CAN-250ML' : 1, 'IKEA-CUP-SOLBRAEND-BLUE' : 5}
        
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        
        self._pub = rospy.Publisher('ar_pose_marker', AlvarMarkers)
        self._visualPub = rospy.Publisher('visualization_marker', Marker)
        self._tf_broadcaster = tf.TransformBroadcaster()
        
        rospy.loginfo("gazebo object tracker started")

    def publish(self):
        
        
        seq = 0
        while not rospy.is_shutdown():
            markers = AlvarMarkers()
            markers.header.stamp = rospy.get_rostime()
            markers.header.seq = seq
            markers.header.frame_id = _frame_id
            
            for name in self.objs.keys():
                if name in self.objDict.keys():
                    ################################
                    # publish AR marker
                    ################################
                    
                    marker = AlvarMarker()
                    marker.id = self.objDict[name]
                    marker.pose.pose = self.objs[name]
                    
                    marker.confidence = 1
                    marker.header.frame_id = _frame_id
                    
                    markers.markers.append(marker)
                    
                    ################################
                    # publish tf
                    ################################
                    translation = (self.objs[name].position.x, 
                              self.objs[name].position.y, 
                              self.objs[name].position.z)
                    
                    rotation = (self.objs[name].orientation.x,
                                self.objs[name].orientation.y,
                                self.objs[name].orientation.z,
                                self.objs[name].orientation.w)
                    
                    self._tf_broadcaster.sendTransform(translation, 
                                                       rotation, 
                                                       markers.header.stamp, 
                                                       name, 
                                                       _frame_id)
                    
                    ################################
                    # publish rviz marker
                    ################################
                    rviz_marker = Marker()
                    rviz_marker.header = markers.header
                    rviz_marker.id = marker.id
                    rviz_marker.pose = self.objs[name]
                    rviz_marker.lifetime = rospy.Duration(1.0)
                    rviz_marker.scale.x = 0.03
                    rviz_marker.scale.y = 0.03
                    rviz_marker.scale.z = 0.015
                    rviz_marker.ns = "basic_shapes"
                    rviz_marker.type = Marker.CUBE
                    rviz_marker.action = Marker.ADD
                    if rviz_marker.id == 1:
                        # red
                        rviz_marker.color.r = 1.0
                        rviz_marker.color.g = 0.0
                        rviz_marker.color.b = 0.0
                        rviz_marker.color.a = 1.0
                    elif rviz_marker.id == 2:
                        # blue
                        rviz_marker.color.r = 0.0
                        rviz_marker.color.g = 0.0
                        rviz_marker.color.b = 1.0
                        rviz_marker.color.a = 1.0
                    else:
                        # green
                        rviz_marker.color.r = 0.0
                        rviz_marker.color.g = 1.0
                        rviz_marker.color.b = 0.0
                        rviz_marker.color.a = 1.0
                        
                    
                    self._visualPub.publish(rviz_marker)
            
            self._pub.publish(markers)
            seq = seq + 1
            
            # no spinning needed in rospy
            rospy.sleep(1 / _pub_freq)
     
    def callback(self, data):
         for objIdx in range(len(data.name)):
             self.objs[str(data.name[objIdx])] = data.pose[objIdx]
     
if __name__ == '__main__':
    tracker = GazeboObjectTracker()
    try:
        tracker.publish()
    except rospy.ROSInterruptException:
        pass
