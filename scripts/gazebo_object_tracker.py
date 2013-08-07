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
IKEA-CUP-SOLBRAEND-BLU Marker 2

"""

class GazeboObjectTracker:
    def __init__(self):
        rospy.init_node('gazebo_object_tracker')
        
        self.objs = {}
        self.objDict = {'COCA-COLA-CAN-250ML' : 1, 'IKEA-CUP-SOLBRAEND-BLUE' : 2}
        
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        
        self._pub = rospy.Publisher('ar_pose_marker', ARMarkers)
        self._visualPub = rospy.Publisher('visualization_marker', Marker)
        self._tf_broadcaster = tf.TransformBroadcaster()
        
        rospy.loginfo("gazebo object tracker started")

    def publish(self):
        
        
        seq = 0
        while not rospy.is_shutdown():
            markers = ARMarkers()
            markers.header.stamp = rospy.get_rostime()
            markers.header.seq = seq
            markers.header.frame_id = _frame_id
            
            for name in self.objs.keys():
                if name in self.objDict.keys():
                    ################################
                    # publish AR marker
                    ################################
                    
                    marker = ARMarker()
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
