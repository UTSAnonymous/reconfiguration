#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import *

class Connection:

    def __init__(self):
        # connect to attachment ServiceProxy
        # both uses the attach srv msg (might be confusing at first)
        self.__attachSrv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.__detactSrv = rospy.ServiceProxy('/link_attacher_node/detach', Detach)

        self.__attachSrv.wait_for_service()
        self.__detactSrv.wait_for_service()
        rospy.loginfo("Connection service initialised")

    def Attach(self,model_name_1, link_name_1, model_name_2,link_name_2,type, pose):
        req = AttachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2
        req.type = type
        req.pose = pose

        result = self.__attachSrv.call(req)
        return result

    def Detach(self,model_name_1, link_name_1, model_name_2, link_name_2):
        req = DetachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2

        result = self.__detactSrv.call(req)
        return result
