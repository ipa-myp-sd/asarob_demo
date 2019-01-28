#!/usr/bin/env python

import rospy
import actionlib
import simple_script_server
sss = simple_script_server.simple_script_server()

class TrajectoryExecution(simple_script_server.script):
    def __init__(self):
        rospy.loginfo("Initializing Trajectory Execution")
        sss.init("arm_left")
        sss.init("arm_right")
        sss.move("arm_right","side")
        point0 = [[1.080563267708147, 1.217155545308433, -1.3958713748888183, -1.6480694373057911, -0.8877923106149632, -0.35130350564666646, 1.068305519131009]]
        point1 = [[1.1042523228704306, 1.1305731443564335, -1.4643351484527116, -1.6461103026252353, -0.856494667666925, -0.283204827987106, 1.068305519131009]]
        point2 = [[1.4293331317214442, 0.9202928011615645, -1.7371342144837492, -1.6066390315926524, -0.7578441564939773, -0.2915972507295326, 1.068305519131009]]
        point3 = [[1.6955817735558911, 0.7480505182423105, -1.9605708619286162, -1.5743321318898014, -0.6770427714243787, -0.2984573917592144, 1.068305519131009]]
        point4 = [[2.0365471687616896, 0.5274752981259336, -2.246708212935909, -1.5329572790293566, -0.5735659588841662, -0.30724068380699165, 1.068305519131009]]
        point5 = [[2.8349540630098407, 0.010985892324975488, -2.9167374669334363, -1.4360783398255421, -0.33125283046196596, -0.3278046281006306, 1.068305519131009]]
        point6 = [[2.2671826983406342, -0.36800267278300436, -2.29318810419535, -1.2845797827603465, -0.09478883167581204, -0.3888069974667768, 1.0699740979351238]] # used for pointing inthe torso motion
        self.point = [point0, point1, point2, point3, point4, point5, point6]

    def execute(self):
         rospy.loginfo("Executing Trajectory")
         for point in self.point:
             move = sss.move("arm_right",point,blocking=True) # blocking=False means that move continously without waiting for results
         rospy.loginfo("Executing Trajectory Finished")

if __name__ == "__main__":
    rospy.init_node("asarob_demo")
    SCRIPT = TrajectoryExecution()
    SCRIPT.execute()
    rospy.spin()
