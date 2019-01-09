#! /usr/bin/python

import sys
import rospy

from ed_navigation.srv import *
from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *

if __name__ == '__main__':
    
    if len(sys.argv) < 4:
        print("Please provide: ROBOT_NAME ENTITY_ID AREA_NAME")
        exit(1)

    robot_name = sys.argv[1]
    entity_id = sys.argv[2]
    area_name = sys.argv[3]

    rospy.init_node('show_ed_object_area')

    rospy.wait_for_service('/{}/ed/navigation/get_constraint'.format(robot_name))
    rospy.wait_for_service('/{}/global_planner/get_plan_srv'.format(robot_name))

    get_contraint = rospy.ServiceProxy('/{}/ed/navigation/get_constraint'.format(robot_name), GetGoalConstraint)
    get_plan = rospy.ServiceProxy('/{}/global_planner/get_plan_srv'.format(robot_name), GetPlan)

    res1 = get_contraint(entity_ids=[entity_id], area_names=[area_name])    

    print("Constraint: {}".format(res1.position_constraint_map_frame))

    constr = PositionConstraint()
    constr.frame = "map"
    constr.constraint = res1.position_constraint_map_frame

    res2 = get_plan(goal_position_constraints=[constr])
