#! /usr/bin/python

import sys
import rospy

from ed_navigation.srv import *
from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *

if __name__ == '__main__':
    
    if len(sys.argv) < 3:
        print "Please provide: ENTITY_ID AREA_NAME"
        exit(1)

    entity_id = sys.argv[1]
    area_name = sys.argv[2]

    rospy.init_node('show_ed_object_area')

    rospy.wait_for_service('/amigo/ed/navigation/get_constraint')
    rospy.wait_for_service('/amigo/global_planner/get_plan_srv')

    get_contraint = rospy.ServiceProxy('/amigo/ed/navigation/get_constraint', GetGoalConstraint)
    get_plan = rospy.ServiceProxy('/amigo/global_planner/get_plan_srv', GetPlan)

    res1 = get_contraint(entity_ids=[entity_id], area_names=[area_name])    

    print "Constraint: {}".format(res1.position_constraint_map_frame)

    constr = PositionConstraint()
    constr.frame = "map"
    constr.constraint = res1.position_constraint_map_frame

    resp2 = get_plan(goal_position_constraints=[constr])

