#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from agv_communication import agv_communication_deploy, agv_communication_lookup


def agv_task_deply_wrapper(task_id, task_number):
    agv_task_id_current, _ = agv_communication_deploy(task_id, task_number)
    rospy.sleep(5)
    agv_task_id_current, agv_task_lookup_result = agv_communication_lookup(task_id, task_number)
    while agv_task_lookup_result != 3:
        rospy.loginfo("AGV executing task No %s", task_number)
        rospy.sleep(0.5)
        agv_task_id_current, agv_task_lookup_result = agv_communication_lookup(task_id, task_number)
    if agv_task_lookup_result == 3:
        rospy.loginfo("AGV finished task No %s", task_number)
        agv_task_id_deploy = agv_task_id_current + 1
    return agv_task_id_deploy
