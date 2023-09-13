#!/usr/bin/env python3

import math
import rospy
import random
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import Ptp, Lin, Robot, Gripper, from_euler, Sequence
from prbt_hardware_support.srv import WriteModbusRegister
from prbt_hardware_support.msg import ModbusRegisterBlock, ModbusMsgInStamped
from agv_prbt_2022.srv import GetMarkerCenter
from agv_task_deploy_wrapper import agv_task_deply_wrapper


# 位置常量（因涉及到实际机械位置，因此不要修改）
HOME_POSE = [
    math.radians(-90),
    math.radians(0),
    math.radians(-90),
    math.radians(0),
    math.radians(-90),
    math.radians(135),
]  # 起始关节角度
CAPTURE_POSE = [
    math.radians(90),
    math.radians(0),
    math.radians(-60),
    math.radians(0),
    math.radians(-120),
    math.radians(135),
]  # 相机拍照角度
GRIPPER_ORIENTATION = from_euler(0, math.radians(180), math.radians(180))  # 夹爪方向
GRIPPER_ORIENTATION_AGV = from_euler(0, math.radians(180), math.radians(181))  # AGV夹爪方向
AGV_PLATE_FULL_POSE = Pose(
    position=Point(-0.128, -0.357, 0.138), orientation=GRIPPER_ORIENTATION_AGV
)  # AGV满料盘位置
AGV_PLATE_EMPTY_POSE = Pose(
    position=Point(0.116, -0.361, 0.138), orientation=GRIPPER_ORIENTATION_AGV
)  # AGV空料盘位置
SAFETY_HEIGHT = 0.32
FEED_TABLE_BOX_FULL_OFFSET_X = 0.102
FEED_TABLE_BOX_EMPTY_OFFSET_X = 0.102 + 0.2135
FEED_TABLE_PEN_FULL_OFFSET_X = 0.102 - 0.214 * 2
FEED_TABLE_PEN_EMPTY_OFFSET_X = 0.102 - 0.2135
FEED_TABLE_OFFSET_Y = 0.121
FEED_TABLE_PNP_OFFSET_Z = 0.073
SMF_TABLE_BOX_OFFSET_X = 0.264
SMF_TABLE_PEN_OFFSET_X = 0.264 - 0.27
SMF_TABLE_OFFSET_Y = 0.1515
CAMERA_GRIPPER_OFFSET = 0.066
PLATE_HEIGHT = 0.025

# 速度常量
PTP_SCALE = 0.1  # 点到点移动速度比例
LIN_SCALE = 0.05  # 直线移动速度
PNP_SCALE = 0.02  # 拾取与放置速度

# 发送至PSS Modbus寄存器地址
pss_modbus_write_dic = {
    "agv_ros_program_run": 1000,
    "agv_at_robot_station": 1010,
    "agv_prbt_changing_box": 1011,
    "agv_prbt_changing_pen": 1012,
    "agv_prbt_at_home": 1013,
}


def pss_modbus_write(start_idx, values):
    rospy.wait_for_service("/pilz_modbus_client_node/modbus_write")
    try:
        modbus_write_client = rospy.ServiceProxy(
            "/pilz_modbus_client_node/modbus_write", WriteModbusRegister
        )
        modbus_write_client(ModbusRegisterBlock(start_idx, values))
    except rospy.ServiceException as e:
        print("Modbus write service call failed: %s", e)


def pss_modbus_read_callback(data):
    global pen_plate_pick_number
    global pen_plate_place_number
    global box_plate_pick_number
    global box_plate_place_number
    global box_missing
    global pen_missing
    global smfPrbt_picking_box
    global smfPrbt_picking_pen
    global smfPrbt_placing_box
    global smfPrbt_placing_pen
    global smfPrbt_at_home
    external_start = data.holding_registers.data[1]
    external_stop = data.holding_registers.data[2]
    external_reset = data.holding_registers.data[3]
    robot_run_permission = data.holding_registers.data[4]
    pen_plate_pick_number = data.holding_registers.data[10]
    pen_plate_place_number = data.holding_registers.data[11]
    box_plate_pick_number = data.holding_registers.data[12]
    box_plate_place_number = data.holding_registers.data[13]
    smfPrbt_picking_box = data.holding_registers.data[15]
    smfPrbt_picking_pen = data.holding_registers.data[17]
    smfPrbt_placing_box = data.holding_registers.data[19]
    smfPrbt_placing_pen = data.holding_registers.data[21]
    smfPrbt_at_home = data.holding_registers.data[23]
    box_missing = data.holding_registers.data[26]
    pen_missing = data.holding_registers.data[27]

    # if not robot_run_permission or external_stop:
    if external_stop:
        r.pause()

    if external_start:
        rospy.sleep(1)
        r.resume()


def get_marker_center():
    global feedTable_x, feedTable_y
    global smfTable_x, smfTable_y
    global angle
    rospy.wait_for_service("get_marker_center")
    try:
        get_marker_center = rospy.ServiceProxy("get_marker_center", GetMarkerCenter)
        response = get_marker_center()
        feedTable_x = response.feedTable_x
        feedTable_y = response.feedTable_y
        smfTable_x = response.smfTable_x
        smfTable_y = response.smfTable_y
        angle = response.angle
    except rospy.ServiceException as e:
        print("Service call failed: %s", e)


if __name__ == "__main__":
    rospy.init_node("agv_prbt_application")
    rospy.loginfo("AGV PRBT application started")
    rospy.Subscriber(
        "/pilz_modbus_client_node/modbus_read",
        ModbusMsgInStamped,
        pss_modbus_read_callback,
        queue_size=1,
    )
    pss_modbus_write(pss_modbus_write_dic["agv_ros_program_run"], [1])
    pss_modbus_write(pss_modbus_write_dic["agv_at_robot_station"], [0])

    # 初始化
    r = Robot("1")  # 创建化机器人实例

    agv_task_id_deploy = random.randint(1, 100)
    agv_task_id_lookup = agv_task_id_deploy

    """
    安全位置检测
    获取当前机器人位置
    若当前Z位置小于安全高度，则反向提升至安全高度
    """
    current_pose = r.get_current_pose()
    while current_pose.position.z < SAFETY_HEIGHT:
        r.move(
            Lin(
                goal=Pose(
                    position=Point(
                        0,
                        0,
                        -0.05,
                    )
                ),
                reference_frame="prbt_tcp",
                vel_scale=LIN_SCALE,
                acc_scale=0.1,
            )
        )
        current_pose = r.get_current_pose()

    if current_pose.position.y > 0:
        r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
    r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
    pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [1])

    while not rospy.is_shutdown():
        if box_missing:
            if 0 < box_plate_pick_number <= 5:
                # 拍照并移动至满料盘位置
                pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [0])
                r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                rospy.sleep(0.5)
                get_marker_center()
                r.move(
                    Ptp(
                        goal=Pose(orientation=from_euler(0, 0, math.radians(angle))),
                        reference_frame="prbt_tcp",
                        vel_scale=PTP_SCALE,
                        acc_scale=0.1,
                    )
                )

                rospy.sleep(0.5)
                get_marker_center()
                r.move(
                    Lin(
                        goal=Pose(
                            position=Point(
                                -feedTable_x - FEED_TABLE_BOX_FULL_OFFSET_X,
                                feedTable_y
                                + FEED_TABLE_OFFSET_Y
                                + CAMERA_GRIPPER_OFFSET,
                                0.3,
                            ),
                        ),
                        reference_frame="prbt_tcp",
                        vel_scale=LIN_SCALE,
                        acc_scale=0.1,
                    )
                )

                # 开始抓取满料盘
                pick_start_pose = r.get_current_pose()
                pick_height = (
                    FEED_TABLE_PNP_OFFSET_Z + (5 - box_plate_pick_number) * 0.025
                )
                r.move(Gripper(goal=0.029))
                r.move(
                    Lin(
                        goal=Pose(position=Point(0, 0, pick_height)),
                        reference_frame="prbt_tcp",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.022))
                rospy.sleep(0.5)
                r.move(
                    Lin(
                        goal=pick_start_pose,
                        reference_frame="prbt_base_link",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
                r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [1])

                # 将满料盘放到小车
                pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [0])
                r.move(
                    Lin(
                        goal=AGV_PLATE_FULL_POSE,
                        reference_frame="prbt_base_link",
                        vel_scale=LIN_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(
                    Lin(
                        goal=Pose(position=Point(0, 0, 0.05)),
                        reference_frame="prbt_tcp",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.029))
                rospy.sleep(0.5)
                r.move(
                    Lin(
                        goal=AGV_PLATE_FULL_POSE,
                        reference_frame="prbt_base_link",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [1])

                # AGV去工作站位置
                agv_task_id_deploy = agv_task_deply_wrapper(agv_task_id_deploy, 5)
                pss_modbus_write(pss_modbus_write_dic["agv_at_robot_station"], [1])

                # 机器人从工作站拾取空料盘
                while not smfPrbt_at_home:
                    r.pause()
                r.resume()
                pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [0])
                r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                rospy.sleep(0.5)
                get_marker_center()
                r.move(
                    Ptp(
                        goal=Pose(orientation=from_euler(0, 0, math.radians(angle))),
                        reference_frame="prbt_tcp",
                        vel_scale=PTP_SCALE,
                        acc_scale=0.1,
                    )
                )

                rospy.sleep(0.5)
                get_marker_center()

                r.move(
                    Lin(
                        goal=Pose(
                            position=Point(
                                -smfTable_x - SMF_TABLE_BOX_OFFSET_X,
                                smfTable_y + SMF_TABLE_OFFSET_Y + CAMERA_GRIPPER_OFFSET,
                                0.28,
                            )
                        ),
                        reference_frame="prbt_tcp",
                        vel_scale=LIN_SCALE,
                        acc_scale=0.1,
                    )
                )
                pick_start_pose = r.get_current_pose()
                pick_height = 0.041
                r.move(Gripper(goal=0.029))
                r.move(
                    Lin(
                        goal=Pose(position=Point(0, 0, pick_height)),
                        reference_frame="prbt_tcp",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.022))
                rospy.sleep(0.5)
                r.move(
                    Lin(
                        goal=pick_start_pose,
                        reference_frame="prbt_base_link",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))

                # 将空料盘放到小车
                r.move(
                    Ptp(
                        goal=AGV_PLATE_EMPTY_POSE,
                        reference_frame="prbt_base_link",
                        vel_scale=PTP_SCALE,
                        acc_scale=0.2,
                    )
                )
                r.move(
                    Lin(
                        goal=Pose(position=Point(0, 0, 0.05)),
                        reference_frame="prbt_tcp",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.029))
                rospy.sleep(0.5)
                r.move(
                    Lin(
                        goal=AGV_PLATE_EMPTY_POSE,
                        reference_frame="prbt_base_link",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )

                # 从小车拾取满料盘放到工作站
                r.move(
                    Lin(
                        goal=AGV_PLATE_FULL_POSE,
                        reference_frame="prbt_base_link",
                        vel_scale=LIN_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.029))
                r.move(
                    Lin(
                        goal=Pose(position=Point(0, 0, 0.048)),
                        reference_frame="prbt_tcp",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.022))
                rospy.sleep(0.5)
                r.move(
                    Lin(
                        goal=AGV_PLATE_FULL_POSE,
                        reference_frame="prbt_base_link",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                r.move(
                    Lin(
                        goal=pick_start_pose,
                        reference_frame="prbt_base_link",
                        vel_scale=LIN_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(
                    Lin(
                        goal=Pose(position=Point(0, 0, pick_height)),
                        reference_frame="prbt_tcp",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.029))
                rospy.sleep(0.5)
                r.move(
                    Lin(
                        goal=pick_start_pose,
                        reference_frame="prbt_base_link",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
                r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [1])

                # AGV去上料台位置
                pss_modbus_write(pss_modbus_write_dic["agv_at_robot_station"], [0])
                agv_task_id_deploy = agv_task_deply_wrapper(agv_task_id_deploy, 1)

                while not (0 <= box_plate_place_number < 5):
                    r.pause()

                if 0 <= box_plate_place_number < 5:
                    r.resume()
                    pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [0])
                    r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                    rospy.sleep(0.5)
                    get_marker_center()
                    r.move(
                        Ptp(
                            goal=Pose(
                                orientation=from_euler(0, 0, math.radians(angle))
                            ),
                            reference_frame="prbt_tcp",
                            vel_scale=PTP_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    rospy.sleep(0.5)
                    get_marker_center()
                    place_middle_waypoint = r.get_current_pose()
                    r.move(
                        Ptp(
                            goal=AGV_PLATE_EMPTY_POSE,
                            reference_frame="prbt_base_link",
                            vel_scale=PTP_SCALE,
                            acc_scale=0.2,
                        )
                    )
                    r.move(Gripper(goal=0.029))
                    r.move(
                        Lin(
                            goal=Pose(position=Point(0, 0, 0.048)),
                            reference_frame="prbt_tcp",
                            vel_scale=PNP_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    r.move(Gripper(goal=0.022))
                    rospy.sleep(0.5)
                    r.move(
                        Lin(
                            goal=AGV_PLATE_EMPTY_POSE,
                            reference_frame="prbt_base_link",
                            vel_scale=PNP_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    r.move(
                        Ptp(
                            goal=place_middle_waypoint,
                            reference_frame="prbt_base_link",
                            vel_scale=PTP_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    r.move(
                        Lin(
                            goal=Pose(
                                position=Point(
                                    -feedTable_x - FEED_TABLE_BOX_EMPTY_OFFSET_X,
                                    feedTable_y
                                    + FEED_TABLE_OFFSET_Y
                                    + CAMERA_GRIPPER_OFFSET
                                    + 0.002,
                                    0.3,
                                )
                            ),
                            reference_frame="prbt_tcp",
                            vel_scale=LIN_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    place_start_pose = r.get_current_pose()
                    place_height = (
                        FEED_TABLE_PNP_OFFSET_Z
                        + (5 - box_plate_place_number) * 0.025
                        - 0.023
                    )
                    r.move(
                        Lin(
                            goal=Pose(position=Point(0, 0, place_height)),
                            reference_frame="prbt_tcp",
                            vel_scale=PNP_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    r.move(Gripper(goal=0.029))
                    rospy.sleep(0.5)
                    r.move(
                        Lin(
                            goal=place_start_pose,
                            reference_frame="prbt_base_link",
                            vel_scale=PNP_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
                    r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                    pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [1])

        if pen_missing:
            if 0 < pen_plate_pick_number <= 5:
                # 拍照并移动至满料盘位置
                pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [0])
                r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                rospy.sleep(0.5)
                get_marker_center()
                r.move(
                    Ptp(
                        goal=Pose(orientation=from_euler(0, 0, math.radians(angle))),
                        reference_frame="prbt_tcp",
                        vel_scale=PTP_SCALE,
                        acc_scale=0.1,
                    )
                )

                rospy.sleep(0.5)
                get_marker_center()
                r.move(
                    Lin(
                        goal=Pose(
                            position=Point(
                                -feedTable_x - FEED_TABLE_PEN_FULL_OFFSET_X,
                                feedTable_y
                                + FEED_TABLE_OFFSET_Y
                                + CAMERA_GRIPPER_OFFSET
                                - 0.003,
                                0.3,
                            ),
                        ),
                        reference_frame="prbt_tcp",
                        vel_scale=LIN_SCALE,
                        acc_scale=0.1,
                    )
                )

                # 开始抓取满料盘
                pick_start_pose = r.get_current_pose()
                pick_height = (
                    FEED_TABLE_PNP_OFFSET_Z + (5 - pen_plate_pick_number) * 0.025
                )
                r.move(Gripper(goal=0.029))
                r.move(
                    Lin(
                        goal=Pose(position=Point(0, 0, pick_height)),
                        reference_frame="prbt_tcp",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.022))
                rospy.sleep(0.5)
                r.move(
                    Lin(
                        goal=pick_start_pose,
                        reference_frame="prbt_base_link",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
                r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [1])

                # 将满料盘放到小车
                pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [0])
                r.move(
                    Lin(
                        goal=AGV_PLATE_FULL_POSE,
                        reference_frame="prbt_base_link",
                        vel_scale=LIN_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(
                    Lin(
                        goal=Pose(position=Point(0, 0, 0.05)),
                        reference_frame="prbt_tcp",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.029))
                rospy.sleep(0.5)
                r.move(
                    Lin(
                        goal=AGV_PLATE_FULL_POSE,
                        reference_frame="prbt_base_link",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [1])

                # AGV去工作站位置
                agv_task_id_deploy = agv_task_deply_wrapper(agv_task_id_deploy, 5)
                pss_modbus_write(pss_modbus_write_dic["agv_at_robot_station"], [1])

                # 机器人从工作站拾取空料盘
                while not smfPrbt_at_home:
                    r.pause()
                r.resume()
                pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [0])
                r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                rospy.sleep(0.5)
                get_marker_center()
                r.move(
                    Ptp(
                        goal=Pose(orientation=from_euler(0, 0, math.radians(angle))),
                        reference_frame="prbt_tcp",
                        vel_scale=PTP_SCALE,
                        acc_scale=0.1,
                    )
                )

                rospy.sleep(0.5)
                get_marker_center()

                r.move(
                    Lin(
                        goal=Pose(
                            position=Point(
                                -smfTable_x - SMF_TABLE_PEN_OFFSET_X,
                                smfTable_y + SMF_TABLE_OFFSET_Y + CAMERA_GRIPPER_OFFSET,
                                0.28,
                            )
                        ),
                        reference_frame="prbt_tcp",
                        vel_scale=LIN_SCALE,
                        acc_scale=0.1,
                    )
                )
                pick_start_pose = r.get_current_pose()
                pick_height = 0.041
                r.move(Gripper(goal=0.029))
                r.move(
                    Lin(
                        goal=Pose(position=Point(0, 0, pick_height)),
                        reference_frame="prbt_tcp",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.022))
                rospy.sleep(0.5)
                r.move(
                    Lin(
                        goal=pick_start_pose,
                        reference_frame="prbt_base_link",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))

                # 将空料盘放到小车
                r.move(
                    Ptp(
                        goal=AGV_PLATE_EMPTY_POSE,
                        reference_frame="prbt_base_link",
                        vel_scale=PTP_SCALE,
                        acc_scale=0.2,
                    )
                )
                r.move(
                    Lin(
                        goal=Pose(position=Point(0, 0, 0.05)),
                        reference_frame="prbt_tcp",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.029))
                rospy.sleep(0.5)
                r.move(
                    Lin(
                        goal=AGV_PLATE_EMPTY_POSE,
                        reference_frame="prbt_base_link",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )

                # 从小车拾取满料盘放到工作站
                r.move(
                    Lin(
                        goal=AGV_PLATE_FULL_POSE,
                        reference_frame="prbt_base_link",
                        vel_scale=LIN_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.029))
                r.move(
                    Lin(
                        goal=Pose(position=Point(0, 0, 0.048)),
                        reference_frame="prbt_tcp",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.022))
                rospy.sleep(0.5)
                r.move(
                    Lin(
                        goal=AGV_PLATE_FULL_POSE,
                        reference_frame="prbt_base_link",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                r.move(
                    Lin(
                        goal=pick_start_pose,
                        reference_frame="prbt_base_link",
                        vel_scale=LIN_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(
                    Lin(
                        goal=Pose(position=Point(0, 0, pick_height)),
                        reference_frame="prbt_tcp",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Gripper(goal=0.029))
                rospy.sleep(0.5)
                r.move(
                    Lin(
                        goal=pick_start_pose,
                        reference_frame="prbt_base_link",
                        vel_scale=PNP_SCALE,
                        acc_scale=0.1,
                    )
                )
                r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
                r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [1])

                # AGV去上料台位置
                pss_modbus_write(pss_modbus_write_dic["agv_at_robot_station"], [0])
                agv_task_id_deploy = agv_task_deply_wrapper(agv_task_id_deploy, 1)

                while not (0 <= pen_plate_place_number < 5):
                    r.pause()

                if 0 <= pen_plate_place_number < 5:
                    r.resume()
                    pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [0])
                    r.move(Ptp(goal=CAPTURE_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                    rospy.sleep(0.5)
                    get_marker_center()
                    r.move(
                        Ptp(
                            goal=Pose(
                                orientation=from_euler(0, 0, math.radians(angle))
                            ),
                            reference_frame="prbt_tcp",
                            vel_scale=PTP_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    rospy.sleep(0.5)
                    get_marker_center()
                    place_middle_waypoint = r.get_current_pose()
                    r.move(
                        Ptp(
                            goal=AGV_PLATE_EMPTY_POSE,
                            reference_frame="prbt_base_link",
                            vel_scale=PTP_SCALE,
                            acc_scale=0.2,
                        )
                    )
                    r.move(Gripper(goal=0.029))
                    r.move(
                        Lin(
                            goal=Pose(position=Point(0, 0, 0.048)),
                            reference_frame="prbt_tcp",
                            vel_scale=PNP_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    r.move(Gripper(goal=0.022))
                    rospy.sleep(0.5)
                    r.move(
                        Lin(
                            goal=AGV_PLATE_EMPTY_POSE,
                            reference_frame="prbt_base_link",
                            vel_scale=PNP_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    r.move(
                        Ptp(
                            goal=place_middle_waypoint,
                            reference_frame="prbt_base_link",
                            vel_scale=PTP_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    r.move(
                        Lin(
                            goal=Pose(
                                position=Point(
                                    -feedTable_x - FEED_TABLE_PEN_EMPTY_OFFSET_X,
                                    feedTable_y
                                    + FEED_TABLE_OFFSET_Y
                                    + CAMERA_GRIPPER_OFFSET
                                    - 0.002,
                                    0.3,
                                )
                            ),
                            reference_frame="prbt_tcp",
                            vel_scale=LIN_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    place_start_pose = r.get_current_pose()
                    place_height = (
                        FEED_TABLE_PNP_OFFSET_Z
                        + (5 - pen_plate_place_number) * 0.025
                        - 0.023
                    )
                    r.move(
                        Lin(
                            goal=Pose(position=Point(0, 0, place_height)),
                            reference_frame="prbt_tcp",
                            vel_scale=PNP_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    r.move(Gripper(goal=0.029))
                    rospy.sleep(0.5)
                    r.move(
                        Lin(
                            goal=place_start_pose,
                            reference_frame="prbt_base_link",
                            vel_scale=PNP_SCALE,
                            acc_scale=0.1,
                        )
                    )
                    r.move(Lin(goal=CAPTURE_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
                    r.move(Ptp(goal=HOME_POSE, vel_scale=PTP_SCALE, acc_scale=0.2))
                    pss_modbus_write(pss_modbus_write_dic["agv_prbt_at_home"], [1])

    rospy.spin()
