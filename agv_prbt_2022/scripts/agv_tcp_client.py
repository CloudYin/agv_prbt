#!/usr/bin/env python3

import rospy
import socket
import struct
from agv_prbt_2022.msg import AgvToSmfStatus, SmfToAgvStatus


class AgvTcpClient:
    def __init__(self):
        rospy.init_node("agv_tcp_client")
        self.server_ip_ = rospy.get_param("~ip", "192.168.251.112")
        self.port_ = rospy.get_param("~port", 65432)
        self.sock_ = None
        self.agvAtSmf_ = False
        self.agvPrbtChangingBox_ = False
        self.agvPrbtChangingPen_ = False
        self.agvPrbtAtHome_ = False
        self.publisher_ = rospy.Publisher(
            "smf_to_agv_status", SmfToAgvStatus, queue_size=10
        )
        rospy.Subscriber(
            "agv_to_smf_status", AgvToSmfStatus, self.callback_AgvToSmfStatus
        )
        self.start_tcp_client()
        self.transmit_tcp_data()

    def callback_AgvToSmfStatus(self, data):
        self.agvAtSmf_ = data.agvAtSmf
        self.agvPrbtChangingBox_ = data.agvPrbtChangingBox
        self.agvPrbtChangingPen_ = data.agvPrbtChangingPen
        self.agvPrbtAtHome_ = data.agvPrbtAtHome

    def start_tcp_client(self):
        while not rospy.is_shutdown():
            try:
                self.sock_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock_.connect((self.server_ip_, self.port_))
                rospy.loginfo("Connected to %s", self.server_ip_)
                break
            except socket.error as e:
                self.sock_.close()
                rospy.logerr("Connection error: %s, reconnecting...", e)
                rospy.sleep(1)

    def transmit_tcp_data(self):
        while not rospy.is_shutdown():
            try:
                tcp_send_msg = struct.pack(
                    "!????",
                    self.agvAtSmf_,
                    self.agvPrbtChangingBox_,
                    self.agvPrbtChangingPen_,
                    self.agvPrbtAtHome_,
                )
                self.sock_.send(tcp_send_msg)
                data = self.sock_.recv(1024)
                tcp_receive_msg = struct.unpack("!???????", data)
                smfToAgvMsg = SmfToAgvStatus()
                smfToAgvMsg.boxMissing = tcp_receive_msg[0]
                smfToAgvMsg.penMissing = tcp_receive_msg[1]
                smfToAgvMsg.smfPrbtPickingBox = tcp_receive_msg[2]
                smfToAgvMsg.smfPrbtPickingPen = tcp_receive_msg[3]
                smfToAgvMsg.smfPrbtPlacingBox = tcp_receive_msg[4]
                smfToAgvMsg.smfPrbtPlacingPen = tcp_receive_msg[5]
                smfToAgvMsg.smfPrbtAtHome = tcp_receive_msg[6]
                self.publisher_.publish(smfToAgvMsg)
            except socket.error as e:
                self.sock_.close()
                rospy.logerr("Connection error: %s", e)
                self.start_tcp_client()
            rospy.sleep(0.1)


if __name__ == "__main__":
    AgvTcpClient()
    rospy.spin()
