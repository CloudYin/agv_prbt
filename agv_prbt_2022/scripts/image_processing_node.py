#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from agv_prbt_2022.srv import GetMarkerCenter, GetMarkerCenterResponse


class ImageProcessing:
    def __init__(self):
        rospy.init_node("image_processing")
        self.bridge_ = CvBridge()
        self.cv_image_ = None
        self.image_subscriber_ = rospy.Subscriber(
            "pylon_camera_node/image_rect_color", Image, self.image_callback
        )
        self.image_processing_server_ = rospy.Service(
            "get_marker_center", GetMarkerCenter, self.callback_GetMarkerCenter
        )

    def image_callback(self, data):
        try:
            self.cv_image_ = self.bridge_.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s" % e)

    def callback_GetMarkerCenter(self, req):
        image_hsv = cv2.cvtColor(self.cv_image_, cv2.COLOR_BGR2HSV)
        lower_b = np.array([100, 50, 50])
        upper_b = np.array([120, 255, 255])
        mask = cv2.inRange(image_hsv, lower_b, upper_b)
        cv2.bitwise_and(self.cv_image_, self.cv_image_, mask=mask)

        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
        )

        # Select inner contours
        hierarchy = hierarchy[0]
        for component in zip(contours, hierarchy):
            currentContour = component[0]
            currentHierarchy = component[1]
            if currentHierarchy[3] == -1:
                # these are the outermost child components
                img_contour = cv2.drawContours(
                    self.cv_image_, currentContour, -1, (0, 0, 0)
                )
                if cv2.moments(currentContour)["m00"] > 800:
                    m00 = cv2.moments(currentContour)["m00"]
                    m01 = cv2.moments(currentContour)["m01"]
                    m10 = cv2.moments(currentContour)["m10"]
                    centerX = round(m10 / m00)
                    centerY = round(m01 / m00)
                    rect = cv2.minAreaRect(currentContour)
                    points = cv2.boxPoints(rect)
                    points = np.int0(points)
                    cv2.drawContours(img_contour, [points], -1, (0, 255, 0), 2)
                    angle = cv2.minAreaRect(currentContour)[2]
                    if 45 < angle <= 90:
                        angle = angle - 90
                    cv2.circle(
                        img_contour, (int(centerX), int(centerY)), 2, (0, 0, 255)
                    )
                    feedTable_x = ((centerX - 1000) * 0.1127) / 1000
                    feedTable_y = ((800 - centerY) * 0.1062) / 1000
                    smfTable_x = ((centerX - 1000) * 0.1199) / 1000
                    smfTable_y = ((800 - centerY) * 0.1163) / 1000
        # cv2.imshow("img_contour", img_contour)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        cv2.imwrite("/home/pilz/Pictures/agv_prbt/table_contour.jpg", img_contour)
        return GetMarkerCenterResponse(
            feedTable_x, feedTable_y, smfTable_x, smfTable_y, angle
        )


if __name__ == "__main__":
    ImageProcessing()
    rospy.spin()
