#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Float32MultiArray
from ggcnn.ggcnn_torch import predict, process_depth_image
from dougsm_helpers.timeit import TimeIt

bridge = CvBridge()
crop_size = 300
crop_offset = 40
out_size = 300

def depth_callback(depth_message):
    global best_g_unr, angle, width
    with TimeIt('Predict'):
        depth = bridge.imgmsg_to_cv2(depth_message)
        #  Crop a square out of the middle of the depth and resize it to 300*300
        crop_size = 300
        crop_offset = 40
        out_size = 300

        depth_crop, depth_nan_mask = process_depth_image(depth, crop_size, 300, return_mask=True,
                                                         crop_y_offset=crop_offset)
        points, angle, width_img, _ = predict(depth_crop, process_depth=False, depth_nan_mask=depth_nan_mask,
                                              filters=(2.0, 2.0, 2.0))
        best_g = np.argmax(points)
        best_g_unr = np.unravel_index(best_g, points.shape)

        angle = angle[best_g_unr]
        width = width_img[best_g_unr]

        best_g_unr = ((np.array(best_g_unr) / out_size * crop_size) + np.array(
                 [(480 - crop_size) // 2 - crop_offset, (640 - crop_size) // 2]))

        # msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=50)
        msg = rospy.Subscriber('/camera/color/image_raw', Image, convert_msg_to_cv, queue_size=1)


def convert_msg_to_cv(msg):
    global img_rgb, best_g_unr, angle, width
    img_rgb = bridge.imgmsg_to_cv2(msg)
    img_rgb = img_rgb[(480 - crop_size) // 2 - crop_offset:(480 - crop_size) // 2 + crop_size - crop_offset,
                           (640 - crop_size) // 2:(640 - crop_size) // 2 + crop_size]
    as_gr = rectangle_point(best_g_unr, angle, width)
    draw_rectangle(img_rgb, as_gr)
    print(img_rgb.shape)
    publish_img = bridge.cv2_to_imgmsg(img_rgb)
    ggcnn_draw_pub.publish(publish_img)


        # Convert max_pixel back to uncropped/resized image coordinates in order to do the camera transform.
        # max_pixel = ((np.array(max_pixel) / out_size * crop_size) + np.array(
        #     [(480 - crop_size) // 2 - crop_offset, (640 - crop_size) // 2]))
        # max_pixel = np.round(max_pixel).astype(np.int)


def rectangle_point(center, angle, length):
    """
    center: [y, x]
    """
    xo = np.cos(angle)
    yo = np.sin(angle)
    width = length / 2

    y1 = center[0] + length / 2 * yo
    x1 = center[1] - length / 2 * xo
    y2 = center[0] - length / 2 * yo
    x2 = center[1] + length / 2 * xo

    return np.array(
        [
            [y1 - width / 2 * xo, x1 - width / 2 * yo],
            [y2 - width / 2 * xo, x2 - width / 2 * yo],
            [y2 + width / 2 * xo, x2 + width / 2 * yo],
            [y1 + width / 2 * xo, x1 + width / 2 * yo],
        ]
    ).astype(np.int)


def draw_rectangle(img, as_gr):
    cv2.line(img, (as_gr[0][1], as_gr[0][0]), (as_gr[1][1], as_gr[1][0]), (255, 0, 0), 1)
    cv2.line(img, (as_gr[2][1], as_gr[2][0]), (as_gr[3][1], as_gr[3][0]), (255, 0, 0), 1)
    cv2.line(img, (as_gr[3][1], as_gr[3][0]), (as_gr[0][1], as_gr[0][0]), (0, 0, 255), 1)
    cv2.line(img, (as_gr[1][1], as_gr[1][0]), (as_gr[2][1], as_gr[2][0]), (0, 0, 255), 1)


if __name__ == '__main__':
    global img_rgb
    rospy.init_node('ggcnn_draw_rectangle')
    # img_rgb = rospy.Subscriber('/camera/color/image_raw', Image, convert_msg_to_cv, queue_size=1)
    depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, depth_callback, queue_size=1)
    ggcnn_draw_pub = rospy.Publisher('ggcnn/draw/grasp_rectangle', Image, queue_size=1)
    while not rospy.is_shutdown():
        rospy.spin()
