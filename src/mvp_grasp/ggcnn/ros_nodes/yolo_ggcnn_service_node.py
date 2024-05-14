#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function

import rospy

import time

import numpy as np
import cv2
import time

from tf import transformations as tft
from dougsm_helpers.timeit import TimeIt

from ggcnn.ggcnn_torch import predict, process_depth_image
from dougsm_helpers.gridshow import gridshow
# from dougsm_helpers.tf_helpers_node import tf_helpers
from ggcnn.srv import GraspPrediction, GraspPredictionResponse
from sensor_msgs.msg import Image, CameraInfo
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes

import cv_bridge
bridge = cv_bridge.CvBridge()

TimeIt.print_output = False



class GGCNNService:
    def __init__(self):
        # Get the camera parameters
        cam_info_topic = rospy.get_param('~camera/info_topic')
        self.yolo_topic = '/yolov5/BoundingBoxes'
        camera_info_msg = rospy.wait_for_message(cam_info_topic, CameraInfo)
        self.cam_K = np.array(camera_info_msg.K).reshape((3, 3))

        self.img_pub = rospy.Publisher('~visualisation', Image, queue_size=1)
        rospy.Service('~predict', GraspPrediction, self.compute_service_handler)

        self.base_frame = rospy.get_param('~camera/robot_base_frame')
        self.camera_frame = rospy.get_param('~camera/camera_frame')
        self.img_crop_size = rospy.get_param('~camera/crop_size')
        self.img_crop_y_offset = rospy.get_param('~camera/crop_y_offset')
        self.cam_fov = rospy.get_param('~camera/fov')

        self.counter = 0
        self.curr_depth_img = None
        self.curr_img_time = 0
        self.last_image_pose = None
        rospy.Subscriber(rospy.get_param('~camera/depth_topic'), Image, self._depth_img_callback, queue_size=1)

        self.waiting = False
        self.received = False

    def _depth_img_callback(self, msg):
        # Doing a rospy.wait_for_message is super slow, compared to just subscribing and keeping the newest one.
        if not self.waiting:
          return
        self.curr_img_time = time.time()
        self.last_image_pose = tfh.current_robot_pose(self.base_frame, self.camera_frame)
        self.curr_depth_img = bridge.imgmsg_to_cv2(msg)
        self.received = True
        print("depth_img_callback is ok")

    def yolo_callback(self, boxes):
        rospy.loginfo("yolo callback")
        objects = []
        boxs = boxes.bounding_boxes
        for box in boxs:
            objects.append(box.Class)
        print("Detected: ", objects)
        grasp_object = str(input("Please enter object: "))
        print(grasp_object)
        print(type(grasp_object))
        for box in boxs:
            if box.Class == grasp_object:
                xmin = box.xmin
                ymin = box.ymin
                xmax = box.xmax
                ymax = box.ymax
                break
            else:
                continue
        return xmin, ymin, xmax, ymax
    
    def boxInter(self, x1, y1, x2, y2, crop_size=300, crop_y_offset=40):
        imw1 = 640
        imh1 = 480
        print("x1, y1, x2, y2: ", x1, y1, x2, y2)
        x3 = (imw1 - crop_size) // 2
        y3 = (imh1 - crop_size) // 2 - crop_y_offset
        x4 = (imw1 - crop_size) // 2 + crop_size
        y4 = (imh1 - crop_size) // 2 + crop_size - crop_y_offset
        print("x3, y3, x4, y4: ", x3, y3, x4, y4)

        xinter_min = max(x1, x3)
        yinter_min = max(y1, y3)
        xinter_max = min(x2, x4)
        yinter_max = min(y2, y4)
        print(xinter_min, yinter_min, xinter_max, yinter_max)

        inter_area = max(0, xinter_max - xinter_min) * max(0, yinter_max - yinter_min)
        if inter_area > 0:
            return xinter_min - x3, yinter_min - y3, xinter_max - x3, yinter_max - y3
        else:
            raise ValueError("No matched object")
        

    def compute_service_handler(self, req):
        # if self.curr_depth_img is None:
        #     rospy.logerr('No depth image received yet.')
        #     rospy.sleep(0.5)

        # if time.time() - self.curr_img_time > 0.5:
        #     rospy.logerr('The Realsense node has died')
        #     return GraspPredictionResponse()
        print("ggcnn_service_node.py compute_service_handler start")
        self.waiting = True
        while not self.received:
          rospy.sleep(0.01)
        self.waiting = False
        self.received = False
        with TimeIt('Total'):
            depth = self.curr_depth_img.copy()
            camera_pose = self.last_image_pose
            cam_p = camera_pose.position

            camera_rot = tft.quaternion_matrix(tfh.quaternion_to_list(camera_pose.orientation))[0:3, 0:3]

            # Do grasp prediction
            depth_crop, depth_nan_mask = process_depth_image(depth, self.img_crop_size, 300, return_mask=True, crop_y_offset=self.img_crop_y_offset)
            print("ggcnn_test.py process_depth_image done")
            points, angle, width_img, _ = predict(depth_crop, process_depth=False, depth_nan_mask=depth_nan_mask, filters=(2.0, 2.0, 2.0))

            # Mask Points Here
            angle -= np.arcsin(camera_rot[0, 1])  # Correct for the rotation of the camera
            print('camera_rot[0, 1]: ', np.arcsin(camera_rot[0, 1]))
            angle = (angle + np.pi/2) % np.pi - np.pi/2  # Wrap [-np.pi/2, np.pi/2]

            # yolo
            boundingboxes = rospy.wait_for_message(self.yolo_topic, BoundingBoxes)
            xmin, ymin, xmax, ymax = self.yolo_callback(boundingboxes)

            print("xmin, ymin, xmax, ymax: ", xmin, ymin, xmax, ymax)

            xInterMin, yInterMin, xInterMax, yInterMax = self.boxInter(xmin, ymin, xmax, ymax)
            print("Detected box: ", "xmin ", xInterMin, "ymin ", yInterMin, "xmax ", xInterMax, "ymax ", yInterMax)


            # Convert to 3D positions.
            imh, imw = depth.shape
            x = ((np.vstack((np.linspace((imw - self.img_crop_size) // 2, (imw - self.img_crop_size) // 2 + self.img_crop_size, depth_crop.shape[1], np.float), )*depth_crop.shape[0]) - self.cam_K[0, 2])/self.cam_K[0, 0] * depth_crop).flatten()
            y = ((np.vstack((np.linspace((imh - self.img_crop_size) // 2 - self.img_crop_y_offset, (imh - self.img_crop_size) // 2 + self.img_crop_size - self.img_crop_y_offset, depth_crop.shape[0], np.float), )*depth_crop.shape[1]).T - self.cam_K[1,2])/self.cam_K[1, 1] * depth_crop).flatten()
            pos = np.dot(camera_rot, np.stack((x, y, depth_crop.flatten()))).T + np.array([[cam_p.x, cam_p.y, cam_p.z]])

            width_m = width_img / 300.0 * 2.0 * depth_crop * np.tan(self.cam_fov * self.img_crop_size/depth.shape[0] / 2.0 / 180.0 * np.pi)

            # 全局最大抓取质量分数
            # best_g = np.argmax(points)
            # best_g_unr = np.unravel_index(best_g, points.shape)

            # 局部最大抓取质量分数
            best_g_detect = np.argmax(points[yInterMin:yInterMax, xInterMin:xInterMax])
            print("检测框内的最大抓取质量分数坐标: ", best_g_detect)
            best_g_unr = np.unravel_index(best_g_detect, points[yInterMin:yInterMax, xInterMin:xInterMax].shape) 
            tuple_num = (yInterMin, xInterMin)
            best_g_unr = tuple(x + y for x, y in zip(best_g_unr, tuple_num))

            best_g = best_g_unr[0] * self.img_crop_size + best_g_unr[1]



            print("best_g: ", best_g)
            print("best_g type: ", type(best_g))
            print("best_g_unr: ", best_g_unr)
            print("best_g_unr type: ", type(best_g_unr))
        
            ret = GraspPredictionResponse()
            ret.success = True
            g = ret.best_grasp
            g.pose.position.x = pos[best_g, 0]
            g.pose.position.y = pos[best_g, 1]
            print("position.z")
            print(g.pose.position.z)
            g.pose.position.z = pos[best_g, 2]

            output_angle = angle[best_g_unr]
            run_angle = (angle[best_g_unr] % np.pi) - np.pi/2
            print('ggcnn output angle: ', output_angle)
            print('panda run angle: ', run_angle)
            g.pose.orientation = tfh.list_to_quaternion(tft.quaternion_from_euler(np.pi, 0, ((angle[best_g_unr] % np.pi) - 3 * np.pi/4)))
            g.width = width_m[best_g_unr]
            g.quality = points[best_g_unr]

            show = gridshow('Display',
                     [depth_crop, points],
                     [(0.30, 0.55), None, (-np.pi/2, np.pi/2)],
                     [cv2.COLORMAP_BONE, cv2.COLORMAP_JET, cv2.COLORMAP_BONE],
                     3,
                     False)
            
            save_path = "/home/gh/" + str(time.time()) + ".png"
            cv2.imwrite(save_path, show)
            self.img_pub.publish(bridge.cv2_to_imgmsg(show))
            print("publish the dealed image to /ggcnn_service/visualisation")
            print(ret.best_grasp)
            return ret
    

if __name__ == '__main__':
    # print("rospy.get_param('~camera/depth_topic')")
    rospy.init_node('ggcnn_service')
    import dougsm_helpers.tf_helpers as tfh
    GGCNN = GGCNNService()
    rospy.spin()
