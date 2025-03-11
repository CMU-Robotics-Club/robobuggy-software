#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
# from zed_msgs import Object
import pyzed.sl as sl
from ultralytics import YOLO
import cv2
import numpy as np
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation


class Detector(Node):

    def __init__(self):
        super().__init__('detector')
        self.get_logger().info("INITIALIZED.")

        self.SC_pose = None  # will hold msg.pose.pose of SC/self/state

        self.cam = sl.Camera()
        self.initialize_camera()
        self.raw_image = sl.Mat()
        self.objects = sl.Objects()
        self.model = YOLO("src/buggy/scripts/vision/01-15-25_no_pushbar_yolov11n.pt")

        self.runtime_params = sl.RuntimeParameters()
        self.object_det_params = sl.ObjectDetectionRuntimeParameters()

        self.bridge = CvBridge()

        # Subscribers
        self.SC_state_subscriber = self.create_subscription(
            Odometry, "self/state", self.set_SC_state, 1
        )

        # Publishers
        self.observed_NAND_odom_publisher = self.create_publisher(
            Odometry, "other/state", 1
        )
        self.raw_camera_frame_publisher = self.create_publisher(
                    Image, "debug/raw_camera_frame", 1
                )
        self.annotated_camera_frame_publisher = self.create_publisher(
                    Image, "debug/annotated_camera_frame", 1
                )
        self.num_detections_publisher = self.create_publisher(
                    Int32, "debug/num_detections", 1
                )

        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.loop)

    def set_SC_state(self, msg):
        # TODO: does this need locking to prevent conflict with object detection? --> probably not, but needs to be tested
        self.SC_pose = msg.pose.pose
        self.get_logger().debug("SC state received: " + str(self.SC_pose.position))

    def initialize_camera(self):
        init_params = sl.InitParameters(svo_real_time_mode=True)
        positional_tracking_params = sl.PositionalTrackingParameters()
        obj_params = sl.ObjectDetectionParameters()

        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_maximum_distance = 50

        # testing with a sample SVO file
        # TODO: comment out when running
        # input_path = "../vision/workflow-test/sc-purnell-pass-1.svo2"
        # init_params.set_from_svo_file(input_path)

        obj_params.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_params.enable_tracking = True
        obj_params.enable_segmentation = False  # designed to give person pixel mask

        # TODO: is exiting a node this way safe? will it interfere with other operation? (might need to test by trying to run node without camera plugged in)
        # --> changed from exit(1) to destroy_node(), still needs to be tested
        status = self.cam.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("Camera Open", status, "Exiting program.")
            self.destroy_node()
            rclpy.shutdown()

        self.cam.enable_positional_tracking(positional_tracking_params)
        self.cam.enable_object_detection(obj_params)

    def detections_to_custom_box(self, detections, im0):
        def xywh2abcd(xywh, im_shape):
            output = np.zeros((4, 2))

            # Center / Width / Height -> BBox corners coordinates
            x_min = (xywh[0] - 0.5*xywh[2]) #* im_shape[1]
            x_max = (xywh[0] + 0.5*xywh[2]) #* im_shape[1]
            y_min = (xywh[1] - 0.5*xywh[3]) #* im_shape[0]
            y_max = (xywh[1] + 0.5*xywh[3]) #* im_shape[0]

            # A ------ B
            # | Object |
            # D ------ C

            output[0][0] = x_min
            output[0][1] = y_min

            output[1][0] = x_max
            output[1][1] = y_min

            output[2][0] = x_max
            output[2][1] = y_max

            output[3][0] = x_min
            output[3][1] = y_max
            return output

        output = []
        for i, det in enumerate(detections):
            xywh = det.xywh[0]

            # Creating ingestable objects for the ZED SDK
            obj = sl.CustomBoxObjectData()
            obj.bounding_box_2d = xywh2abcd(xywh, im0.shape)
            obj.label = int(det.cls.item())
            obj.probability = det.conf.item()
            obj.is_grounded = False
            output.append(obj)
        return output

    def objects_to_utm(self):

        # TODO: MOVE TO CONSTANTS FILE
        CAMERA_OFFSET = 0.6  # Distance from INS to camera in meters

        buggy_position = self.SC_pose.position
        buggy_orientation = self.SC_pose.orientation

        # TODO: conversion to UTM needs to be tested
        utms = []
        for obj in self.objects.object_list:
            detection_position = obj.position

            rot = Rotation.from_euler(
                "xyz", [buggy_orientation.x, buggy_orientation.y, buggy_orientation.z]
            )
            vec = rot.apply(
                np.array(
                    [
                        detection_position[0] + CAMERA_OFFSET,
                        detection_position[1],
                        detection_position[2],
                    ]
                )
            )

            utm_position = (
                np.array([buggy_position.x, buggy_position.y, buggy_position.z]) + vec
            )

            utms.append(utm_position)

        return utms

    def loop(self):
        # raw_frame_publish = None
        annotated_frame_publish = None
        num_detections = 0
        NAND_utm = None

        # Loop for the code that operates every 10ms
        # get a new frame from camera and get objects in that frame
        if self.cam.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.cam.retrieve_image(self.raw_image, sl.VIEW.LEFT)
            image_net = self.raw_image.get_data()

            # get raw frame
            raw_image_np = cv2.cvtColor(image_net, cv2.COLOR_BGRA2RGB)
            # raw_frame_publish = self.bridge.cv2_to_imgmsg(raw_image_np, encoding="rgb8")

            # pass frame into YOLO model (get 2D)
            detections = self.model.predict(raw_image_np, save=False)
            detection_boxes = (
                detections[0].cpu().numpy().boxes
            )  # what is the [0] indexing into, does this pull out the first detection?
            custom_boxes = self.detections_to_custom_box(detection_boxes, image_net)

            # publish annotated frame
            annotated_frame_publish = self.bridge.cv2_to_imgmsg(detections[0].plot(), encoding="rgb8")

            # pass into 2D to 3D to get approximate depth
            self.cam.ingest_custom_box_objects(custom_boxes)
            self.cam.retrieve_objects(self.objects, self.object_det_params)

            # sort object_list by confidence
            self.objects.object_list.sort(key=lambda obj: obj.confidence, reverse=True)

            num_detections = len(self.objects.object_list)
            NAND_pose = None
            if num_detections > 0:
                utms = self.objects_to_utm()
                # NOTE: we're currently defining NAND to just be the first bounding box, we might change how we figure out what NAND is if there are multiple detections
                NAND_pose = Odometry()
                NAND_utm = utms[0]
                NAND_pose.pose.pose.position.x = NAND_utm[0]
                NAND_pose.pose.pose.position.y = NAND_utm[1]
                NAND_pose.pose.pose.position.z = NAND_utm[2]

            # self.raw_camera_frame_publisher.publish(raw_frame_publish)
            self.num_detections_publisher.publish(Int32(data=num_detections))
            if annotated_frame_publish is not None:  # should always be true
                self.annotated_camera_frame_publisher.publish(annotated_frame_publish)
            if NAND_pose is not None:
                self.observed_NAND_odom_publisher.publish(NAND_pose)

        else:
            self.get_logger().error("Zed camera frame grab failed.")

def main(args=None):

    rclpy.init(args=args)
    detector_node = Detector()

    rclpy.spin(detector_node)

    detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
