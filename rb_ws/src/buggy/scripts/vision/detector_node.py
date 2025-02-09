import rclpy
from rclpy.node import Node
from sensor_msgs import Image
from std_msgs import Int32
from nav_msgs import Odometry
from zed_msgs import Object
import pyzed.sl as sl
from ultralytics import YOLO
import cv2
import numpy as np
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation



class Detector(Node):

    def __init__(self):
        super().__init__('detector')

        self.cam = sl.Camera()
        self.initialize_camera()
        self.raw_image = sl.Mat()
        self.objects = sl.Objects()
        self.model = YOLO("model")

        self.runtime_params = sl.RuntimeParameters()
        self.object_det_params = sl.ObjectDetectionRuntimeParameters()

        self.bridge = CvBridge()

        # Subscribers:
        # TODO: need to subscribe to get SC position

        # Publishers
        self.observed_nand_odom_publisher = self.create_publisher(
                    Odometry, "/NAND_raw_state", 1
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

        timer_period = 0.01  # seconds (100 Hz)
        self.timer = self.create_timer(timer_period, self.loop)

    def initialize_camera(self):
        init_params = sl.InitParameters(svo_real_time_mode=True)
        positional_tracking_params = sl.PositionalTrackingParameters()
        obj_params = sl.ObjectDetectionParameters()


        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_maximum_distance = 50

        obj_params.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_params.enable_tracking = True
        obj_params.enable_segmentation = False  # designed to give person pixel mask

        # TODO: is exiting a node this way safe? will it interfere with other operation? (might need to test by trying to run node without camera plugged in)
        status = self.cam.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open", status, "Exit program.")
            exit(1)

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

    def convert_to_utm(self):

        # TODO: MOVE TO CONSTANTS FILE
        CAMERA_OFFSET = 0.6  # Distance from INS to camera in meters

        # TODO: get detection_pos from self.objects
        # TODO: get buggy_pos and buggy_pitch from the subscriber (you probably need to add a listener to regularly update a self.self_state variable)
        self.objects

        buggy_pos = None
        buggy_pitch = None()
        detection_pos = None()

        rot = Rotation.from_euler('xyz', [0, -buggy_pitch, buggy_pos.heading])
        vec = rot.apply(np.array([-detection_pos.z + CAMERA_OFFSET, -detection_pos.x, detection_pos.y]))

        return buggy_pos.pos + vec

    def loop(self):
        raw_frame_publish = None
        annotated_frame_publish = None
        num_detections = 0
        nand_utm = None

        # Loop for the code that operates every 10ms
        # get a new frame from camera and get objects in that frame
        if self.cam.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.cam.retrieve_image(self.raw_image, sl.VIEW.LEFT)
            image_net = self.raw_image.get_data()

            self.raw_image = cv2.cvtColor(image_net, cv2.COLOR_RGBA2RGB)
            raw_frame_publish = self.bridge.cv2_to_imgmsg(image_net, encoding="rgb8")

            # pass frame into YOLO model (get 2D)
            detections = self.model.predict(self.raw_image, save=False)
            custom_boxes = self.detections_to_custom_box(detections, image_net)

            # pass into 2D to 3D to get approximate depth
            self.cam.ingest_custom_box_objects(custom_boxes)
            self.cam.retrieve_objects(self.objects, self.object_det_params)

        num_detections = len(self.objects.object_list)
        if num_detections > 0:
            # TODO: this function needs to be written!
            utms = self.objects_to_utm()
            # NOTE: we're currently defining NAND to just be the first bounding box, we might change how we figure out what NAND is if there are multiple detections
            nand_utm = utms[0]

        self.raw_camera_frame_publisher.publish(raw_frame_publish)
        self.annotated_camera_frame_publisher.publish(annotated_frame_publish)
        self.num_detections_publisher.publish(num_detections)
        self.observed_nand_odom_publisher.publish(nand_utm)


def main(args=None):
    rclpy.init(args=args)
    detector_node = Detector()
    rclpy.spin(detector_node)
    detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()