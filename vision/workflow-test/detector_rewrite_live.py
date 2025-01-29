import numpy as np
import argparse
import torch
import cv2
import pyzed.sl as sl
from ultralytics import YOLO

import ogl_viewer.viewer as gl
import cv_viewer.tracking_viewer as cv_viewer


def initialize_camera_params(zed, input_type):
    init_params = sl.InitParameters(
        input_t=input_type, svo_real_time_mode=True
    )  # input vs input_t??
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_maximum_distance = 50

    # runtime_params = sl.RuntimeParameters()
    status = zed.open(init_params)

    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open camera: {repr(status)}")
        exit()

    positional_tracking_parameters = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(positional_tracking_parameters)

    obj_param = sl.ObjectDetectionParameters()
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_param.enable_tracking = True
    obj_param.enable_segmentation = (
        False  # designed to give person pixel mask with internal OD
    )
    zed.enable_object_detection(obj_param)

    return obj_param


def xywh2abcd(xywh, im_shape):
    output = np.zeros((4, 2))

    # Center / Width / Height -> BBox corners coordinates
    x_min = xywh[0] - 0.5 * xywh[2]  # * im_shape[1]
    x_max = xywh[0] + 0.5 * xywh[2]  # * im_shape[1]
    y_min = xywh[1] - 0.5 * xywh[3]  # * im_shape[0]
    y_max = xywh[1] + 0.5 * xywh[3]  # * im_shape[0]

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--weights", type=str, default="yolov11n.pt", help="Path to YOLO model weights."
    )
    parser.add_argument("--svo", type=str, required=True, help="Path to the SVO file.")
    parser.add_argument(
        "--output",
        type=str,
        default="./output.mp4",
        help="Path to save the annotated video.",
    )
    args = parser.parse_args()

    # Initialize ZED camera
    zed = sl.Camera()
    input_type = sl.InputType()
    input_type.set_from_svo_file(args.svo)
    obj_param = initialize_camera_params(zed, input_type)

    # Load YOLO model
    model = YOLO(args.weights)

    svo_image = sl.Mat()
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 40

    camera_infos = zed.get_camera_information()
    camera_res = camera_infos.camera_configuration.resolution

    # Create OpenGL viewer
    viewer = gl.GLViewer()
    point_cloud_res = sl.Resolution(
        min(camera_res.width, 720), min(camera_res.height, 404)
    )
    point_cloud_render = sl.Mat()
    viewer.init(camera_infos.camera_model, point_cloud_res, obj_param.enable_tracking)
    point_cloud = sl.Mat(
        point_cloud_res.width, point_cloud_res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU
    )
    image_left = sl.Mat()
    
    # Camera pose
    cam_w_pose = sl.Pose()
    print("Initialized display settings")


    # Video Writer setup -- ignore
    # image_size = zed.get_camera_information().camera_configuration.resolution
    # width, height = image_size.width, image_size.height
    # fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    # out = cv2.VideoWriter(args.output, fourcc, 30.0, (width, height))

    # Detection parameters
    conf = 0.2
    iou = 0.45

    while zed.grab() == sl.ERROR_CODE.SUCCESS:
        # Retrieve the left RGB image and depth map
        zed.retrieve_image(svo_image, sl.VIEW.LEFT)
        image_net = svo_image.get_data()
        img = cv2.cvtColor(image_net, cv2.COLOR_RGBA2RGB)

        # YOLO detection
        results = model.predict(img, save=False, conf=conf, iou=iou)
        detections = results[0].cpu().numpy().boxes

        objects_in = []
        for box in detections:
            tmp = sl.CustomBoxObjectData()
            tmp.unique_object_id = sl.generate_unique_id()
            tmp.probability = box.conf.item()  # what if array is bigger than 1?
            tmp.label = int(box.cls.item())
            tmp.bounding_box_2d = xywh2abcd(box.xywh[0], image_net.shape)
            tmp.is_grounded = True
            objects_in.append(tmp)

        # Ingest custom 3D objects into ZED SDK for tracking
        zed.ingest_custom_box_objects(objects_in)

        # Retrieve 3D bounding boxes from ZED
        objects = sl.Objects()
        zed.retrieve_objects(objects, obj_runtime_param)

        if objects.object_list:
            first_object = objects.object_list[0]
            print(f"Object ID: {first_object.id}, Position: {first_object.position}")

    # out.close()
    zed.close()


if __name__ == "__main__":
    main()
