import sys
import numpy as np
import json
import argparse
import torch
import cv2
import pyzed.sl as sl
from ultralytics import YOLO

from threading import Lock, Thread
from time import sleep


def initialize_camera_params(zed, input_type):
    init_params = sl.InitParameters(
        input_t=input_type, svo_real_time_mode=True
    )  # input vs input_t??
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_maximum_distance = 50

    runtime_params = sl.RuntimeParameters()
    status = zed.open(init_params)

    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open camera: {repr(status)}")
        exit()

    image_left_tmp = sl.Mat()  # not needed?

    positional_tracking_parameters = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(positional_tracking_parameters)

    obj_param = sl.ObjectDetectionParameters()
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_param.enable_tracking = True
    obj_param.enable_segmentation = (
        False  # designed to give person pixel mask with internal OD
    )
    zed.enable_object_detection(obj_param)

    # return runtime_params


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--weights", type=str, default="yolov8m.pt", help="Path to YOLO model weights."
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
    initialize_camera_params(zed, input_type)

    # Load YOLO model
    model = YOLO(args.weights)

    svo_image = sl.Mat()
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 40

    # Video Writer setup
    image_size = zed.get_camera_information().camera_configuration.resolution
    width, height = image_size.width, image_size.height
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(args.output, fourcc, 30.0, (width, height))

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
        detections = results[0].boxes  # .cpu().numpy()?

        objects_in = []
        for box in detections:
            tmp = sl.CustomBoxObjectData()
            tmp.unique_object_id = sl.generate_unique_id()
            tmp.probability = box.conf
            tmp.label = int(box.cls)
            tmp.bounding_box_2d = box.xyxy.cpu().numpy()
            tmp.is_grounded = True
            objects_in.append(tmp)

        # objects_in = []
        # for box in detections:
        #     tmp = sl.CustomBoxObjectData()
        #     tmp.unique_object_id = sl.generate_unique_id()
        #     tmp.probability = box.conf
        #     tmp.label = int(box.cls)
        #     tmp.bounding_box_2d = box.bounding_box
        #     tmp.is_grounded = (
        #         True  # objects are moving on the floor plane and tracked in 2D only
        #     )
        #     objects_in.append(tmp)

        # Ingest custom 3D objects into ZED SDK for tracking
        zed.ingest_custom_box_objects(objects_in)

        # Retrieve 3D bounding boxes from ZED
        objects = sl.Objects()
        zed.retrieve_objects(objects, obj_runtime_param)

        if objects.object_list:
            first_object = objects.object_list[0]
            print(f"Object ID: {first_object.id}, Position: {first_object.position}")

            # Draw 3D bounding box
            for obj in objects.object_list:
                bbox = obj.bounding_box
                if bbox:
                    for i in range(4):
                        start = (int(bbox[i][0]), int(bbox[i][1]))
                        end = (int(bbox[(i + 1) % 4][0]), int(bbox[(i + 1) % 4][1]))
                        cv2.line(img, start, end, (0, 255, 0), 2)
                    for i in range(4, 8):
                        start = (int(bbox[i][0]), int(bbox[i][1]))
                        end = (
                            int(bbox[(i + 1) % 4 + 4][0]),
                            int(bbox[(i + 1) % 4 + 4][1]),
                        )
                        cv2.line(img, start, end, (0, 255, 0), 2)
                    for i in range(4):
                        cv2.line(
                            img,
                            (int(bbox[i][0]), int(bbox[i][1])),
                            (int(bbox[i + 4][0]), int(bbox[i + 4][1])),
                            (0, 255, 0),
                            2,
                        )

        out.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

    out.release()
    zed.close()


if __name__ == "__main__":
    main()
