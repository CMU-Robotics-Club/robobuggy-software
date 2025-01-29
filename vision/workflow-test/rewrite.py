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
    init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_maximum_distance = 50

    runtime_params = sl.RuntimeParameters()
    status = zed.open(init_params)

    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    image_left_tmp = sl.Mat()


    positional_tracking_parameters = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(positional_tracking_parameters)

    obj_param = sl.ObjectDetectionParameters()
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_param.enable_tracking = True
    obj_param.enable_segmentation = False  # designed to give person pixel mask with internal OD
    zed.enable_object_detection(obj_param)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--weights", type=str, default="yolov11n.pt", help="model.pt path(s)"
    )
    parser.add_argument('--svo', type=str, default=None, help=' svo file')

    args = parser.parse_args()
    zed = sl.Camera()
    input_type = sl.InputType()
    input_type.set_from_svo_file(args.svo)

    # values i stole from sample code
    conf = 0.2
    iou = 0.45

    initialize_camera_params(zed, input_type)

    model = YOLO(args.weights)

    detections = []
    svo_image = sl.Mat()
    while zed.grab() == sl.ERROR_CODE.SUCCESS:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Read side by side frames stored in the SVO
            zed.retrieve_image(svo_image, sl.VIEW.LEFT)
            image_net = svo_image.get_data()
            img = cv2.cvtColor(image_net, cv2.COLOR_RGBA2RGB)
            det = model.predict(img, save=False, conf=conf, iou=iou)[0].cpu().numpy().boxes
            detections.append(det)

    objects_in = []
    # The "detections" variable contains your custom 2D detections
    for it in detections:
        tmp = sl.CustomBoxObjectData()
        # Fill the detections into the correct SDK format
        tmp.unique_object_id = sl.generate_unique_id()
        tmp.probability = it.conf
        tmp.label = int(it.class_id)
        tmp.bounding_box_2d = it.bounding_box
        tmp.is_grounded = True # objects are moving on the floor plane and tracked in 2D only
        objects_in.append(tmp)
    zed.ingest_custom_box_objects(objects_in)

    objects = sl.Objects() # Structure containing all the detected objects

    zed.retrieve_objects(objects, obj_runtime_param) # Retrieve the 3D tracked objects

    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 40

    for object in objects.object_list:
        print("{} {}".format(object.id, object.position))

    object_id = object.id # Get the object id
    object_label = object.raw_label; # Get the label
    object_position = object.position # Get the object position
    object_velocity = object.velocity # Get the object velocity
    object_tracking_state = object.tracking_state # Get the tracking state of the object
    if object_tracking_state == sl.OBJECT_TRACKING_STATE.OK:
        print("Object {0} is tracked\n".format(object_id))


if __name__ == '__main__':
    main()
