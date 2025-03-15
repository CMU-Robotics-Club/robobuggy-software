# Release resources
cap.release()
out.release()
cv2.destroyAllWindows()
# Create a ZED camera object
zed = sl.Camera()

# Set SVO path for playback
input_path = sys.argv[1]
init_parameters = sl.InitParameters()
init_parameters.set_from_svo_file(input_path)

# Open the ZED
err = zed.open(init_parameters)
init_parameters.set_from_svo_file(input_path)

# Open the ZED
zed = sl.Camera()
err = zed.open(init_parameters)

width = 1280
height = 480
fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # Or other codec
out = cv2.VideoWriter("test_output_video.mp4", fourcc, 30, (width, height))

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

svo_image = sl.Mat()
while True:
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        # Read side by side frames stored in the SVO
        zed.retrieve_image(svo_image, sl.VIEW.SIDE_BY_SIDE)

        # Use get_data() to get the numpy array
        image_ocv = svo_image.get_data()
        # Display the left image from the numpy array
        # cv2.imshow("Image", image_ocv)
        out.write(image_ocv)

        # Press Q on keyboard to exit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    elif zed.grab() == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
        print("SVO end has been reached. Looping back to first frame")
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        break
    else:
        break
