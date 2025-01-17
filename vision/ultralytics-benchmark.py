import cv2

from ultralytics import YOLO

# Load the YOLO11 model
model = YOLO("trained-models/01-15-25 no_pushbar yolov11 model 985_augmented.pt")

# Open the video file
video_path = "data/9-21-whobaat1.avi"
cap = cv2.VideoCapture(video_path)

subsample = 5

i = 0
# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()
    i += 1
    if (i % subsample != 0):
        continue
    if success:
        # Run YOLO11 tracking on the frame, persisting tracks between frames
        results = model.track(frame, persist=True)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Display the annotated frame
        cv2.imshow("YOLO11 Tracking", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()