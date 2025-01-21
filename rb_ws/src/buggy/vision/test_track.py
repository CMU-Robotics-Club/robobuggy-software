import cv2
from ultralytics import YOLO

# Load the YOLO11 model
model = YOLO("yolo11n.pt")

# Open the video file
video_path = "robobuggy test detect.mp4"
cap = cv2.VideoCapture(video_path)
i = 0

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
  success, frame = cap.read()
  if i % 800 == 0:
    if success:
        # Run YOLO11 tracking on the frame, persisting tracks between frames
        results = model.track(frame, persist=True, classes=[0])

        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        # try:
        #   index = results[0].boxes.id.tolist().index(64.) 
        #   print(results[0].boxes.xyxy[index])
        # except:
        #   print("None found")
      
        print(results[0].boxes.xyxy)

        # Display the annotated frame
        cv2.imshow("YOLO11 Tracking", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break
  i += 1

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()