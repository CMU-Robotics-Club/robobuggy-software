from ultralytics import YOLO

# Load a model
model = YOLO("yolo11n.pt")  # load a pretrained model (recommended for training)
yaml_path = "C:\\Users\\Mehul Goel\\Documents\\robobuggy-software\\vision\\data\\Buggy Vision.v1i.yolov11\\data.yaml"


# Train the model with MPS
results = model.train(data=yaml_path, epochs=100, imgsz=640)
print(results)