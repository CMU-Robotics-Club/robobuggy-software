from ultralytics import YOLO

# Load a model
model = YOLO("yolo11n.pt")  # load a pretrained model (recommended for training)

# Train the model with MPS
results = model.train(data="/Users/mehulgoel/Documents/robobuggy-software/vision/data/Buggy Vision.v1i.yolov11/data.yaml", epochs=100, imgsz=640, device="mps")
print(results)