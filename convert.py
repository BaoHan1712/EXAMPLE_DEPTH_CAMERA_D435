from ultralytics import YOLO

# Load a pretrained YOLO11n model
model = YOLO(r"yolo11n.pt")

# model.export(format="engine",half = True, simplify=True, imgsz=640)

# # # # # Run inference on 'bus.jpg' with arguments
model.predict(source=0, imgsz= 640,conf = 0.7, show = True, verbose = False)


