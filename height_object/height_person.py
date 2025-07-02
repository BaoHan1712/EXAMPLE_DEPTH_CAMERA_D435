import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

W = 848
H = 480

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

pipeline.start(config)

aligned_stream = rs.align(rs.stream.color)
point_cloud = rs.pointcloud()

print("[INFO] loading model...")
model = YOLO("model\yolo11n.pt") 

# Bộ lọc Kalman 
class SimpleKalman:
    def __init__(self, init_x=0, init_p=1.0, Q=0.018, R=0.642):
        self.x = init_x
        self.p = init_p
        self.Q = Q
        self.R = R

    def update(self, measurement):
        x_pred = self.x
        p_pred = self.p + self.Q

        K = p_pred / (p_pred + self.R)

        self.x = x_pred + K * (measurement - x_pred)
        self.p = (1 - K) * p_pred

        return self.x
height_kalman = SimpleKalman()

def compensate_height(measured_height_cm, scale=1.0, offset=0.0):
    """
    Hàm bù trừ chiều cao đo được.
    - measured_height_cm: chiều cao đã đo (cm).
    - scale: hệ số tỉ lệ (ví dụ 1.05 nếu muốn cộng 5%).
    - offset: cộng thêm hoặc trừ đi trực tiếp (cm).
    """
    compensated = measured_height_cm * scale + offset
    return compensated


while True:
    frames = pipeline.wait_for_frames()
    frames = aligned_stream.process(frames)
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    points = point_cloud.calculate(depth_frame)
    verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, W, 3)

    color_image = np.asanyarray(color_frame.get_data())

    results = model(color_image, verbose=False)

    for result in results:
        boxes = result.boxes
        for box in boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].cpu().numpy()

            if conf > 0.4:  
                left = int(xyxy[0])
                top = int(xyxy[1])
                right = int(xyxy[2])
                bottom = int(xyxy[3])

                width = right - left
                height = bottom - top

                bbox = (left, top, width, height)
                p1 = (bbox[0], bbox[1])
                p2 = (bbox[0] + bbox[2], bbox[1] + bbox[3])
                cv2.rectangle(color_image, p1, p2, (255, 0, 0), 2)

                # Lấy point cloud trong bbox
                obj_points = verts[top:bottom, left:right].reshape(-1, 3)
                zs = obj_points[:, 2]
                z = np.median(zs)

                ys = obj_points[:, 1]
                ys = np.delete(ys, np.where((zs < z - 1) | (zs > z + 1)))  

                my = np.amin(ys, initial=1)
                My = np.amax(ys, initial=-1)

                height_m = My - my
                height_cm = float("{:.1f}".format(height_m * 100))
               
                filtered_height_cm = height_kalman.update(height_cm)
                compensated_height_cm = compensate_height(filtered_height_cm, scale=1, offset=-1)
                height_txt = str(round(compensated_height_cm)) + " [cm]"

                cv2.putText(color_image, height_txt,
                            (p1[0], p1[1] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (0, 0, 255),
                            2)

    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', color_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
