import pyrealsense2 as rs
import time
import math
import cv2
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)
pipeline.start(config)

current_angle_x = 0.0  # pitch
current_angle_y = 0.0  # roll

prev_time = time.time()

accel_history = []
HISTORY_SIZE = 10
ACCEL_DELTA_THRESHOLD = 0.031  # Ngưỡng thay đổi lớn (m/s^2)

print("Khởi động, set góc = 0 độ. Nhấn Ctrl+C để dừng.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        gyro_frame = frames.first(rs.stream.gyro)
        accel_frame = frames.first(rs.stream.accel)

        if not color_frame or not gyro_frame or not accel_frame:
            continue

        # Lấy data gyro & accel
        gyro_data = gyro_frame.as_motion_frame().get_motion_data()
        accel_data = accel_frame.as_motion_frame().get_motion_data()

        # Tính độ lớn vector gia tốc
        accel_magnitude = math.sqrt(accel_data.x**2 + accel_data.y**2 + accel_data.z**2)


        accel_history.append(accel_magnitude)
        if len(accel_history) > HISTORY_SIZE:
            accel_history.pop(0)

        # Kiểm tra sự thay đổi
        if len(accel_history) == HISTORY_SIZE:
            accel_delta = max(accel_history) - min(accel_history)
            is_moving = accel_delta > ACCEL_DELTA_THRESHOLD
        else:
            is_moving = False

        # Tính delta t
        now = time.time()
        dt = now - prev_time
        prev_time = now

        # Chỉ tích phân khi phát hiện chuyển động mạnh
        if is_moving:
            current_angle_x += gyro_data.x * dt
            current_angle_y += gyro_data.y * dt

        # Chuyển sang độ
        angle_pitch = math.degrees(current_angle_x)
        angle_roll = math.degrees(current_angle_y)
        color_image = cv2.cvtColor(np.asanyarray(color_frame.get_data()), cv2.COLOR_BGR2RGB)
        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

        # Text overlay
        if len(accel_history) == HISTORY_SIZE:
            text = f"Pitch: {angle_pitch:.2f} | Roll: {angle_roll:.2f}"
            print(f"Pitch: {angle_pitch:.2f}° | Roll: {angle_roll:.2f}°|Đang di chuyển: {is_moving} | Δa: {accel_delta:.3f} m/s²")
        else:
            text = "Dang khoi tao..."

        cv2.putText(color_image, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

        cv2.imshow("RealSense Color + IMU", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nĐã dừng.")
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
