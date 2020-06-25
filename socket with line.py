import math
import socket
import sys
import cv2
import pickle
import numpy as np
import struct
import zlib
from imutils.video import VideoStream
from imutils.video import FPS
import imutils
import time
import pyscreenshot as ImageGrab

'''
Onemli
raspberry yi kablosuz bilgisayar ustunden internete baglamak icin yapilcaklar:
1. bilgisarin kablosuz paylasimini ac
2. ipconfig den alinan wireless lan adapter yerel ag baglantisi *2 den IPv4 adresini socket adresi olarak yaz
'''

HOST = '192.168.137.1'
PORT = 8000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Socket created')

s.bind((HOST, PORT))
print('Socket bind complete')
s.listen(10)
print('Socket now listening')

conn, addr = s.accept()

data = b""
payload_size = struct.calcsize(">L")


class Lane:

    def __init__(self):
        self.lower = np.array([255, 255, 255])
        self.upper = np.array([200, 200, 200])
        self.steering_angle = 90
        self.stabilized_steering_angle = 90
        self.stop_flag = False

    def drive(self, frame):
        lane_lines, lane_frame = self.detect_lane(frame)
        self.steering_angle, self.stop_flag = self.compute_steering_angle(lane_frame, lane_lines)
        self.stabilized_steering_angle = self.stabilize_steering_angle(self.stabilized_steering_angle,
                                                                       self.steering_angle,
                                                                       len(lane_lines))

        return self.stabilized_steering_angle, self.stop_flag

    def detect_lane(self, frame):
        edges = self.detect_edge(frame)
        edges = self.region_of_interest(edges)
        line_segments = self.detect_line_segments(edges)
        lane_lines = self.average_slope_intercept(frame, line_segments)
        lane_frame = self.display_lines(frame, lane_lines)
        return lane_lines, lane_frame

    def detect_edge(self, frame):
        edge_mask = cv2.inRange(frame, self.upper, self.lower)
        mask = cv2.bitwise_and(frame, frame, mask=edge_mask)
        edges = cv2.Canny(mask, 200, 400)
        return edges

    def region_of_interest(self, edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height * 1 / 2),  # sol ust
            (width, height * 1 / 2),  # sag ust
            (width, height),  # sag alt
            (0, height),  # sol alt
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_image = cv2.bitwise_and(edges, mask)
        return masked_image

    def detect_line_segments(self, edges):
        rho = 1  # precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # degree in radian, i.e. 1 degree
        min_threshold = 55  # minimal of votes
        line_segments = cv2.HoughLinesP(edges, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                        maxLineGap=4)
        # print("line segment", line_segments)
        return line_segments

    def average_slope_intercept(self, frame, line_segments):
        lane_lines = []

        if line_segments is None:
            return lane_lines
        height, width, _ = frame.shape
        left_fit = []
        right_fit = []
        boundary = 1 / 3
        left_region_boundary = width * 2 / 3
        right_region_boundary = width * 1 / 3
        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                # print(intercept)
                if slope < -0.05:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                        # print("yey1")
                elif slope > 0.05:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))
        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(self.make_points(frame, left_fit_average))
        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, right_fit_average))
        return lane_lines

    def compute_steering_angle(self, lane_frame, lane_lines):
        if len(lane_lines) == 0:
            self.stop_flag = True
            return 90, self.stop_flag

        height, width, _ = lane_frame.shape
        if len(lane_lines) == 1:
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
        else:
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]
            camera_mid_offset_percent = 0.02
            mid = int(width / 2 * (1 + camera_mid_offset_percent))
            x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
        angle_to_mid_radian = math.atan(x_offset / y_offset)
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
        steering_angle = angle_to_mid_deg + 90
        return steering_angle

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height
        y2 = int(y1 * 1 / 2)
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]

    def display_lines(self, frame, lane_lines, line_color=(0, 255, 0), line_width=10):
        line_image = np.zeros_like(frame)
        if lane_lines is not None:
            for line in lane_lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        lane_frame = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        cv2.imshow('masked', lane_frame)
        return lane_frame

    def stabilize_steering_angle(self, stabilized_steering_angle, steering_angle, num_of_lane_line):

        max_angle_deviation_two_lines = 5
        max_angle_deviation_one_lane = 1
        if num_of_lane_line == 2:
            max_angle_deviation = max_angle_deviation_two_lines
        else:
            max_angle_deviation = max_angle_deviation_one_lane
        angle_deviation = steering_angle - stabilized_steering_angle
        if abs(angle_deviation) > max_angle_deviation:
            stabilized_steering_angle = int(stabilized_steering_angle
                                            + max_angle_deviation * angle_deviation / abs(angle_deviation))
        else:
            stabilized_steering_angle = steering_angle
        return stabilized_steering_angle


class Object:
    def __init__(self):
        self.classes = ["background", "aeroplane", "bicycle", "bird", "boat",
                        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                        "sofa", "train", "tvmonitor"]
        self.relevant = ["background", "bicycle", "bus", "car", "dog", "motorbike", "person"]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
        self.net = cv2.dnn.readNetFromCaffe("MobileNetSSD_deploy.prototxt.txt", "MobileNetSSD_deploy.caffemodel")
        self.stop_flag = False

    def detect(self, frame):
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()
        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.5:
                idx = int(detections[0, 0, i, 1])
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                label = "{}: {:.2f}%".format(self.classes[idx], confidence * 100)
                if self.classes[idx] in self.relevant:
                    cv2.rectangle(frame, (startX, startY), (endX, endY), self.colors[idx], 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors[idx], 2)
                    if self.classes[idx] == "person" and confidence > 0.8:
                        self.stop_flag = True
                    else:
                        self.stop_flag = False
                # print(label, abs(startX - endX))
        return frame, self.stop_flag


class AutonomousCar:

    def __init__(self):
        self.speed = 0
        self.steering_angle = 90

    def drive(self, speed, steering_angle, obj_flag, lane_flag):
        if obj_flag or lane_flag:
            self.speed = 0
        else:
            if steering_angle in range(50, 70):
                self.speed = speed - 10
            elif steering_angle in range(70, 110):
                self.speed = speed + 1
            elif steering_angle in range(110, 130):
                self.speed = speed - 10
            else:
                self.speed = 0

            if self.speed < 25:
                self.speed = 25
            elif self.speed > 40:
                self.speed = 40
        self.steering_angle = steering_angle
        return self.speed, self.steering_angle


lane = Lane()
obj = Object()
car = AutonomousCar()
fps = FPS().start()

while True:
    while len(data) < payload_size:
        data += conn.recv(4096)
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack(">L", packed_msg_size)[0]
    while len(data) < msg_size:
        data += conn.recv(4096)
    frame_data = data[:msg_size]
    data = data[msg_size:]
    frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

    lane.drive(frame)
    obj.detect(frame)

    car.drive(speed=car.speed, steering_angle=lane.stabilized_steering_angle, obj_flag=obj.stop_flag,
              lane_flag=lane.stop_flag)
    stabilized = str(car.steering_angle)
    speed = str(car.speed)
    package = stabilized.encode() + str('/').encode() + speed.encode()
    conn.send(package)

    # cv2.imshow('ImageWindow', frame)
    # plt.imshow(frame)
    # plt.show()
    sys.stdout.write("\rSpeed: {speed:} / Steering Angle: {angle:}".format(speed=car.speed, angle=car.steering_angle))
    sys.stdout.flush()
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    fps.update()
fps.stop()
print("\n[INFO] Elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] Approx. FPS: {:.2f}".format(fps.fps()))
cv2.destroyAllWindows()
