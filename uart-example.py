import time
import serial
from ultralytics import YOLO
import supervision as sv
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

print("UART Demonstration Program")
print("NVIDIA Jetson Nano Developer Kit")


serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
# Wait a second to let the port initialize
time.sleep(1)


def calc_box_area(xyxy: list):
    return max(0, xyxy[2] - xyxy[0]) * max(0, xyxy[3] - xyxy[1])

# should return a frame, not sure what class it needs to be
def get_frame():
    frame = None
    # camera stuff
    return frame

def boxes_from_frame(frame):
    results = model(frame)
    detections = sv.Detections.from_ultralytics(results[0])
    detections = detections[detections.class_id == 0]
    
    boxes = []
    for detection in boxes:
        boxes.append(detection[0]) # detection[0] is the box coordinates [xmin, ymin, xmax, ymax]
    return boxes

# peoplexyxy is a list of lists with [xmin, ymin, xmax, ymax], predictedxyxy is the predicted [xmin, ymin, xmax, ymax]
def find_best_box(boxes: list, predictedbox: list):
    best_index = None
    best_overlap_rate = -1
    for i, box in enumerate(boxes):
        overlapxyxy = [None] * 4
        print(box)

        for j in range(4):
            if j < 2:
                func = max
            else:
                func = min
            overlapxyxy[j] = func(predictedbox[j], box[j])
        
        overlap_area = calc_box_area(overlapxyxy)
        total_area = calc_box_area(predictedbox) + calc_box_area(box) - overlap_area
        overlap_rate = overlap_area / total_area

        if overlap_rate > best_overlap_rate:
            best_overlap_rate = overlap_rate
            best_index = i
    
    return best_index

try:
    # Load the YOLOv8 model
    model = YOLO('yolov8n.pt')

    # initalize kalman filters
    kalmanfilters = []
    for i in range(4):
        f = KalmanFilter(dim_x=2, dim_z=1)
        f.x = np.array([2., 0.])
        f.F = np.array([[1.,1.],
                        [0.,1.]])
        f.H = np.array([[1.,0.]])
        f.P = np.array([[1000.,    0.],
                        [   0., 1000.]])
        f.R = 5
        f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)
        kalmanfilters.append(f)

    # Send a simple header
    serial_port.write("UART Demonstration Program\r\n".encode())
    serial_port.write("NVIDIA Jetson Nano Developer Kit\r\n".encode())
    while True:
        if serial_port.inWaiting() > 0:
            data = serial_port.read()
            print(data)

            
            frame = get_frame()
            ydim, xdim = frame.shape[:2]

            boxes = boxes_from_frame(frame)
            
            if len(boxes) > 0:
                predictedbox = []
                for i in range(4):
                    f = kalmanfilters[i]
                    f.predict()
                    predictedbox.append(f.x[0])
                best_box_index = find_best_box(boxes, predictedbox)
                best_box = boxes[best_box_index]

                boxleft, boxright = best_box[0], best_box[2]
                boxcenter = (boxleft + boxright) / 2

                if boxcenter < xdim / 2:
                    data = "left"
                elif boxcenter > xdim / 2:
                    data = "right"

                serial_port.write(data)

            # if we get a carriage return, add a line feed too
            # \r is a carriage return; \n is a line feed
            # This is to help the tty program on the other end 
            # Windows is \r\n for carriage return, line feed
            # Macintosh and Linux use \n
            if data == "\r".encode():
                # For Windows boxen on the other end
                serial_port.write("\n".encode())


except KeyboardInterrupt:
    print("Exiting Program")

except Exception as exception_error:
    print("Error occurred. Exiting Program")
    print("Error: " + str(exception_error))

finally:
    serial_port.close()
    pass