import cv2
from ultralytics import YOLO
import supervision as sv
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
from copy import deepcopy


# Load the YOLOv8 model
model = YOLO('yolov8n.pt')

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

cap = cv2.VideoCapture(0)

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLOv8 inference on the frame
        results = model(frame)
        detections = sv.Detections.from_ultralytics(results[0]) # Detections class is not very nice to work with
        detections = detections[detections.class_id == 0]
        labels = ["person"] * len(detections.class_id)

        
        if len(detections) > 0:
            predictedxyxy = []
            for i in range(4):
                f = kalmanfilters[i]
                f.predict()
                predictedxyxy.append(f.x[0])
            
            best_index = None
            best_overlap_rate = -1
            for i, detection in enumerate(detections):
                print(i, detection)
                overlapxyxy = [None] * 4
                xyxy = detection[0]
                print(xyxy)

                for j in range(4):
                    if j < 2:
                        func = max
                    else:
                        func = min
                    overlapxyxy[j] = func(predictedxyxy[j], xyxy[j])
                
                calc_box_area = lambda box : max(0, box[2] - box[0]) * max(0, box[3] - box[1])
                overlap_area = calc_box_area(overlapxyxy)
                total_area = calc_box_area(predictedxyxy) + calc_box_area(xyxy) - overlap_area
                overlap_rate = overlap_area / total_area

                if overlap_rate > best_overlap_rate:
                    best_overlap_rate = overlap_rate
                    best_index = i

            print(best_index, best_overlap_rate)
            detection = detections[best_index]
            ydim, xdim = frame.shape[:2]

            xyxy = detection.xyxy[0]
            boxleft = detection.xyxy[0][0]
            boxright = detection.xyxy[0][2]
            boxcenter = (boxleft + boxright) / 2

            for i in range(4):
                f = kalmanfilters[i]
                f.update(xyxy[i])
            
            predictedxyxy = np.array(predictedxyxy)
            detections.xyxy = np.concatenate((detections.xyxy,[predictedxyxy]),axis=0)
            detections.class_id = np.append(detections.class_id, 1)
            labels.append("kalman")

            if boxcenter < xdim / 2:
                print("go left")
            elif boxcenter > xdim / 2:
                print("go right")

        #print(detections)
        box_annotator = sv.BoxAnnotator()
        annotated_frame = box_annotator.annotate(
            scene=frame.copy(),
            detections=detections,
            labels=labels
        )

        # Visualize the results on the frame

        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()