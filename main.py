import cv2
from ultralytics import YOLO
import supervision as sv

# Load the YOLOv8 model
model = YOLO('yolov8n.pt')


cap = cv2.VideoCapture(0)

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLOv8 inference on the frame
        results = model(frame)
        detections = sv.Detections.from_ultralytics(results[0])
        detections = detections[detections.class_id == 0]

        detection = detections[0]
        ydim, xdim = frame.shape[:2]

        boxleft = detection.xyxy[0][0]
        boxright = detection.xyxy[0][2]
        boxcenter = (boxleft + boxright) / 2

        if boxcenter < xdim / 2:
            print("go left")
        elif boxcenter > xdim / 2:
            print("go right")

        box_annotator = sv.BoxAnnotator()
        annotated_frame = box_annotator.annotate(
            scene=frame.copy(),
            detections=detections,
            labels=["person"]*len(detections.class_id)
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