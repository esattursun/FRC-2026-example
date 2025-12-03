import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model

# Load the YOLO model
model = load_model('yolov3.h5')
# Object classes
classes = ["pedestrian", "vehicle", "emergency"]
# Set the number of roads
num_roads = 2
# Initialize road counters
road_counters = {f"road{i+1}": 0 for i in range(num_roads)}
# Load the image
image = cv2.imread('image.jpg')
# Get image dimensions
height, width = image.shape[:2]
# Object detection using YOLO
blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
model.setInput(blob)
outputs = model.forward(output_layers)

# Process detection results
boxes = []
confidences = []
class_ids = []
for output in outputs:
    for detection in output:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > 0.5:
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[2] * width)
            h = int(detection[3] * height)
            x = int(center_x - w / 2)
            y = int(center_y - h / 2)
            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)

# Draw detected objects and count vehicles for each road
has_emergency = False
count_pedestrians = 0
indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
for i in range(len(boxes)):
    if i in indexes:
        x, y, w, h = boxes[i]
        label = str(classes[class_ids[i]])
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if label == "emergency":
            has_emergency = True
        if label == "pedestrian":
            count_pedestrians += 1
        # Determine which road the vehicle is on
        road_index = 1 if x < width // 2 else 2
        if label == "vehicle":
            road_counters[f"road{road_index}"] += 1

# Traffic light control
if has_emergency:
    print("Emergency vehicle detected. Green light will be activated for emergency route.")
elif road_counters["road1"] > road_counters["road2"]:
    print("Traffic is heavy on road 1. Green light will be activated for road 1.")
elif road_counters["road2"] > road_counters["road1"]:
    print("Traffic is heavy on road 2. Green light will be activated for road 2.")
elif count_pedestrians > 0:
    print("Pedestrian detected. Green light will be activated for pedestrians.")
else:
    print("Normal traffic flow. Lights will be adjusted accordingly.")

# Display the results
cv2.imshow('Image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
