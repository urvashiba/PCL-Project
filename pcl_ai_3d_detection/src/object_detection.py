import os
import torch
import rospy
import logging

logging.basicConfig(level=logging.INFO)

# Check GPU availability
if torch.cuda.is_available():
    logging.info("CUDA is available. GPU: %s", torch.cuda.get_device_name(0))
else:
    logging.warning("CUDA is not available. Running on CPU.")

try:
    # Load the YOLOv8n model
    model = torch.hub.load('ultralytics/yolov8', 'yolov8n', force_reload=False)
    logging.info("YOLOv8n model loaded successfully!")
except Exception as e:
    logging.error("Error loading YOLOv8 model: %s", e)
    exit(1)

def detect_objects(image_path=None):
    if image_path is None:
        image_path = rospy.get_param("input_image", default="/path/to/default/image.jpg")

    # Validate the input image path
    if not os.path.exists(image_path):
        logging.error("Error: The file '%s' does not exist.", image_path)
        return None

    # Perform object detection
    try:
        results = model(image_path)
        logging.info("Detection results: %s", results.pandas().xyxy[0])
        return results.pandas().xyxy[0]  # Return detection results as a Pandas DataFrame
    except Exception as e:
        logging.error("Error processing image: %s", e)
        return None

# ROS parameter setup (remove this if using detection.launch for parameter setup)
# rospy.set_param("input_image", "c:/Users/Urvashiba Parmar/Desktop/PCL-Project/images/sample_image.jpg")

# Get the input image path from ROS parameter
input_image = rospy.get_param("input_image")
detect_objects(input_image)