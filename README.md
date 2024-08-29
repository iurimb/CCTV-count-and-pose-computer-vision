# CCTV-count-and-pose-computer-vision

## Objectives: 

CCTV project. Detect, track and count people. Count ins and outs. Extract and display keypoints from pose yolov8 model. Define interest area with Polygon for payment checking (future development)
Cool project, many developments possible to be made. With extracted keypoints it might be possible to go into action recognition, which would be the next step (for example, with CNN-LSTM's architectures). 

In this case, the custom dataset is the footage, but it's only used for testing. There was no specific model training, since yolov8 already posesses class "person". I've applied yolov8-n for detection and yolov8-n-pose for keypoints extraction.


## Disclaimers 
The code is functionable, but not yet organized, so I also couldn't yet organize the python environment. You might note (until I solve this) that there are many commented lines in the codes, and even parts of it that are 'outdated' (for example, some polygon shapes not being used). 


## Folders 

- detect_track_count: folder containing two scripts to detect, track and count (ins and outs) of people in store. See section "Running the codes" for details. 
- results: folder with short videos representing the outputs of each method. 
- training_loop: script to train a yolo model for object detection.
- line_zone: line_zone code I use that contains a modified function for annotating info. You could substitute the original line_zone file with mine or only copy the modified method code into yours ("_annotate_anything_count"). Alternatevely, you can give it a different name and import it. 

## About the task 

This is a regular task. I was testing yolov8-pose network and preparing some data to go into action recognition tasks. 

## Running the codes
Codes are found in the "training_loop" and "detect_count_and_track" folders. 
The training_loop code is easily executed by just inserting your 'data.yaml' file_path into the "data" variable. 

The detect, track and count codes are also easily executable (abstracting from the dependencies... I'm on it ASAP) by plugging in the video_file path into the code. Below the imports, you'll find:


- VIDEO_PATH = "INSERT_VIDEO_PATH"

That should do it. 


### Some final observations 
#### 1) I used a modified version of the annotator method in "LineZoneAnnotator" to display information on the videos. I uploaded my "Line_Zone" file together with the code. 
#### 2) I did not have the time to adequately organize the code, I reckon it is a bit messy and there's work to be done there. I'm currently working on refactoring the personal projects I upload to github. 

