import supervision as sv
from ultralytics import YOLO
import cv2
import numpy as np
from PIL import Image
from supervision import Position
from shapely.geometry import Polygon
from shapely.geometry.point import Point
from collections import defaultdict
from ultralytics.utils.plotting import Annotator, colors
import os
#from ultralytics import YOLOv10

# Importing datetime 
import datetime 
import time
from time import perf_counter
from timeit import default_timer as cronometro

# importing whole module
from tkinter import *
from tkinter.ttk import *
import datetime 
# importing strftime function to
# retrieve system's time
from time import strftime
#import time
from datetime import timedelta
import math 
# Initializing a date and time 
'''
inicio = cronometro()
# Seu código aqui


#start = time.perf_counter()
#print(start)

#input(dt)

while True: 
    fim = cronometro()
    time_delta = datetime.timedelta(seconds=fim-inicio) 
    #print(time_delta)
    #print(inicio)    
    date_and_time_of_video = (date_and_time_of_video + time_delta)
    #date_and_time_of_video = date_and_time_of_video + time_sec
    print(date_and_time_of_video)
#current_time_24hr = time.strftime("%H:%M:%S")
#current_time#_12hr = time.strftime("%I:%M:%S %p")
#current_date = time.strftime("%Y-%m-%d")
'''
#model = YOLO('best_new_bees.pt')
ROOT = os.getcwd()
annotator = sv.LineZoneAnnotator()
#input(ROOT)
#model = YOLO(r"C:\Users\imich\OneDrive\Documentos\Clutch\Abelhas\abelhas_chegando.pt")
model = YOLO("yolov8n.pt")
#model.names
#model.names = {0: 'Abelha'}
#input(model.names)

CCTV = os.path.join(ROOT, "rael_cctv.mp4")
CCTV2 = os.path.join(ROOT, "rael_CCTV_2.mp4")
#input(CCTV)
frames_generator = sv.get_video_frames_generator(CCTV2, start = 0, stride=3)
#frames_generator = sv.get_video_frames_generator(CCTV2, start = 0)

bounding_box_annotator = sv.BoxAnnotator()
label_annotator = sv.LabelAnnotator()
lost_track_buffer = 100
tracker = sv.ByteTrack(track_activation_threshold=0.7, lost_track_buffer=100, minimum_consecutive_frames=10)
vcap = cv2.VideoCapture(CCTV2)
width  = vcap.get(cv2.CAP_PROP_FRAME_WIDTH)   # float `width`
height = vcap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = vcap.get(cv2.CAP_PROP_FPS)
#input(fps)
 # choose codec according to format needed
fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
video = cv2.VideoWriter('Every_two_frames.mp4', fourcc, fps, (int(width), int(height)))
heat_map_annotator = sv.HeatMapAnnotator()
COLORS = sv.ColorPalette.DEFAULT
length = int(vcap.get(cv2.CAP_PROP_FRAME_COUNT))
#input(length)

polygon = np.array([
    [0, 211], #top_left
    [168, 216], #top_right
    [168, 480], #bottom_right
    [0, 480] #bottom_left
])

detections_list = []

polygon = sv.PolygonZone(polygon=polygon)
lost_tracks_list = []
out_count = 0 
index_count = 1

for frame_idx, frame in enumerate(frames_generator):
    #result = model.track(frame, persist=True)[0]
    result = model.predict(frame, conf=0.6, classes=[0,1])[0]
    detections = sv.Detections.from_ultralytics(result)
    detections = tracker.update_with_detections(detections)
#results = model.track(source=CCTV, persist=True, stream=True, show=True)
    
    if detections.tracker_id.any():
        labels_tracker = [
        f"#{tracker_id} {class_name} {confidence:.2f}"
        for tracker_id, class_name, confidence
        in zip(detections.tracker_id, detections['class_name'], detections.confidence)
        ]

    if detections:
        labels = [
        f"{class_name} {confidence:.2f}"
        for class_name, confidence
        in zip(detections['class_name'], detections.confidence)
        ]
    
    
    annotated_frame = bounding_box_annotator.annotate(scene=frame.copy(), detections=detections)
    #annotated_frame = sv.draw_polygon(scene=annotated_frame, polygon=polygon, color=COLORS.colors[0])
    zone_annotator = sv.PolygonZoneAnnotator(zone=polygon, color=sv.Color.WHITE, thickness=6, text_thickness=6, text_scale=4)
    #input(type(detections))
    track_ids_buffer = []
    #tracked_tracks; lost_tracks and removed_tracks -> get from tracker 
    #get from tracker self.track_id, self.start_frame, self.end_frame
    if len(detections.tracker_id) > 0:
        #tracked_tracks = input(tracker.tracked_tracks)
        #input(type(detections.tracker_id))
        #self.track_id, self.start_frame, self.end_frame
        for track in tracker.tracked_tracks:
            
        #for track in detections.tracker_id:
            #print(tracker.tracked_tracks, tracker.lost_tracks, tracker.removed_tracks, tracker.frame_id) #== track 
            #input("Para")
        #    if 
            #tracker.tracked_tracks[0].track_id, tracker.tracked_tracks[0].start_frame, tracker.tracked_tracks[0].end_frame,
            #time_detected = track.end_frame - track.start_frame
            #if track.external_track_id not in detections_list and track.end_frame - track.start_frame > 30 and track.external_track_id not in track_ids_buffer: #descarta as menores
            if track.external_track_id not in detections_list and track.external_track_id not in track_ids_buffer: #descarta as menores
            
                if track.external_track_id not in track_ids_buffer:
                    track_ids_buffer.append(track.external_track_id)
                #ele aparece mais de uma vez e isso buga, tem que pegar só uma aparicao 
                #detections_list.append(track.track_id) 
                detections_list.append(track.external_track_id)
                #index_count = len(detections_list)

        track_ids_buffer = []
        '''
        for detected in detections_list:
            if detected not in detections.tracker_id and detected not in lost_tracks_list :
                out_count = out_count+1
                lost_tracks_list.append(detected)
        '''
        #if detections.tracker_id not in detections_list:
        #    detections_list.append(detections.tracker_id)
            #input(detections.tracker_id)
        
        #input(detections.tracker_id)
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections, labels=labels_tracker)
        polygon.trigger(detections=detections)
        annotated_frame = zone_annotator.annotate(scene=annotated_frame)
        boxes = result.boxes.xyxy.cpu()
        track_ids = detections.tracker_id.tolist()
        clss = result.boxes.cls.cpu().tolist() 
    
        for box, track_id, cls in zip(boxes, track_ids, clss):
            bbox_center = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2  # Bbox center

            #if polygon_special.contains(Point((bbox_center[0], bbox_center[1]))):
    elif(len(detections)>0 and detections.tracker_id == 0):
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections, labels=labels)

    
    if tracker.removed_tracks:
        #input('am i here')
    #    input(tracker.lost_tracks[0])
    #    input(type(tracker.lost_tracks[0]))
        for losttrack in tracker.removed_tracks:
            #if track not in detections_list and track.end_frame - track.start_frame > 30:
                #detections_list.append(track.track_id) 

            #if losttrack.external_track_id not in lost_tracks_list and losttrack.end_frame - losttrack.start_frame > lost_track_buffer:
            if losttrack.external_track_id not in lost_tracks_list and losttrack.external_track_id > 0:
                #input('am i here')
                lost_tracks_list.append(losttrack.external_track_id)
    
    out_count = len(lost_tracks_list)
    in_count = len(detections_list)
    annotator._annotate_anything_count(annotated_frame, sv.Point(120, 20), f"Ins: {in_count}")
    annotator._annotate_anything_count(annotated_frame, sv.Point(120, 50), f"Outs {out_count}")
    annotator._annotate_anything_count(annotated_frame, sv.Point(120, 80), f"Ins_IDS {detections_list}")
    annotator._annotate_anything_count(annotated_frame, sv.Point(120, 110), f"Out_IDS {lost_tracks_list}")
    if frame_idx % 2 == 0:
        video.write(annotated_frame)
    #cv2.imwrite("frame.jpg", frame)
    #break
    cv2.imshow("frame", annotated_frame)
    
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break
