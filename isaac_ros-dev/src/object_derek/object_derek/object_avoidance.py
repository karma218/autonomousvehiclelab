from ultralytics import YOLO
import numpy as np
from PIL import Image
import requests
from io import BytesIO
import cv2
from matplotlib import pyplot as plt
import glob


cnt = 0

def box_label(image, box, b1, b2, label='', color=(128, 128, 128), txt_color=(255, 255, 255)):
  lw = max(round(sum(image.shape) / 2 * 0.003), 2)
  p1, p2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
  b1.append(p1)
  b2.append(p2)
  cv2.rectangle(image, p1, p2, color, thickness=lw, lineType=cv2.LINE_AA)

    
def is_point_in_trapezoid(point, trapezoid_points):
    return cv2.pointPolygonTest(trapezoid_points, point, False) >= 0
    
def plot_bboxes(image, boxes, b1, b2, trapezoid_points, labels=[], colors=[], score=True, conf=None):
  #Define COCO Labels
  # global cb # box center 
  # global cnt
  if labels == []:
    labels = {0: u'__background__', 1: u'person'}
  #Define colors
  if colors == []:
    #colors = [(6, 112, 83), (253, 246, 160), (40, 132, 70), (205, 97, 162), (149, 196, 30), (106, 19, 161), (127, 175, 225), (115, 133, 176), (83, 156, 8), (182, 29, 77), (180, 11, 251), (31, 12, 123), (23, 6, 115), (167, 34, 31), (176, 216, 69), (110, 229, 222), (72, 183, 159), (90, 168, 209), (195, 4, 209), (135, 236, 21), (62, 209, 199), (87, 1, 70), (75, 40, 168), (121, 90, 126), (11, 86, 86), (40, 218, 53), (234, 76, 20), (129, 174, 192), (13, 18, 254), (45, 183, 149), (77, 234, 120), (182, 83, 207), (172, 138, 252), (201, 7, 159), (147, 240, 17), (134, 19, 233), (202, 61, 206), (177, 253, 26), (10, 139, 17), (130, 148, 106), (174, 197, 128), (106, 59, 168), (124, 180, 83), (78, 169, 4), (26, 79, 176), (185, 149, 150), (165, 253, 206), (220, 87, 0), (72, 22, 226), (64, 174, 4), (245, 131, 96), (35, 217, 142), (89, 86, 32), (80, 56, 196), (222, 136, 159), (145, 6, 219), (143, 132, 162), (175, 97, 221), (72, 3, 79), (196, 184, 237), (18, 210, 116), (8, 185, 81), (99, 181, 254), (9, 127, 123), (140, 94, 215), (39, 229, 121), (230, 51, 96), (84, 225, 33), (218, 202, 139), (129, 223, 182), (167, 46, 157), (15, 252, 5), (128, 103, 203), (197, 223, 199), (19, 238, 181), (64, 142, 167), (12, 203, 242), (69, 21, 41), (177, 184, 2), (35, 97, 56), (241, 22, 161)]
    colors = [(89, 161, 197),(67, 161, 255),(19, 222, 24),(186, 55, 2),(167, 146, 11),(190, 76, 98),(130, 172, 179),(115, 209, 128),(204, 79, 135),(136, 126, 185),(209, 213, 45),(44, 52, 10),(101, 158, 121),(179, 124, 12),(25, 33, 189),(45, 115, 11),(73, 197, 184),(62, 225, 221),(32, 46, 52),(20, 165, 16),(54, 15, 57),(12, 150, 9),(10, 46, 99),(94, 89, 46),(48, 37, 106),(42, 10, 96),(7, 164, 128),(98, 213, 120),(40, 5, 219),(54, 25, 150),(251, 74, 172),(0, 236, 196),(21, 104, 190),(226, 74, 232),(120, 67, 25),(191, 106, 197),(8, 15, 134),(21, 2, 1),(142, 63, 109),(133, 148, 146),(187, 77, 253),(155, 22, 122),(218, 130, 77),(164, 102, 79),(43, 152, 125),(185, 124, 151),(95, 159, 238),(128, 89, 85),(228, 6, 60),(6, 41, 210),(11, 1, 133),(30, 96, 58),(230, 136, 109),(126, 45, 174),(164, 63, 165),(32, 111, 29),(232, 40, 70),(55, 31, 198),(148, 211, 129),(10, 186, 211),(181, 201, 94),(55, 35, 92),(129, 140, 233),(70, 250, 116),(61, 209, 152),(216, 21, 138),(100, 0, 176),(3, 42, 70),(151, 13, 44),(216, 102, 88),(125, 216, 93),(171, 236, 47),(253, 127, 103),(205, 137, 244),(193, 137, 224),(36, 152, 214),(17, 50, 238),(154, 165, 67),(114, 129, 60),(119, 24, 48),(73, 8, 110)]
  # add largest area so the biggest bbox is selected if more are there.
  largest_area = 0
  largest_p1 = None
  largest_p2 = None 

  #plot each boxes
  for box in boxes:
    #add score in label if score=True 
    if int(box[-1]) == 0:
      if score :
        label = labels[int(box[-1])+1] + " " + str(round(100 * float(box[-2]),1)) + "%"
      else :
        label = labels[int(box[-1])+1]
        #print(box[-2])
      #filter every box under conf threshold if conf threshold setted
      if conf :
        if box[-2] > conf:
          color = colors[int(box[-1])]
          box_label(image, box, b1, b2, label, color)
          p1 = (int(box[0]), int(box[1]))
          p2 = (int(box[2]), int(box[3]))
          area = (p2[0] - p1[0]) * (p2[1] - p1[1])
          if area > largest_area:
              largest_area = area 
              largest_p1 = p1
              largest_p2 = p2
          
      else:
        color = colors[int(box[-1])]
        box_label(image, box, b1, b2, label, color)
        p1 = (int(box[0]), int(box[1]))
        p2 = (int(box[2]), int(box[3]))
        area = (p2[0] - p1[0]) * (p2[1] - p1[1])
        if area > largest_area:
            largest_area = area 
            largest_p1 = None
            largest_p2 = None  
          

  return image, largest_p1, largest_p2


cf =  320

model = YOLO("yolov8n.pt")

# #path = './soccer_pics/frame_30.jpg'
# path = './A/a.jpg'

# image = Image.open(path)
# image = np.asarray(image)

# image = cv2.resize(image, (640, 480))

# current_result = model.predict(image)

# boxes = current_result[0].boxes.data
# #print("prev boxes", len(boxes))

# # plotting boxes in original image and saving the coordinates   

# black1 = np.zeros_like(image, dtype=np.uint8)
# b1 = []
# b2 = []
# # Define trapezoid points
# trapezoid_points = np.array([[140, 480], [500, 480], [370, 280], [270, 280]], np.int32)
# trapezoid_points = trapezoid_points.reshape((-1, 1, 2))
# cv2.polylines(image, [trapezoid_points], isClosed=True, color=(0, 0, 255), thickness=2)

# image = plot_bboxes(image, boxes, b1, b2, trapezoid_points, score=True, conf=0.90)

# # plt.imshow(image)
# # plt.show()


# ----- COMMENTED OUT ER 08/19/2024 - 09:06 AM ---------------------------------------------------
# def detect_object(msg):
#   image = cv2.resize(msg, (640, 480))

#   current_result = model.predict(image)

#   boxes = current_result[0].boxes.data
#   #print("prev boxes", len(boxes))

#   # plotting boxes in original image and saving the coordinates   

#   black1 = np.zeros_like(image, dtype=np.uint8)
#   b1 = []
#   b2 = []
#   # Define trapezoid points
#   trapezoid_points = np.array([[140, 480], [500, 480], [370, 280], [270, 280]], np.int32)
#   trapezoid_points = trapezoid_points.reshape((-1, 1, 2))
#   cv2.polylines(image, [trapezoid_points], isClosed=True, color=(0, 0, 255), thickness=2)

#   image, result = plot_bboxes(image, boxes, b1, b2, trapezoid_points, score=True, conf=0.50)

#   return image, result
#   # cv2.imwrite(filename, image)
#   # cv2.imshow(image)
# # plt.show()

#   return image, "MOVE"
#-------------------------------------------------------------------------------------------------
# cf =  320

# model = YOLO("yolov8n.pt")

# #path = './soccer_pics/frame_30.jpg'
# path = './A/a.jpg'

# image = Image.open(path)
# image = np.asarray(image)

# image = cv2.resize(image, (640, 480))

# current_result = model.predict(image)

# boxes = current_result[0].boxes.data
# #print("prev boxes", len(boxes))

# # plotting boxes in original image and saving the coordinates   

# black1 = np.zeros_like(image, dtype=np.uint8)
# b1 = []
# b2 = []
# # Define trapezoid points
# trapezoid_points = np.array([[140, 480], [500, 480], [370, 280], [270, 280]], np.int32)
# trapezoid_points = trapezoid_points.reshape((-1, 1, 2))
# cv2.polylines(image, [trapezoid_points], isClosed=True, color=(0, 0, 255), thickness=2)

# image = plot_bboxes(image, boxes, b1, b2, trapezoid_points, score=True, conf=0.90)

# # plt.imshow(image)
# # plt.show()



def detect_object(msg):
  image = cv2.resize(msg, (640, 480))

  current_result = model.predict(image)

  boxes = current_result[0].boxes.data
  #print("prev boxes", len(boxes))

  # plotting boxes in original image and saving the coordinates   
  b1 = []
  b2 = []
  # Define trapezoid points
  trapezoid_points = np.array([[140, 480], [500, 480], [370, 280], [270, 280]], np.int32)
  trapezoid_points = trapezoid_points.reshape((-1, 1, 2))
  cv2.polylines(image, [trapezoid_points], isClosed=True, color=(0, 0, 255), thickness=2)

  image, p1, p2 = plot_bboxes(image, boxes, b1, b2, trapezoid_points, score=True, conf=0.50)
  person = False
  if p1 is not None:
    cb = int((p1[0] + p2[0])/2)
    person = True
    if p2[1] > 280 and cb > cf and p1[0] < 530:
      direction = f"left"
    elif p2[1] > 280 and cb < cf and p2[0] > 110:
      direction = f"right"
    else:
      direction = f"straigth"

  else:
    direction = f"straigth"

  return image, direction


