import cv2
from pathlib import Path

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
img = cv2.aruco.generateImageMarker(d, 0, 200)            # id 0, 200 px
img = cv2.copyMakeBorder(img, 25, 25, 25, 25, cv2.BORDER_CONSTANT, value=255)  # white quiet zone
img = cv2.flip(img, 1)

# the scene loads the marker image from robotis_tb3
cv2.imwrite(str(Path(__file__).resolve().parent.parent / "robotis_tb3" / "aruco_0.png"), img)