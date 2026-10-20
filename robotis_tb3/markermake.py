import cv2

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
img = cv2.aruco.generateImageMarker(d, 0, 200)            # id 0, 200 px
img = cv2.copyMakeBorder(img, 25, 25, 25, 25, cv2.BORDER_CONSTANT, value=255)  # white quiet zone
img = cv2.flip(img, 1)

cv2.imwrite("aruco_0.png", img)