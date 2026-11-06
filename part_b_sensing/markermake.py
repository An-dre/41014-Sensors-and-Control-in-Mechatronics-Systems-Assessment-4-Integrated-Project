"""Make the dock's ArUco marker image, robotis_tb3/aruco_0.png (id 0, DICT_4X4_50).

The marker is 200 px with a 25 px white border, so on the 0.2 m plate in sceneforturtle.xml
the black square is 0.16 m: that is sensing.MARKER_SIZE. The image is mirrored (cv2.flip)
so the marker reads the right way round on the face of the plate the robot sees.

    python -m part_b_sensing.markermake
"""

from pathlib import Path

import cv2

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
img = cv2.aruco.generateImageMarker(d, 0, 200)            # id 0, 200 px
img = cv2.copyMakeBorder(img, 25, 25, 25, 25, cv2.BORDER_CONSTANT, value=255)  # white quiet zone
img = cv2.flip(img, 1)

# the scene loads the marker image from robotis_tb3
cv2.imwrite(str(Path(__file__).resolve().parents[1] / "robotis_tb3" / "aruco_0.png"), img)
