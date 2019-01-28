import cv2
import numpy as np


def map():
    img = np.zeros((600, 430))
    # outer border
    cv2.rectangle(img, (0, 0), (429, 599), 255)
    cv2.rectangle(img, (1, 1), (428, 598), 255)

    # outer circle
    cv2.line(img, (30, 195), (30, 404), 255)
    cv2.line(img, (31, 195), (31, 404), 255)
    cv2.line(img, (400, 195), (400, 404), 255)
    cv2.line(img, (399, 195), (399, 404), 255)

    cv2.ellipse(img, (215, 195), (185, 185), 0, 180, 360, 255)
    cv2.ellipse(img, (215, 195), (184, 184), 0, 180, 360, 255)
    cv2.ellipse(img, (215, 404), (185, 185), 180, 180, 360, 255)
    cv2.ellipse(img, (215, 404), (184, 184), 180, 180, 360, 255)

    # inner circle
    cv2.line(img, (94, 195), (94, 404), 255)
    cv2.line(img, (95, 195), (95, 404), 255)
    cv2.line(img, (336, 195), (336, 404), 255)
    cv2.line(img, (335, 195), (335, 404), 255)

    cv2.ellipse(img, (215, 195), (121, 121), 0, 180, 360, 255)
    cv2.ellipse(img, (215, 195), (120, 120), 0, 180, 360, 255)
    cv2.ellipse(img, (215, 404), (121, 121), 180, 180, 360, 255)
    cv2.ellipse(img, (215, 404), (120, 120), 180, 180, 360, 255)

    # middle circle
    cv2.line(img, (62, 195), (62, 404), 255)
    cv2.line(img, (63, 195), (63, 404), 255)
    cv2.line(img, (367, 195), (367, 404), 255)
    cv2.line(img, (368, 195), (368, 404), 255)

    cv2.ellipse(img, (215, 195), (152, 152), 0, 180, 360, 255)
    cv2.ellipse(img, (215, 195), (151, 151), 0, 180, 360, 255)
    cv2.ellipse(img, (215, 404), (152, 152), 180, 180, 360, 255)
    cv2.ellipse(img, (215, 404), (151, 151), 180, 180, 360, 255)

    # dilate because rasterization is not perfect for the circles
    img = cv2.dilate(img, (2, 2))

    marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    marker = cv2.aruco.drawMarker(marker_dict, 0, 20, 20)

    img[2:33, 2:24] = 255
    img[200:230, 2:24] = 255
    img[400:430, 2:24] = 255
    img[568:598, 2:24] = 255
    img[2:33, 406:428] = 255
    img[200:230, 406:428] = 255
    img[400:430, 406:428] = 255
    img[568:598, 406:428] = 255

    img[8:28, 3:23] = cv2.aruco.drawMarker(marker_dict, 1, 20, 20)
    img[205:225, 3:23] = cv2.aruco.drawMarker(marker_dict, 4, 20, 20)
    img[405:425, 3:23] = cv2.aruco.drawMarker(marker_dict, 6, 20, 20)
    img[573:593, 3:23] = cv2.aruco.drawMarker(marker_dict, 2, 20, 20)

    img[8:28, 407:427] = cv2.aruco.drawMarker(marker_dict, 7, 20, 20)
    img[205:225, 407:427] = cv2.aruco.drawMarker(marker_dict, 5, 20, 20)
    img[405:425, 407:427] = cv2.aruco.drawMarker(marker_dict, 3, 20, 20)
    img[573:593, 407:427] = cv2.aruco.drawMarker(marker_dict, 0, 20, 20)

    cv2.imwrite("map.bmp", img)


if __name__ == '__main__':
    map()


