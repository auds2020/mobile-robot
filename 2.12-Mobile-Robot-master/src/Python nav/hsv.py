import cv2
from imutils.video import VideoStream

def callback(value):
    pass
def create_trackbars():
    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255.
        for j in "HSV":
            cv2.createTrackbar("%s_%s" % (j, i), "trackbars", v, 255, callback)
def get_trackbar_values():
    values = []
    for i in ["MIN", "MAX"]:
        for j in "HSV":
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "trackbars")
            values.append(v)
    return values

camera = VideoStream(src=0).start()
# camera = cv2.VideoCapture(0)
cv2.namedWindow("original", cv2.WINDOW_NORMAL)
cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
cv2.namedWindow("trackbars", 0)
create_trackbars()
while True:
    image = camera.read()
    frame_to_mask = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    H_min,S_min,V_min,H_max,S_max,V_max = get_trackbar_values()
    # whitespace is ignored inside parentheses:
    mask = cv2.inRange(frame_to_mask , (H_min, S_min, V_min), (H_max, S_max, V_max))
    cv2.imshow("original", image)
    cv2.imshow("mask", mask)
    if cv2.waitKey(1) & 0xFF is ord('q'):
        break