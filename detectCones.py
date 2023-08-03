import cv2
from imutils.video import VideoStream
import imutils

def callback(value):
    pass
def create_trackbars():
    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255
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
# cv2.namedWindow("trackbars", 0)
# create_trackbars()
while True:
    image = camera.read()
    height, width, channels = image.shape
    frame_to_mask = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # H_min,S_min,V_min,H_max,S_max,V_max = get_trackbar_values()

    # HSV values tuned to downstairs lighting conditions
    H_min = 0
    S_min =119
    V_min = 228
    H_max = 43
    S_max = 255
    V_max = 255

    # whitespace is ignored inside parentheses:
    mask = cv2.inRange(frame_to_mask, (H_min, S_min, V_min), (H_max, S_max, V_max))
    # blur with an 11x11-pixel kernel:
    blurred = cv2.GaussianBlur(image, (11,11), 0)
    # erode a mask with standard settings, twice:
    maskErode = cv2.erode(mask, None, iterations=2)
    # dilate a mask with standard settings, twice:
    maskDilate = cv2.dilate(mask, None, iterations=2)
    # find the coutours in a mask:
    contr = cv2.findContours(maskDilate.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contr = imutils.grab_contours(contr)
    # if you find more than zero contours, do something with them:
    if len(contr) > 0:
    # select the largest area:
        c = max(contr, key=cv2.contourArea)
    # calculate the minimum circle enclosing contour c:
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        # print(x, y, radius)
    # draw a circle onto our original image
    # using a 2-pixel-wide yellow line
        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        if x < (width/2):
            print("left")
        else:
            print("right")
    # print(mask)
    
    cv2.imshow("original", image)
    cv2.imshow("mask", mask)
    if cv2.waitKey(1) & 0xFF is ord('q'):
        break

