from imutils.video import VideoStream
import numpy as np
import cv2
from pyapriltags import Detector

def draw(frame_, corner, points):
	corner = tuple(np.int0(corner))
	cv2.line(frame_, corner, tuple(np.int0(points[0,0])), (0,0,255), 2)
	cv2.line(frame_, corner, tuple(np.int0(points[1,0])), (0,255,0), 2)
	cv2.line(frame_, corner, tuple(np.int0(points[2,0])), (255,0,0), 2)
	return frame_
	
vs = VideoStream(src=0).start()
# vs = cv2.VideoCapture(0)
# vs.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

at_detector = Detector(families='tag36h11')

tagWidth = 0.1
axisLength = 0.07
axis = axisLength * np.float32([[1,0,0], [0,1,0],[0,0,-1]])

fx = 642.210
fy = 645.48384
cx = 366.90074
cy = 258.30506

camVector = [fx,fy,cx,cy]
camMatrix = [[fx,0,cx],[0,fy,cy],[0,0,1]]
camAxis = np.float32(camMatrix)

font = cv2.FONT_HERSHEY_SIMPLEX
fScale = 0.45
fColor = (0, 255, 255)
fStroke = 2

while True:
	frame = vs.read()
	height, width, channels = frame.shape
	# scale_percent = 25
	# width = int(frame.shape[1]*scale_percent/100)
	# height = int(frame.shape[0]*scale_percent/100)
	# dim = (width, height)
	# frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
	# print(frame)
	frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	tags = at_detector.detect(frameGray, estimate_tag_pose=True,camera_params=camVector, tag_size=tagWidth)
	
	for tag in tags:
		if tag.hamming == 0:
			# print(tag.corners)
			startX = np.int0(min(tag.corners[:,0]))
			startY = np.int0(min(tag.corners[:,1]))
			cv2.drawContours(frame, np.int0([tag.corners]), 0, fColor, 2)
			(cX, cY) = (int(tag.center[0]), int(tag.center[1]))
			cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

			if cX < (width/2):
				print("detecting tag on left")
			else:
				print("detecting tag on right")

			phi = (360/6.28)*np.arcsin(-tag.pose_R[2,0])
			print("phi: ", phi)
			# if phi < 0:
			# 	print("robot too far left")
			# else:
			# 	print("robot too far right")
	
			y = startY-10
			text = "Tag %s" % (tag.tag_id)
			pos = f" A: {(360/6.28)*np.arcsin(-tag.pose_R[2,0]):.3f}"
			pos = pos + f" B: {(360/6.28)*np.arctan(tag.pose_R[2,1]/tag.pose_R[2,2]):.3f}"
			pos = pos + f" C: {(360/6.28)*np.arctan(tag.pose_R[1,0]/tag.pose_R[0,0]):.3f}"
			text = text + pos
			cv2.putText(frame, text, (startX, y), font, fScale, (255, 0, 0), fStroke)
			center_tuple = tuple([int(q) for q in tag.center])
			
			imgpts, jac = cv2.projectPoints(axis, tag.pose_R, tag.pose_t, camAxis, np.array([]))
			frame = draw(frame, center_tuple, imgpts)
			
	cv2.imshow("frame", frame)
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		break
vs.stop()

cv2.destroyAllWindows()
