import numpy as np
import cv2

"""
def gstreamer_pipeline(
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=21,
        flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


print(gstreamer_pipeline(flip_method=0))
"""
gst_str = ('nvarguscamerasrc ! '
           'video/x-raw(memory:NVMM), '
           'width=(int)1280, height=(int)720, '
           'format=(string)NV12, framerate=(fraction)15/1 ! '
           'nvvidconv flip-method=0 ! '
           'video/x-raw, width=(int){}, height=(int){}, '
           'format=(string)BGRx ! '
           'videoconvert ! appsink').format(1280, 720)
#cap = cv2.VideoCapture(0,cv2.CAP_GSTREAMER)
cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
#cap = cv2.VideoCapture("imxv4l2videosrc ! video/x-raw ! appsink")
while(True):
    if cap.isOpened():
        break
while(True):
    ret, frame = cap.read()
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    lines = cv2.HoughLines(edges, 1, np.pi/180, 200)

    for r,theta in lines[0]:
        a = np.cos(theta)

        b = np.sin(theta)

        x0 = a*r
        y0 = b*r

        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))

        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
    """
    cv2.imshow('frame', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        cap.release()
        cv2.destroyAllWindows()
        break
