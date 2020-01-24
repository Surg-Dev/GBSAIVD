import math
import os
import time

import numpy as np

import cv2


def gstreamer_pipeline(
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=2,
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


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 2 / 3)  # make points from bottom 1/3 of the frame down
    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]


def stabilize_steering_angle(
        curr_steering_angle,
        new_steering_angle,
        num_of_lane_lines,
        max_angle_deviation_two_lines=5,
        max_angle_deviation_one_lane=1):
    """
    Using last steering angle to stabilize the steering angle
    if new angle is too different from current angle, 
    only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2:
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else:
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane

    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(
            curr_steering_angle + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    return stabilized_steering_angle

def camera():
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(
        flip_method=0), cv2.CAP_GSTREAMER)
    if cap.isOpened():
        window_handle = cv2.namedWindow("Line Detector", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("Line Detector", 0) >= 0:
            ret_val, image = cap.read()
            #cv2.imshow("CSI Camera", image)
            
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            #cv2.imshow("HSV", hsv)
            lower_color = np.array([130, 0, 185])
            upper_color = np.array([180, 100, 255])
            mask = cv2.inRange(hsv, lower_color, upper_color)
            imageEdges = cv2.Canny(mask, 200, 400)
            #cv2.imshow("HSV Mask", mask)
            #cv2.imshow("Mask Edges", imageEdges)

            height, width = imageEdges.shape
            maskEdges = np.zeros_like(imageEdges)
            polygon = np.array([[
                (0, height * 2 / 3),
                (width, height * 2 / 3),
                (width, height),
                (0, height),
            ]], np.int32)
            cv2.fillPoly(maskEdges, polygon, 255)
            croppedEdges = cv2.bitwise_and(imageEdges, maskEdges)
            #cv2.imshow("Cropped Mask Edges", croppedEdges)

            rho = 1  # distance precision in pixel, i.e. 1 pixel
            angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
            min_threshold = 80  # minimal number of votes
            lineSegments = cv2.HoughLinesP(croppedEdges, rho, angle, min_threshold,
                                           minLineLength=60, maxLineGap=100)
            # print(lineSegments)
            line_image = np.zeros_like(image)
            line_color = (0, 255, 0)
            line_width = 6
            if lineSegments is not None:
                for line in lineSegments:
                    for x1, y1, x2, y2 in line:
                        if x1 == x2:
                            # Skip vertical line segment
                            continue
                        fit = np.polyfit((x1, x2), (y1, y2), 1)
                        slope = fit[0]
                        intercept = fit[1]
                        if abs(slope) < 0.2:
                            # Skip horizontal line
                            continue
                        cv2.line(line_image, (x1, y1), (x2, y2),
                                 line_color, line_width)
                        line_image = cv2.addWeighted(
                            image, 0.8, line_image, 1, 1)
            #cv2.imshow("Line Segment Overlay", line_image)

            lane_lines = []
            if lineSegments is None:
                print('No line_segment segments detected', lineSegments)
            else:
                height, width, _ = image.shape
                left_fit = []
                right_fit = []
                boundary = 1/3
                # left lane line segment should be on left 2/3 of the screen
                left_region_boundary = width * (1 - boundary)
                # right lane line segment should be on right 2/3 of the screen
                right_region_boundary = width * boundary
                for lineSegment in lineSegments:
                    for x1, y1, x2, y2 in lineSegment:
                        if x1 == x2:
                            # Skip vertical line segment
                            continue
                    fit = np.polyfit((x1, x2), (y1, y2), 1)
                    slope = fit[0]
                    #print("SLOPE", slope)
                    intercept = fit[1]
                    if abs(slope) < 0.2:
                        # Skip horizontal line
                        continue
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))
            left_fit_average = np.average(left_fit, axis=0)
            if len(left_fit) > 0:
                lane_lines.append(make_points(image, left_fit_average))
                right_fit_average = np.average(right_fit, axis=0)
                if len(right_fit) > 0:
                    lane_lines.append(make_points(image, right_fit_average))
                # print(lane_lines)

            line_image2 = np.zeros_like(image)
            line_color = (255, 0, 0)
            line_width = 6
            if lane_lines is not None:
                for line in lane_lines:
                    for x1, y1, x2, y2 in line:
                        cv2.line(line_image2, (x1, y1), (x2, y2),
                                 line_color, line_width)
                        line_image2 = cv2.addWeighted(
                            line_image, 0.8, line_image2, 1, 1)
                #cv2.imshow("Line Overlay", line_image2)

            if len(lane_lines) == 0:
                print('No lane lines detected.')
            steeringAngle = -90

            height, width, _ = image.shape
            if len(lane_lines) == 1:
                print('Only detected one lane line. Following.')
                x1, _, x2, _ = lane_lines[0][0]
                x_offset = x2 - x1
                X1, Y1, X2, Y2 = lane_lines[0][0]
                MIDPOINTX1 = X1
                MIDPOINTX2 = X2
                MIDPOINTY1 = Y1
                MIDPOINTY2 = Y2
            else:
                print('Detected two lane lines. All is well.')
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]
            # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
            camera_mid_offset_percent = 0.02
            mid = int(width / 2 * (1 + camera_mid_offset_percent))
            x_offset = (left_x2 + right_x2) / 2 - mid

            LX1, LY1, LX2, LY2 = lane_lines[0][0]
            RX1, RY1, RX2, RY2 = lane_lines[1][0]
            MIDPOINTX1 = (LX1 + RX1) / 2
            MIDPOINTY1 = (LY1 + RY1) / 2
            MIDPOINTX2 = (LX2 + RX2) / 2
            MIDPOINTY2 = (LY2 + RY2) / 2

            # find the steering angle, which is angle between navigation direction to end of center line
            y_offset = int(height * 2 / 3)

            # angle (in radian) to center vertical line
            angle_to_mid_radian = math.atan(x_offset / y_offset)
            # angle (in degrees) to center vertical line
            angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
            # this is the steering angle needed by picar front wheel
            steeringAngle = angle_to_mid_deg + 90

            #print('Steering Angle: ', steeringAngle)

            headingImage = np.zeros_like(image)
            height, width, _ = image.shape
            line_color = (0, 0, 255)
            line_width = 10

            # figure out the heading line from steering angle
            # heading line (x1,y1) is always center bottom of the screen
            # (x2, y2) requires trigonometry

            # Note: the steering angle of:
            # 0-89 degree: turn left
            # 90 degree: go straight
            # 91-180 degree: turn right
            steering_angle_radian = steeringAngle / 180.0 * math.pi
            x1 = int(width / 2)
            y1 = height
            x2 = int(x1 - height * 2 / 3 / math.tan(steering_angle_radian))
            y2 = int(height * 2 / 3)

            cv2.line(headingImage, (MIDPOINTX1, MIDPOINTY1),
                     (MIDPOINTX2, MIDPOINTY2), line_color, line_width)
            headingImage = cv2.addWeighted(
                line_image2, 0.8, headingImage, 1, 1)
            cv2.imshow("Directional/Follow Line", headingImage)

            # Add image here that shows follow line and line overlay at same time for debugging, along with edges (red, green, and blue)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break


camera()
