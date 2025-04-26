#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Purpose: Handles frame analysis for object tracking and motion decision logic.
    This module is independent of drone hardware interfaces.
"""

import cv2
import math
import time
import numpy as np
import imutils
from collections import deque
import xml.etree.ElementTree as ET
from pathlib import Path

"""
ImageAnalyzer is responsible for locating and tracking a colored object in video frames,
and issuing movement commands to help a drone center itself vertically and horizontally
on the object. It uses HSV thresholding to identify the target object.
"""
class ImageAnalyzer:

    MAX_YAW = np.pi / 4  # Maximum allowable yaw rate
    MAX_VSPEED = 0.1     # Maximum allowable vertical speed
    MOVE_SPEED = 0.5     # Constant forward movement speed

    HORIZ_TOL = 0.075    # Tolerance for horizontal centering
    VERT_TOL = 0.04      # Tolerance for vertical centering
    VERTICAL_OFFSET = -0.2  # Offset to account for camera mounting position

    """
    Initializes the ImageAnalyzer class.
    Loads HSV thresholds and sets up initial flags and buffer.

    Args:
        color_profile (str): The tag in the XML file to determine HSV thresholds.
        rotate_on_loss (bool): Whether the drone should rotate to search for target when it's lost.
    """
    def __init__(self, color_profile="default", rotate_on_loss=False):
        self.rotate_on_loss = rotate_on_loss
        self.horiz_centered = False
        self.vert_centered = False
        self.target_visible = False
        self.recent_points = deque(maxlen=10)
        self.hsv_profiles = [profile for profile in self._load_hsv_thresholds(color_profile)]

    """
    Checks if the object (blob) is visible in the current frame.

    Args:
        frame (ndarray): The input video frame.
        show (bool): Whether to display the processed frame.

    Returns:
        bool: True if the object is detected, False otherwise.
    """
    def blob_present(self, frame, show=False):
        self._detect_blob(frame, show=show)
        return self.target_visible

    """
    Calculates motion to align the drone vertically with the detected object.

    Args:
        frame (ndarray): The current frame from the video feed.
        show (bool): Whether to show debug visual output.
        stop_when_aligned (bool): If True, stops movement once aligned.

    Returns:
        tuple: A MotionCommand object and a boolean indicating whether the drone should stop.
    """
    def adjust_vertical(self, frame, show=False, stop_when_aligned=True):
        self._detect_blob(frame, show=show)
        if not stop_when_aligned:                       #only really used in test image to constantly get vertical measurements
            self.vert_centered = False
        if not self.vert_centered:
            return self._track_blob(horizontal=False)
        return self._halt()

    """
    Aligns horizontally and issues forward motion if the alignment is complete.

    Args:
        frame (ndarray): Current frame from the video feed.
        show (bool): Whether to display the frame for debugging.
        move (bool): Whether to allow movement forward if aligned.

    Returns:
        tuple: MotionCommand and boolean (False).
    """
    def align_and_move_forward(self, frame, show=False, move=True):
        self._detect_blob(frame, show=show)
        if not move:
            self.horiz_centered = False
        if not self.horiz_centered:
            return self._track_blob(horizontal=True)
        print("Moving forward")
        return self._move_forward()

    """
    Resets the flag indicating horizontal alignment.
    Useful when starting a new alignment phase.
    """
    def reset_alignment(self):
        self.horiz_centered = False

    """
    Core logic to calculate motion commands to center on the target.

    Args:
        horizontal (bool): Whether the adjustment is for horizontal or vertical alignment.
        modify_state (bool): If True, updates internal state flags.
        record_cmd (bool): If True, saves command and timestamp for future reference.

    Returns:
        tuple: MotionCommand and boolean False (indicating continuous motion).
        
    Explanation:
        If we did not find the blob, then rotate to look for it
        Calcualtes the error the mask is from the center
        Then multiplies the error by the max speed and velocity instiated from the top
        The drone will only move vertically or with yaw one at a time
        Checks to see if the error is within the tolerance, if so then stop
        If record_cmd is high then it will be saved with timestamp for later info
        
    """
    def _track_blob(self, horizontal, modify_state=True, record_cmd=True):
        if not self.target_visible:
            print("searching")
            return self._rotate_search()
        dx, dy = self._get_relative_position()
        
        vertical_speed = 0 if horizontal else self.MAX_VSPEED * dy
        yaw_speed = self.MAX_YAW * dx if horizontal else 0
        cmd = MotionCommand(0, vertical_speed, yaw_speed)

        if modify_state:
            if horizontal:
                self.horiz_centered = abs(dx) < self.HORIZ_TOL
            else:
                if abs(dy) < self.VERT_TOL:
                    self.vert_centered = True
                    return self._halt()
                self.vert_centered = False

        if record_cmd:
            self.last_cmd = cmd
            self.last_cmd_time = time.time()

        return cmd, False

    """
    Commands the drone to move forward while continuing to face the object.
    If object is lost, continues old command briefly then halts.

    Returns:
        tuple: MotionCommand and a boolean False.
        
    Explanation:
        If there is no target, do the last command, if the blob is lost for more than a second, realign horizontally
        If the blob is visible, call the horizontal adjustment, while also adding a forward movement
        Saves the command if we need to keep moving forward
    """
    def _move_forward(self):
        if not self.target_visible:
            if self.last_cmd_time is None or (time.time() - self.last_cmd_time <= 1.0):
                return self._repeat_last_cmd()
            self.horiz_centered = False
            return self._halt()

        cmd, _ = self._track_blob(horizontal=True, modify_state=False, record_cmd=False)
        cmd.forward = self.MOVE_SPEED
        self.last_cmd = cmd
        self.last_cmd_time = time.time()
        return cmd, False

    """
    Issues a yaw command if the target is lost and rotation is allowed.

    Returns:
        tuple: MotionCommand and boolean False.
    """
    def _rotate_search(self):
        return (MotionCommand(0, 0, -1.0) if self.rotate_on_loss else MotionCommand(0, 0, 0)), False

    """
    Returns the last issued motion command (used as fallback).

    Returns:
        tuple: Last MotionCommand and boolean False.
    """
    def _repeat_last_cmd(self):
        return self.last_cmd, False

    """
    Returns a zero-motion command to stop all movement.

    Returns:
        tuple: Zero-motion MotionCommand and boolean True.
    """
    def _halt(self):
        return MotionCommand(0, 0, 0), True

    """
    Analyzes the input frame for a valid colored blob based on HSV thresholds.

    Args:
        frame (ndarray): Raw camera frame.
        show (bool): Whether to visualize the detection process.
     
    Explanation: 
        Rescales the frame to 600 pixels for consistency and stores the current frame into __frame
        Applyes an 11x11 gaussian blur converts the blur from BGR to HSV 
        Reads the center pixel/prints it out 
        Calls mask function to get mask
        Expands white areas and gets rid of noise
        Finds all contours (white) in the mask and find which is the biggest one (that should be target)
        Calculates the center of the contour and makes a circle with a center dot around the contour
        Saves middle x,y corrdinate and returns true
        
    """
    def _detect_blob(self, frame, show=False):
        self.target_visible = False
        frame = imutils.resize(frame, width=600)
        self._frame = frame

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        b, g, r = frame[frame.shape[0] // 2, frame.shape[1] // 2]
        center_hsv = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)[0][0]
        print(f"Center HSV: {center_hsv}")

        mask = self._build_mask(hsv_frame)
        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.erode(mask, None, iterations=1)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        if cnts:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if radius > 5:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    self.target_visible = True
                    self.blob_x = x
                    self.blob_y = y

    """
    Computes the normalized position of the detected blob relative to the center of the frame.

    Returns:
        tuple: dx, dy as normalized displacements along X and Y.
    
    Explanation: 
        __frame.shape returns [height, width, channels (3 for bgr)]
        Divides the rows and cols by two to get center bit
        Then calculates the error of the mask center coordinates from __detect_blob__
    """
    def _get_relative_position(self):
        rows = float(self._frame.shape[0])
        cols = float(self._frame.shape[1])
        cx, cy = 0.5 * cols, 0.5 * rows
        return (self.blob_x - cx) / cx, (self.blob_y - cy) / cy

    """
    Combines all HSV threshold ranges into a single binary mask.

    Args:
        hsv_img (ndarray): HSV-transformed image.

    Returns:
        ndarray: Binary image mask.
    
    Explanation: 
        Get the current image and mask it out based on the threshold values from the XML file
        Result= mask[0] gets the first mask as a strting image
        The the loops goes over every other mask in the list
        Then the add statement bit-wise addition each pixel of the mask and result (meaning that 
        if white in either mask then it stays white)
    
    """
    def _build_mask(self, hsv_img):
        masks = [cv2.inRange(hsv_img, tuple(h[:3]), tuple(h[3:])) for h in self.hsv_profiles]
        result = masks[0]
        for m in masks[1:]:
            result = cv2.add(result, m)
        return result

    """
    Loads HSV color threshold data from an XML file.

    Args:
        tag (str): XML node name to read.

    Returns:
        list: List of HSV value ranges.
        
    Explanation:
        Based on tag, return a list of [Hmin, Smin, Vmin, Hmax, Smax, Vmax]
    """
    def _load_hsv_thresholds(self, tag):
        xml_file = Path(__file__).parent / 'thresholds.xml'
        tree = ET.parse(str(xml_file))
        root = tree.getroot()
        node = root.find(tag)
        return [[int(val.text) for val in item] for item in list(node)]

"""
Simple data structure for storing motion commands issued to the drone.
Each command includes forward, vertical, and yaw (rotation) components.
"""
class MotionCommand:
    def __init__(self, forward, vertical, yaw):
        self.forward = forward
        self.vertical = vertical
        self.yaw = yaw
