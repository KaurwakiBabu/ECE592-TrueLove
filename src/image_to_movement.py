"""
Provides high-level control for drone movement using vision input.
Handles tasks such as seeking, centering, and commanding drone velocities
based on blob tracking from the onboard camera.
"""

from pymavlink import mavutil
from imutils.video import VideoStream
from image_analyzer import ImageAnalyzer
import time

class VisualAligner:

    DEFAULT_VELOCITY = 2          #for sideways movement

    def __init__(self, vehicle):
        """Initializes the controller with a vehicle and a video stream."""
        self.vehicle = vehicle
        self.analyzer = ImageAnalyzer(color_profile="default", rotate_on_loss=False)
        self.cap = VideoStream(src=0).start()

    def __del__(self):
        """Stops the video stream and closes vehicle connection on cleanup."""
        print("Exiting")
        self.vehicle.close()
        self.cap.stop()

         """
        Aligns the drone to the blob either vertically or horizontally and commands movement.

        Args:
            horizontal (bool): Whether to align horizontally (True) or vertically (False).
            stop (bool): If True, loop ends when alignment is achieved.
            show (bool): If True, video output is shown.
            advance (bool): If True and aligning horizontally, drone moves forward.

        Returns:
            bool: True if alignment is successful
            
        Explanation:
            Calls horizontal and vertical alignment and sends cmd to MAVLINK to align it self
            Once aligned, exit the loop
        """ 
    def center_on_target(self, horizontal, stop=True, show=False, advance=True):

        aligned = False
        while True:
            frame = self.cap.read()
            if frame is None:
                print("No frame received.")
                break

            if horizontal:
                cmd, finished = self.analyzer.align_and_move_forward(frame, show=show, move=advance)
            else:
                cmd, finished = self.analyzer.adjust_vertical(frame, show=show, stop_when_aligned=stop)

            self.vehicle.send_mavlink(self._velocity_command(cmd))

            if stop and finished:
                aligned = True
                break

        self.analyzer.reset_alignment()
        return aligned
        """
        Explanation: 
            Loops over 10 frames
            If any of the 10 frames has the blob in it, then return true, else false
        """
    def _check_multiple_frames(self, show=False):
        """Checks several frames in a row to see if the blob is detected."""
        for _ in range(10):
            frame = self.cap.read()
            if frame is None:
                print("No frame received.")
                return True
            if self.analyzer.blob_present(frame, show):
                print("Target found.")
                return True
        return False
        
        
        
        
        
        
        """
        Explanation:
            Checks 10 frames, if there then exit, else slide right 1 meter
            Checks 10 frames, again, if not there then slide left 1 meter
            Same stuff in this order: Right Left Left Right
            
        """    
    def translate_seek(self, show=False):
        """Slides left and right while checking for the presence of a target blob."""
        while True:
            if self._check_multiple_frames(show): return

            self._slide(1, 1, 1, show)
            
            if self._check_multiple_frames(show): return 
            
            self._slide(1, 1, -1, show)
            
            if self._check_multiple_frames(show): return 
            
            self._slide(1, 1, -1, show)
            
            if self._check_multiple_frames(show): return  

            self._slide(1, 1, 1, show)
            
            if self._check_multiple_frames(show): return
        
        
        
        """
        Slides the drone laterally.

        Args:
            repeats (int): Number of slide steps.
            distance (float): Distance per step.
            direction (int): 1 for right, -1 for left.
        """
    def _slide(self, repeats, distance, direction, show=False):

        for _ in range(repeats):
            msg = self._sideways_command(distance, self.DEFAULT_VELOCITY * direction)
            self.vehicle.send_mavlink(msg)
            print("Sliding...")
            time.sleep(10)
            
            
            
       """
        Converts a MotionCommand into a MAVLink velocity message.

        Args:
            cmd (MotionCommand): Contains forward, vertical, and yaw rate.

        Returns:
            MAVLink message: Velocity command.
        """
    def _velocity_command(self, cmd):
 
        return self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b010111000111,
            0, 0, 0,
            cmd.forward, 0, cmd.vertical,
            0, 0, 0,
            0, cmd.yaw
        )
        
        
        
        """
        Creates a MAVLink command to slide sideways.

        Args:
            distance (float): Distance to slide.
            velocity (float): Lateral velocity.

        Returns:
            MAVLink message: Sideways movement command.
        """
    def _sideways_command(self, distance, velocity):

        return self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b000111000101,
            0, distance, 0,
            0, velocity, 0,
            0, 0, 0,
            0, 0
        )
