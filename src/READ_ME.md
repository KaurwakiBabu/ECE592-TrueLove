### About Each File 

All documents in this folder (./src) are Python scripts with functions and classes critical to achieving our proposed drone movement. The pythoon scripts on SITL for drone movement, code including MAVLINK commands, are in the MAVLINK folder. 

`image_analyser.py` creates the class ImageAnalyser and includes functions such as `adjust_vertical()` and `_track_blob()`, which allow for the locating and tracking of a colored object in the vision frame. To identify the 'colored' object, HSV thresholds are set in the `thresholds.xml` file. While creating a class for these functions isn't required, it helped keep all functions related to image movement bundled and located in one easy-to-address location. 

`image_to_movement.py` creates the class VisualAligner and includes functions such as `_check_multiple_frames()` and `_velocity_command()`, which oftentimes use functions in `image_analyser.py` to return MAVLINK commands. This function is a stepping stone to the final demo code, as it is an intermediary between opencv and MAVLINK. 

`lidar.py` initializes the rPi3 UART and establishes a connection to the TFLuna Lidar. 

`Manual_threshold_management.py` is directly copied from the imutils library <a href="(https://github.com/jrosebr1/imutils/blob/master/bin/range-detector)">imutils library</a>. 
