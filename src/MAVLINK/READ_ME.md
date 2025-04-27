### About These Files

These files use functions in the main ./src to alongside MAVLINK commands to communicate with the drone. 

`drop_off_only.py` prompts the drone to take off, move to the drop-off location, and move the servo into the down/release position. 

`field_centering.py` prompts the drone to scan the field and center itself on an object of a certain color (in our case red).

`with_lidar&centering.py` builds on the field_centering code to move forward until the table with the letter is detected using lidar, after which the servo motor is moved to the pickup position. After pick up, the drone navigates to a different location for drop off, lowers, and moves the servo into drop off position, before RTL. 
