### Demo Command Line Code
You will need two windows open in terminal

`cd [path]/ardupilot/ArduCopter`

`sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14552`

Run script

`python3.10 [file_name].py --connect udp:127.0.0.1:14552`

  STABILIZE> mode guided

  <b> Wait for professor to arm in field </b>



### Gazebo Testing 

`cd [path]ardupilot_gazebo`

`gz sim -v4 -r [world]/[sdf]`

### VRone Testing 

`python3.10 [file_name].py --connect udp:127.0.0.1:14552`

  STABILIZE> mode guided

  <b> Only in virtual testing </b>
  
  GUIDED> arm throttle
  GUIDED> takeoff 5
