# GitHub Repo for ECE 492/592 (A2) True Love Project

This repository includes all documents necessary in final project deliverables including the final report and short video. 

#### Team Members
Jennifer Jimenez

Kaurwaki Babu

Michael Felt 

The following information is located in the ./bin folder, but reiterated below for easy access. 

## Dependencies for ./src code

### Pi Environment
#### For Servo GPIO configuration
- `pip3 install gpiozero`
- `pip3 install RPi.GPIO`
  
#### For Image Processing 
- `pip3 install opencv-python`
- `pip3 install imutils`
- `sudo apt-get install libatlas-base-dev`
- `pip3 install -U numpy`

#### For Drone Communication 
- `pip3 install dronekit`
- `pip3 install pymavlink`

It is helpful to have the above packages downloaded onto your GCS as well. `python3 -m venv [path]/venv` will create a folder labeled 'venv' in which packages can be downloaded without impacting your entire system. 
Make sure to run `source [path]/venv/bin/activate` to activate the environment and be able to use the desired packages. 

### GCS Environment
- `pip3 install dronekit`
- `pip3 install pymavlink`

<i> Make sure to import files when using classes, functions from different files </i> 
