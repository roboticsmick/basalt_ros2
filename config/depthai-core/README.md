# DEPTHAI-CORE CALIBRATION

The depthai-ros scripts publish the OAK-FFC-3P camera calibration parameters as a ROS2 topic. To calibrate the cameras and flash the parameters to the camera module follow the steps here: [DepthAI V3 Camera Calibration for the OAK-FFC-3P](https://github.com/roboticsmick/calibration_depthai_v3)

Note: The calibration generally fails on the first attempt calibrating directly from the images. 

```bash
python3 calibrate_depthai_v3.py -s 4.0 -brd OAK-FFC-3P-HQ83.json -nx 17 -ny 9 --ssh-preview
```

Rerunning it on the captured images seems to succeed.

```bash
python3 calibrate_depthai_v3.py -s 4.0 -brd OAK-FFC-3P-HQ83.json -m process -nx 17 -ny 9
```