sawIntuitiveResearchKit
=======================

# Bilateral Controller

## dependecies

The only dependency is the python package `onnxruntime` which you can install in a virtual environment using:

`pip install onnxruntime`

This is used to efficiently run the deep learning model for contact detection. 

## setup

This forked repo contains all of the code used to run my bilateral controller. In your dVRK workspace, rm -rf sawIntuitiveResearchKit dir in your ros2 workspace under src/cisst-saw, cp this repo in your ros2 workspace, then colcon build. Inside of the bilateral controller folder
ros/examples/bilateralTeleop, move the config files and study_gui.py script in the share folder into your dVRK config folder. For this application to run, set the contact detection model path in MTMR_PSM2_teleop_config.json. 
The contact detection model paths lives in contact_detection/model/deployable_models/ . The model used in the paper lives in 

## run
To run the bilateral controller in the embedded python interpreter, run this command: 

`ros2 run dvrk_robot dvrk_system -j system-MTMR-PSM2-bilateral-teleop.json -m manager-MTMR-PSM2-bilateral-ros.json -K -C -p 0.001 -e IRE_IPYTHON`

To run the GUI to switch between haptic modes run `exec(open('study_gui.py').read())`.

## haptic modes
- UNI: Unilateral Mode -> no feedback
- BIL_OFF: Bilateral Controller (with disturbance observer OFF) -> continuous feedback
- BIL_ON: Bilateral Controller (with disturbance observer ON) -> continuous feedback
- CSB: Contact-Switched-Bilateral (switching between unilateral and bilateral based on contact detection model) -> gated feedback


## configuration
The default configuration for this controller:

- Teleop Config: MTMR-PSM2
- Position Scaling: 1
- Cf_{translation} : 1
- Cf_{wrist} : 0



Libraries and applications for the da Vinci Research Kit

Links
=====

 * Documentation: http://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/
 * [Code of conduct](CODE_OF_CONDUCT.md)
 
Dependencies
============
 * da Vinci Research Kit hardware: http://research.intusurg.com/dvrkwiki
 * Linux only
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * sawRobotIO1394: http://github.com/jhu-saw/sawRobotIO1394
 * sawControllers: http://github.com/jhu-saw/sawControllers
