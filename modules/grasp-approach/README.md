# grasp-approach
Glue code for the mobile manipulation demo.

### Dependencies for building
- [Yarp](https://github.com/robotology/yarp)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)

### Dependencies for running the demo
- [navigation](https://github.com/robotology/navigation)
- [cardinal-points-grasp](https://github.com/robotology/cardinal-points-grasp.git): branch `moonshot`
- [localize-superquadrics](https://github.com/robotology/)
- [iol](https://github.com/robotology/iol)
- [cer](https://github.com/robotology/cer)
- [AFAR detection]

### Command-line parameters

- `--verbosity <level>`: set the level of verbosity of the output of the module (2 for extended debug info)

### RPC commands

- `PlanApproachObjectName <objectNameAFAR> <objectNameOPC>`: plan the optimal base pose to grasp the object. `<objectNameAFAR>` is the name of the object in the afar detector database and `<objectNameOPC>` is the object name in the OPC (IOL) database.
- `ApproachObjectName <objectNameAFAR> <objectNameOPC> <distance>`: same as previous command but then navigates towards the optimal base pose until it is reached or until it is at less than `<distance>` from the object. The robot then looks at the object after it stops.
- `ContinuousApproachObjectName <objectNameAFAR> <objectNameOPC> <step>`: perform optimal base pose planning, navigation for a distance of `<step>` and then look at the object repeatively until the optimal base pose can be reached.
- `PlanApproachObjectPosition <x> <y>,<z> <frame>`: same as `PlanApproachObjectName`, but the object is defined by its 3D position `(<x>, <y>, <z>)` instead of its name. The 3D position can be expressed in the robot frame, setting `<frame>="local"`, or in the map frame by setting `<frame>="map"`. (used for debug only)
- `ApproachObjectPosition <x> <y> <z> <distance> <frame>`: same as `PlanApproachObjectPosition`, but then navigates towards the optimal base pose until it is reached or until it is at less than `<distance>` from the object. The robot then looks at the object after it stops. (used for debug only)

### Steps for running the demo

1) start the navigation pipeline (for localization and navigation)

- on `r1-base`:
  - run `roscore`
  - run `yarp server --ros`
  - run `yarprobotinterface`
  - run `roslaunch robotStatePublisher.launch` from `/usr/local/src/robot/cer/app/robots/CER02`
  - run `roslaunch amcl_map_from_yarp.launch` from `/usr/local/src/robot/cer/app/cerRos/launch/localization`

- on `r1-console-linux` (IITICUBLAP121)
  - check that the kitchen area is defined properly in `/usr/local/src/robot/carve-scenarios-config/build/share/carve-scenarios-config/contexts/sanquirico/locations.ini` (if not, see `r1-grasping/modules/grasp-approach/app/conf/location.ini`)
  - run script `CARVE_R1_Navigation` in yarpmanager
  - run modules one by one + connect (may require to manually start some `yarprun` if yarpmanager cannot do it properly)
  - run `rviz`
  - load `/usr/local/src/robot/cer/app/cerRos/rviz/amcl_yarp.rviz`
  - check if no navigation module crashed (if yes, stop all yarpdev and rerun them one by one + connect)
  - change `Goto` into `Localize` in navigationGUI + initialize robot pose (map should then appear in rviz)
  - initialize robot pose in rviz (button 2D Pose Estimate)
  - move the robot in the room until good convergence of the localization (with joystick)

2) start the AFAR detection pipeline (for afar detection)

- on `r1-console-cuda` (IITICUBLAP122):
  - launch the Windows virtual machine
  - run the `yarprun` script

- on `r1-console-linux` (IITICUBLAP121)
  - check that line 59 in `/usr/local/src/robot/iCubContrib/bin/detection_speech_helper.lua` is uncommented (it should contain "Domino")
  - check that file `/home/r1-user/.local/share/yarp/contexts/yarp-augmentation/config_640x480.ini` exists (if not, see `r1-grasping/modules/grasp-approach/app/conf/config_640x480.ini`)
  - run script `05 - Online Detection Afar 640x480` in yarpmanager (or `r1-grasping/modules/grasp-approach/app/scripts/demo-mobile-manipulation_detection_AFAR_640x480.xml`)
  - run modules + connect

- on r1-console-cuda (IITICUBLAP122):
  - check that height h and width w are set to 640 and 480 in `/usr/local/src/robot/onlineDetection/matlab-online-detection/Conf/yarp_initialization.m`
  - run `matlab -nodesktop` from `/usr/local/src/robot/onlineDetection/matlab-online-detection`
  - run Detection_main` in Matlab
  - connect everything in yarpmanager (on `r1-console-linux`)
  - run `yarp write ... /detection/command:i` + write `load dataset jason_data_domino2.mat` (or retrain the robot detection from scratch) 

3) start the IOL pipeline (for near detection and final grasping)

- on r1-console-cuda (IITICUBLAP122):
  - run `./kickstart.sh` from `cd ~/virtualenv_projects/Mask_RCNN/samples/tabletop`
    
- on r1-console-linux (IITICUBLAP121):
  - run script `Demo mobile manipulation - IOL` in yarpmanager (or `r1-grasping/modules/grasp-approach/app/scripts/demo-mobile-manipulation_IOL.xml`)
  - run modules + connect
  - run `yarp rpc /iolStateMahineHandler/human:rpc` + `>>attention stop`

4) start the base pose planner pipeline (for object approach phase)

- on r1-console-linux (IITICUBLAP121):
  - run script `Demo mobile manipulation` in yarpmanager (or `r1-grasping/modules/grasp-approach/app/scripts/demo-mobile-manipulation.xml`)
  - run modules + connect

5) Run the demo:

 - lift the right arm: run `yarp rpc /action-gateway/cmd:io` + `>>home right`
 - perform the approach phase: run `yarp rpc /grasp-approach/rpc:i` + run the adequate command (see list of available commands above, typically`>>ApproachObjectName Domino Domino 0.0`)
 - perform the final grasping: run `yarp rpc /graspProcessor/cmd:rpc` + `>>grasp Domino right`

