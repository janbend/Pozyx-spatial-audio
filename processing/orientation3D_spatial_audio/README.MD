# processing configuration

## prerequisite
- download processing ide (tested V3.54)
- include controlP5, oscP5, netP5 libary with processing libary manager
- include gwoptics.jar libary manually by copying into processing core folder
- (more details in dependencies.txt file)

## required code modifications
- define com port of your pozyx workstation tag (prcoessing = arduino port), e.g COM8
- define local host ip-address (e.g. 127.0.0.1)
- define osc port for osc interface of digital audio workstation
- define room dimension (room_x, room_y) - distance between anchor coordinates in meter

## setup
- verify the code and check for missing libaries or other issues in processing ide
- successful patch, if pozyx remote tag and anchors are visualized and sensor data is presented
- otherwise, test the [official pozyx orientation3D](https://github.com/pozyxLabs/Pozyx-processing)

## digital audio workstation
- download and install reaper
- open a new project and navigate to options -> preferences -> control/OSC/web
- add new control surface -> select control surface mode OSC (Open Soujnd Control)
- define device name and select Device IP/port
- define osc port e.g. 8888 and device ip-address e.g. 127.0.0.1
- allow binding messages to REAPER actions and FX learn
- (guidance on how to setup an spatial audio scene in reaper can be requested)

## vst plugins
- add vst to effects (FX) of selected channel (e.g. master)
- navigate to MIDI/OSC learn in fx parameterlist (Param -> FX paramater list -> Learn -> Action)
- 
