# processing configuration

![Screenshot](ressources/images/processing.png "processing gui")

## prerequisite
- download processing ide (tested V3.54)
- include controlP5, oscP5, netP5 libary with processing libary manager
- include gwoptics.jar libary manually by copying into processing core folder
- (more details in dependencies.txt)

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

## vst plugin
- add vst to effects (FX) of selected channel (e.g. master bus)
- navigate to MIDI/OSC learn in parameter list (Param -> FX paramater list -> Learn -> Select paramater)
- to bind specific OSC message to MIDI paramater you have to deactivate all others in processing gui (otherwise overflow)
- (guidance on how to setup an spatial audio scene in reaper can be requested)

## osc commands
Parameter | Index | VST Plugin
--- | --- | ---
Azimuth | /imu_az/ | Dear VR Polar - Azimuth
Elevation | /imu_el/ | Dear VR Polar - Elevation
Position X | /dear_x/ | Dear VR Cartesian - X
Position Z | /dear_z/ | Dear VR Cartesian - Z
Yaw| /imu_x/ | Ambix / IEM / Sparta - Yaw
Yaw (inv.) | /imu_x_inv/ | Ambix / IEM / Sparta - Yaw inv.
Roll | /imu_y/ | Ambix / IEM / Sparta - Roll
Pitch | /imu_z/ | Ambix / IEM / Sparta - Pitch
Position X | /pos_x/ | IEM Room Encoder - X
Position Y | /pos_y/ | IEM Room Encoder - Y
Position Z | /pos_z/ | IEM Room Encoder - Z
Room Zone 1 | /zone1/ | Volume of Audio Scene 1
Room Zone 2 | /zone2/ | Volume of Audio Scene 2
Room Zone 3 | /zone3/ | Volume of Audio Scene 3
Room Zone 4 | /zone4/ | Volume of Audio Scene 4
Room Zone 1 | /marker_zone1/ | Trigger of Audio Scene 1 (tbd)
Room Zone 2 | /marker_zone2/ | Trigger of Audio Scene 2 (tbd)
Room Zone 3 | /marker_zone3/ | Trigger of Audio Scene 3 (tbd)
Room Zone 4 | /marker_zone4/ | Trigger of Audio Scene 4 (tbd)
