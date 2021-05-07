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
- define room dimension (room_x, room_y) - as distance between the anchors

## setup
- verify the code and check for missing libaries or other issues in processing ide
- successful patch, if pozyx remote tag and anchors are visualized
- otherwise, test the [official pozyx orientation3D](https://github.com/pozyxLabs/Pozyx-processing]
