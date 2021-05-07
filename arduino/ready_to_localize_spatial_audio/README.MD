# arduino configuration

## prerequisite
- include pozyx libary (tested V1.1.3) with arduino libary manager
- select board: arduino / genuino uno
- select port: com port of the arduino with pozyx workstation tag, e.g. COM8

## required code modifications
- define your pozyx remote id of remote tag, e.g. 0x6029
- define number of your pozyx anchor and num_anchors, e.g. 0x604f, 0x6009c, 0x605c, 0x6078 
- define x- and y-coordinates of your pozyx anchor

## setup
- verify the code and check for missing libaries or other issues in arduino ide
- successful patch, if green and red led's of your pozyx workstation tag start blinking
- otherwise, test the official [pozyx ready to localize tutorial](https://github.com/pozyxLabs/Pozyx-processing) (tested V1.0.1)
