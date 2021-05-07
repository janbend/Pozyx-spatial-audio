# arduino configuration

## arduino prerequisite
- include pozyx (tested V1.1.3) libary with arduino libary manager
- select board: arduino / genuino uno
- select port: usb com port of the arduino with workstation tag, e.g. COM3

## required code modifications
- define your pozyx remote id of remote tag, e.g. 0x6029
- define number of your pozyx anchor and num_anchors, e.g. 0x604f, 0x6009c, 0x605c, 0x6078 
- define x- and y-coordinates of your pozyx anchor

## setup
- verify the code and check for missing libaries or other issues
- successful, if green and red led's of your pozyx tag start blinking
- (otherwise test the official pozyx ready to localize tutorial)
