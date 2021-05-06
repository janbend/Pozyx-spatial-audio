# processing configuration

## processing prerequisite (tested V3.54)
- include controlP5, oscP5, netP5 libary with processing libary manager
- include gwoptics.jar libary manually by copying into processing core folder

## required code modifications
- define your pozyx remote id of remote tag, e.g. 0x6029
- define number of your pozyx anchor and num_anchors, e.g. 0x604f, 0x6009c, 0x605c, 0x6078 
- define x- and y-coordinates of your pozyx anchor

## setup
- verify the code and check for missing libaries or other issues
- sucessful, if green and red led's of your pozyx tag start blinking
- (test the official pozyx ready to localize tutorial first)

