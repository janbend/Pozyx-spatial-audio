# pozyx-spatial-audio

## idea

- create 6dof spatial audio enviornment with pozyx tracking and digital audio workstation
- additional control for digital signage players or effect lighting via osc

## system requirements

- hardware: pozyx creator / developer kit, arduino uno & headphones
- software: arduino ide & processing ide (recommended 3.54)
- digital audio workstation: reaper (recommended, osc interface)
- spatial audio vst plugins: dear vr (pro), iem plugin suite (tu graz) or sparta (aalto university)
- limitations: overall delay ~75ms - blocksize 256 samples (asio4all)
- (could be lowered to ~45ms with pozyx uwb_only tracking algorithm -> more jitter in positioning)

## software dependencies

- libaries for processing: controlP5, oscP5, netP5, gwoptics
- guidance provided in processing repository folder

## images

- signal flow shematic of pozyx tracking, workstation, processing, daw, vst's and listener

![Screenshot](ressources/images/systemflow.png "system flow")

- graphical shematic of the system setup with pozyx tags and anchors

![Screenshot](ressources/images/systemsetup.png "system setup")

- processing gui to control signal flow and calibrate the pozyx system

![Screenshot](ressources/images/processing.png "processing gui")

- pozyx remote tag with lipo mounted on headphones and workstation tag 

![Screenshot](ressources/images/pozyx_tags.png "pozyx tags")


