# pozyx-spatial-audio
## what it's all about

- software to create an positiondynamic spatial audio eviornment with pozyx
- additional features: control for digital signage players or effect lights via osc

## system requirements

- hardware: pozyx creator / developer kit, arduino uno
- software: arduino ide, processing ide (recommended 3.54)
- spatial audio rendering: daw: reaper + vst's: dear vr pro, iem plugin suite (tu graz), sparta (aalto university)
- limitations: overall delay ~75ms - blocksize 256 samples (asio4all)
- (could be lowered to ~45ms with pozyx uwb_only tracking algorithm -> more positioning jitter)

## system setup
