# This msg file is a copy of 'grSim_Commands.proto'
# https://github.com/RoboCup-SSL/grSim/blob/master/src/proto/grSim_Commands.proto
# wheelsspeed はonにする。
# wheel1, wheel2, wheel3, wheel4で進行方向のベクトル成分を決めれる。

uint32 id
float32 kickspeedx
float32 kickspeedz
float32 veltangent
float32 velnormal
float32 velangular
bool spinner
bool wheelsspeed
float32[<=1] wheel1  # optional
float32[<=1] wheel2  # optional
float32[<=1] wheel3  # optional
float32[<=1] wheel4  # optional
