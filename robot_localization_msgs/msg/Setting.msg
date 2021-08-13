##camera setting
int32[] FieldHSV
int32[] LineHSV

##particle filter setting
#noise
float32[] ForwardNoise
float32[] BackwardNoise
float32[] LeftNoise
float32[] RightNoise
float32[] LeftTurnNoise
float32[] RightTurnNoise
float32[] ExpandedLeftNoise
float32[] ExpandedRightNoise
float32[] CenteredLeftNoise
float32[] CenteredRightNoise

float32 SenseNoise

#etc
float32[] InitRangeX
float32[] InitRangeY
float32[] InitRangeW

float32[] KidnapRangeX
float32[] KidnapRangeY
float32[] KidnapRangeW

float32 NoSamplingSigma
float32 ReinitWeight
int32 NumberofParticle
float32 IntervalAngle
float32 Unit
