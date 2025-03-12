# MotorStatus.msg
bool[8] state # [0,1,2,3] are driving and [4,5,6,7] are steering

float64[8] current
float64[8] average_current

float64[4] position # steering absolute encoders

float64[4] velocity # driving velocity

bool[8] fault_state