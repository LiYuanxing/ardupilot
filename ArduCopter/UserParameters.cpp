#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_RES", 0, UserParameters, _int8, 0),
    AP_GROUPINFO("_ALT_m", 1, UserParameters, _int16, 30),
    AP_GROUPINFO("_THR_%", 2, UserParameters, _float, 0.2),
    
    AP_GROUPEND
};
