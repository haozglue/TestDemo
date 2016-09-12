#define CFG_MODULE_ACCEL_DRIVER 3630

#if CFG_MODULE_ACCEL_DRIVER == 3610
#include "m_drv_mc3610.h"
#else
#include "m_drv_mc3630.h"
#endif