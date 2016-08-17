#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
//#include <mach/mt6577_pm_ldo.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 4,
    .polling_mode_ps =0,
    .polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x72, 0x48, 0x78, 0x00},
    /*Lenovo-sw chenlj2 add 2011-06-03,modify parameter below two lines*/
    //.als_level  = { 4, 40,  80,   120,   160, 250,  400, 800, 1200,  1600, 2000, 3000, 5000, 10000, 65535},
    //.als_value  = {10, 20,20,  120, 120, 280,  280,  280, 1600,  1600,  1600,  6000,  6000, 9000,  10240, 10240},
    .als_level  = { 5, 10,  25,   50,  100, 150,  200, 400, 1000,  1500, 2000, 3000, 5000, 8000, 10000},
    .als_value  = {10, 50,  100,  150, 200, 250,  280,  280, 1600,  1600,  1600,  6000,  6000, 9000,  10240, 10240},
    /* MTK: modified to support AAL */
    //.als_level  = { 6, 9, 17, 38, 56, 74, 116, 342, 778, 1082,  1386, 1914, 3000, 5000, 8000 },
    //.als_value  = {136, 218, 312, 730, 1065, 1400, 2250, 4286, 5745, 7390, 9034, 11000, 12250, 12250, 12250, 12250},
    //.als_level  = {2*100, 30*100, 60*100, 90*100, 155*100, 230*100, 640*100, 855*100, 1045*100, 1340*100, 1805*100, 2100*100, 2850*100, 3820*100, 6600*100, 10600*100, 12600*100, 16236*100},
    //.als_value  = {5*100, 30*100, 60*100, 90*100, 150*100, 200*100, 350*100,  450*100, 550*100, 720*100, 950*100, 1100*100, 1500*100, 2000*100, 3500*100, 5600*100, 6700*100, 8700*100},
    .ps_threshold_high = 600,
    .ps_threshold_low = 400,
    .ps_threshold = 600,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}
int APDS9930_CMM_PPCOUNT_VALUE = 0x07;//0x06;//0x08;
int APDS9930_CMM_CONTROL_VALUE = 0x68;//0x68;//0xe4;
int ZOOM_TIME = 4;
