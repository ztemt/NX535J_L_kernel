#ifndef _CUST_SETTING_H_
#define _CUST_SETTING_H_
/******************************************************************************
 * Charging Parameter Define.
 *****************************************************************************/
#define USB_INVALID_CHARGER_CURRENT			100
#define USB_SDP_LIMIT_MAX_CURRENT			500
#define USB_DCP_LIMIT_MAX_CURRENT			2000
#define USB_HVDCP_LIMIT_MAX_CURRENT         2000
#define USB_MAX_CHARGE_CURRENT				3500

#define PARALLEL_LIMT_CURRENT_PERCENTAGE    50
#define PARALLEL_AICL_CURRENT_PERCENTAGE    50

#define BATTERY_RECHAEGE_THRESHOLD			4250
#define BATTERY_FULL_TYPICAL_CAPACITY		5000
#define BATTERY_FULL_DESIGN_CAPACITY		4900000

/** 0.1 degree   eg:470 = 47C */
#define CHG_TEMP_MAX    	                700
#define CHG_TEMP_HOT    	                500
#define CHG_TEMP_WARM   	                450
#define CHG_TEMP_GOOD   	                230
#define CHG_TEMP_COOL   	                100
#define CHG_TEMP_COLD   	                -50
#define CHG_TEMP_MIN   		                -200

#define BATT_COLD_CURRENT                   0
#define BATT_GOOD_CURRENT                   3500
#define BATT_COOL_CURRENT                   1200
#define BATT_WARM_CURRENT                   1200
#define BATT_HOT_CURRENT                    0
#endif