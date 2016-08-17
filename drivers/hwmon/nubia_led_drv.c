/***********************************************************************************/
/* File Name: nubia_led_drv.c */
/* File Description: this file is used to make breathled driver to be added in kernel or module. */

/*  Copyright (c) 2016, NUBIA corporation department 2 section 3 wifi group xiaofeng. All rights reserved.           */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: nubia, Inc.,            */
/*
*/

/***********************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <linux/kthread.h>

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock_types.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
//#include <linux/delay.h>// xiaofeng add for debug
//add timer function kheader for once-breath function begin
#include<linux/module.h>
#include<linux/timer.h>
#include<linux/jiffies.h>
//add timer function kheader for once-breath function end

#include  <../../include/linux/printk.h>
#include <linux/ctype.h>

#include <linux/regulator/consumer.h>

//#include <cust_gpio_usage.h>//xiaofengadd 
#include "mt_pm_ldo.h"   //xiaofeng add for hwPowerOn and its parameter

//#include "upmu_common.h"//xiaofeng add
//#include "pmic.h"
#include "upmu_hw.h"//xiaofeng add for the former commit of expell charging pre-charge indication


#include "nubia_led_drv.h"
#include "mt_typedefs.h"//maybe will encounter compile error
#include"cust_leds.h"
#include"cust_leds_def.h"
#include"leds_drv.h"
#include"leds_hal.h"



//static DEFINE_MUTEX(leds_mutex);
//static DEFINE_MUTEX(leds_pmic_mutex);


extern struct mutex leds_mutex;
extern struct mutex leds_pmic_mutex;
extern struct wake_lock leds_suspend_lock;


static bool aw2013_SUSPEND_FLAG=false; 
//static int blink_mode = 0;

static char last_fade_parameter[10];
static char last_grade_parameter[10];
static char last_outn[10];

int g_breath_time;

int g_breath_once_flag = 1;


static struct timer_list nubia_breathlight_timer;






extern unsigned short pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname, unsigned int val);
extern int platform_device_register(struct platform_device *pdev);
extern int platform_driver_register(struct platform_driver *drv);
extern int led_classdev_register(struct device *parent, struct led_classdev *led_cdev);
extern void led_classdev_unregister(struct led_classdev *led_cdev);
extern int sysfs_create_group(struct kobject *kobj, const struct attribute_group *grp);
extern void sysfs_remove_group(struct kobject * kobj, const struct attribute_group * grp);
extern bool cancel_work_sync(struct work_struct *work);



#define DRV_NAME "class/leds/nubia_led/outn"

#define AW2013_I2C_BUSNUM 1 //xiaofeng

#define xf_ext_flag //xiaofeng add


#define NLED_OFF 0
#define NLED_ON 1

//#define PMIC_CHRIND_EN  1712

#define __NUBIA__REG(a,b,c)  a##b##c
#define NUBIA_REG(a,b,c) __NUBIA__REG(a,b,c)


static  float duty_mapping[16] = {
	0.123, 0.338, 0.523, 0.707, 
	0.926, 1.107, 1.291, 1.507, 
	1.691, 1.876, 2.091, 2.276,
	2.460, 2.676, 2.860, 3.075,
};





enum nubia_outn_mode{
	SW_RESET,	    // 0  soft_reset , all regs revert to default value.
	CONST_ON,	    // 1 work on a constant lightness.
	CONST_OFF,	    // 2 darkness is comming
	AUTO_BREATH, 	// 3 self breathing, used in sences such as missing message.
	STEP_FADE_IN,	// 4  fade in means that the lightness is getting stronger.
	STEP_FADE_OUT,	// 5  fade out means that the lightness is getting weaker
	BREATH_ONCE,     // 6 only breath once, touch the home menu for instance.
	RESERVED,		// 7 reserverd.
};

//according to schematic, isink0-LED_RIGHT, isink1-LED_LEFT, isink2-INDICATOR-R,isink3-INDICATOR-G
enum nubia_breathlight_isink_num{
	LED_RIGHT,
	LED_LEFT,
	INDICATOR_R,
	INDICATOR_G,
};

#define GRADE_PARAM_LEN 20
#define CONST_MIN_GRADE  10
#define CONST_MAX_GRADE  200
#define FADE_PARAM_LEN 20

#define NUBIA_MAX_LOOP 		50




extern int led_classdev_register(struct device *parent, struct led_classdev *led_cdev);
//extern int led_classdev_unregister(struct device *parent, struct led_classdev *led_cdev);//at last ,comment this due to conflict with new added leds.h about statement for this interface





static struct nubia_breathlight_regs_data  nubia_breathlight_regs =
{
	.pmic_isink_ch0_en = MT6351_PMIC_ISINK_CH0_EN,
	.pmic_isink_ch1_en = MT6351_PMIC_ISINK_CH1_EN,
	.pmic_isink_ch4_en = MT6351_PMIC_ISINK_CH4_EN,//xiaofeng modify to 4,5
	.pmic_isink_ch5_en = MT6351_PMIC_ISINK_CH5_EN,
	
	.pmic_rg_drv_32k_ck_pdn = MT6351_PMIC_RG_DRV_32K_CK_PDN,

	//isink0
	.pmic_rg_drv_isink0_ck_pdn = MT6351_PMIC_RG_DRV_ISINK0_CK_PDN,
	.pmic_rg_drv_isink0_ck_cksel = MT6351_PMIC_RG_DRV_ISINK0_CK_CKSEL,
	.pmic_isink_ch0_mode = MT6351_PMIC_ISINK_CH0_MODE,
	.pmic_isink_ch0_step = MT6351_PMIC_ISINK_CH0_STEP,//0-4ma,1-8ma,
	.pmic_isink_breath0_tr1_sel = MT6351_PMIC_ISINK_BREATH0_TR1_SEL,
	.pmic_isink_breath0_tr2_sel = MT6351_PMIC_ISINK_BREATH0_TR2_SEL,
	.pmic_isink_breath0_tf1_sel = MT6351_PMIC_ISINK_BREATH0_TF1_SEL,
	.pmic_isink_breath0_tf2_sel = MT6351_PMIC_ISINK_BREATH0_TF2_SEL,
	.pmic_isink_breath0_ton_sel = MT6351_PMIC_ISINK_BREATH0_TON_SEL,
	.pmic_isink_breath0_toff_sel = MT6351_PMIC_ISINK_BREATH0_TOFF_SEL,
	.pmic_isink_dim0_duty = MT6351_PMIC_ISINK_DIM0_DUTY,
	.pmic_isink_dim0_fsel = MT6351_PMIC_ISINK_DIM0_FSEL,
	.pmic_isink_chop0_en = MT6351_PMIC_ISINK_CHOP0_EN,
	.pmic_isink_ch0_bias_en = MT6351_PMIC_ISINK_CH0_BIAS_EN,
	//int  pmic_isink_ch0_en = MT6351_PMIC_ISINK_CH0_EN; //have above


	//isink1
	.pmic_rg_drv_isink1_ck_pdn = MT6351_PMIC_RG_DRV_ISINK1_CK_PDN,
	.pmic_rg_drv_isink1_ck_cksel = MT6351_PMIC_RG_DRV_ISINK1_CK_CKSEL,
	.pmic_isink_ch1_mode = MT6351_PMIC_ISINK_CH1_MODE,
	.pmic_isink_ch1_step = MT6351_PMIC_ISINK_CH1_STEP,
	.pmic_isink_breath1_tr1_sel = MT6351_PMIC_ISINK_BREATH1_TR1_SEL,
	.pmic_isink_breath1_tr2_sel = MT6351_PMIC_ISINK_BREATH1_TR2_SEL,
	.pmic_isink_breath1_tf1_sel = MT6351_PMIC_ISINK_BREATH1_TF1_SEL,
	.pmic_isink_breath1_tf2_sel = MT6351_PMIC_ISINK_BREATH1_TF2_SEL,
	.pmic_isink_breath1_ton_sel = MT6351_PMIC_ISINK_BREATH1_TON_SEL,
	.pmic_isink_breath1_toff_sel = MT6351_PMIC_ISINK_BREATH1_TOFF_SEL,
	.pmic_isink_dim1_duty = MT6351_PMIC_ISINK_DIM1_DUTY,
	.pmic_isink_dim1_fsel = MT6351_PMIC_ISINK_DIM1_FSEL,
	.pmic_isink_chop1_en = MT6351_PMIC_ISINK_CHOP1_EN,
	.pmic_isink_ch1_bias_en = MT6351_PMIC_ISINK_CH1_BIAS_EN,
	//int  pmic_isink_ch1_en; 	//PMIC_ISINK_CH1_EN //have above


	//isink2//NO,NO,NO,no isink2,maybe isink4
	.pmic_rg_drv_isink4_ck_pdn = MT6351_PMIC_RG_DRV_ISINK4_CK_PDN,
	.pmic_rg_drv_isink4_ck_cksel = MT6351_PMIC_RG_DRV_ISINK4_CK_CKSEL,
	.pmic_isink_ch4_mode = MT6351_PMIC_ISINK_CH4_MODE,
	.pmic_isink_ch4_step = MT6351_PMIC_ISINK_CH4_STEP,
	.pmic_isink_breath4_tr1_sel = MT6351_PMIC_ISINK_BREATH4_TR1_SEL,
	.pmic_isink_breath4_tr2_sel = MT6351_PMIC_ISINK_BREATH4_TR2_SEL,
	.pmic_isink_breath4_tf1_sel = MT6351_PMIC_ISINK_BREATH4_TF1_SEL,
	.pmic_isink_breath4_tf2_sel = MT6351_PMIC_ISINK_BREATH4_TF2_SEL,
	.pmic_isink_breath4_ton_sel = MT6351_PMIC_ISINK_BREATH4_TON_SEL,
	.pmic_isink_breath4_toff_sel = MT6351_PMIC_ISINK_BREATH4_TOFF_SEL,
	.pmic_isink_dim4_duty = MT6351_PMIC_ISINK_DIM4_DUTY,
	.pmic_isink_dim4_fsel = MT6351_PMIC_ISINK_DIM4_FSEL,
	.pmic_isink_chop4_en = MT6351_PMIC_ISINK_CHOP4_EN,
	.pmic_isink_ch4_bias_en = MT6351_PMIC_ISINK_CH4_BIAS_EN,
	//int  pmic_isink_ch2_en; 	//MT6351_PMIC_ISINK_CH2_EN //have above


	//isink3//NO,NO,NO,no isink2,maybe isink5
	.pmic_rg_drv_isink5_ck_pdn = MT6351_PMIC_RG_DRV_ISINK5_CK_PDN,
	.pmic_rg_drv_isink5_ck_cksel = MT6351_PMIC_RG_DRV_ISINK5_CK_CKSEL,
	.pmic_isink_ch5_mode = MT6351_PMIC_ISINK_CH5_MODE,
	.pmic_isink_ch5_step = MT6351_PMIC_ISINK_CH5_STEP,
	.pmic_isink_breath5_tr1_sel = MT6351_PMIC_ISINK_BREATH5_TR1_SEL,
	.pmic_isink_breath5_tr2_sel = MT6351_PMIC_ISINK_BREATH5_TR2_SEL,
	.pmic_isink_breath5_tf1_sel = MT6351_PMIC_ISINK_BREATH5_TF1_SEL,
	.pmic_isink_breath5_tf2_sel = MT6351_PMIC_ISINK_BREATH5_TF2_SEL,
	.pmic_isink_breath5_ton_sel = MT6351_PMIC_ISINK_BREATH5_TON_SEL,
	.pmic_isink_breath5_toff_sel = MT6351_PMIC_ISINK_BREATH5_TOFF_SEL,
	.pmic_isink_dim5_duty = MT6351_PMIC_ISINK_DIM5_DUTY,
	.pmic_isink_dim5_fsel = MT6351_PMIC_ISINK_DIM5_FSEL,
	.pmic_isink_chop5_en = MT6351_PMIC_ISINK_CHOP5_EN,
	.pmic_isink_ch5_bias_en = MT6351_PMIC_ISINK_CH5_BIAS_EN,
	//int  pmic_isink_ch3_en; 	//MT6351_PMIC_ISINK_CH3_EN //have above
};



// for debug issue.
static int debug_mask = 0;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
#define NUBIA_BREATH_DBG(x...) do {if (debug_mask) pr_info("nubia_breathlight  " x); } while (0)

#if 1 
#define LED_DEBUG(fmt,args...)  printk(KERN_ERR"[NUBIA_BREATHLED:%s,%d]" fmt,__func__,__LINE__,##args)
#else
#define LED_DEBUG(fmt,args...)  do {} while (0)
#endif


unsigned char read_reg(struct nubia_breathlight_control_data *led,unsigned char regaddr) 
{

}


static int write_reg(struct nubia_breathlight_control_data *led,unsigned char reg,unsigned char data)
{

}




//#define PMIC_SET_REGISTER_VALUE(num)  
static int nubia_set_pmic_reg_mode_const_on(struct nubia_breathlight_control_data *led, unsigned int n)
{
	int ret = 0;
	//char *num;
	//sprintf(num,"%d",n);
#if 0
	//The following is try to set regs in breath mode.
	//first turn off it
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _en, NLED_OFF);

	pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);
	
	pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink ## num ## _ck_pdn, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink ## num ## _ck_cksel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _mode, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _step, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath ## num ## _tr1_sel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath ## num ## _tr2_sel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath ## num ## _tf1_sel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath ## num ## _tf2_sel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath ## num ## _ton_sel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath ## num ## _toff_sel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim ## num ## _duty, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim ## num ## _fsel, value);
	
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_chop ## num ## _en, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _bias_en, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _en, NLED_ON);
#endif
	
#if 0
	
	if ((button_flag_isink3 == 0) && (first_time == true)) {	/* button flag ==0, means this ISINK is not for button backlight */
		if (button_flag_isink0 == 0)
			pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN, NLED_OFF);
		if (button_flag_isink1 == 0)
			pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN, NLED_OFF);	/* sw workround for sync leds status */
		if (button_flag_isink2 == 0)
			pmic_set_register_value(MT6351_PMIC_ISINK_CH4_EN, NLED_OFF);
		first_time = false;
	}
#endif

	if(LED_RIGHT == n)
	{
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch0_en, NLED_OFF);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink0_ck_pdn, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink0_ck_cksel, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch0_mode, ISINK_PWM_MODE);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch0_step, ISINK_3);

		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim0_duty, 15);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim0_fsel, ISINK_1KHZ);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);//set it second time according leds.c in mtk native impl
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch0_en, NLED_ON);
	}else if(LED_LEFT == n)
	{
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch1_en, NLED_OFF);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink1_ck_pdn, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink1_ck_cksel, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch1_mode, ISINK_PWM_MODE);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch1_step, ISINK_3);

		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim1_duty, 15);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim1_fsel, ISINK_1KHZ);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);//set it second time according leds.c in mtk native impl
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch1_en, NLED_ON);
	}else if(INDICATOR_R == n)
	{
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_en, NLED_OFF);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink4_ck_pdn, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink4_ck_cksel, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_mode, ISINK_PWM_MODE);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_step, ISINK_3);

		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim4_duty, 15);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim4_fsel, ISINK_1KHZ);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);//set it second time according leds.c in mtk native impl
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_en, NLED_ON);
	}else if(INDICATOR_G == n)
	{
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_en, NLED_OFF);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink5_ck_pdn, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink5_ck_cksel, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_mode, ISINK_PWM_MODE);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_step, ISINK_3);

		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim5_duty, 15);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim5_fsel, ISINK_1KHZ);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);//set it second time according leds.c in mtk native impl
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_en, NLED_ON);
	}else
		LED_DEBUG("error parameter n\n");
		
	LED_DEBUG("exit\n");
	return ret;
	
}


static int nubia_set_pmic_reg_mode_const_off(struct nubia_breathlight_control_data *led, unsigned int n)
{
	int ret =0;
	//char *num;
	//sprintf(num,"%d",n);

	//first turn off it.copy from auto_breath mode
#if 0
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _en, NLED_OFF);

	pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);
	
	pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink ## num ## _ck_pdn, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink ## num ## _ck_cksel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _mode, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _step, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath ## num ## _tr1_sel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath ## num ## _tr2_sel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath ## num ## _tf1_sel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath ## num ## _tf2_sel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath ## num ## _ton_sel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath ## num ## _toff_sel, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim ## num ## _duty, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim ## num ## _fsel, value);
	
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_chop ## num ## _en, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _bias_en, value);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _en, NLED_ON);

#endif
	if(LED_RIGHT == n)
	{
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch0_en, NLED_OFF);
	}else if(LED_LEFT == n)
	{
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch1_en, NLED_OFF);

	}else if(INDICATOR_R == n)
	{
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_en, NLED_OFF);

	}else if(INDICATOR_G == n)
	{
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_en, NLED_OFF);

	}else
	{
		LED_DEBUG("error parameter\n");
	}

	pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);
#if 0
	pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink ## num ## _ck_pdn, 0);
	pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink ## num ## _ck_cksel, 0);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _mode, ISINK_PWM_MODE);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _step, ISINK_3);

	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim ## num ## _duty, 15);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim ## num ## _fsel, ISINK_1KHZ);
	
	pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);//set it second time according leds.c in mtk native impl
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch ## num ## _en, NLED_ON);
#endif
	
	LED_DEBUG("exit\n");
	return ret;
	
}


static int nubia_set_pmic_reg_mode_auto_breath(struct nubia_breathlight_control_data *led, unsigned int n)
{
	int ret = 0;
	//char *num;
	//sprintf(num,"%d",n);


	//first turn off it
	if(LED_RIGHT == n)//at the beginning, I use numeric num instead, and I sunk the consequence.
	{
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch0_en, NLED_OFF);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);
		
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink0_ck_pdn, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink0_ck_cksel, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch0_mode, ISINK_BREATH_MODE);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch0_step, ISINK_3);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath0_tr1_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath0_tr2_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath0_tf1_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath0_tf2_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath0_ton_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath0_toff_sel, 0x03);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim0_duty, 15);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim0_fsel, 1999);
		
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_chop0_en, 1);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch0_bias_en, 1);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch0_en, NLED_ON);
	}else if(LED_LEFT == n)
	{
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch1_en, NLED_OFF);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);
		
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink1_ck_pdn, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink1_ck_cksel, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch1_mode, ISINK_BREATH_MODE);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch1_step, ISINK_3);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath1_tr1_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath1_tr2_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath1_tf1_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath1_tf2_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath1_ton_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath1_toff_sel, 0x03);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim1_duty, 15);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim1_fsel, 1999);
		
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_chop1_en, 1);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch1_bias_en, 1);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch1_en, NLED_ON);
	}else if(INDICATOR_R == n)
	{
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_en, NLED_OFF);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);
		
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink4_ck_pdn, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink4_ck_cksel, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_mode, ISINK_BREATH_MODE);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_step, ISINK_3);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath4_tr1_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath4_tr2_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath4_tf1_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath4_tf2_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath4_ton_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath4_toff_sel, 0x03);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim4_duty, 15);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim4_fsel, 1999);
		
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_chop4_en, 1);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_bias_en, 1);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_en, NLED_ON);
	}else if(INDICATOR_G == n)
	{
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_en, NLED_OFF);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);
		
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink5_ck_pdn, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink5_ck_cksel, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_mode, ISINK_BREATH_MODE);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_step, ISINK_3);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath5_tr1_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath5_tr2_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath5_tf1_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath5_tf2_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath5_ton_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath5_toff_sel, 0x01);//try 0x03 -> 0x01
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim5_duty, 15);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim5_fsel, 1999);
		
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_chop5_en, 1);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_bias_en, 1);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_en, NLED_ON);
	}else
	{
		LED_DEBUG("error parameter\n");
	}
	
	LED_DEBUG("exit\n");
	return ret;
	
}


static int nubia_set_pmic_reg_mode_breath_once(struct nubia_breathlight_control_data *led, unsigned int n)
{
	int ret = 0;
	int tall_int;

	float tall;
	float tr1_sel;
	float tr2_sel;
	float tf1_sel;
	float tf2_sel;
	float ton_sel;
	float toff_sel;

	//char *num;
	//sprintf(num,"%d",n);

	if(g_breath_once_flag == 0)
	{
		return 0;
	}else{
		g_breath_once_flag = 0;
	}

	if(n == INDICATOR_R)
	{
		//first turn off it
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_en, NLED_OFF);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);
		
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink4_ck_pdn, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink4_ck_cksel, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_mode, ISINK_BREATH_MODE);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_step, ISINK_3);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath4_tr1_sel, 0x01);//try 0x02 -> 0x01
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath4_tr2_sel, 0x01);//try 0x02 -> 0x01
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath4_tf1_sel, 0x01);//try 0x02 -> 0x01
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath4_tf2_sel, 0x01);//try 0x02 -> 0x01
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath4_ton_sel, 0x01);//try 0x02 -> 0x01
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath4_toff_sel, 0x01);//try 0x03 -> 0x01
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim4_duty, 15);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim4_fsel, 999);//try 1999 -> 999
		
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_chop4_en, 1);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_bias_en, 1);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_en, NLED_ON);

		LED_DEBUG("timing:tr1=%d,tr2=%d,tf1=%d,tf2=%d,ton=%d,toff=%d.\n",
			nubia_breathlight_regs.pmic_isink_breath4_tr1_sel,
				nubia_breathlight_regs.pmic_isink_breath4_tr2_sel,
					nubia_breathlight_regs.pmic_isink_breath4_tf1_sel, 
						nubia_breathlight_regs.pmic_isink_breath4_tf2_sel, 
							nubia_breathlight_regs.pmic_isink_breath4_ton_sel, 
								nubia_breathlight_regs.pmic_isink_breath4_toff_sel);
		

		tr1_sel = duty_mapping[1];
		tr2_sel = duty_mapping[1];
		tf1_sel = duty_mapping[1];
		tf2_sel = duty_mapping[1];
		ton_sel = duty_mapping[1];
		toff_sel = duty_mapping[1] * 2.0;//the toff time is so special according to spec

		tall = tr1_sel + tr2_sel + tf1_sel + tf2_sel + ton_sel + toff_sel;
		tall_int = (int) tall;
		g_breath_time = (int)(tall * HZ);
		//g_breath_time = tall_int;//in order to reduce start-up stime,dont transform tall to tall_int,but transform tall * HZ to int.

		
		LED_DEBUG("g_breath_time = %d\n",g_breath_time);
		nubia_breathlight_timer.expires = jiffies + g_breath_time;//so g_breath_time * HZ should turn to g_breath_time.
		add_timer(&nubia_breathlight_timer);//we add timer here,but where we do del_timer or del_timer_sync?		
	}else if(n == INDICATOR_G)
	{
		//first turn off it
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_en, NLED_OFF);

		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_32k_ck_pdn,0x0);
		
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink5_ck_pdn, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_rg_drv_isink5_ck_cksel, 0);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_mode, ISINK_BREATH_MODE);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_step, ISINK_3);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath5_tr1_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath5_tr2_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath5_tf1_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath5_tf2_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath5_ton_sel, 0x02);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_breath5_toff_sel, 0x03);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim5_duty, 15);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_dim5_fsel, 1999);
		
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_chop5_en, 1);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_bias_en, 1);
		pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_en, NLED_ON);

		LED_DEBUG("timing:tr1=%d,tr2=%d,tf1=%d,tf2=%d,ton=%d,toff=%d.\n",
			nubia_breathlight_regs.pmic_isink_breath5_tr1_sel,
				nubia_breathlight_regs.pmic_isink_breath5_tr2_sel,
					nubia_breathlight_regs.pmic_isink_breath5_tf1_sel, 
						nubia_breathlight_regs.pmic_isink_breath5_tf2_sel, 
							nubia_breathlight_regs.pmic_isink_breath5_ton_sel, 
								nubia_breathlight_regs.pmic_isink_breath5_toff_sel);
		

		tr1_sel = duty_mapping[2];
		tr2_sel = duty_mapping[2];
		tf1_sel = duty_mapping[2];
		tf2_sel = duty_mapping[2];
		ton_sel = duty_mapping[2];
		toff_sel = duty_mapping[3] * 2.0;//the toff time is so special according to spec

		tall = tr1_sel + tr2_sel + tf1_sel + tf2_sel + ton_sel + toff_sel;
		tall_int = (int) tall;
		g_breath_time = tall_int;
		LED_DEBUG("g_breath_time = %d\n",g_breath_time);
		
		nubia_breathlight_timer.expires = jiffies + g_breath_time * HZ;
		add_timer(&nubia_breathlight_timer);//we add timer here,but where we do del_timer or del_timer_sync?		
	}else
	{
		LED_DEBUG("error parameter n, we not support it temporarily.\n");
	}

	LED_DEBUG(" exit\n");
	return ret;
	
}

#if 0
void aw2013_debug(struct nubia_breathlight_control_data * led)
{	
	unsigned int addr, buf;

	for(addr = nubia_breathlight_regs.enable_ledx; addr < 0x40; addr ++)
		{
		buf = read_reg(led,addr);
		LED_DEBUG("at addr %x value is %x\n",addr, buf);
	}
}
#endif



static int nubia_breathlight_soft_reset( struct nubia_breathlight_control_data *led)
{
	//char buf;
	int ret=0;
	//to be fix

	return ret;
}


void nubia_breathlight_const_on(struct nubia_breathlight_control_data *led)
{
	unsigned char buf;
	unsigned int isink_num;
	LED_DEBUG(" entry\n");
	//isink_num = led->outn;
	if(led->outn == 0){
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = INDICATOR_R;
		nubia_set_pmic_reg_mode_const_on(led,isink_num);
		//msleep(2);
		//isink_num = INDICATOR_G;
		//nubia_set_pmic_reg_mode_const_on(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
	}
	else if(led->outn == 1){
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = LED_RIGHT;
		nubia_set_pmic_reg_mode_const_on(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
	}
	else if(led->outn == 2){
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = LED_LEFT;
		nubia_set_pmic_reg_mode_const_on(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
	}
	else
		LED_DEBUG("error outn value\n");

}



void nubia_breathlight_const_off(struct nubia_breathlight_control_data *led)
{
	unsigned char buf;
	unsigned int isink_num;
	LED_DEBUG(" entry\n");
	//isink_num = led->outn;
	if(led->outn == 0){
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = INDICATOR_R;
		nubia_set_pmic_reg_mode_const_off(led,isink_num);
		//msleep(2);
		//isink_num = INDICATOR_G;
		//nubia_set_pmic_reg_mode_const_off(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
	}
	else if(led->outn == 1){
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = LED_RIGHT;
		nubia_set_pmic_reg_mode_const_off(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
	}
	else if(led->outn == 2){
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = LED_LEFT;
		nubia_set_pmic_reg_mode_const_off(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
	}
	else
		LED_DEBUG(" error outn value\n");

}



void nubia_breathlight_auto_breath(struct nubia_breathlight_control_data *led) // self breath 0 - 255 version.
{
	unsigned char buf;
	unsigned int isink_num;
	//isink_num = led->outn;
	LED_DEBUG(" entry\n");
	if(led->outn == 0){
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = INDICATOR_R;
		nubia_set_pmic_reg_mode_auto_breath(led,isink_num);
		//msleep(2);
		//isink_num = INDICATOR_G;
		//nubia_set_pmic_reg_mode_auto_breath(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
	}
	else if(led->outn == 1){
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = LED_RIGHT;
		nubia_set_pmic_reg_mode_auto_breath(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
	}
	else if(led->outn == 2){
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = LED_LEFT;
		nubia_set_pmic_reg_mode_auto_breath(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
	}
	else
		LED_DEBUG(" error outn value\n");
}




void nubia_breathlight_step_fade_in(struct nubia_breathlight_control_data *led)
{
	LED_DEBUG(" entry\n");

}



void nubia_breathlight_step_fade_out(struct nubia_breathlight_control_data *led)
{
	LED_DEBUG(" entry\n");
}



void nubia_breathlight_breath_once(struct nubia_breathlight_control_data *led)
{

	unsigned char buf;
	unsigned int isink_num;
	LED_DEBUG(" entry\n");
	//isink_num = led->outn;
	if(led->outn == 0){
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = INDICATOR_R;
		nubia_set_pmic_reg_mode_breath_once(led,isink_num);
		msleep(2);
		//isink_num = INDICATOR_G;
		//nubia_set_pmic_reg_mode_breath_once(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
	}
	else if(led->outn == 1){
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = LED_RIGHT;
		nubia_set_pmic_reg_mode_breath_once(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
	}
	else if(led->outn == 2){
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = LED_LEFT;
		nubia_set_pmic_reg_mode_breath_once(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
	}
	else
		LED_DEBUG(" error outn value\n");
}




void all_leds_breath_once_(struct nubia_breathlight_control_data *led)
{

}



static ssize_t blink_mode_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct nubia_breathlight_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);

	return sprintf(buf, "%d\n", led->blink_mode);
}


static ssize_t  blink_mode_set(struct device *dev,
		struct device_attribute *attr, const char *buf,size_t count)
{

	//int val = brightness;
	//int rc = 0;
	struct nubia_breathlight_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);
	sscanf(buf, "%d", &led->blink_mode);
	
#ifndef CONFIG_NUBIA_BREATH_USE_WORKSTRUCT
	switch (led->blink_mode) {
		case SW_RESET:
			nubia_breathlight_soft_reset(led);
			break;

		case CONST_ON: 
			nubia_breathlight_const_on(led);
			break;

		case CONST_OFF:
			nubia_breathlight_const_off(led);
			break;

		case AUTO_BREATH:
			nubia_breathlight_auto_breath(led);
			break;

		case STEP_FADE_IN:
		    nubia_breathlight_step_fade_in(led);
			break;

		case STEP_FADE_OUT:
			nubia_breathlight_step_fade_out(led);
			break;

		case BREATH_ONCE:
		   	nubia_breathlight_breath_once(led);		 
			break;

		case RESERVED://to be extended
			break;

		default:
			break;
	}
#else
	if((strcmp(led->grade_parameter, last_grade_parameter) != 0) || (strcmp(led->fade_parameter, last_fade_parameter) != 0) ||(strcmp(led->outn, last_outn)!= 0))
	{
		schedule_work(&led->work);
	}

#endif
	return count;
}

static ssize_t fade_parameter_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct nubia_breathlight_control_data *led;
	char *after, *parm2,*parm3;

	
	unsigned long delay_off,delay_off_1;


	
        if(NULL != buf)
        {
                copy_from_user(last_fade_parameter,buf,10);
        }
        else
        {
                LED_DEBUG(" NULL POINTER\n");
                return -1;
        }



	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long delay_on = simple_strtoul(buf, &after, 10);
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);

	while(isspace(*after))
		after++;
	parm2 = after;
	delay_off = simple_strtoul(parm2, &after, 10);

	while(isspace(*after))
		after++;
	parm3 = after;
	delay_off_1 = simple_strtoul(parm3, &after, 10);
	led->Rise_Fall_time = (int)delay_on;
	led->Hold_time = (int)delay_off;
	led->Off_time = (int)delay_off_1; 
	LED_DEBUG("fade_time=%d ,on_time=%d , off_time=%d\n",
		led->Rise_Fall_time,led->Hold_time,led->Off_time);
	return count;
}

static ssize_t fade_parameter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct nubia_breathlight_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);

	return snprintf(buf, FADE_PARAM_LEN, "%4d %4d %4d\n",
			led->Rise_Fall_time, led->Hold_time, led->Off_time);
}
	
static ssize_t grade_parameter_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{

	struct nubia_breathlight_control_data *led;
	char *after, *parm2;
	unsigned long parameter_two;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	

	
        if(NULL != buf)
        {
                copy_from_user(last_grade_parameter,buf,10);
        }
        else
        {
                LED_DEBUG("NULL POINTER\n");
                return -1;
        }



	unsigned long parameter_one = simple_strtoul(buf, &after, 10);

	
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);

	while(isspace(*after))
		after++;
	parm2 = after;
	parameter_two = simple_strtoul(parm2, &after, 10);

	led->min_grade=(int)parameter_one;

	
	led->max_grade=(int)parameter_two;

	LED_DEBUG("min_grade=%d , max_grade=%d\n",
		led->min_grade,led->max_grade);
	return count;
}
	
static ssize_t grade_parameter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	struct nubia_breathlight_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);

	return snprintf(buf, GRADE_PARAM_LEN,	"%4d %4d\n",
			led->min_grade,led->max_grade);
}

static ssize_t outn_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct nubia_breathlight_control_data *led;
	char *after;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	
	//assert(NULL != buf);


	if(NULL != buf)
	{
		copy_from_user(last_outn,buf,10);
	}
	else
	{
		LED_DEBUG(" NULL POINTER\n");
		return -1;
	}


	unsigned long parameter_one = simple_strtoul(buf, &after, 10);
	
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);


	parameter_one = (parameter_one >> 4) & 0x0f;
	if(parameter_one == 0x01)
		parameter_one = 0x00;
	else if(parameter_one == 0x00)
		parameter_one = 0x01;


	led->outn =(int) parameter_one;
	LED_DEBUG("nubia_channel=%d \n",led->outn);
	return count;
}

static ssize_t outn_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct nubia_breathlight_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);

	return sprintf(buf, "%d\n",led->outn);	
}

static DEVICE_ATTR(fade_parameter, 0664, fade_parameter_show, fade_parameter_store);
static DEVICE_ATTR(grade_parameter, 0664, grade_parameter_show, grade_parameter_store);
static DEVICE_ATTR(outn, 0664, outn_show, outn_store);
static DEVICE_ATTR(blink_mode, 0664, blink_mode_get, blink_mode_set);


static struct attribute *nubia_led_attrs[] = {
	&dev_attr_fade_parameter.attr,
	&dev_attr_grade_parameter.attr,
	&dev_attr_outn.attr,
	&dev_attr_blink_mode.attr,
	NULL
};

static const struct attribute_group nubia_led_attr_group = {
	.attrs = nubia_led_attrs,
};


#if 0
static int aw2013_power_on(struct device *dev)
{
#if 0
	int rc;
    	static struct regulator *vcc_ana;
    	static struct regulator *vcc_i2c;
		
	pr_info("\n --- aw2013_power_on ---\n");
	vcc_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(vcc_ana))
    {
		rc = PTR_ERR(vcc_ana);
		dev_err(dev, "Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(vcc_ana) > 0)
    {
		rc = regulator_set_voltage(vcc_ana, 2850000, 3300000);
		if (rc)
        {
			dev_err(dev, "Regulator set ana vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}
  
    rc = reg_set_optimum_mode_check(vcc_ana, 15000);
    if (rc < 0)
    {
        dev_err(dev, "Regulator vcc_ana set_opt failed rc=%d\n", rc);
        return rc;
    }
    
    rc = regulator_enable(vcc_ana);
    if (rc)
    {
        dev_err(dev, "Regulator vcc_ana enable failed rc=%d\n", rc);
        goto error_reg_en_vcc_ana;
    }

	vcc_i2c = regulator_get(dev, "vcc_i2c");


	if (IS_ERR(vcc_i2c))
    {
		rc = PTR_ERR(vcc_i2c);
		dev_err(dev, "Regulator get failed rc=%d\n", rc);
		goto error_reg_opt_vcc_dig;
	}
   
	if (regulator_count_voltages(vcc_i2c) > 0)
    {
 		rc = regulator_set_voltage(vcc_i2c, 1800000, 1800000);
		if (rc)
        {
			dev_err(dev, "Regulator set i2c vtg failed rc=%d\n", rc);
			goto error_set_vtg_i2c;
		}
	}
  
    rc = reg_set_optimum_mode_check(vcc_i2c, 10000);
    if (rc < 0)
    {
        dev_err(dev, "Regulator vcc_i2c set_opt failed rc=%d\n", rc);
        goto error_set_vtg_i2c;
    }

    rc = regulator_enable(vcc_i2c);
    if (rc)
    {
        dev_err(dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
        goto error_reg_en_vcc_i2c;
    }

    msleep(100);
    
    return 0;


error_reg_en_vcc_i2c:
    reg_set_optimum_mode_check(vcc_i2c, 0);
error_set_vtg_i2c:
    regulator_put(vcc_i2c);
error_reg_opt_vcc_dig:
    regulator_disable(vcc_ana);
error_reg_en_vcc_ana:
    reg_set_optimum_mode_check(vcc_ana, 0);
error_set_vtg_vcc_ana:
	regulator_put(vcc_ana);
	return rc;
#endif
	//LED_DEBUG(" before hwPowerOn MT6351_POWER_LDO_VIO28\n");

	hwPowerOn(MT6351_POWER_LDO_VIO28, VOL_2800 * 1000, "VIO28_PMU");


	hwPowerOn(MT6351_POWER_LDO_VIO18, VOL_1800 * 1000, "VIO18_PMU");
	//LED_DEBUG("after VIO18_PMU!\n");

}
#endif



void nubia_led_work(struct work_struct * work)
{
	struct nubia_breathlight_control_data *led_data =
		container_of(work, struct nubia_breathlight_control_data, work);

	LED_DEBUG("%s:%d\n", led_data->cust.name, led_data->level);//level value will risk,never assign it 
	mutex_lock(&leds_mutex);//this mutex defined in leds.c,we should export it in that file
		switch (led_data->blink_mode) {
		case SW_RESET:
			nubia_breathlight_soft_reset(led_data);
			break;

		case CONST_ON: 
			nubia_breathlight_const_on(led_data);
			break;

		case CONST_OFF:
			nubia_breathlight_const_off(led_data);
			break;

		case AUTO_BREATH:
			nubia_breathlight_auto_breath(led_data);
			break;

		case STEP_FADE_IN:
		    nubia_breathlight_step_fade_in(led_data);
			break;

		case STEP_FADE_OUT:
			nubia_breathlight_step_fade_out(led_data);
			break;

		case BREATH_ONCE:
		   	nubia_breathlight_breath_once(led_data);		 
			break;

		case RESERVED://to be extended
			break;

		default:
			break;
	}
	mutex_unlock(&leds_mutex);
}


static void timer_breath_once_func(unsigned long data)
{
	LED_DEBUG(" entry\n");
	
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch4_en, NLED_OFF);
	pmic_set_register_value(nubia_breathlight_regs.pmic_isink_ch5_en, NLED_OFF);

	g_breath_once_flag = 1;
	//mod_timer(&nubia_breathlight_timer, jiffies + g_breath_time * HZ);//after deep thinking, I think we no need mod_timer all the same.
}





static void nubia_breathlight_preinit()
{
	LED_DEBUG(" entry\n");
	/*
	pmic_set_register_value(MT6351_PMIC_CHRIND_EN_SEL,1);
	msleep(10);
	//printk(KERN_ERR"xiaofeng:set PMIC_CHRIND_EN to 0\n");
	pmic_set_register_value(PMIC_CHRIND_EN,0);//xiaofeng add
	*/
}





//struct nubia_breathlight_control_data
//isink0-LED_RIGHT, isink1-LED_LEFT, isink2-INDICATOR-R,isink3-INDICATOR-G
static int nubia_breathlight_probe(struct platform_device *pdev) 
{
	int status;
	int i;
	int ret;
	struct nubia_breathlight_control_data *breathlight_ctl_data;
	LED_DEBUG(" entry\n");
	struct cust_mt65xx_led *cust_led_list = mt_get_cust_led_list();//should modify cust_leds name.

	for (i = 0; i < MT65XX_LED_TYPE_TOTAL; i++) {
		if (cust_led_list[i].mode != MT65XX_LED_MODE_PMIC) {
			//g_breathlight_ctl_data = NULL;//we need it?
			continue;
		}
		else if((0 == strcmp(cust_led_list[i].name, "LED_RIGHT")) || (0 == strcmp(cust_led_list[i].name, "LED_LEFT")) || (0 == strcmp(cust_led_list[i].name, "INDICATOR-R")) || (0 == strcmp(cust_led_list[i].name, "INDICATOR-G")))
		{ //ATTENTIONS: strings above should keep steps with cust_mt65xx_led definition for name field in cust_leds.c
			
			LED_DEBUG("we have find pmic isink node!\n");
			breathlight_ctl_data = kzalloc(sizeof(struct nubia_breathlight_control_data), GFP_KERNEL);//we use devm_kzalloc?
			if (!breathlight_ctl_data) {
				ret = -ENOMEM;
				goto err;
			}

			breathlight_ctl_data->cust.mode = cust_led_list[i].mode;
			breathlight_ctl_data->cust.data = cust_led_list[i].data;
			breathlight_ctl_data->cust.name = cust_led_list[i].name;

			//g_breathlight_ctl_data->cdev.name = cust_led_list[i].name;
			breathlight_ctl_data->cdev.name = "nubia_led";
			breathlight_ctl_data->cust.config_data = cust_led_list[i].config_data;	/* bei add */

			//breathlight_ctl_data->cdev.brightness_set = mt65xx_led_set;//maybe not useful
			//breathlight_ctl_data->cdev.blink_set = mt65xx_blink_set;//maybe not useful

			INIT_WORK(&breathlight_ctl_data->work, nubia_led_work);//AW  not use this logic

			ret = led_classdev_register(&pdev->dev, &breathlight_ctl_data->cdev);

			if (strcmp(breathlight_ctl_data->cdev.name, "nubia_led") == 0) {// the if clause is suitable???
				ret = sysfs_create_group(&breathlight_ctl_data->cdev.dev->kobj,
					&nubia_led_attr_group);
				if (ret)
					goto init_fail;
			}

			platform_set_drvdata(pdev,breathlight_ctl_data);

			nubia_breathlight_preinit();//for some initialization

			LED_DEBUG("after nubia_breathlight_preinit!\n");
			
			break;
		}
	}
	
	return 0;

 err:
	led_classdev_unregister(&breathlight_ctl_data->cdev);
	cancel_work_sync(&breathlight_ctl_data->work);
	kfree(breathlight_ctl_data);
	breathlight_ctl_data = NULL;
init_fail:
	return ret;
}
/*----------------------------------------------------------------------------*/
static int nubia_breathlight_remove(struct platform_device *pdev)
{
	struct nubia_breathlight_control_data *led_data = platform_get_drvdata(pdev);

	led_classdev_unregister(&led_data->cdev);//add para parent->del para parent due to conflict with leds.h

	sysfs_remove_group(&led_data->cdev.dev->kobj, &nubia_led_attrs);
	LED_DEBUG(" entry\n");
	return 0;
}


static void nubia_breathlight_shutdown(struct platform_device *pdev)
{
	//TO BE FIX
	struct nubia_breathlight_control_data *led_data = platform_get_drvdata(pdev);
	LED_DEBUG(" entry\n");
}


static int nubia_breathlight_suspend(struct platform_device *pdev, pm_message_t state)
{
	//TO BE FIX

	struct nubia_breathlight_control_data *led_data = platform_get_drvdata(pdev);
	LED_DEBUG(" entry\n");	
	if((led_data->outn == 0) &&
	       ((led_data->blink_mode == SW_RESET)||
	             (led_data->blink_mode == CONST_OFF)))
	{	          
		//write_reg(aw2013_data, 0x01, 0x00);// for aw2013 suspend setting GCR Register 0

	}
	return 0;
}


static int nubia_breathlight_resume(struct platform_device *pdev)
{
	//TO BE FIX

	struct nubia_breathlight_control_data *led_data = platform_get_drvdata(pdev);
	LED_DEBUG(" entry\n");
	if((led_data->outn == 0) &&
		((led_data->blink_mode == SW_RESET)||
			(led_data->blink_mode == CONST_OFF)))
	{
		//write_reg(aw2013_data, 0x01, 0x01);// for aw2013 resume setting GCR Register 1
		
	}
	return 0;
}



#ifdef CONFIG_OF
static struct of_device_id nubia_breathlight_match_table[] = {
	{ .compatible = "nubia,breathlight", },
	{}
};
#endif


//void	(*release)(struct device *dev);//prototype
void nubia_breathlight_release(struct device *dev)
{
	LED_DEBUG("entry");
}

struct resource nubia_breathlight_res[1] = {
		[0] = {
				.start = 0x12345678, //just a joke,to be fix
				.end = 0x12345678, //just a joke
				.flags = IORESOURCE_MEM,
		},
};	


static struct platform_device nubia_breathlight_dev = {
		.name = "pmic_breathlight",
		.id = -1,
		.dev = {
				.release = nubia_breathlight_release,
		},
		.num_resources = ARRAY_SIZE(nubia_breathlight_res),
		.resource = nubia_breathlight_res,
};



static struct platform_driver nubia_breathlight_drv = {
	.probe      = nubia_breathlight_probe,
	.remove     = nubia_breathlight_remove,
	.shutdown = nubia_breathlight_shutdown,
	.suspend = nubia_breathlight_suspend,
	.resume = nubia_breathlight_resume,
	.driver     =
	{
		.name  = "pmic_breathlight",
		.owner  = THIS_MODULE,
        #ifdef CONFIG_OF
		.of_match_table = nubia_breathlight_match_table,
	#endif
	}
};

static int __init nubia_breathlight_init(void)
{
	int err = 0;
	LED_DEBUG(" entry\n");
	err = platform_device_register(&nubia_breathlight_dev);
	if(err != 0)
	{
		LED_DEBUG("failed to register nubia_breathlight_dev,err =%d\n",err);
		return err;
	}

	msleep(1000);
	
	if(err = platform_driver_register(&nubia_breathlight_drv))
	{
		LED_DEBUG("failed to register nubia_breathlight_drv,err =%d\n",err);
		return -ENODEV;
	}

	setup_timer(&nubia_breathlight_timer,timer_breath_once_func,(unsigned long)0);//add for breath once function
	//nubia_breathlight_timer.expires = jiffies + 5 * HZ;//no need , we do in nubia_set_pmic_reg_mode_breath_once
	return err;
}

static void __exit nubia_breathlight_exit(void)
{
	LED_DEBUG(" entry\n");
	del_timer(&nubia_breathlight_timer);
}


late_initcall(nubia_breathlight_init);
module_exit(nubia_breathlight_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("xiaofeng,<xiao.feng1@zte.com.cn>, lvsen,yangkui");
MODULE_DESCRIPTION("nubia breathlight Linux driver");
MODULE_ALIAS("platform:nubia_led_drv");
MODULE_LICENSE("GPL");
