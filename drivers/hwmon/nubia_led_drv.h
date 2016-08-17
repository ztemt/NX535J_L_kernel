/* Copyright (c) 2016, NUBIA corporation department 2 section 3 wifi group xiaofeng. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __NUBIA_LED_DRV_H__
#define __NUBIA_LED_DRV_H__

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
//#include "../../arch/arm/mach-msm/include/mach/board.h"//xiaofeng comment due to no factual meaning
//#include <linux/mfd/pm8xxx/pm8921-charger.h>//xiaofeng comment due to no factual meaning
#include "cust_leds_def.h"
#define AW_DEVICE_NAME "aw2013"
//extern int mt_mt65xx_led_set_cust(struct cust_mt65xx_led *cust, int level);

/****************************************************************************
 * structures
 ***************************************************************************/



struct nubia_breathlight_control_data {
	struct led_classdev cdev;
	struct cust_mt65xx_led cust;
	struct work_struct work;
	int level;
	int delay_on;//useful?
	int delay_off;//useful?

	//add pmic-specific parameters
	/*
	int 32k_ck_pdn;	//pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down
	int ck_pdn;		//pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN,0);
	int ck_cksel;	       //pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL,0);
	
	int ch_mode;
	int ch_step;
	int ch_en;

	int t_rising_1;
	int t_rising_2;
	int t_falling_1;
	int t_falling_2;
	int t_on;
	int t_off;
	int dim_duty;
	int dim_fsel;
	int ch_bias_en;
	int chop_en;
	*/
    	int Rise_Fall_time; // intergrated as a ingrater.
	int Hold_time;
    	// int Fall_time;
	int Off_time;

	int Delay_time;
	int Repeat_time;
	
	int min_grade;
    	int max_grade;
	

	//add info parameters
	int outn;
	int fade_parameter;
	int grade_parameter;
	int blink_mode;
};


struct nubia_breathlight_regs_data {
	int  pmic_isink_ch0_en;    //PMIC_ISINK_CH0_EN    use for
	int  pmic_isink_ch1_en;    //PMIC_ISINK_CH1_EN    use for
	int  pmic_isink_ch4_en;    //PMIC_ISINK_CH2_EN
	int  pmic_isink_ch5_en;    //PMIC_ISINK_CH3_EN
	
	int  pmic_rg_drv_32k_ck_pdn; //PMIC_RG_DRV_32K_CK_PDN

	//isink0
	int  pmic_rg_drv_isink0_ck_pdn; //PMIC_RG_DRV_ISINK0_CK_PDN
	int  pmic_rg_drv_isink0_ck_cksel; //PMIC_RG_DRV_ISINK0_CK_CKSEL
	int  pmic_isink_ch0_mode; //PMIC_ISINK_CH0_MODE
	int  pmic_isink_ch0_step; //PMIC_ISINK_CH0_STEP
	int  pmic_isink_breath0_tr1_sel; 	//PMIC_ISINK_BREATH0_TR1_SEL
	int  pmic_isink_breath0_tr2_sel; 	//PMIC_ISINK_BREATH0_TR2_SEL
	int  pmic_isink_breath0_tf1_sel; 	//PMIC_ISINK_BREATH0_TF1_SEL
	int  pmic_isink_breath0_tf2_sel; 	//PMIC_ISINK_BREATH0_TF2_SEL
	int  pmic_isink_breath0_ton_sel; 	//PMIC_ISINK_BREATH0_TON_SEL
	int  pmic_isink_breath0_toff_sel; 	//PMIC_ISINK_BREATH0_TOFF_SEL
	int  pmic_isink_dim0_duty; 	//PMIC_ISINK_DIM0_DUTY
	int  pmic_isink_dim0_fsel; 	//PMIC_ISINK_DIM0_FSEL
	int  pmic_isink_chop0_en; 	//PMIC_ISINK_CHOP0_EN
	int  pmic_isink_ch0_bias_en; 	//PMIC_ISINK_CH0_BIAS_EN
	//int  pmic_isink_ch0_en; 	//PMIC_ISINK_CH0_EN //have above


	//isink1
	int  pmic_rg_drv_isink1_ck_pdn; //PMIC_RG_DRV_ISINK1_CK_PDN
	int  pmic_rg_drv_isink1_ck_cksel; //PMIC_RG_DRV_ISINK1_CK_CKSEL
	int  pmic_isink_ch1_mode; //PMIC_ISINK_CH1_MODE
	int  pmic_isink_ch1_step; //PMIC_ISINK_CH1_STEP
	int  pmic_isink_breath1_tr1_sel; 	//PMIC_ISINK_BREATH1_TR1_SEL
	int  pmic_isink_breath1_tr2_sel; 	//PMIC_ISINK_BREATH1_TR2_SEL
	int  pmic_isink_breath1_tf1_sel; 	//PMIC_ISINK_BREATH1_TF1_SEL
	int  pmic_isink_breath1_tf2_sel; 	//PMIC_ISINK_BREATH1_TF2_SEL
	int  pmic_isink_breath1_ton_sel; 	//PMIC_ISINK_BREATH1_TON_SEL
	int  pmic_isink_breath1_toff_sel; 	//PMIC_ISINK_BREATH1_TOFF_SEL
	int  pmic_isink_dim1_duty; 	//PMIC_ISINK_DIM1_DUTY
	int  pmic_isink_dim1_fsel; 	//PMIC_ISINK_DIM1_FSEL
	int  pmic_isink_chop1_en; 	//PMIC_ISINK_CHOP1_EN
	int  pmic_isink_ch1_bias_en; 	//PMIC_ISINK_CH1_BIAS_EN
	//int  pmic_isink_ch1_en; 	//PMIC_ISINK_CH1_EN //have above


	//isink2
	int  pmic_rg_drv_isink4_ck_pdn; //PMIC_RG_DRV_ISINK2_CK_PDN
	int  pmic_rg_drv_isink4_ck_cksel; //PMIC_RG_DRV_ISINK2_CK_CKSEL
	int  pmic_isink_ch4_mode; //PMIC_ISINK_CH2_MODE
	int  pmic_isink_ch4_step; //PMIC_ISINK_CH2_STEP
	int  pmic_isink_breath4_tr1_sel; 	//PMIC_ISINK_BREATH2_TR1_SEL
	int  pmic_isink_breath4_tr2_sel; 	//PMIC_ISINK_BREATH2_TR2_SEL
	int  pmic_isink_breath4_tf1_sel; 	//PMIC_ISINK_BREATH2_TF1_SEL
	int  pmic_isink_breath4_tf2_sel; 	//PMIC_ISINK_BREATH2_TF2_SEL
	int  pmic_isink_breath4_ton_sel; 	//PMIC_ISINK_BREATH2_TON_SEL
	int  pmic_isink_breath4_toff_sel; 	//PMIC_ISINK_BREATH2_TOFF_SEL
	int  pmic_isink_dim4_duty; 	//PMIC_ISINK_DIM2_DUTY
	int  pmic_isink_dim4_fsel; 	//PMIC_ISINK_DIM2_FSEL
	int  pmic_isink_chop4_en; 	//PMIC_ISINK_CHOP2_EN
	int  pmic_isink_ch4_bias_en; 	//PMIC_ISINK_CH2_BIAS_EN
	//int  pmic_isink_ch2_en; 	//PMIC_ISINK_CH2_EN //have above


	//isink3
	int  pmic_rg_drv_isink5_ck_pdn; //PMIC_RG_DRV_ISINK3_CK_PDN
	int  pmic_rg_drv_isink5_ck_cksel; //PMIC_RG_DRV_ISINK3_CK_CKSEL
	int  pmic_isink_ch5_mode; //PMIC_ISINK_CH3_MODE
	int  pmic_isink_ch5_step; //PMIC_ISINK_CH3_STEP
	int  pmic_isink_breath5_tr1_sel; 	//PMIC_ISINK_BREATH3_TR1_SEL
	int  pmic_isink_breath5_tr2_sel; 	//PMIC_ISINK_BREATH3_TR2_SEL
	int  pmic_isink_breath5_tf1_sel; 	//PMIC_ISINK_BREATH3_TF1_SEL
	int  pmic_isink_breath5_tf2_sel; 	//PMIC_ISINK_BREATH3_TF2_SEL
	int  pmic_isink_breath5_ton_sel; 	//PMIC_ISINK_BREATH3_TON_SEL
	int  pmic_isink_breath5_toff_sel; 	//PMIC_ISINK_BREATH3_TOFF_SEL
	int  pmic_isink_dim5_duty; 	//PMIC_ISINK_DIM3_DUTY
	int  pmic_isink_dim5_fsel; 	//PMIC_ISINK_DIM3_FSEL
	int  pmic_isink_chop5_en; 	//PMIC_ISINK_CHOP3_EN
	int  pmic_isink_ch5_bias_en; 	//PMIC_ISINK_CH3_BIAS_EN
	//int  pmic_isink_ch3_en; 	//PMIC_ISINK_CH3_EN //have above
};

#endif /* __NUBIA_LED_DRV_H__ */

