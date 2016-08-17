/*
** =============================================================================
** Copyright (c) 2014  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
** File:
**     drv2605.c
**
** Description:
**     DRV2605 chip driver
**
** =============================================================================
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
//#include <linux/haptics/drv2605.h>
#include "drv2605.h"

#include <linux/of_gpio.h>

#define DRV2605_I2C_BUSNUM 4 //xiaofeng,according to lvsen, should modify 3 to 4
//#define LINUX_NATIVE_GPIO
//#include <mach/gpio_const.h>

#ifndef LINUX_NATIVE_GPIO
//#include <mach/mt_gpio.h>
//#include <mach/mt_gpio.h>

//#include <mach/cust_gpio_usage.h>
/******************************************************************************
* GPIO Driver interface
******************************************************************************/
/*direction*/
extern int mt_set_gpio_dir(unsigned long pin, unsigned long dir);
extern int mt_get_gpio_dir(unsigned long pin);

/*pull enable*/
extern int mt_set_gpio_pull_enable(unsigned long pin, unsigned long enable);
extern int mt_get_gpio_pull_enable(unsigned long pin);

/*schmitt trigger*/
extern int mt_set_gpio_smt(unsigned long pin, unsigned long enable);
extern int mt_get_gpio_smt(unsigned long pin);

/*IES*/
extern int mt_set_gpio_ies(unsigned long pin, unsigned long enable);
extern int mt_get_gpio_ies(unsigned long pin);

/*pull select*/
extern int mt_set_gpio_pull_select(unsigned long pin, unsigned long select);
extern int mt_get_gpio_pull_select(unsigned long pin);

/*data inversion*/
extern int mt_set_gpio_inversion(unsigned long pin, unsigned long enable);
extern int mt_get_gpio_inversion(unsigned long pin);

/*input/output*/
extern int mt_set_gpio_out(unsigned long pin, unsigned long output);
extern int mt_get_gpio_out(unsigned long pin);
extern int mt_get_gpio_in(unsigned long pin);

/*mode control*/
extern int mt_set_gpio_mode(unsigned long pin, unsigned long mode);
extern int mt_get_gpio_mode(unsigned long pin);

/*misc functions for protect GPIO*/
/* void mt_gpio_dump(GPIO_REGS *regs,GPIOEXT_REGS *regs_ext); */
extern void gpio_dump_regs(void);
//#include <mach/cust_gpio_usage.h>//xiaofengadd 
#endif

//#define DRV2605_GPIO_ENABLE GPIO_SPEAKER_EN_PIN

static struct i2c_client *drv_client;//1231

static struct drv2605_data *pDRV2605data = NULL;

static struct i2c_board_info __initdata drv2605_i2c_info = { I2C_BOARD_INFO(HAPTICS_DEVICE_NAME, 0x5a) };//xiaofeng

static struct platform_driver drv2605_platform_driver;//

static struct drv2605_platform_data  drv2605_plat_data = {	
	//xiaofeng mod GpioEnable to 115
	.GpioEnable = 115,			
	//enable the chip
	.GpioTrigger = 0,		
	//external trigger pin, (0: internal trigger)
	//rated = 1.5Vrms, ov=2.1Vrms, f=204hz	
	.loop = CLOSE_LOOP,	
	.RTPFormat = Signed,	
	.BIDIRInput = BiDirectional,	
	.actuator = {		
	.device_type = LRA,		
	.rated_vol = 0x45,//0x3d,		
	.g_effect_bank = LIBRARY_F,		
	.over_drive_vol = 0x76,//0x87,		
	.LRAFreq = 235,	},
	.a2h = {		
		.a2h_min_input = AUDIO_HAPTICS_MIN_INPUT_VOLTAGE,		
			.a2h_max_input = AUDIO_HAPTICS_MAX_INPUT_VOLTAGE,		
			.a2h_min_output = AUDIO_HAPTICS_MIN_OUTPUT_VOLTAGE,		
			.a2h_max_output = AUDIO_HAPTICS_MAX_OUTPUT_VOLTAGE,			
			},
};

static int drv2605_reg_read(struct drv2605_data *pDrv2605data, unsigned int reg)
{
/*	unsigned int val;
	int ret;
	
	ret = regmap_read(pDrv2605data->regmap, reg, &val);
    
	if (ret < 0)
		return ret;
	else
		return val;
		*/
	s32 ret;

	ret = i2c_smbus_read_byte_data(pDrv2605data->client, reg);
	if (ret < 0)
		return ret;

	return ret;
}

static int drv2605_reg_write(struct drv2605_data *pDrv2605data, unsigned char reg, char val)
{
    //return regmap_write(pDrv2605data->regmap, reg, val);
    return i2c_smbus_write_byte_data(pDrv2605data->client, reg, val);
}

static int drv2605_bulk_read(struct drv2605_data *pDrv2605data, unsigned char reg, unsigned int count, u8 *buf)
{
	int i=0;
	for(; i < count; i++){
		buf[i] = drv2605_reg_read(pDrv2605data, reg + i);
	}
	
	return count;
}

static int drv2605_bulk_write(struct drv2605_data *pDrv2605data, unsigned char reg, unsigned int count, const u8 *buf)
{
//	return regmap_bulk_write(pDrv2605data->regmap, reg, buf, count);
	int i=0;

	for(; i < count; i++){
		drv2605_reg_write(pDrv2605data, reg + i, buf[i]);
	}
	
	return count;
}

static int drv2605_set_bits(struct drv2605_data *pDrv2605data, unsigned char reg, unsigned char mask, unsigned char val)
{
	int reg_original = drv2605_reg_read(pDrv2605data, reg);
	if((reg_original & mask) != (val & mask))
	{
		reg_original &=~mask;
		reg_original |= (val&mask);
		drv2605_reg_write(pDrv2605data, reg, reg_original);
	}
	return 1;
//	return regmap_update_bits(pDrv2605data->regmap, reg, mask, val);
}

static int drv2605_set_go_bit(struct drv2605_data *pDrv2605data, unsigned char val)
{
	return drv2605_reg_write(pDrv2605data, GO_REG, (val&0x01));
}

static void drv2605_poll_go_bit(struct drv2605_data *pDrv2605data)
{
    while (drv2605_reg_read(pDrv2605data, GO_REG) == GO)
      schedule_timeout_interruptible(msecs_to_jiffies(GO_BIT_POLL_INTERVAL));
}

static int drv2605_select_library(struct drv2605_data *pDrv2605data, unsigned char lib)
{
	return drv2605_reg_write(pDrv2605data, LIBRARY_SELECTION_REG, (lib&0x07));
}

static int drv2605_set_rtp_val(struct drv2605_data *pDrv2605data, char value)
{
	/* please be noted: in unsigned mode, maximum is 0xff, in signed mode, maximum is 0x7f */
	return drv2605_reg_write(pDrv2605data, REAL_TIME_PLAYBACK_REG, value);
}

static int drv2605_set_waveform_sequence(struct drv2605_data *pDrv2605data, unsigned char* seq, unsigned int size)
{
	return drv2605_bulk_write(pDrv2605data, WAVEFORM_SEQUENCER_REG, (size>WAVEFORM_SEQUENCER_MAX)?WAVEFORM_SEQUENCER_MAX:size, seq);
}

static void drv2605_change_mode(struct drv2605_data *pDrv2605data, char work_mode, char dev_mode)
{
	/* please be noted : LRA open loop cannot be used with analog input mode */
	if(dev_mode == DEV_IDLE){
		pDrv2605data->dev_mode = dev_mode;
		pDrv2605data->work_mode = work_mode;
	}else if(dev_mode == DEV_STANDBY){
		if(pDrv2605data->dev_mode != DEV_STANDBY){
			pDrv2605data->dev_mode = DEV_STANDBY;
			drv2605_set_rtp_val(pDrv2605data, 0);
			schedule_timeout_interruptible(msecs_to_jiffies(25));
			drv2605_reg_write(pDrv2605data, MODE_REG, MODE_STANDBY);
			schedule_timeout_interruptible(msecs_to_jiffies(WAKE_STANDBY_DELAY));
		}
		pDrv2605data->work_mode = WORK_IDLE;
	}else if(dev_mode == DEV_READY){
		if((work_mode != pDrv2605data->work_mode)
			||(dev_mode != pDrv2605data->dev_mode)){
			pDrv2605data->work_mode = work_mode;
			pDrv2605data->dev_mode = dev_mode;
			if((pDrv2605data->work_mode == WORK_VIBRATOR)
				||(pDrv2605data->work_mode == WORK_PATTERN_RTP_ON)
				||(pDrv2605data->work_mode == WORK_SEQ_RTP_ON)
				||(pDrv2605data->work_mode == WORK_RTP)){
					drv2605_reg_write(pDrv2605data, MODE_REG, MODE_REAL_TIME_PLAYBACK);
			}else if(pDrv2605data->work_mode == WORK_AUDIO2HAPTIC){
				drv2605_reg_write(pDrv2605data, MODE_REG, MODE_AUDIOHAPTIC);
			}else if(pDrv2605data->work_mode == WORK_CALIBRATION){
				drv2605_reg_write(pDrv2605data, MODE_REG, AUTO_CALIBRATION);
			}else{
				drv2605_reg_write(pDrv2605data, MODE_REG, MODE_INTERNAL_TRIGGER);
				schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));
			}
		}
	}
}

static void setAudioHapticsEnabled(struct drv2605_data *pDrv2605data, int enable)
{
    if (enable)
    {
		if(pDrv2605data->work_mode != WORK_AUDIO2HAPTIC){
			pDrv2605data->vibrator_is_playing = YES;
			drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_READY);
			
			drv2605_set_bits(pDrv2605data, 
					Control1_REG, 
					Control1_REG_AC_COUPLE_MASK, 
					AC_COUPLE_ENABLED );
					
			drv2605_set_bits(pDrv2605data, 
					Control3_REG, 
					Control3_REG_PWMANALOG_MASK, 
					INPUT_ANALOG);	

			drv2605_change_mode(pDrv2605data, WORK_AUDIO2HAPTIC, DEV_READY);
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_AUDIO2HAPTIC);
		}
    } else {
        // Chip needs to be brought out of standby to change the registers
		if(pDrv2605data->work_mode == WORK_AUDIO2HAPTIC){
			pDrv2605data->vibrator_is_playing = NO;
			drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_READY);
						
			drv2605_set_bits(pDrv2605data, 
					Control1_REG, 
					Control1_REG_AC_COUPLE_MASK, 
					AC_COUPLE_DISABLED );
					
			drv2605_set_bits(pDrv2605data, 
					Control3_REG, 
					Control3_REG_PWMANALOG_MASK, 
					INPUT_PWM);	
					
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);		
			drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_STANDBY); // Disable audio-to-haptics
		}
    }
}

static void play_effect(struct drv2605_data *pDrv2605data)
{
	switch_set_state(&pDrv2605data->sw_dev, SW_STATE_SEQUENCE_PLAYBACK);
	drv2605_change_mode(pDrv2605data, WORK_SEQ_PLAYBACK, DEV_READY);
    drv2605_set_waveform_sequence(pDrv2605data, pDrv2605data->sequence, WAVEFORM_SEQUENCER_MAX);
	pDrv2605data->vibrator_is_playing = YES;
    drv2605_set_go_bit(pDrv2605data, GO);

    while((drv2605_reg_read(pDrv2605data, GO_REG) == GO) && (pDrv2605data->should_stop == NO)){
        schedule_timeout_interruptible(msecs_to_jiffies(GO_BIT_POLL_INTERVAL));
	}
	
	if(pDrv2605data->should_stop == YES){
		drv2605_set_go_bit(pDrv2605data, STOP);
	}
  
    if (pDrv2605data->audio_haptics_enabled){
        setAudioHapticsEnabled(pDrv2605data, YES);
    } else {
        drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_STANDBY);
		switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);		
		pDrv2605data->vibrator_is_playing = NO;
		wake_unlock(&pDrv2605data->wklock);
    }
}

static void play_Pattern_RTP(struct drv2605_data *pDrv2605data)
{
	if(pDrv2605data->work_mode == WORK_PATTERN_RTP_ON){
		drv2605_change_mode(pDrv2605data, WORK_PATTERN_RTP_OFF, DEV_READY);
		if(pDrv2605data->repeat_times == 0){
			drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_STANDBY);
			pDrv2605data->vibrator_is_playing = NO;
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);	
			wake_unlock(&pDrv2605data->wklock);
		}else{
			hrtimer_start(&pDrv2605data->timer, ns_to_ktime((u64)pDrv2605data->silience_time * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		}
	}else if(pDrv2605data->work_mode == WORK_PATTERN_RTP_OFF){
		pDrv2605data->repeat_times--;
		drv2605_change_mode(pDrv2605data, WORK_PATTERN_RTP_ON, DEV_READY);
		hrtimer_start(&pDrv2605data->timer, ns_to_ktime((u64)pDrv2605data->vibration_time * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	}
}

static void play_Seq_RTP(struct drv2605_data *pDrv2605data)
{
	if(pDrv2605data->RTPSeq.RTPindex < pDrv2605data->RTPSeq.RTPCounts){
		int RTPTime = pDrv2605data->RTPSeq.RTPData[pDrv2605data->RTPSeq.RTPindex] >> 8;
		int RTPVal = pDrv2605data->RTPSeq.RTPData[pDrv2605data->RTPSeq.RTPindex] & 0x00ff ;
			
		pDrv2605data->vibrator_is_playing = YES;
		pDrv2605data->RTPSeq.RTPindex++;
		drv2605_change_mode(pDrv2605data, WORK_SEQ_RTP_ON, DEV_READY);
		drv2605_set_rtp_val(pDrv2605data,  RTPVal);
							
		hrtimer_start(&pDrv2605data->timer, ns_to_ktime((u64)RTPTime * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	}else{
		drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_STANDBY);
		pDrv2605data->vibrator_is_playing = NO;
		switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);	
		wake_unlock(&pDrv2605data->wklock);
	}
}

static void vibrator_off(struct drv2605_data *pDrv2605data)
{
    if (pDrv2605data->vibrator_is_playing) {
		if(pDrv2605data->audio_haptics_enabled == YES){
			setAudioHapticsEnabled(pDrv2605data, YES);
		}else{
			pDrv2605data->vibrator_is_playing = NO;
			drv2605_set_go_bit(pDrv2605data, STOP);
			drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_STANDBY);
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);				
			wake_unlock(&pDrv2605data->wklock);		
		}
    }
}

static void drv2605_stop(struct drv2605_data *pDrv2605data)
{
	if(pDrv2605data->vibrator_is_playing){
		if(pDrv2605data->work_mode == WORK_AUDIO2HAPTIC){
			setAudioHapticsEnabled(pDrv2605data, NO);		
		}else if((pDrv2605data->work_mode == WORK_VIBRATOR)
				||(pDrv2605data->work_mode == WORK_PATTERN_RTP_ON)
				||(pDrv2605data->work_mode == WORK_PATTERN_RTP_OFF)
				||(pDrv2605data->work_mode == WORK_SEQ_RTP_ON)
				||(pDrv2605data->work_mode == WORK_SEQ_RTP_OFF)
				||(pDrv2605data->work_mode == WORK_RTP)){
			vibrator_off(pDrv2605data);
		}else if(pDrv2605data->work_mode == WORK_SEQ_PLAYBACK){
		}else{
			printk("%s, err mode=%d \n", __FUNCTION__, pDrv2605data->work_mode);
		}
	}	
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct drv2605_data *pDrv2605data = container_of(dev, struct drv2605_data, to_dev);

    if (hrtimer_active(&pDrv2605data->timer)) {
        ktime_t r = hrtimer_get_remaining(&pDrv2605data->timer);
        return ktime_to_ms(r);
    }

    return 0;
}

static void vibrator_enable( struct timed_output_dev *dev, int value)
{
	struct drv2605_data *pDrv2605data = container_of(dev, struct drv2605_data, to_dev);
	
	pDrv2605data->should_stop = YES;	
	hrtimer_cancel(&pDrv2605data->timer);
	cancel_work_sync(&pDrv2605data->vibrator_work);

    mutex_lock(&pDrv2605data->lock);
	
	drv2605_stop(pDrv2605data);

    if (value > 0) {
		if(pDrv2605data->audio_haptics_enabled == NO){
			wake_lock(&pDrv2605data->wklock);
		}

		drv2605_change_mode(pDrv2605data, WORK_VIBRATOR, DEV_READY);
		drv2605_set_rtp_val(pDrv2605data,  0x7f);
		pDrv2605data->vibrator_is_playing = YES;
		switch_set_state(&pDrv2605data->sw_dev, SW_STATE_RTP_PLAYBACK);			

		value = (value>MAX_TIMEOUT)?MAX_TIMEOUT:value;
        hrtimer_start(&pDrv2605data->timer, ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
    }
	
	mutex_unlock(&pDrv2605data->lock);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct drv2605_data *pDrv2605data = container_of(timer, struct drv2605_data, timer);

    schedule_work(&pDrv2605data->vibrator_work);
	
    return HRTIMER_NORESTART;
}

static void vibrator_work_routine(struct work_struct *work)
{
	struct drv2605_data *pDrv2605data = container_of(work, struct drv2605_data, vibrator_work);

	mutex_lock(&pDrv2605data->lock);
	
	if((pDrv2605data->work_mode == WORK_VIBRATOR)
		||(pDrv2605data->work_mode == WORK_RTP)){
		vibrator_off(pDrv2605data);
	}else if(pDrv2605data->work_mode == WORK_SEQ_PLAYBACK){
		play_effect(pDrv2605data);
	}else if((pDrv2605data->work_mode == WORK_PATTERN_RTP_ON)||(pDrv2605data->work_mode == WORK_PATTERN_RTP_OFF)){
		play_Pattern_RTP(pDrv2605data);
	}else if((pDrv2605data->work_mode == WORK_SEQ_RTP_ON)||(pDrv2605data->work_mode == WORK_SEQ_RTP_OFF)){
		play_Seq_RTP(pDrv2605data);
	}
	
	mutex_unlock(&pDrv2605data->lock);
}

static int dev2605_open (struct inode * i_node, struct file * filp)
{
	if(pDRV2605data == NULL){
		return -ENODEV;
	}
	
	filp->private_data = pDRV2605data;
	return 0;
}

static ssize_t dev2605_read(struct file* filp, char* buff, size_t length, loff_t* offset)
{
	struct drv2605_data *pDrv2605data = (struct drv2605_data *)filp->private_data;
	int ret = 0;

	if(pDrv2605data->ReadLen > 0){
		ret = copy_to_user(buff, pDrv2605data->ReadBuff, pDrv2605data->ReadLen);
		if (ret != 0){
			printk("%s, copy_to_user err=%d \n", __FUNCTION__, ret);
		}else{
			ret = pDrv2605data->ReadLen;
		}
		pDrv2605data->ReadLen = 0;
	}
	
    return ret;
}

static bool isforDebug(int cmd){
	return ((cmd == HAPTIC_CMDID_REG_WRITE)
		||(cmd == HAPTIC_CMDID_REG_READ)
		||(cmd == HAPTIC_CMDID_REG_SETBIT));
}

static ssize_t dev2605_write(struct file* filp, const char* buff, size_t len, loff_t* off)
{
	struct drv2605_data *pDrv2605data = (struct drv2605_data *)filp->private_data;
	
	if(isforDebug(buff[0])){
	}else{
		pDrv2605data->should_stop = YES;	
		hrtimer_cancel(&pDrv2605data->timer);
		cancel_work_sync(&pDrv2605data->vibrator_work);
	}
	
    mutex_lock(&pDrv2605data->lock);
	
	if(isforDebug(buff[0])){
	}else{
		drv2605_stop(pDrv2605data);
	}
	
    switch(buff[0])
    {
        case HAPTIC_CMDID_PLAY_SINGLE_EFFECT:
        case HAPTIC_CMDID_PLAY_EFFECT_SEQUENCE:
		{	
            memset(&pDrv2605data->sequence, 0, WAVEFORM_SEQUENCER_MAX);
            if (!copy_from_user(&pDrv2605data->sequence, &buff[1], len - 1))
            {
				if(pDrv2605data->audio_haptics_enabled == NO){
					wake_lock(&pDrv2605data->wklock);
				}
				pDrv2605data->should_stop = NO;
				drv2605_change_mode(pDrv2605data, WORK_SEQ_PLAYBACK, DEV_IDLE);
                schedule_work(&pDrv2605data->vibrator_work);
            }
            break;
        }
        case HAPTIC_CMDID_PLAY_TIMED_EFFECT:
        {	
            unsigned int value = 0;
            value = buff[2];
            value <<= 8;
            value |= buff[1];
		
            if (value > 0)
            {
				if(pDrv2605data->audio_haptics_enabled == NO){			
					wake_lock(&pDrv2605data->wklock);
				}
				switch_set_state(&pDrv2605data->sw_dev, SW_STATE_RTP_PLAYBACK);
				pDrv2605data->vibrator_is_playing = YES;
  				value = (value > MAX_TIMEOUT)?MAX_TIMEOUT:value;
				drv2605_change_mode(pDrv2605data, WORK_RTP, DEV_READY);
				
				hrtimer_start(&pDrv2605data->timer, ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
            }
            break;
        }

       case HAPTIC_CMDID_PATTERN_RTP:
        {
			unsigned char strength = 0;

			pDrv2605data->vibration_time = (int)((((int)buff[2])<<8) | (int)buff[1]);
			pDrv2605data->silience_time = (int)((((int)buff[4])<<8) | (int)buff[3]);
			strength = buff[5];
			pDrv2605data->repeat_times = buff[6];
			
            if(pDrv2605data->vibration_time > 0){
				if(pDrv2605data->audio_haptics_enabled == NO){
					wake_lock(&pDrv2605data->wklock);			
				}
				switch_set_state(&pDrv2605data->sw_dev, SW_STATE_RTP_PLAYBACK);
				pDrv2605data->vibrator_is_playing = YES;
                if(pDrv2605data->repeat_times > 0)
					pDrv2605data->repeat_times--;
                if (pDrv2605data->vibration_time > MAX_TIMEOUT)
                    pDrv2605data->vibration_time = MAX_TIMEOUT;
				drv2605_change_mode(pDrv2605data, WORK_PATTERN_RTP_ON, DEV_READY);
				drv2605_set_rtp_val(pDrv2605data, strength);
				
                hrtimer_start(&pDrv2605data->timer, ns_to_ktime((u64)pDrv2605data->vibration_time * NSEC_PER_MSEC), HRTIMER_MODE_REL);
            }
            break;
        }		
 		
		case HAPTIC_CMDID_RTP_SEQUENCE:
		{
            memset(&pDrv2605data->RTPSeq, 0, sizeof(struct RTP_Seq));
			if(((len-1)%2) == 0){
				pDrv2605data->RTPSeq.RTPCounts = (len-1)/2;
				if((pDrv2605data->RTPSeq.RTPCounts <= MAX_RTP_SEQ)&&(pDrv2605data->RTPSeq.RTPCounts>0)){
					if(copy_from_user(pDrv2605data->RTPSeq.RTPData, &buff[1], pDrv2605data->RTPSeq.RTPCounts*2) != 0){
						printk("%s, rtp_seq copy seq err\n", __FUNCTION__);	
						break;
					}
					
					if(pDrv2605data->audio_haptics_enabled == NO){
						wake_lock(&pDrv2605data->wklock);
					}
					switch_set_state(&pDrv2605data->sw_dev, SW_STATE_RTP_PLAYBACK);
					drv2605_change_mode(pDrv2605data, WORK_SEQ_RTP_OFF, DEV_IDLE);
					schedule_work(&pDrv2605data->vibrator_work);
				}else{
					printk("%s, rtp_seq count error,maximum=%d\n", __FUNCTION__,MAX_RTP_SEQ);
				}
			}else{
				printk("%s, rtp_seq len error\n", __FUNCTION__);
			}
			break;
		}
		
        case HAPTIC_CMDID_STOP:
        {
            break;
        }
		
        case HAPTIC_CMDID_AUDIOHAPTIC_ENABLE:
        {
			if(pDrv2605data->audio_haptics_enabled == NO){
				wake_lock(&pDrv2605data->wklock);
			}
			pDrv2605data->audio_haptics_enabled = YES;
			setAudioHapticsEnabled(pDrv2605data, YES);
            break;
        }
		
        case HAPTIC_CMDID_AUDIOHAPTIC_DISABLE:
        {
			if(pDrv2605data->audio_haptics_enabled == YES){
				pDrv2605data->audio_haptics_enabled = NO;
				wake_unlock(&pDrv2605data->wklock);	
			}
            break;
        }
		
		case HAPTIC_CMDID_REG_READ:
		{
			if(len == 2){
				pDrv2605data->ReadLen = 1;
				pDrv2605data->ReadBuff[0] = drv2605_reg_read(pDrv2605data, buff[1]);
			}else if(len == 3){
				pDrv2605data->ReadLen = (buff[2]>MAX_READ_BYTES)?MAX_READ_BYTES:buff[2];
				drv2605_bulk_read(pDrv2605data, buff[1], pDrv2605data->ReadLen, pDrv2605data->ReadBuff);
			}else{
				printk("%s, reg_read len error\n", __FUNCTION__);
			}
			break;
		}
		
		case HAPTIC_CMDID_REG_WRITE:
		{
			if((len-1) == 2){
				drv2605_reg_write(pDrv2605data, buff[1], buff[2]);	
			}else if((len-1)>2){
				unsigned char *data = (unsigned char *)kzalloc(len-2, GFP_KERNEL);
				if(data != NULL){
					if(copy_from_user(data, &buff[2], len-2) != 0){
						printk("%s, reg copy err\n", __FUNCTION__);	
					}else{
						drv2605_bulk_write(pDrv2605data, buff[1], len-2, data);
					}
					kfree(data);
				}
			}else{
				printk("%s, reg_write len error\n", __FUNCTION__);
			}
			break;
		}
		
		case HAPTIC_CMDID_REG_SETBIT:
		{
			int i=1;			
			for(i=1; i< len; ){
				drv2605_set_bits(pDrv2605data, buff[i], buff[i+1], buff[i+2]);
				i += 3;
			}
			break;
		}		
    default:
		printk("%s, unknown HAPTIC cmd\n", __FUNCTION__);
      break;
    }

    mutex_unlock(&pDrv2605data->lock);

    return len;
}


static struct file_operations fops =
{
	.open = dev2605_open,
    .read = dev2605_read,
    .write = dev2605_write,
};

/*
void drv2605_early_suspend(struct early_suspend *h){
	struct drv2605_data *pDrv2605data = container_of(h, struct drv2605_data, early_suspend); 

	pDrv2605data->should_stop = YES;	
	hrtimer_cancel(&pDrv2605data->timer);
	cancel_work_sync(&pDrv2605data->vibrator_work);
	
	mutex_lock(&pDrv2605data->lock);	
	
	drv2605_stop(pDrv2605data);
	if(pDrv2605data->audio_haptics_enabled == YES){
		wake_unlock(&pDrv2605data->wklock);
	}
	
	mutex_unlock(&pDrv2605data->lock);
    return ;
}

void drv2605_late_resume(struct early_suspend *h) {
	struct drv2605_data *pDrv2605data = container_of(h, struct drv2605_data, early_suspend); 
	
	mutex_lock(&pDrv2605data->lock);	
	if(pDrv2605data->audio_haptics_enabled == YES){
		wake_lock(&pDrv2605data->wklock);
		setAudioHapticsEnabled(pDrv2605data, YES);
	}
	mutex_unlock(&pDrv2605data->lock);
    return ; 
 }
 */
static int Haptics_init(struct drv2605_data *pDrv2605data)
{
    int reval = -ENOMEM;
    printk(KERN_ERR"into Haptics_init"); 
    pDrv2605data->version = MKDEV(0,0);
    reval = alloc_chrdev_region(&pDrv2605data->version, 0, 1, HAPTICS_DEVICE_NAME);
    if (reval < 0)
    {
        printk(KERN_ALERT"drv2605: error getting major number %d\n", reval);
        goto fail0;
    }

    pDrv2605data->class = class_create(THIS_MODULE, HAPTICS_DEVICE_NAME);
    if (!pDrv2605data->class)
    {
        printk(KERN_ALERT"drv2605: error creating class\n");
        goto fail1;
    }

    pDrv2605data->device = device_create(pDrv2605data->class, NULL, pDrv2605data->version, NULL, HAPTICS_DEVICE_NAME);
    if (!pDrv2605data->device)
    {
        printk(KERN_ALERT"drv2605: error creating device 2605\n");
        goto fail2;
    }

    cdev_init(&pDrv2605data->cdev, &fops);
    pDrv2605data->cdev.owner = THIS_MODULE;
    pDrv2605data->cdev.ops = &fops;
    reval = cdev_add(&pDrv2605data->cdev, pDrv2605data->version, 1);
    if (reval)
    {
        printk(KERN_ALERT"drv2605: fail to add cdev\n");
        goto fail3;
    }

	pDrv2605data->sw_dev.name = "haptics";
	reval = switch_dev_register(&pDrv2605data->sw_dev);
	if (reval < 0) {
		printk(KERN_ALERT"drv2605: fail to register switch\n");
		goto fail4;
	}
//add by yesichao
#ifdef CONFIG_FEATURE_ZTEMT_DRV2605_COMPATIBLE
	pDrv2605data->to_dev.name = "vibrator_drv2605";
#else
	pDrv2605data->to_dev.name = "vibrator";
#endif
	pDrv2605data->to_dev.get_time = vibrator_get_time;
	pDrv2605data->to_dev.enable = vibrator_enable;

    if (timed_output_dev_register(&(pDrv2605data->to_dev)) < 0)
    {
        printk(KERN_ALERT"drv2605: fail to create timed output dev\n");
        goto fail3;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
 //   pDrv2605data->early_suspend.suspend = drv2605_early_suspend;
//	pDrv2605data->early_suspend.resume = drv2605_late_resume;
//	pDrv2605data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
//	register_early_suspend(&pDrv2605data->early_suspend);
#endif  
	
    hrtimer_init(&pDrv2605data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    pDrv2605data->timer.function = vibrator_timer_func;
    INIT_WORK(&pDrv2605data->vibrator_work, vibrator_work_routine);
    
    wake_lock_init(&pDrv2605data->wklock, WAKE_LOCK_SUSPEND, "vibrator");
    mutex_init(&pDrv2605data->lock);
	
    return 0;

fail4:
	switch_dev_unregister(&pDrv2605data->sw_dev);
fail3:
	device_destroy(pDrv2605data->class, pDrv2605data->version);
fail2:
    class_destroy(pDrv2605data->class);	
fail1:
    unregister_chrdev_region(pDrv2605data->version, 1);	
fail0:
    return reval;
}

static void dev_init_platform_data(struct drv2605_data *pDrv2605data)
{
	struct drv2605_platform_data *pDrv2605Platdata = &pDrv2605data->PlatData;
	struct actuator_data actuator = pDrv2605Platdata->actuator;
	struct audio2haptics_data a2h = pDrv2605Platdata->a2h;
	unsigned char temp = 0;
	
	drv2605_select_library(pDrv2605data, actuator.g_effect_bank);
	
	//OTP memory saves data from 0x16 to 0x1a
	if(pDrv2605data->OTP == 0) {
		if(actuator.rated_vol != 0){
			drv2605_reg_write(pDrv2605data, RATED_VOLTAGE_REG, actuator.rated_vol);
		}else{
			printk("%s, ERROR Rated ZERO\n", __FUNCTION__);
		}

		if(actuator.over_drive_vol != 0){
			drv2605_reg_write(pDrv2605data, OVERDRIVE_CLAMP_VOLTAGE_REG, actuator.over_drive_vol);
		}else{
			printk("%s, ERROR OverDriveVol ZERO\n", __FUNCTION__);
		}
		
		drv2605_set_bits(pDrv2605data, 
						FEEDBACK_CONTROL_REG, 
						FEEDBACK_CONTROL_DEVICE_TYPE_MASK, 
						(actuator.device_type == LRA)?FEEDBACK_CONTROL_MODE_LRA:FEEDBACK_CONTROL_MODE_ERM);
	}else{
		printk("%s, OTP programmed\n", __FUNCTION__);
	}
	
	if(pDrv2605Platdata->loop == OPEN_LOOP){
		temp = BIDIR_INPUT_BIDIRECTIONAL;
	}else{
		if(pDrv2605Platdata->BIDIRInput == UniDirectional){
			temp = BIDIR_INPUT_UNIDIRECTIONAL;
		}else{
			temp = BIDIR_INPUT_BIDIRECTIONAL;
		}
	}

	if(actuator.device_type == LRA){
		unsigned char DriveTime = 5*(1000 - actuator.LRAFreq)/actuator.LRAFreq;
		drv2605_set_bits(pDrv2605data, 
				Control1_REG, 
				Control1_REG_DRIVE_TIME_MASK, 
				DriveTime);	
		printk("%s, LRA = %d, DriveTime=0x%x\n", __FUNCTION__, actuator.LRAFreq, DriveTime);
	}
	
	drv2605_set_bits(pDrv2605data, 
				Control2_REG, 
				Control2_REG_BIDIR_INPUT_MASK, 
				temp);	
		
	if((pDrv2605Platdata->loop == OPEN_LOOP)&&(actuator.device_type == LRA))
	{
		temp = LRA_OpenLoop_Enabled;
	}
	else if((pDrv2605Platdata->loop == OPEN_LOOP)&&(actuator.device_type == ERM))
	{
		temp = ERM_OpenLoop_Enabled;
	}
	else
	{
		temp = ERM_OpenLoop_Disable|LRA_OpenLoop_Disable;
	}

	if((pDrv2605Platdata->loop == CLOSE_LOOP) &&(pDrv2605Platdata->BIDIRInput == UniDirectional))
	{
		temp |= RTP_FORMAT_UNSIGNED;
		drv2605_reg_write(pDrv2605data, REAL_TIME_PLAYBACK_REG, 0xff);
	}
	else
	{
		if(pDrv2605Platdata->RTPFormat == Signed)
		{
			temp |= RTP_FORMAT_SIGNED;
			drv2605_reg_write(pDrv2605data, REAL_TIME_PLAYBACK_REG, 0x7f);
		}
		else
		{
			temp |= RTP_FORMAT_UNSIGNED;
			drv2605_reg_write(pDrv2605data, REAL_TIME_PLAYBACK_REG, 0xff);
		}
	}
	drv2605_set_bits(pDrv2605data, 
					Control3_REG, 
					Control3_REG_LOOP_MASK|Control3_REG_FORMAT_MASK, 
					temp);	

	//for audio to haptics
	if(pDrv2605Platdata->GpioTrigger == 0)	//not used as external trigger
	{
		drv2605_reg_write(pDrv2605data, AUDIO_HAPTICS_MIN_INPUT_REG, a2h.a2h_min_input);
		drv2605_reg_write(pDrv2605data, AUDIO_HAPTICS_MAX_INPUT_REG, a2h.a2h_max_input);
		drv2605_reg_write(pDrv2605data, AUDIO_HAPTICS_MIN_OUTPUT_REG, a2h.a2h_min_output);
		drv2605_reg_write(pDrv2605data, AUDIO_HAPTICS_MAX_OUTPUT_REG, a2h.a2h_max_output);
	}
}

static int dev_auto_calibrate(struct drv2605_data *pDrv2605data)
{
	int err = 0, status=0;
	
	drv2605_change_mode(pDrv2605data, WORK_CALIBRATION, DEV_READY);
	drv2605_set_go_bit(pDrv2605data, GO);
			
	/* Wait until the procedure is done */
	drv2605_poll_go_bit(pDrv2605data);
	/* Read status */
	status = drv2605_reg_read(pDrv2605data, STATUS_REG);

	printk("%s, calibration status =0x%x\n", __FUNCTION__, status);

	/* Read calibration results */
	drv2605_reg_read(pDrv2605data, AUTO_CALI_RESULT_REG);
	drv2605_reg_read(pDrv2605data, AUTO_CALI_BACK_EMF_RESULT_REG);
	drv2605_reg_read(pDrv2605data, FEEDBACK_CONTROL_REG);
	
	return err;
}

/*
static struct regmap_config drv2605_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};
*/

static int drv2605_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	struct drv2605_data *pDrv2605data;
//	struct drv2605_platform_data *pDrv2605Platdata = client->dev.platform_data;
	
	int err = 0;
	int status = 0;

	
        printk(KERN_ERR"into drv2605_probe"); 
	struct device_node *of_node = NULL;
	printk("---drv2605_probe\n");
	//xiao comment begin 
	/* 
	//of_node=client->dev.of_node;//xiaofeng comment
	of_node = of_find_compatible_node(NULL, NULL, "ti,drv2605");//xiaofeng add 
	//drv2605_plat_data. GpioEnable= of_get_named_gpio(of_node, "ti,enable-gpio", 0);//xiaofeng comment
	if(of_node){
		printk("xiaofeng into if of_node \n");
    of_property_read_u32_array(node, "enable-gpio",&(drv2605_plat_data.GpioEnable),1);
	}
	if (drv2605_plat_data. GpioEnable < 0) {
		printk("[drv2605]%s: Can't get drv2605-enable-gpio\n",__func__);
		return -EINVAL;
	}
	*/
	//xiao comment end
	printk("drv2605 gpio enable :%d\n",drv2605_plat_data. GpioEnable);

	printk("---drv2605_probe\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR"%s:I2C check failed\n", __FUNCTION__);
		return -ENODEV;
	}

	pDrv2605data = devm_kzalloc(&client->dev, sizeof(struct drv2605_data), GFP_KERNEL);
	if (pDrv2605data == NULL){
		printk(KERN_ERR"%s:no memory\n", __FUNCTION__);
		return -ENOMEM;
	}

	//pDrv2605data->regmap = devm_regmap_init_i2c(client, &drv2605_i2c_regmap);
	pDrv2605data->client = client;
/*	if (IS_ERR(pDrv2605data->regmap)) {
		err = PTR_ERR(pDrv2605data->regmap);
		printk(KERN_ERR"%s:Failed to allocate register map: %d\n",__FUNCTION__,err);
		return err;
	}*/

	memcpy(&pDrv2605data->PlatData, &drv2605_plat_data, sizeof(struct drv2605_platform_data));
	i2c_set_clientdata(client,pDrv2605data);

#ifdef LINUX_NATIVE_GPIO
    	printk(KERN_ERR"defined LINUX_NATIVE_GPIO"); 
	if(pDrv2605data->PlatData.GpioTrigger){
		err = gpio_request(pDrv2605data->PlatData.GpioTrigger,HAPTICS_DEVICE_NAME"Trigger");
		if(err < 0){
			printk(KERN_ERR"%s: GPIO request Trigger error\n", __FUNCTION__);				
			goto exit_gpio_request_failed;
		}
	}

	if(pDrv2605data->PlatData.GpioEnable){
		err = gpio_request(pDrv2605data->PlatData.GpioEnable,HAPTICS_DEVICE_NAME"Enable");
		if(err < 0){
			printk(KERN_ERR"%s: GPIO request enable error\n", __FUNCTION__);					
			goto exit_gpio_request_failed;
		}

	    /* Enable power to the chip */
	    gpio_direction_output(pDrv2605data->PlatData.GpioEnable, 1);


	    /* Wait 30 us */
	    udelay(30);
	}
	
#else
	//GpioTrigger
	/*
	mt_set_gpio_mode(pDrv2605data->PlatData.GpioTrigger, GPIO_IRQ_NFC_PIN_M_EINT);
	mt_set_gpio_dir(pDrv2605data->PlatData.GpioTrigger, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(pDrv2605data->PlatData.GpioTrigger, GPIO_PULL_ENABLE);
	//mt_set_gpio_pull_select(pDrv2605data->PlatData.GpioTrigger, GPIO_PULL_DOWN);
	mt_set_gpio_out(pDrv2605data->PlatData.GpioTrigger, 0);
	*/

    	printk(KERN_ERR"undefined LINUX_NATIVE_GPIO"); 
	//GpioEnable
	mt_set_gpio_mode((pDrv2605data->PlatData.GpioEnable | 0x80000000), 0);//#define GPIO_SPEAKER_EN_PIN_M_GPIO   GPIO_MODE_00; #define GPIO_MODE_00 0
	mt_set_gpio_dir((pDrv2605data->PlatData.GpioEnable | 0x80000000), 1);//GPIO_DIR_OUT=1
	mt_set_gpio_pull_enable((pDrv2605data->PlatData.GpioEnable | 0x80000000), 1);//GPIO_PULL_ENABLE=1
	//mt_set_gpio_pull_select(pDrv2605data->PlatData.GpioEnable, GPIO_PULL_DOWN);
	mt_set_gpio_out((pDrv2605data->PlatData.GpioEnable | 0x80000000), 1);//xiaofeng mod 0 to 1
	/* Wait 30 us */
	udelay(30);	
#endif

	err = drv2605_reg_read(pDrv2605data, STATUS_REG);
	if(err < 0){
		printk("%s, i2c bus fail (%d)\n", __FUNCTION__, err);
		goto exit_gpio_request_failed;
	}else{
		printk("%s, i2c status (0x%x)\n", __FUNCTION__, err);
		status = err;
	}
	/* Read device ID */
	pDrv2605data->device_id = (status & DEV_ID_MASK);
	switch (pDrv2605data->device_id)
	{
		case DRV2605_VER_1DOT1:
		printk("drv2605 driver found: drv2605 v1.1.\n");
		break;
		case DRV2605_VER_1DOT0:
		printk("drv2605 driver found: drv2605 v1.0.\n");
		break;
		case DRV2604:
		printk(KERN_ALERT"drv2605 driver found: drv2604.\n");
		break;
		default:
		printk(KERN_ERR"drv2605 driver found: unknown.\n");
		break;
	}

	if((pDrv2605data->device_id != DRV2605_VER_1DOT1)
		&&(pDrv2605data->device_id != DRV2605_VER_1DOT0)){
		printk("%s, status(0x%x),device_id(%d) fail\n",
			__FUNCTION__, status, pDrv2605data->device_id);
		goto exit_gpio_request_failed;
	}

	drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_READY);
	schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));
	
	pDrv2605data->OTP = drv2605_reg_read(pDrv2605data, AUTOCAL_MEM_INTERFACE_REG) & AUTOCAL_MEM_INTERFACE_REG_OTP_MASK;
	
	dev_init_platform_data(pDrv2605data);
	
	if(pDrv2605data->OTP == 0){
		err = dev_auto_calibrate(pDrv2605data);
		if(err < 0){
			printk("%s, ERROR, calibration fail\n",	__FUNCTION__);
		}
	}

    /* Put hardware in standby */
    drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_STANDBY);

    Haptics_init(pDrv2605data);
	
	pDRV2605data = pDrv2605data;
    printk("drv2605 probe succeeded\n");

    return 0;

exit_gpio_request_failed:
	if(pDrv2605data->PlatData.GpioTrigger){
		gpio_free(pDrv2605data->PlatData.GpioTrigger);
	}

	if(pDrv2605data->PlatData.GpioEnable){
		gpio_free(pDrv2605data->PlatData.GpioEnable);
	}
	
    printk(KERN_ERR"%s failed, err=%d\n",__FUNCTION__, err);
	return err;
}

static int drv2605_remove(struct i2c_client* client)
{
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

    device_destroy(pDrv2605data->class, pDrv2605data->version);
    class_destroy(pDrv2605data->class);
    unregister_chrdev_region(pDrv2605data->version, 1);

	if(pDrv2605data->PlatData.GpioTrigger)
		gpio_free(pDrv2605data->PlatData.GpioTrigger);

	if(pDrv2605data->PlatData.GpioEnable)
		gpio_free(pDrv2605data->PlatData.GpioEnable);

//xiaofeng comment due to no early_suspend member 
/*
#ifdef CONFIG_HAS_EARLYSUSPEND		
	unregister_early_suspend(&pDrv2605data->early_suspend);
#endif
*/

    printk(KERN_ALERT"drv2605 remove");
	
    return 0;
}


MODULE_DEVICE_TABLE(i2c, drv2605_id_table);


static struct i2c_device_id drv2605_id_table[] =
{
    { HAPTICS_DEVICE_NAME, 0 },//xiaofeng mod from 0x5a -> 0
    {}
};


static struct i2c_driver drv2605_driver =
{
    .driver = {
        .name = HAPTICS_DEVICE_NAME,
		.owner = THIS_MODULE,
		//.of_match_table = ti_match_table,
    },
    .id_table = drv2605_id_table,
    .probe = drv2605_probe,
    .remove = drv2605_remove,
};


/*----------------------------------------------------------------------------*/
static int drv2605_platform_probe(struct platform_device *pdev) 
{
	printk(KERN_ERR" mode id_table.mod i2c name.into drv2605_platform_probe\n");
	if(i2c_add_driver(&drv2605_driver))
	{
		printk(KERN_ERR" drv2605_platform_probe add driver error\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int drv2605_platform_remove(struct platform_device *pdev)
{
	printk(KERN_ERR"into drv2605_platform_remove\n");
    i2c_del_driver(&drv2605_driver);
    return 0;
}





static const struct of_device_id ti_match_table[] = {
	{.compatible = DRV2605_NAME},
	{}
};



static struct platform_driver drv2605_platform_driver = {
	.probe      = drv2605_platform_probe,
	.remove     = drv2605_platform_remove,    
	.driver     = 
	{
		.name  = "drv2605",
	//	.owner  = THIS_MODULE,
        #ifdef CONFIG_OF
		.of_match_table = ti_match_table,
	#endif
	}
};

static int __init drv2605_init(void)
{
	int status = 0;
	int err = 0;
	struct i2c_adapter *i2c3_host;
	i2c3_host = i2c_get_adapter(4);
	drv_client = i2c_new_device(i2c3_host,&drv2605_i2c_info);	
	printk(KERN_ERR"before i2c_register_board_info \n");
	//status = i2c_register_board_info(DRV2605_I2C_BUSNUM, &drv2605_i2c_info,1);//xiaofeng
	printk(KERN_ERR"status = %d.after i2c_register_board_info,before  i2c_add_driver\n",status);
	//err = i2c_add_driver(&drv2605_driver);
	//printk(KERN_ERR"xiaofeng:err =%d. i2c_add_driver return value\n",err);
	//i2c_put_adapter;
	if(err = platform_driver_register(&drv2605_platform_driver))
	{
		printk(KERN_ERR"failed to register drv2605_platform_driver,err value=%d\n",err);
		return -ENODEV;
	}
	return err;
}

static void __exit drv2605_exit(void)
{
	i2c_del_driver(&drv2605_driver);
}

module_init(drv2605_init);
module_exit(drv2605_exit);

MODULE_AUTHOR("wireless section: xiaofeng");
MODULE_DESCRIPTION("Driver for "HAPTICS_DEVICE_NAME);
