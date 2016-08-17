/* drivers/hwmon/mt6516/amit/APDS9930.c - APDS9930 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>

#include <alsps.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "APDS9930.h"
#include <linux/sched.h>
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define APDS9930_DEV_NAME     "APDS9930"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args) 

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);

/*----------------------------------------------------------------------------*/
static struct i2c_client *APDS9930_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id APDS9930_i2c_id[] = {{APDS9930_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_APDS9930={ I2C_BOARD_INFO("APDS9930", 0x39)};
/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int APDS9930_i2c_remove(struct i2c_client *client);
static int APDS9930_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int APDS9930_i2c_resume(struct i2c_client *client);
static int APDS9930_remove(void);
static int APDS9930_local_init(void);
static int APDS9930_prox_offset_cal_process(void);
static int APDS9930_read_cal_file(char *file_path);
static int APDS9930_write_cal_file(char *file_path,unsigned int value);
static int APDS9930_load_calibration_data(struct i2c_client *client);

static int APDS9930_init_flag =-1; // 0<==>OK -1 <==> fail
static struct alsps_init_info APDS9930_init_info = {
		.name = "APDS9930",
		.init = APDS9930_local_init,
		.uninit = APDS9930_remove,
};


static DEFINE_MUTEX(APDS9930_mutex);


static struct APDS9930_priv *g_APDS9930_ptr = NULL;

 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;
static unsigned long long int_top_time = 0;
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct APDS9930_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct APDS9930_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;
    struct delayed_work ps_cal_poll_work;

#if defined(CONFIG_OF)
	struct device_node *irq_node;
	int irq;
#endif

    /*i2c address group*/
    struct APDS9930_i2c_addr  addr;
    
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
	int		ps_cali;
	bool		debug;
	bool		prox_cal_valid;
	u32		prox_threshold_hi;
	u32		prox_threshold_lo;
	u32		prox_thres_hi_max;
	u32		prox_thres_hi_min;
	u32		prox_thres_lo_max;
	u32		prox_thres_lo_min;
	u32		prox_uncover_data;
	u32		prox_offset;
	u32		prox_manual_calibrate_threshold;

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver APDS9930_i2c_driver = {	
	.probe      = APDS9930_i2c_probe,
	.remove     = APDS9930_i2c_remove,
	.detect     = APDS9930_i2c_detect,
	.suspend    = APDS9930_i2c_suspend,
	.resume     = APDS9930_i2c_resume,
	.id_table   = APDS9930_i2c_id,
	.driver = {
		.name           = APDS9930_DEV_NAME,
	},
};

static struct APDS9930_priv *APDS9930_obj = NULL;
static struct platform_driver APDS9930_alsps_driver;
/*------------------------i2c function for 89-------------------------------------*/
int APDS9930_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag)
{
	int res = 0;
	mutex_lock(&APDS9930_mutex);
	switch(i2c_flag){	
	case I2C_FLAG_WRITE:
	client->addr &=I2C_MASK_FLAG;
	res = i2c_master_send(client, buf, count);
	client->addr &=I2C_MASK_FLAG;
	break;
	
	case I2C_FLAG_READ:
	client->addr &=I2C_MASK_FLAG;
	client->addr |=I2C_WR_FLAG;
	client->addr |=I2C_RS_FLAG;
	res = i2c_master_send(client, buf, count);
	client->addr &=I2C_MASK_FLAG;
	break;
	default:
	APS_LOG("APDS9930_i2c_master_operate i2c_flag command not support!\n");
	break;
	}
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	mutex_unlock(&APDS9930_mutex);
	return res;
	EXIT_ERR:
	mutex_unlock(&APDS9930_mutex);
	APS_ERR("APDS9930_i2c_transfer fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
int APDS9930_get_addr(struct alsps_hw *hw, struct APDS9930_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void APDS9930_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "APDS9930")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "APDS9930")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}
/*----------------------------------------------------------------------------*/
static long APDS9930_enable_als(struct i2c_client *client, int enable)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];	  
	long res = 0;

	databuf[0]= APDS9930_CMM_ENABLE;
	res = APDS9930_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//APS_LOG("APDS9930_CMM_ENABLE als value = %x\n",databuf[0]);
	
	if(enable)
		{
			databuf[1] = databuf[0]|0x03;
			databuf[0] = APDS9930_CMM_ENABLE;
			//APS_LOG("APDS9930_CMM_ENABLE enable als value = %x\n",databuf[1]);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
		}
	else {
		if(test_bit(CMC_BIT_PS, &obj->enable))
			databuf[1] = databuf[0]&0xFD;
		else
			databuf[1] = databuf[0]&0xF8;
		
			databuf[0] = APDS9930_CMM_ENABLE;
			//APS_LOG("APDS9930_CMM_ENABLE disable als value = %x\n",databuf[1]);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
	return 0;
		
EXIT_ERR:
	APS_ERR("APDS9930_enable_als fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static long APDS9930_enable_ps(struct i2c_client *client, int enable)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	long res = 0;
	
	databuf[0]= APDS9930_CMM_ENABLE;
	res = APDS9930_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	//APS_LOG("APDS9930_CMM_ENABLE ps value = %x\n",databuf[0]);
	
	if(enable)
		{
			databuf[1] = databuf[0]|0x05;
			databuf[0] = APDS9930_CMM_ENABLE;
			//APS_LOG("APDS9930_CMM_ENABLE enable ps value = %x\n",databuf[1]);	
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->ps_deb_on, 1);
			atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
			if(obj->prox_cal_valid == false)
			{
				res = APDS9930_load_calibration_data(client);
				if(res >= 0)
				{
					obj->prox_cal_valid = true;
				}
			}
		}
	else{
		if(test_bit(CMC_BIT_ALS, &obj->enable))
			databuf[1] = databuf[0]&0xFB;
		else
			databuf[1] = databuf[0]&0xF8;
		
			databuf[0] = APDS9930_CMM_ENABLE;
			//APS_LOG("APDS9930_CMM_ENABLE disable ps value = %x\n",databuf[1]);	
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			/*fix bug*/
			databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
			databuf[1] = (u8)(750 & 0x00FF);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)((750 & 0xFF00) >> 8);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)(900 & 0x00FF);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH;	
			databuf[1] = (u8)((900 & 0xFF00) >> 8);;
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			/*fix bug*/
		}
	return 0;
	
EXIT_ERR:
	APS_ERR("APDS9930_enable_ps fail\n");
	return res;
}

/*add by yesichao start*/
/*----------------------------------------------------------------------------*/
static int APDS9930_read_cal_file(char *file_path)
{
    struct file *file_p;
    int vfs_read_retval = 0;
    mm_segment_t old_fs;
    char read_buf[32];
    unsigned short read_value;

    if (NULL==file_path)
    {
        APS_ERR("file_path is NULL\n");
        goto error;
    }

    memset(read_buf, 0, 32);

    file_p = filp_open(file_path, O_RDONLY , 0);
    if (IS_ERR(file_p))
    {
        APS_ERR("[open file <%s>failed]\n",file_path);
        goto error;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    vfs_read_retval = vfs_read(file_p, (char*)read_buf, 16, &file_p->f_pos);
    if (vfs_read_retval < 0)
    {
        APS_ERR("[read file <%s>failed]\n",file_path);
        goto error;
    }

    set_fs(old_fs);
    filp_close(file_p, NULL);

    if (kstrtou16(read_buf, 10, &read_value) < 0)
    {
        APS_ERR("[kstrtou16 %s failed]\n",read_buf);
        goto error;
    }

    APS_ERR("[the content of %s is %s]\n", file_path, read_buf);

    return read_value;

error:
    return -1;
}

static int APDS9930_write_cal_file(char *file_path,unsigned int value)
{
    struct file *file_p;
    char write_buf[10];
	 mm_segment_t old_fs;
    int vfs_write_retval=0;
    if (NULL==file_path)
    {
        APS_ERR("file_path is NULL\n");

    }
       memset(write_buf, 0, sizeof(write_buf));
      sprintf(write_buf, "%d\n", value);
    file_p = filp_open(file_path, O_CREAT|O_RDWR , 0665);
    if (IS_ERR(file_p))
    {
        APS_ERR("[open file <%s>failed]\n",file_path);
        goto error;
    }
    old_fs = get_fs();
    set_fs(KERNEL_DS);

    vfs_write_retval = vfs_write(file_p, (char*)write_buf, sizeof(write_buf), &file_p->f_pos);
    if (vfs_write_retval < 0)
    {
        APS_ERR("[write file <%s>failed]\n",file_path);
        goto error;
    }

    set_fs(old_fs);
    filp_close(file_p, NULL);

    return 1;

error:
    return -1;
}

static int APDS9930_read_file(char *filename,u8* param)
{
	struct file  *fop;
	mm_segment_t old_fs;

	fop = filp_open(filename,O_RDONLY,0);
	if(IS_ERR(fop))
	{
		APS_LOG("Filp_open error!! Path = %s\n",filename);
		return -1;
	}

	old_fs = get_fs();
	set_fs(get_ds()); //set_fs(KERNEL_DS);

	fop->f_op->llseek(fop,0,0);
	fop->f_op->read(fop, param, strlen(param), &fop->f_pos);

	set_fs(old_fs);

	filp_close(fop,NULL);

	return 0;

}

static ssize_t APDS9930_write_file(char *filename,u8* param)
{
	struct file  *fop;
	mm_segment_t old_fs;

	fop = filp_open(filename,O_CREAT | O_RDWR,0666);
	if(IS_ERR(fop))
	{
		APS_LOG("Create file error!! Path = %s\n",filename);
		return -1;
	}

	old_fs = get_fs();
	set_fs(get_ds()); //set_fs(KERNEL_DS);
	fop->f_op->write(fop, (char *)param, sizeof(param), &fop->f_pos);
	set_fs(old_fs);

	filp_close(fop,NULL);

	return 0;
}

static int APDS9930_load_calibration_data(struct i2c_client *client)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;

	APS_DBG("%s\n", __FUNCTION__);
	obj->prox_uncover_data = APDS9930_read_cal_file(PATH_PROX_UNCOVER_DATA);
	if (obj->prox_uncover_data < 0)
	{
		obj->prox_uncover_data = 0x00;
		APS_ERR("APDS9930_load_calibration_data read PATH_PROX_UNCOVER_DATA error\n");
	}
	if (obj->prox_uncover_data > 0 && obj->prox_uncover_data < APDS9930_DATA_SAFE_RANGE_MAX)
	{
		obj->prox_thres_hi_min = obj->prox_uncover_data + APDS9930_THRESHOLD_SAFE_DISTANCE;
		obj->prox_thres_hi_min = (obj->prox_thres_hi_min > APDS9930_THRESHOLD_HIGH_MIN) ? obj->prox_thres_hi_min : APDS9930_THRESHOLD_HIGH_MIN;

		obj->prox_thres_hi_max = obj->prox_thres_hi_min + APDS9930_THRESHOLD_DISTANCE / 2;
		obj->prox_thres_hi_max = (obj->prox_thres_hi_max > APDS9930_THRESHOLD_HIGH_MAX) ? APDS9930_THRESHOLD_HIGH_MAX : obj->prox_thres_hi_max;

		obj->prox_thres_lo_min = obj->prox_uncover_data + APDS9930_THRESHOLD_DISTANCE;
		obj->prox_thres_lo_min = (obj->prox_thres_lo_min > APDS9930_THRESHOLD_LOW_MIN) ? obj->prox_thres_lo_min : APDS9930_THRESHOLD_LOW_MIN;

		obj->prox_thres_lo_max = obj->prox_thres_lo_min + APDS9930_THRESHOLD_DISTANCE;
		if (obj->prox_thres_lo_max > (obj->prox_thres_hi_min - 100))
		{
			obj->prox_thres_lo_max = (obj->prox_thres_hi_min - 100);
		}
		APS_LOG("get uncover data success\n");
	}
	else
	{
		obj->prox_thres_hi_min = APDS9930_DEFAULT_THRESHOLD_HIGH;
		obj->prox_thres_hi_max = APDS9930_DEFAULT_THRESHOLD_HIGH;
		obj->prox_thres_lo_min = APDS9930_DEFAULT_THRESHOLD_LOW;
		obj->prox_thres_lo_max = APDS9930_DEFAULT_THRESHOLD_LOW;
		APS_ERR("get uncover data failed for data too high\n");
	}
	APS_ERR("prox_thres_hi range is [%d--%d]\n", obj->prox_thres_hi_min, obj->prox_thres_hi_max);
	APS_ERR("prox_thres_lo range is [%d--%d]\n", obj->prox_thres_lo_min, obj->prox_thres_lo_max);

	res = APDS9930_read_cal_file(CAL_THRESHOLD);
	if (res <= 0)
	{
		APS_ERR("APDS9930_load_calibration_data read CAL_THRESHOLD error\n");
		res = APDS9930_write_cal_file(CAL_THRESHOLD, 0);
		if (res < 0)
		{
			APS_ERR("APDS9930_load_calibration_data create CAL_THRESHOLD error\n");
		}
		obj->prox_threshold_hi = APDS9930_DEFAULT_THRESHOLD_HIGH;
		obj->prox_threshold_lo = APDS9930_DEFAULT_THRESHOLD_LOW;
		goto EXIT_ERR;
	}
	else
	{
		APS_LOG("APDS9930_load_calibration_data CAL_THRESHOLD= %d\n", res);
		obj->prox_manual_calibrate_threshold = res;
		obj->prox_threshold_hi = res;

		obj->prox_threshold_hi = (obj->prox_threshold_hi < obj->prox_thres_hi_max) ? obj->prox_threshold_hi : obj->prox_thres_hi_max;
		obj->prox_threshold_hi = (obj->prox_threshold_hi > obj->prox_thres_hi_min) ? obj->prox_threshold_hi : obj->prox_thres_hi_min;
		obj->prox_threshold_lo = obj->prox_threshold_hi - APDS9930_THRESHOLD_DISTANCE;
		obj->prox_threshold_lo = (obj->prox_threshold_lo < obj->prox_thres_lo_max) ? obj->prox_threshold_lo : obj->prox_thres_lo_max;
		obj->prox_threshold_lo = (obj->prox_threshold_lo > obj->prox_thres_lo_min) ? obj->prox_threshold_lo : obj->prox_thres_lo_min;
	}
	APS_ERR("prox_threshold_lo %d-- prox_threshold_hi %d\n", obj->prox_threshold_lo, obj->prox_threshold_hi);
	databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;
	databuf[1] = (u8)(obj->prox_threshold_lo & 0x00FF);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;
	databuf[1] = (u8)((obj->prox_threshold_lo & 0xFF00) >> 8);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;
	databuf[1] = (u8)(obj->prox_threshold_hi & 0x00FF);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH;
	databuf[1] = (u8)((obj->prox_threshold_hi & 0xFF00) >> 8);;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	res = APDS9930_read_cal_file(PATH_PROX_OFFSET);
	if(res < 0)
	{
		APS_ERR("APDS9930_load_calibration_data read PATH_PROX_OFFSET error\n");
		goto EXIT_ERR;
	}
	else
	{
		obj->prox_offset = res;
	}

	databuf[0] = 0x9E;
	databuf[1] = obj->prox_offset;
	APS_ERR("APDS9930_load_calibration_data offset = %d\n", databuf[1]);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	APS_LOG("APDS9930_load_calibration_data done\n");
	return 0;

EXIT_ERR:
	APS_ERR("APDS9930_load_calibration_data fail\n");
	return -1;
}

static int APDS9930_get_psoffset(struct i2c_client *client)
{
        APS_DBG("%s\n", __FUNCTION__);
        return 0;
}

static int APDS9930_run_calibration(struct i2c_client *client)
{
        APS_DBG("%s\n", __FUNCTION__);
        return 0;
}

/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/

static ssize_t APDS9930_show_chip_name(struct device_driver *ddri, char *buf)
{
       ssize_t res;

       APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%s\n", APDS9930_CHIP_NAME);
	return res;
}

static ssize_t APDS9930_show_prox_thres_min(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_THRES_MIN);
	return res;
}

static ssize_t APDS9930_show_prox_thres_max(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_THRES_MAX);
	return res;
}

static ssize_t APDS9930_show_prox_thres(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_obj->prox_threshold_hi);
	return res;
}

static ssize_t APDS9930_store_prox_thres(struct device_driver *ddri, char *buf, size_t count)
{
	ssize_t res;
	 int value = 0;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = sscanf(buf, "%d", &value);

	if (value == 1)
	{
		APDS9930_load_calibration_data(APDS9930_obj->client);
	}

	return count;
}

static ssize_t APDS9930_show_prox_data_max(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_DATA_MAX);
	return res;
}

static ssize_t APDS9930_show_prox_calibrate_start(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_DATA_MAX);
	return res;
}

static ssize_t APDS9930_store_prox_calibrate_start(struct device_driver *ddri, char *buf, size_t count)
{
       ssize_t res;
	 int value = 0;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = sscanf(buf, "%d", &value);
	if (value == 1)
	{
		APDS9930_obj->debug = true;
		schedule_delayed_work(&APDS9930_obj->ps_cal_poll_work, msecs_to_jiffies(100));
		APS_LOG("ps_cal_poll_work scheduled\n");
	}
	else
	{
		APDS9930_obj->debug = false;
		cancel_delayed_work(&APDS9930_obj->ps_cal_poll_work);
		APS_LOG("ps_cal_poll_work cancelled\n");
	}
	return count;
}

static ssize_t APDS9930_show_prox_calibrate_verify(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_DATA_MAX);
	return res;
}

static ssize_t APDS9930_store_prox_calibrate_verify(struct device_driver *ddri, char *buf, size_t count)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	//res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_DATA_MAX);
	return count;
}

static ssize_t APDS9930_show_prox_data_safe_range_min(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_DATA_SAFE_RANGE_MIN);
	return res;
}

static ssize_t APDS9930_show_prox_data_safe_range_max(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_DATA_SAFE_RANGE_MAX);
	return res;
}

static ssize_t APDS9930_show_prox_offset_cal_start(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_DATA_MAX);
	return res;
}

static ssize_t APDS9930_store_prox_offset_cal_start(struct device_driver *ddri, char *buf, size_t count)
{
       ssize_t res;
	int value = 0;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}
	res = sscanf(buf, "%d", &value);
	if (value == 1)
	{
		APDS9930_obj->debug = true;
		schedule_delayed_work(&APDS9930_obj->ps_cal_poll_work, msecs_to_jiffies(100));
		APS_LOG("ps_cal_poll_work scheduled\n");
	}
	else
	{
		APDS9930_obj->debug = false;
		cancel_delayed_work(&APDS9930_obj->ps_cal_poll_work);
		APS_LOG("ps_cal_poll_work cancelled\n");
	}
	return count;
}

static ssize_t APDS9930_show_prox_offset_cal(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_obj->prox_offset);
	return res;
}

static ssize_t APDS9930_store_prox_offset_cal(struct device_driver *ddri, char *buf, size_t count)
{
       ssize_t res;
	 int value = 0;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = sscanf(buf, "%d", &value);

	if (value == 1)
	{
		APDS9930_prox_offset_cal_process();
	}
	
	return count;
}

static ssize_t APDS9930_show_prox_offset_cal_verify(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_DATA_MAX);
	return res;
}

static ssize_t APDS9930_store_prox_offset_cal_verify(struct device_driver *ddri, char *buf, size_t count)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}

	//res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_DATA_MAX);
	return count;
}

static ssize_t APDS9930_show_prox_manual_calibrate_threshold(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!APDS9930_obj)
	{
		APS_ERR("APDS9930_obj is null!!\n");
		return 0;
	}
	APDS9930_load_calibration_data(APDS9930_obj->client);

	res = snprintf(buf, PAGE_SIZE, "%d\n", APDS9930_obj->prox_manual_calibrate_threshold);
	return res;
}

static DRIVER_ATTR(chip_name,     S_IWUGO | S_IRUGO, APDS9930_show_chip_name, NULL);
static DRIVER_ATTR(prox_thres_min,     S_IWUGO | S_IRUGO, APDS9930_show_prox_thres_min, NULL);
static DRIVER_ATTR(prox_thres_max,     S_IWUGO | S_IRUGO, APDS9930_show_prox_thres_max, NULL);
static DRIVER_ATTR(prox_thres,     S_IWUGO | S_IRUGO, APDS9930_show_prox_thres, APDS9930_store_prox_thres);
static DRIVER_ATTR(prox_data_max,     S_IWUGO | S_IRUGO, APDS9930_show_prox_data_max, NULL);
static DRIVER_ATTR(prox_calibrate_start,     S_IWUGO | S_IRUGO, APDS9930_show_prox_calibrate_start, APDS9930_store_prox_calibrate_start);
static DRIVER_ATTR(prox_calibrate_verify,     S_IWUGO | S_IRUGO, APDS9930_show_prox_calibrate_verify, APDS9930_store_prox_calibrate_verify);
static DRIVER_ATTR(prox_data_safe_range_min,     S_IWUGO | S_IRUGO, APDS9930_show_prox_data_safe_range_min, NULL);
static DRIVER_ATTR(prox_data_safe_range_max,     S_IWUGO | S_IRUGO, APDS9930_show_prox_data_safe_range_max, NULL);
static DRIVER_ATTR(prox_offset_cal_start,     S_IWUGO | S_IRUGO, APDS9930_show_prox_offset_cal_start, APDS9930_store_prox_offset_cal_start);
static DRIVER_ATTR(prox_offset_cal,     S_IWUGO | S_IRUGO, APDS9930_show_prox_offset_cal, APDS9930_store_prox_offset_cal);
static DRIVER_ATTR(prox_offset_cal_verify,     S_IWUGO | S_IRUGO, APDS9930_show_prox_offset_cal_verify, APDS9930_store_prox_offset_cal_verify);
static DRIVER_ATTR(prox_manual_calibrate_threshold,     S_IWUGO | S_IRUGO, APDS9930_show_prox_manual_calibrate_threshold, NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *APDS9930_attr_list[] = {
	&driver_attr_chip_name,
	&driver_attr_prox_thres_min,
	&driver_attr_prox_thres_max,
	&driver_attr_prox_thres,
	&driver_attr_prox_data_max,
	&driver_attr_prox_calibrate_start,
	&driver_attr_prox_calibrate_verify,
	&driver_attr_prox_data_safe_range_min,
	&driver_attr_prox_data_safe_range_max,
	&driver_attr_prox_offset_cal_start,
	&driver_attr_prox_offset_cal,
	&driver_attr_prox_offset_cal_verify,
	&driver_attr_prox_manual_calibrate_threshold,
};

/*----------------------------------------------------------------------------*/
static int APDS9930_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(APDS9930_attr_list)/sizeof(APDS9930_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, APDS9930_attr_list[idx])))
		{
			APS_ERR("driver_create_file (%s) = %d\n", APDS9930_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
	static int APDS9930_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(APDS9930_attr_list)/sizeof(APDS9930_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, APDS9930_attr_list[idx]);
	}

	return err;
}
/*----------------------------------------------------------------------------*/

/*add by yesichao end*/

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int APDS9930_check_and_clear_intr(struct i2c_client *client) 
{
	int res,intp,intl;
	u8 buffer[2];

	if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	    return 0;

	buffer[0] = APDS9930_CMM_STATUS;
	res = APDS9930_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	res = 0;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 1;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 1;
		intl = 1;		
	}

	if(1 == res)
	{
		if((1 == intp) && (0 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
		}
		else if((0 == intp) && (1 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
		}
		else
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
		}

		res = APDS9930_i2c_master_operate(client, buffer, 0x1, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		else
		{
			res = 0;
		}
	}

	return res;

EXIT_ERR:
	APS_ERR("APDS9930_check_and_clear_intr fail\n");
	return 1;
}
/*----------------------------------------------------------------------------*/

/*yucong add for interrupt mode support MTK inc 2012.3.7*/
static int APDS9930_check_intr(struct i2c_client *client) 
{
	int res,intp,intl;
	u8 buffer[2];

	if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	return 0;

	buffer[0] = APDS9930_CMM_STATUS;
	res = APDS9930_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = 0;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;		
	}

	return res;

EXIT_ERR:
	APS_ERR("APDS9930_check_intr fail\n");
	return 1;
}

static int APDS9930_clear_intr(struct i2c_client *client) 
{
	int res;
	u8 buffer[2];
	
	buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
	res = APDS9930_i2c_master_operate(client, buffer, 0x1, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}
	return res;

EXIT_ERR:
	APS_ERR("APDS9930_check_and_clear_intr fail\n");
	return 1;
}


/*-----------------------------------------------------------------------------*/
void APDS9930_eint_func(void)
{
	struct APDS9930_priv *obj = g_APDS9930_ptr;
	if(!obj)
	{
		return;
	}
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}

#if defined(CONFIG_OF)
static irqreturn_t APDS9930_eint_handler(int irq, void *desc)
{
	struct APDS9930_priv *obj = g_APDS9930_ptr;

	if(!obj)
		return IRQ_HANDLED;

	disable_irq_nosync(obj->irq);
	APDS9930_eint_func();

	return IRQ_HANDLED;
}
#endif

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int APDS9930_setup_eint(struct i2c_client *client)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);        

	g_APDS9930_ptr = obj;
#if defined(CONFIG_OF)
	int ints[2] = {0};
	int err = 0;

	if(obj == NULL){
		APS_ERR("apds9930_obj is null!\n");
		return -EINVAL;
	}
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, ALS-eint");
#endif

	APS_LOG("apds9930_setup_eint, GPIO_ALS_EINT_PIN = 0x%x.\n",GPIO_ALS_EINT_PIN);//add for test

	/*configure to GPIO function, external interrupt*/

	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

#if defined(CONFIG_OF)
	if(obj->irq_node != NULL){
		of_property_read_u32_array(obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		APS_LOG("ins[0] = %d, ints[1] = %d\n", ints[0], ints[1]);
		mt_gpio_set_debounce(ints[0], ints[1]);

		obj->irq = irq_of_parse_and_map(obj->irq_node, 0);
#if 1	//add for test
		if(obj->irq != 0){
			err = request_irq(obj->irq, APDS9930_eint_handler, IRQF_TRIGGER_NONE, "ALS-eint", NULL);
			if(err < 0){
				APS_ERR("request_irq failed!\n");
				return -EFAULT;
			}else{
				enable_irq(obj->irq);
			}
		}else{
			APS_ERR("irq_of_parse_and_map failed!\n");
			return -EFAULT;
		}
#endif
	}else{
		APS_ERR("apds9930_obj->irq_node is null!\n");
		return -EFAULT;
	}

#elif defined(CUST_EINT_ALS_TYPE)

	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, APDS9930_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);  
#endif

    return 0;
}

/*----------------------------------------------------------------------------*/

static int APDS9930_init_client(struct i2c_client *client)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;

	databuf[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x00);
	res = APDS9930_i2c_master_operate(client, databuf, 0x1, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	databuf[0] = APDS9930_CMM_ENABLE;
	if(obj->hw->polling_mode_ps == 1)
	databuf[1] = 0x08;
	if(obj->hw->polling_mode_ps == 0)
	databuf[1] = 0x28;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	databuf[0] = APDS9930_CMM_ATIME;    
	databuf[1] = 0xF6;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	databuf[0] = APDS9930_CMM_PTIME;    
	databuf[1] = 0xFF;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	databuf[0] = APDS9930_CMM_WTIME;    
	databuf[1] = 0xFF;//0xFC;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(0 == obj->hw->polling_mode_ps)
	{
		if(1 == ps_cali.valid)
		{
			databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
			databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)(ps_cali.close & 0x00FF);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH;	
			databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		else
		{
			databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
			databuf[1] = (u8)(obj->prox_threshold_lo& 0x00FF);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)((obj->prox_threshold_lo & 0xFF00) >> 8);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)(obj->prox_threshold_hi & 0x00FF);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH;	
			databuf[1] = (u8)((obj->prox_threshold_hi & 0xFF00) >> 8);;
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}

		}

//#ifdef MAXOFFSET //max offset. add for test
//		databuf[0] = 0x9E;
//		databuf[1] = 0x7F;
//		res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
//#endif

		databuf[0] = APDS9930_CMM_Persistence;
		databuf[1] = 0x20;
		res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}

	}

	databuf[0] = APDS9930_CMM_CONFIG;    
	databuf[1] = 0x00;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

       /*Lenovo-sw chenlj2 add 2011-06-03,modified pulse 2  to 4 */
	databuf[0] = APDS9930_CMM_PPCOUNT;    
	databuf[1] = APDS9930_CMM_PPCOUNT_VALUE;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

        /*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16  to 1 */
	databuf[0] = APDS9930_CMM_CONTROL;    
	databuf[1] = APDS9930_CMM_CONTROL_VALUE;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if((res = APDS9930_setup_eint(client))!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	if((res = APDS9930_check_and_clear_intr(client)))
	{
		APS_ERR("check/clear intr: %d\n", res);
	    return res;
	}
	
	return APDS9930_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int APDS9930_read_als(struct i2c_client *client, u16 *data)
{	 
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u16 c0_value, c1_value;	 
	u32 c0_nf, c1_nf;
	u8 buffer[2];
	u16 atio;
	int res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	
	buffer[0]=APDS9930_CMM_C0DATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	c0_value = buffer[0] | (buffer[1]<<8);
	c0_nf = obj->als_modulus*c0_value;
	//APS_LOG("c0_value=%d, c0_nf=%d, als_modulus=%d\n", c0_value, c0_nf, obj->als_modulus);

	buffer[0]=APDS9930_CMM_C1DATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	c1_value = buffer[0] | (buffer[1]<<8);
	c1_nf = obj->als_modulus*c1_value;	
	//APS_LOG("c1_value=%d, c1_nf=%d, als_modulus=%d\n", c1_value, c1_nf, obj->als_modulus);
	
	if((c0_value > c1_value) &&(c0_value < 50000))
	{  	/*Lenovo-sw chenlj2 add 2011-06-03,add {*/
		atio = (c1_nf*100)/c0_nf;

	//APS_LOG("atio = %d\n", atio);
	if(atio<30)
	{
		*data = (13*c0_nf - 24*c1_nf)/10000;
	}
	else if(atio>= 30 && atio<38) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (16*c0_nf - 35*c1_nf)/10000;
	}
	else if(atio>= 38 && atio<45)  /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (9*c0_nf - 17*c1_nf)/10000;
	}
	else if(atio>= 45 && atio<54) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (6*c0_nf - 10*c1_nf)/10000;
	}
	else
		*data = 0;
	/*Lenovo-sw chenlj2 add 2011-06-03,add }*/
    }
	else if (c0_value > 50000)
	{
		*data = 65535;
	}
	else if(c0_value == 0)
        {
                *data = 0;
        }
        else
	{
		APS_DBG("APDS9930_read_als als_value is invalid!!\n");
		return -1;
	}	

	//APS_LOG("APDS9930_read_als als_value_lux = %d\n", *data);
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("APDS9930_read_ps fail\n");
	return res;
}
int APDS9930_read_als_ch0(struct i2c_client *client, u16 *data)
{	 
	//struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u16 c0_value;	 
	u8 buffer[2];
	int res = 0;
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

//get adc channel 0 value
	buffer[0]=APDS9930_CMM_C0DATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	c0_value = buffer[0] | (buffer[1]<<8);
	*data = c0_value;
	//APS_LOG("c0_value=%d\n", c0_value);
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("APDS9930_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/

static int APDS9930_get_als_value(struct APDS9930_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("APDS9930_get_als_value exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
#if defined(CONFIG_NUBIA_APDS9930_NEW_ALS_CONT_VALUE)
        int level_high = obj->hw->als_level[idx];
        int level_low = (idx > 0) ? obj->hw->als_level[idx-1] : 0;
        int level_diff = level_high - level_low;
        int value_high = obj->hw->als_value[idx];
        int value_low = (idx > 0) ? obj->hw->als_value[idx-1] : 0;
        int value_diff = value_high - value_low;
        int value = 0;
        
        if ((level_low >= level_high) || (value_low >= value_high))
            value = value_low;
        else
            value = (level_diff * value_low + (als - level_low) * value_diff + ((level_diff + 1) >> 1)) / level_diff;

		//APS_DBG("ALS: %d [%d, %d] => %d [%d, %d] \n", als, level_low, level_high, value, value_low, value_high);
		return (value > 2) ? value : 0;
#endif			        
		//APS_ERR("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		//APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
long APDS9930_read_ps(struct i2c_client *client, u16 *data)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);	
	u8 buffer[2];
	u16 temp_data;
	long res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=APDS9930_CMM_PDATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	temp_data = buffer[0] | (buffer[1]<<8);
	//APS_LOG("yucong APDS9930_read_ps ps_data=%d, low:%d  high:%d", *data, buffer[0], buffer[1]);
	if(temp_data < obj->ps_cali)
		*data = 0;
	else
		*data = temp_data - obj->ps_cali;
	return 0;	
	return 0;    

EXIT_ERR:
	APS_ERR("APDS9930_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int APDS9930_get_ps_value(struct APDS9930_priv *obj, u16 ps)
{
	int val;// mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	static int val_temp=10;

	if(ps_cali.valid == 1)
		{
			if((ps >ps_cali.close))
			{
				val = 3;  /*close*/
				val_temp = 3;
				intr_flag_value = 1;
			}
			
			else if((ps < ps_cali.far_away))
			{
				val = 10;  /*far away*/
				val_temp = 10;
				intr_flag_value = 0;
			}
			else
				val = val_temp;

			APS_LOG("APDS9930_get_ps_value val  = %d",val);
	}
	else
	{
			if((ps  > atomic_read(&obj->ps_thd_val_high)))
			{
				val = 3;  /*close*/
				val_temp = 3;
				intr_flag_value = 1;
			}
			else if((ps  < atomic_read(&obj->ps_thd_val_low)))
			{
				val = 10;  /*far away*/
				val_temp = 10;
				intr_flag_value = 0;
			}
			else
			       val = val_temp;	
			
	}
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 45000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		if (obj->debug == true)
		{
			return (int)ps;
		}
		else
		{
			return val;
		}
	}	
	else
	{
		return -1;
	}	
}

//add by yesichao
/*----------------------------------------------------------------------------*/
static void APDS9930_cal_poll_work(struct work_struct *work)
{
	struct APDS9930_priv *obj = APDS9930_obj;
	int err;
	hwm_sensor_data sensor_data;
	u8 databuf[3];
		//get raw data
		APDS9930_read_ps(obj->client, &obj->ps);
		APDS9930_read_als_ch0(obj->client, &obj->als);
		APS_LOG("APDS9930_cal_poll_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);

		if(obj->als > 40000)
			{
			APS_LOG("APDS9930_cal_poll_work ALS too large may under lighting als_ch0=%d!\n",obj->als);
			return;
			}
		sensor_data.values[0] = APDS9930_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;	
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("APDS9930_cal_poll_work call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	schedule_delayed_work(&APDS9930_obj->ps_cal_poll_work, msecs_to_jiffies(100));
}

static int APDS9930_prox_offset_calculate(int data, int target)
{
    int offset;

    if (data > target)
    {
        offset = (data - target) * 10 / 60; //offset = (data - APDS9930_DATA_TARGET) * 10 / taos_datap->prox_offset_cal_per_bit;
        if (offset > 0x7F)
        {
		offset = 0x7F;
        }
    }
    else
    {
        offset = (target - data) * 16 / 60 + 128; //offset = (APDS9930_DATA_TARGET - data) * 16 / taos_datap->prox_offset_cal_per_bit + 128;
    }

    APS_LOG("APDS9930_prox_offset_calculate offset=%d!\n", offset);

    return offset;
}

static int APDS9930_prox_offset_cal_process(void)
{
	struct APDS9930_priv *obj = APDS9930_obj;
	int i;
	unsigned int ps_mean = 0, ps_sum = 0, ps_num = 0;
	int err, res;
	unsigned char databuf[2], offset = 0;
	
	cancel_delayed_work(&obj->ps_cal_poll_work);
	
	databuf[0] = 0x9E;
	databuf[1] = 0x00;
	APS_LOG("APDS9930_prox_offset_cal_process offset set to 0!\n");
	res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
	{
		goto prox_offset_cal_process_error;
	}

	for(i = 0; i < 10; i++)
	{
		mdelay(30);
		if(err = APDS9930_read_ps(obj->client, &obj->ps))
		{
			APS_LOG("APDS9930_prox_offset_cal_process read rawdata num:%d failed!\n", i);
			continue;
		}
		else
		{
			ps_sum += obj->ps;
			ps_num++;
			APS_LOG("APDS9930_prox_offset_cal_process rawdata num:%d ps=%d!\n", ps_num, obj->ps);
		}
	}
	if(ps_num != 10)
	{
		APS_ERR("APDS9930_prox_offset_cal_process read rawdata failed!\n");
		goto prox_offset_cal_process_error;
	}
	ps_mean = ps_sum / ps_num;
	APS_LOG("APDS9930_prox_offset_cal_process ps_mean=%d ps_sum=%d ps_num=%d\n", ps_mean, ps_sum, ps_num);

	offset = APDS9930_prox_offset_calculate(ps_mean, APDS9930_DATA_TARGET);
	APS_LOG("APDS9930_prox_offset_cal_process offset=%d!\n", offset);
	
	databuf[0] = 0x9E;
	databuf[1] = offset;
	res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
	{
		goto prox_offset_cal_process_error;
	}

	res = APDS9930_write_cal_file(PATH_PROX_OFFSET, offset);
	if(res < 0)
	{
		APS_ERR("APDS9930_prox_offset_cal_process write PATH_PROX_OFFSET error!\n");
		goto prox_offset_cal_process_error;
	}

	APS_LOG("APDS9930_prox_offset_cal_process write PATH_PROX_OFFSET succeeded!\n");
	
	for(i = 0, ps_sum =0, ps_num = 0; i < 5; i++)
	{
		mdelay(30);
		if(err = APDS9930_read_ps(obj->client, &obj->ps))
		{
			APS_LOG("APDS9930_prox_offset_cal_process after cal read rawdata num:%d failed!\n", i);
			continue;
		}
		else
		{
			ps_sum += obj->ps;
			ps_num++;
			APS_LOG("APDS9930_prox_offset_cal_process after cal rawdata num:%d ps=%d!\n", ps_num, obj->ps);
		}
	}
	if(ps_num != 5)
	{
		APS_ERR("APDS9930_prox_offset_cal_process after cal read rawdata failed!\n");
		goto prox_offset_cal_process_error;
	}
	ps_mean = ps_sum / ps_num;
	
	res = APDS9930_write_cal_file(PATH_PROX_UNCOVER_DATA, ps_mean);
	if(res < 0)
	{
		APS_ERR("APDS9930_prox_offset_cal_process write PATH_PROX_UNCOVER_DATA error!\n");
		goto prox_offset_cal_process_error;
	}
	APS_LOG("APDS9930_prox_offset_cal_process write PATH_PROX_UNCOVER_DATA succeeded!\n");
	
	APS_LOG("APDS9930_prox_offset_cal_process succeeded!\n");
	schedule_delayed_work(&obj->ps_cal_poll_work, msecs_to_jiffies(100));
	return 1;

	
prox_offset_cal_process_error:
	APS_ERR("APDS9930_prox_offset_cal_process failed!\n");
	schedule_delayed_work(&obj->ps_cal_poll_work, msecs_to_jiffies(100));
	return -1;
	
}

//add by yesichao end

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
//#define DEBUG_APDS9930
static void APDS9930_eint_work(struct work_struct *work)
{
	struct APDS9930_priv *obj = (struct APDS9930_priv *)container_of(work, struct APDS9930_priv, eint_work);
	int err;
	hwm_sensor_data sensor_data;
	u8 databuf[3];
	int res = 0;

	if((err = APDS9930_check_intr(obj->client)))
	{
		APS_ERR("APDS9930_eint_work check intrs: %d\n", err);
	}
	else
	{
		//get raw data
		APDS9930_read_ps(obj->client, &obj->ps);
		APDS9930_read_als_ch0(obj->client, &obj->als);
		APS_LOG("APDS9930_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		APS_LOG("APDS9930 int top half time = %lld\n", int_top_time);

		if(obj->als > 40000)
			{
			APS_LOG("APDS9930_eint_work ALS too large may under lighting als_ch0=%d!\n",obj->als);
			return;
			}
		sensor_data.values[0] = APDS9930_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;	

#ifdef DEBUG_APDS9930
		databuf[0]= APDS9930_CMM_ENABLE;
		res = APDS9930_i2c_master_operate(obj->client, databuf, 0x101, I2C_FLAG_READ);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("APDS9930_eint_work APDS9930_CMM_ENABLE ps value = %x\n",databuf[0]);
		
		databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;
		res = APDS9930_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("APDS9930_eint_work APDS9930_CMM_INT_LOW_THD_LOW before databuf[0]=%d databuf[1]=%d!\n",databuf[0],databuf[1]);

		databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;
		res = APDS9930_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("APDS9930_eint_work APDS9930_CMM_INT_HIGH_THD_LOW before databuf[0]=%d databuf[1]=%d!\n",databuf[0],databuf[1]);
#endif
/*singal interrupt function add*/
		if(intr_flag_value){
						databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
						databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
						
						databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;	
						databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
						databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;	
						databuf[1] = (u8)(0x00FF);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
						
						databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH; 
						databuf[1] = (u8)((0xFF00) >> 8);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
						
				}
				else{	
						databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
						databuf[1] = (u8)(0 & 0x00FF);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
						
						databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;	
						databuf[1] = (u8)((0 & 0xFF00) >> 8);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
						
						databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;	
						databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
					
						databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH; 
						databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						res = i2c_master_send(obj->client, databuf, 0x2);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
				}
				
		//let up layer to know
		#ifdef DEBUG_APDS9930
		databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;
		res = APDS9930_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("APDS9930_eint_work APDS9930_CMM_INT_LOW_THD_LOW after databuf[0]=%d databuf[1]=%d!\n",databuf[0],databuf[1]);

		databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;
		res = APDS9930_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("APDS9930_eint_work APDS9930_CMM_INT_HIGH_THD_LOW after databuf[0]=%d databuf[1]=%d!\n",databuf[0],databuf[1]);
		#endif
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	
	APDS9930_clear_intr(obj->client);
#if defined(CONFIG_OF)
	enable_irq(obj->irq);
#elif defined(CUST_EINT_ALS_TYPE)
	mt_eint_unmask(CUST_EINT_ALS_NUM); 
#endif
	return;
	EXIT_ERR:
	APDS9930_clear_intr(obj->client);
#if defined(CONFIG_OF)
	enable_irq(obj->irq);
#elif defined(CUST_EINT_ALS_TYPE)
	mt_eint_unmask(CUST_EINT_ALS_NUM); 
#endif
	APS_ERR("i2c_transfer error = %d\n", res);
	return;
}


/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int APDS9930_open(struct inode *inode, struct file *file)
{
	file->private_data = APDS9930_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int APDS9930_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static int set_psensor_threshold(struct i2c_client *client)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[3];    
	int res = 0;
	APS_ERR("set_psensor_threshold function high: 0x%x, low:0x%x\n",atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));

	databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
	databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_low) & 0x00FF);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		return -1;
	}
	databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH; 
	databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low) & 0xFF00) >> 8);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		return -1;
	}
	databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW; 
	databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_high) & 0x00FF);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		return -1;
	}
	databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH;	
	databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) & 0xFF00) >> 8);;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		return -1;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static long APDS9930_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct APDS9930_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	int ps_result;
	int ps_cali;
	int threshold[2];

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = APDS9930_enable_ps(obj->client, 1)))
				{
					APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if((err = APDS9930_enable_ps(obj->client, 0)))
				{
					APS_ERR("disable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if((err = APDS9930_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = APDS9930_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = APDS9930_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = APDS9930_enable_als(obj->client, 1)))
				{
					APS_ERR("enable als fail: %ld\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if((err = APDS9930_enable_als(obj->client, 0)))
				{
					APS_ERR("disable als fail: %ld\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if((err = APDS9930_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = APDS9930_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if((err = APDS9930_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
		/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
			if((err = APDS9930_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			if(obj->ps > atomic_read(&obj->ps_thd_val_high))
				{
					ps_result = 0;
				}
			else	ps_result = 1;
				
			if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
			{
				err = -EFAULT;
				goto err_out;
			}			   
			break;

			case ALSPS_IOCTL_CLR_CALI:
				if(copy_from_user(&dat, ptr, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(dat == 0)
					obj->ps_cali = 0;
				break;

			case ALSPS_IOCTL_GET_CALI:
				ps_cali = obj->ps_cali ;
				if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;

			case ALSPS_IOCTL_SET_CALI:
				if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
				{
					err = -EFAULT;
					goto err_out;
				}

				obj->ps_cali = ps_cali;
				break;

			case ALSPS_SET_PS_THRESHOLD:
				if(copy_from_user(threshold, ptr, sizeof(threshold)))
				{
					err = -EFAULT;
					goto err_out;
				}
				APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]); 
				atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
				atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm

				set_psensor_threshold(obj->client);
				
				break;
				
			case ALSPS_GET_PS_THRESHOLD_HIGH:
				threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
				APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]); 
				if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
				
			case ALSPS_GET_PS_THRESHOLD_LOW:
				threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
				APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]); 
				if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
			/*------------------------------------------------------------------------------------------*/
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations APDS9930_fops = {
	.owner = THIS_MODULE,
	.open = APDS9930_open,
	.release = APDS9930_release,
	.unlocked_ioctl = APDS9930_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice APDS9930_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &APDS9930_fops,
};
/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
//	struct APDS9930_priv *obj = i2c_get_clientdata(client);    
//	int err;
	APS_FUN();    
#if 0
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		if(err = APDS9930_enable_als(client, 0))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		if(err = APDS9930_enable_ps(client, 0))
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		APDS9930_power(obj->hw, 0);
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_resume(struct i2c_client *client)
{
//	struct APDS9930_priv *obj = i2c_get_clientdata(client);        
//	int err;
	APS_FUN();
#if 0
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	APDS9930_power(obj->hw, 1);
	if(err = APDS9930_init_client(client))
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = APDS9930_enable_als(client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		if(err = APDS9930_enable_ps(client, 1))
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void APDS9930_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct APDS9930_priv *obj = container_of(h, struct APDS9930_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	#if 1
	atomic_set(&obj->als_suspend, 1);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = APDS9930_enable_als(obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	}
	#endif
}
/*----------------------------------------------------------------------------*/
static void APDS9930_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct APDS9930_priv *obj = container_of(h, struct APDS9930_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

        #if 1
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = APDS9930_enable_als(obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
	#endif
}
/*----------------------------------------------------------------------------*/
static int temp_als = 0;
static int ALS_FLAG = 0;

int APDS9930_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int value;
	int err = 0;
	
	hwm_sensor_data* sensor_data;
	struct APDS9930_priv *obj = (struct APDS9930_priv *)self;
	
	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{	
				value = *(int *)buff_in;
				if(value)
				{
					if((err = APDS9930_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
					#if 1
					if(!test_bit(CMC_BIT_ALS, &obj->enable))
					{
						ALS_FLAG = 1;
						if((err = APDS9930_enable_als(obj->client, 1)))
						{
							APS_ERR("enable als fail: %d\n", err); 
							return -1;
						}
					}
					#endif
				}
				else
				{
					if((err = APDS9930_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
					#if 1
					if(ALS_FLAG == 1)
					{
						if((err = APDS9930_enable_als(obj->client, 0)))
						{
							APS_ERR("disable als fail: %d\n", err); 
							return -1;
						}
						ALS_FLAG = 0;
					}
					#endif
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;	
				APDS9930_read_ps(obj->client, &obj->ps);
				APDS9930_read_als_ch0(obj->client, &obj->als);
				APS_ERR("APDS9930_ps_operate als data=%d!\n",obj->als);
				sensor_data->values[0] = APDS9930_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


int APDS9930_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct APDS9930_priv *obj = (struct APDS9930_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
					if((err = APDS9930_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = APDS9930_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				/*yucong MTK add for fixing known issue*/
				APDS9930_read_als(obj->client, &obj->als);
				if(obj->als == 0)
				{
					sensor_data->values[0] = temp_als;				
				}else{
					u16 b[2];
					int i;
					for(i = 0;i < 2;i++){
					APDS9930_read_als(obj->client, &obj->als);
					b[i] = obj->als;
					}
					(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
					sensor_data->values[0] = APDS9930_get_als_value(obj, obj->als);
					temp_als = sensor_data->values[0];
				}
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, APDS9930_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct APDS9930_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	APDS9930_obj = obj;
	obj->hw = get_cust_alsps_hw();
	APDS9930_get_addr(obj->hw, &obj->addr);

	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	INIT_WORK(&obj->eint_work, APDS9930_eint_work);
	INIT_DELAYED_WORK(&obj->ps_cal_poll_work,APDS9930_cal_poll_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 50);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	obj->prox_uncover_data = 0;
	obj->prox_manual_calibrate_threshold = 0;
	obj->prox_offset = 0;
	obj->prox_threshold_hi = APDS9930_DEFAULT_THRESHOLD_HIGH;
	obj->prox_threshold_lo = APDS9930_DEFAULT_THRESHOLD_LOW;
	obj->prox_thres_hi_max = APDS9930_DEFAULT_THRESHOLD_HIGH;
	obj->prox_thres_hi_min = APDS9930_DEFAULT_THRESHOLD_HIGH;
	obj->prox_thres_lo_max = APDS9930_DEFAULT_THRESHOLD_LOW;
	obj->prox_thres_lo_max = APDS9930_DEFAULT_THRESHOLD_LOW;
	obj->prox_cal_valid = false;
	obj->debug = false;
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  
	/*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16 to 1/5 accoring to actual thing */
	obj->als_modulus = (400*100*ZOOM_TIME)/(1*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100 //16
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	obj->ps_cali = 0;
	
	APDS9930_i2c_client = client;
	
	if(1 == obj->hw->polling_mode_ps)
		//if (1)
		{
			obj_ps.polling = 1;
		}
		else
		{
			obj_ps.polling = 0;
		}
	
	if((err = APDS9930_init_client(client)))
	{
		goto exit_init_failed;
	}
	APS_LOG("APDS9930_init_client() OK!\n");

	if((err = misc_register(&APDS9930_device)))
	{
		APS_ERR("APDS9930_device register failed\n");
		goto exit_misc_device_register_failed;
	}
/*
	if(err = APDS9930_create_attr(&APDS9930_alsps_driver.driver))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
*/
	if((err = APDS9930_create_attr(&(APDS9930_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	obj_ps.self = APDS9930_obj;
	
	obj_ps.sensor_operate = APDS9930_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = APDS9930_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = APDS9930_als_operate;
	if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = APDS9930_early_suspend,
	obj->early_drv.resume   = APDS9930_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

    APDS9930_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&APDS9930_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
	//exit_kfree:
	kfree(obj);
	exit:
	APDS9930_i2c_client = NULL;           
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
    APDS9930_init_flag = -1;
	return err;
}
/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_remove(struct i2c_client *client)
{
	int err;	
/*
	if(err = APDS9930_delete_attr(&APDS9930_i2c_driver.driver))
	{
		APS_ERR("APDS9930_delete_attr fail: %d\n", err);
	} 
*/
	if((err = APDS9930_delete_attr(&(APDS9930_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("stk3x1x_delete_attr fail: %d\n", err);
	}
	if((err = misc_deregister(&APDS9930_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	APDS9930_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int APDS9930_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	APDS9930_power(hw, 1);    
	//APDS9930_force[0] = hw->i2c_num;
	//APDS9930_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",APDS9930_force[0],APDS9930_force[1]);
	if(i2c_add_driver(&APDS9930_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}

static int  APDS9930_local_init(void)
{
    struct alsps_hw *hw = get_cust_alsps_hw();
	//printk("fwq loccal init+++\n");

	APDS9930_power(hw, 1);
	if(i2c_add_driver(&APDS9930_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}
	if(-1 == APDS9930_init_flag)
	{
	   return -1;
	}
	//printk("fwq loccal init---\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int APDS9930_remove(void)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	APDS9930_power(hw, 0);    
	i2c_del_driver(&APDS9930_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{ .compatible = "mediatek,als_ps", },
	{},
};
#endif

static struct platform_driver APDS9930_alsps_driver =
{
	.probe      = APDS9930_probe,
	.remove     = APDS9930_remove,    
	.driver     =
	{
		.name = "als_ps",
        #ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
		#endif
	}
};
/*----------------------------------------------------------------------------*/
static int __init APDS9930_init(void)
{
	//APS_FUN();
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 
	i2c_register_board_info(hw->i2c_num, &i2c_APDS9930, 1);
	alsps_driver_add(&APDS9930_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit APDS9930_exit(void)
{
	APS_FUN();
	//platform_driver_unregister(&APDS9930_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(APDS9930_init);
module_exit(APDS9930_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("APDS9930 driver");
MODULE_LICENSE("GPL");

