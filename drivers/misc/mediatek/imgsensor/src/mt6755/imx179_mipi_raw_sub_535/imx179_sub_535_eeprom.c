/*
 * Driver for CAM_CAL
 *
 *
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "imx179_sub_535_eeprom.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>



#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define CAM_CALDB printk
#else
#define CAM_CALDB(x,...)
#endif

//static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 2
#define I2C_REGISTER_ID            0x20
extern u8 OTPData[];

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/

#define CAM_CAL_DRVNAME "CAM_CAL_DRV2"
#define CAM_CAL_I2C_GROUP_ID 2

#define PLATFORM_DRIVER_NAME "eeprom_CAM_CALdrv2"
#define EEPROM_DRIVER_CLASS_NAME "CAM_CALdrv2"
/*******************************************************************************
*
********************************************************************************/
static struct i2c_board_info kd_cam_cal_dev  __initdata ={ 
        I2C_BOARD_INFO(CAM_CAL_DRVNAME, I2C_REGISTER_ID)
};

static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_CAM_CALdevno; // = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
static spinlock_t g_CAM_CALLock;
//spin_lock(&g_CAM_CALLock);
//spin_unlock(&g_CAM_CALLock);

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
//static DEFINE_SPINLOCK(kdcam_cal_drv_lock);
//spin_lock(&kdcam_cal_drv_lock);
//spin_unlock(&kdcam_cal_drv_lock);
static int selective_read_region(u32 addr, u8 *data, u16 i2c_id, u32 size);

#define Read_NUMofEEPROM 2

#define IMX179_I2C_SPEED 400
#define IMX179_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048

static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;
/*#define LSCOTPDATASIZE 0x03c4 //964*/
/*static kal_uint8 lscotpdata[LSCOTPDATASIZE];*/

static void kdSetI2CSpeed(u32 i2cSpeed)
{
	spin_lock(&g_CAM_CALLock);
	g_pstI2Cclient->timing = i2cSpeed;
	spin_unlock(&g_CAM_CALLock);

}


static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > IMX179_MAX_OFFSET)
        return false;
	kdSetI2CSpeed(IMX179_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, IMX179_EEPROM_READ_ID)<0)
		return false;
    return true;
}

static bool byteread_cmos_sensor(unsigned char SLAVEID, unsigned short addr, unsigned char *data)
{
	/* To call your IIC function here*/
	/*char puSendCmd[1] = {(char)(addr & 0xFF) };*/
	/*if(iReadRegI2C(puSendCmd , sizeof(puSendCmd), data, 1, SLAVEID)<0) {*/
	char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	kdSetI2CSpeed(IMX179_I2C_SPEED);
	if (iReadRegI2C(puSendCmd , 2, data, 1, SLAVEID) < 0) {
	//	CAM_CALDB("tanyijun [CAM_CAL] fail imx179_byteread_cmos_sensor addr =0x%x, data = 0x%x", addr, *data);
		return false;
	}
	/*CAM_CALDB("selective_read_byte addr =0x%x data = 0x%x,page %d, offset 0x%x", addr,
	*data,page,offset);*/
	//CAM_CALDB("[CAM_CAL] imx179_cmos_sensor addr =0x%x, data = 0x%x", addr, *data);
	return true;

}


static int selective_read_region(u32 addr, u8 *data, u16 i2c_id, u32 size)
{
	/*u32 page = addr/PAGE_SIZE;//size of page was 256 */
	/*u32 offset = addr%PAGE_SIZE;*/
	unsigned short curAddr = (unsigned short)addr;
	u8 *buff = data;
	u32 size_to_read = size;
	/*kdSetI2CSpeed(EEPROM_I2C_SPEED);*/
	int ret = 0;
	//CAM_CALDB("tanyijun Before byteread_cmos_sensor curAddr =%x count=%d buffData=%x\n", curAddr,
	//size - size_to_read, *buff);
	while (size_to_read > 0) {
		/*if(selective_read_byte(addr,(u8*)buff,S5K2P8_DEVICE_ID)){*/
		if (byteread_cmos_sensor(0x21, curAddr, buff)) {
	//		CAM_CALDB("tanyijun after byteread_cmos_sensor curAddr =%x count=%d buffData=%x\n", curAddr,
	//		size - size_to_read, *buff);
			curAddr += 1;
			buff += 1;
			size_to_read -= 1;
			ret += 1;
		} else {
			break;

		}
	}
	//CAM_CALDB("tanyijun selective_read_region addr =%x size %d readSize = %d\n", addr, size, ret);
	return ret;
}


/* Burst Write Data */
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length,unsigned char pinputdata)
{
    //  char pu_send_cmd[3]= {0};
   //   unsigned char para = 0;
  //    pinputdata = &para;
	 

    // CAM_CALDB("imx179_eeprom iWriteData ui4_offset = 0x%x , ui4_length = %d pinputdata = %d  \n",
	// 	                                                   ui4_offset,ui4_length,pinputdata);
	
       kdSetI2CSpeed(IMX179_I2C_SPEED); // Add this func to set i2c speed by each sensor
	char pu_send_cmd[3] = {(char)(ui4_offset >> 8), (char)(ui4_offset & 0xFF), (char)(pinputdata & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, 0x20);

	return 0;
}




#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
	stCAM_CAL_INFO_STRUCT __user *data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data->u4Offset);
	err |= put_user(i, &data32->u4Offset);
	err |= get_user(i, &data->u4Length);
	err |= put_user(i, &data32->u4Length);
	/* Assume pointer is not change */
#if 1
	err |= get_user(p, &data->pu1Params);
	err |= put_user(p, &data32->pu1Params);
#endif
	return err;
}
static int compat_get_cal_info_struct(
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
	stCAM_CAL_INFO_STRUCT __user *data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data32->u4Offset);
	err |= put_user(i, &data->u4Offset);
	err |= get_user(i, &data32->u4Length);
	err |= put_user(i, &data->u4Length);
	err |= get_user(p, &data32->pu1Params);
	err |= put_user(compat_ptr(p), &data->pu1Params);

	return err;
}


static int  imx179_otp_write_ready(void)
{
    unsigned int addr;
    unsigned int  length;
    unsigned char data;
    int i4RetValue = 0;
  
    addr = 0x0100;
    length = 1;
    data = 0x00;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	
	
    addr = 0x3382;
    length = 1;
    data = 0x05;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	
	
    addr = 0x3383;
    length = 1;
    data = 0xa0;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	
	
    addr = 0x3368;
    length = 1;
    data = 0x18;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	
	
    addr = 0x3369;
    length = 1;
    data = 0x0;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	
	
    addr = 0x3380;
    length = 1;
    data = 0x28;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);
	
    addr = 0x3400;
    length = 1;
    data = 0x03;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	

   /////set page 
#if 0   
    addr = 0x3402;
    length = 1;
    data = 0x00;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);				  
#endif
   return i4RetValue;
}

static int imx179_stream_on(void)
{
    unsigned int addr = 0x0100;
    unsigned int length = 1;
    unsigned int data = 0x1;
    unsigned int  i4RetValue =0;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	
}

static int  imx179_otp_read_ready(void)
{
    unsigned int addr;
    unsigned int  length;
    unsigned char data;
    int i4RetValue = 0;
  
    addr = 0x0100;
    length = 1;
    data = 0x00;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	
	
    addr = 0x3382;
    length = 1;
    data = 0x05;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	
	
    addr = 0x3383;
    length = 1;
    data = 0xa0;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	
	
    addr = 0x3368;
    length = 1;
    data = 0x18;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	
	
    addr = 0x3369;
    length = 1;
    data = 0x0;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	
	
    addr = 0x3380;
    length = 1;
    data = 0x08;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);
	
    addr = 0x3400;
    length = 1;
    data = 0x01;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);	

   /////set page 
#if 0   
    addr = 0x3402;
    length = 1;
    data = 0x00;
    i4RetValue = iWriteData((u16)addr, length, data);
    mdelay(2);				  
#endif
   return i4RetValue;
 CAM_CALDB("[ CAM_CAL] imx179_otp_read_ready --- \n");
}

static long imx179_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
	stCAM_CAL_INFO_STRUCT __user *data;
	int err;
	CAM_CALDB("[CAMERA SENSOR] imx179_Ioctl_Compat,%p %p %x ioc size %d\n", filp->f_op ,
	filp->f_op->unlocked_ioctl, cmd, _IOC_SIZE(cmd));


	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {

	case COMPAT_CAM_CALIOC_G_READ: {
		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_cal_info_struct(data32, data);
		if (err)
			return err;

		ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,
		(unsigned long)data);
		err = compat_put_cal_info_struct(data32, data);


		if (err != 0)
			CAM_CALDB("[CAM_CAL] compat_put_acdk_sensor_getinfo_struct failed\n");
		return ret;
	}
	default:
		return -ENOIOCTLCMD;
	}
}


#endif


/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode *a_pstInode,
			 struct file *a_pstFile,
			 unsigned int a_u4Command,
			 unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
	struct file *file,
	unsigned int a_u4Command,
	unsigned long a_u4Param
)
#endif
{
	int i4RetValue = 0;
	u8 *pBuff = NULL;
	u8 *pWorkingBuff = NULL;
	u8 *pWorkingBuff1 = NULL;
	u8 *pWorkingBuff2 = NULL;
	stCAM_CAL_INFO_STRUCT *ptempbuf;
	ssize_t writeSize;
       u8 readTryagain = 0;
	 int i;


	CAM_CALDB("[CAM_CAL] ioctl\n");

#ifdef CAM_CALGETDLT_DEBUG
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
#endif

	/*if (_IOC_NONE == _IOC_DIR(a_u4Command)) {
	} else {*/
	if (_IOC_NONE != _IOC_DIR(a_u4Command)) {
		pBuff = kmalloc(sizeof(stCAM_CAL_INFO_STRUCT), GFP_KERNEL);

		if (NULL == pBuff) {
			CAM_CALDB("[S24EEPROM] ioctl allocate mem failed\n");
			return -ENOMEM;
		}

		if (_IOC_WRITE & _IOC_DIR(a_u4Command)) {
			if (copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT))) {
				/* get input structure address */
				kfree(pBuff);
				CAM_CALDB("[S24EEPROM] ioctl copy from user failed\n");
				return -EFAULT;
			}
		}
	}

	ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
	pWorkingBuff = kmalloc(ptempbuf->u4Length, GFP_KERNEL);
	if (NULL == pWorkingBuff) {
		kfree(pBuff);
		CAM_CALDB("[CAM_CAL] ioctl allocate mem failed\n");
		return -ENOMEM;
	}
	CAM_CALDB(" init Working buffer address 0x%p  command is 0x%x\n", pWorkingBuff, a_u4Command);


	if (copy_from_user((u8 *)pWorkingBuff , (u8 *)ptempbuf->pu1Params, ptempbuf->u4Length)) {
		kfree(pBuff);
		kfree(pWorkingBuff);
		CAM_CALDB("[S24EEPROM] ioctl copy from user failed\n");
		return -EFAULT;
	}

	switch (a_u4Command) {
	case CAM_CALIOC_S_WRITE:
		for(i=0; i< 1000; i++) 
		CAM_CALDB("[CAM_CAL] Write CMD\n");
		
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
		CAM_CALDB("[tanyijun CAM_CAL] write offset %d\n", ptempbuf->u4Offset);
		CAM_CALDB("[tanyijun CAM_CAL] write length %d\n", ptempbuf->u4Length);
              imx179_otp_write_ready();
		i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
		imx179_stream_on();
              mdelay(2);
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;

		CAM_CALDB("Write data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif
		break;
	case CAM_CALIOC_G_READ:
		CAM_CALDB("[tanyijun CAM_CAL] Read CMD\n");

#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
		CAM_CALDB("[tanyijun CAM_CAL] offset %d\n", ptempbuf->u4Offset);
		CAM_CALDB("[tanyijun CAM_CAL] length %d\n", ptempbuf->u4Length);
		imx179_otp_read_ready();
		i4RetValue = selective_read_region(ptempbuf->u4Offset, pWorkingBuff,IMX179_EEPROM_READ_ID, ptempbuf->u4Length);
		imx179_stream_on();
#if 0
CAM_CALDB("=====read page 1 data ======");
            addr = 0x3402;
           length = 1;
	    data = 0x01;
		i4RetValue = iWriteData((u16)addr, length, &data);
              mdelay(2);				  
		i4RetValue = selective_read_region(ptempbuf->u4Offset, pWorkingBuff2,IMX179_EEPROM_READ_ID, ptempbuf->u4Length);	
#endif		
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;

		CAM_CALDB(" Read data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif

		break;
	default:
		CAM_CALDB("[CAM_CAL] No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	if (_IOC_READ & _IOC_DIR(a_u4Command)) {
		/* copy data to user space buffer, keep other input paremeter unchange. */
		CAM_CALDB("[S24EEPROM] to user length %d\n", ptempbuf->u4Length);
		CAM_CALDB("[S24EEPROM] to user  Working buffer address 0x%p\n", pWorkingBuff);
		if (copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length)) {
			kfree(pBuff);
			kfree(pWorkingBuff);
			CAM_CALDB("[S24EEPROM] ioctl copy to user failed\n");
			return -EFAULT;
		}
	}

	kfree(pBuff);
	kfree(pWorkingBuff);
	return i4RetValue;
}


static u32 g_u4Opened;
/*#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.*/
static int CAM_CAL_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	CAM_CALDB("[CAM_CAL] CAM_CAL_Open111111\n");
	spin_lock(&g_CAM_CALLock);
	if (g_u4Opened) {
		spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[CAM_CAL] Opened, return -EBUSY\n");
		return -EBUSY;
	} else {
		g_u4Opened = 1;
		atomic_set(&g_CAM_CALatomic, 0);
	}
	spin_unlock(&g_CAM_CALLock);
	mdelay(2);
	return 0;
}

/*Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.*/
static int CAM_CAL_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	spin_lock(&g_CAM_CALLock);

	g_u4Opened = 0;

	atomic_set(&g_CAM_CALatomic, 0);

	spin_unlock(&g_CAM_CALLock);

	return 0;
}

static const struct file_operations g_stCAM_CAL_fops = {
	.owner = THIS_MODULE,
	.open = CAM_CAL_Open,
	.release = CAM_CAL_Release,
	/*.ioctl = CAM_CAL_Ioctl*/
#ifdef CONFIG_COMPAT
	.compat_ioctl = imx179_Ioctl_Compat,
#endif
	.unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
static inline int RegisterCAM_CALCharDrv(void)
{
	

	struct device *CAM_CAL_device = NULL;
CAM_CALDB("[CAM_CAL] RegisterCAM_CALCharDrv Start\n");
#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
	if (alloc_chrdev_region(&g_CAM_CALdevno, 0, 1, CAM_CAL_DRVNAME)) {
		CAM_CALDB("[CAM_CAL] Allocate device no failed\n");

		return -EAGAIN;
	}
#else
	if (register_chrdev_region(g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME)) {
		CAM_CALDB("[CAM_CAL] Register device no failed\n");

		return -EAGAIN;
	}
#endif

	/*Allocate driver*/
	g_pCAM_CAL_CharDrv = cdev_alloc();

	if (NULL == g_pCAM_CAL_CharDrv) {
		unregister_chrdev_region(g_CAM_CALdevno, 1);

		CAM_CALDB("[CAM_CAL] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}

	/*Attatch file operation.*/
	cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

	g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

	/*Add to system*/
	if (cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1)) {
		CAM_CALDB("[CAM_CAL] Attatch file operation failed\n");

		unregister_chrdev_region(g_CAM_CALdevno, 1);

		return -EAGAIN;
	}

	CAM_CAL_class = class_create(THIS_MODULE, EEPROM_DRIVER_CLASS_NAME);
	if (IS_ERR(CAM_CAL_class)) {
		int ret = PTR_ERR(CAM_CAL_class);
		CAM_CALDB("[CAM_CAL] Unable to create class, err = %d\n", ret);
		return ret;
	}
	CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);
	CAM_CALDB("[CAM_CAL] RegisterCAM_CALCharDrv End\n");

	if (NULL == CAM_CAL_device)
		return -EIO;
	return 0;
}

static inline void UnregisterCAM_CALCharDrv(void)
{
	/*Release char driver*/
	cdev_del(g_pCAM_CAL_CharDrv);

	unregister_chrdev_region(g_CAM_CALdevno, 1);

	device_destroy(CAM_CAL_class, g_CAM_CALdevno);
	class_destroy(CAM_CAL_class);
}


/* //////////////////////////////////////////////////////////////////// */
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME, 0}, {} };



static struct i2c_driver CAM_CAL_i2c_driver = {
	.probe = CAM_CAL_i2c_probe,
	.remove = CAM_CAL_i2c_remove,
	/*   .detect = CAM_CAL_i2c_detect,*/
	.driver.name = CAM_CAL_DRVNAME,
	.id_table = CAM_CAL_i2c_id,
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, CAM_CAL_DRVNAME);
	return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i4RetValue = 0;
	CAM_CALDB("[CAM_CAL] CAM_CAL_i2c_probe Start!\n");
	/*    spin_lock_init(&g_CAM_CALLock);*/

	/*get sensor i2c client*/
	//spin_lock(&g_CAM_CALLock); /*for SMP*/
	g_pstI2Cclient = client;
	g_pstI2Cclient->addr =IMX179_EEPROM_READ_ID >> 1;
	//spin_unlock(&g_CAM_CALLock); /* for SMP*/

	CAM_CALDB("[CAM_CAL] g_pstI2Cclient->addr = 0x%x\n", g_pstI2Cclient->addr);
	/*Register char driver*/
	i4RetValue = RegisterCAM_CALCharDrv();

	if (i4RetValue) {
		CAM_CALDB("[CAM_CAL] register char device failed!\n");
		return i4RetValue;
	}

	 spin_lock_init(&g_CAM_CALLock);
	CAM_CALDB("[CAM_CAL] CAM_CAL_i2c_probe End!\n");
	return 0;
}

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
	CAM_CALDB("[CAM_CAL] CAM_CAL_probe start!\n");
	return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
	i2c_del_driver(&CAM_CAL_i2c_driver);
	return 0;
}

/*platform structure*/
static struct platform_driver g_stCAM_CAL_Driver = {
	.probe              = CAM_CAL_probe,
	.remove     = CAM_CAL_remove,
	.driver             = {
		.name   = CAM_CAL_DRVNAME,
		.owner  = THIS_MODULE,
	}
};


static struct platform_device g_stCAM_CAL_Device = {
	.name = CAM_CAL_DRVNAME,
	.id = 0,
	.dev = {
	}
};

static int __init CAM_CAL_i2C_init(void)
{
	CAM_CALDB("[CAM_CAL] CAM_CAL_i2C_init Start!\n");
	i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);

	if (platform_device_register(&g_stCAM_CAL_Device)) {
		CAM_CALDB("[CAM_CAL] failed to register CAM_CAL driver, 2nd time\n");
		return -ENODEV;
	}

	if (platform_driver_register(&g_stCAM_CAL_Driver)) {
		CAM_CALDB("[CAM_CAL] failed to register CAM_CAL driver\n");
		return -ENODEV;
	}
	
	CAM_CALDB("[CAM_CAL] CAM_CAL_i2C_init End!\n");
	return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

MODULE_DESCRIPTION("IMX179 CAM_CAL driver");
MODULE_AUTHOR("LukeHu <luke.hu@Mediatek.com>");
MODULE_LICENSE("GPL");



