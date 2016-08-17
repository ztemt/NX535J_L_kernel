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
#include "imx258_main_541_eeprom.h"
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
#define I2C_REGISTER_ID            0x32
extern u8 OTPData[];

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/

#define CAM_CAL_DRVNAME "CAM_CAL_DRV1"
#define CAM_CAL_I2C_GROUP_ID 2

#define PLATFORM_DRIVER_NAME "eeprom_CAM_CALdrv1r"
#define EEPROM_DRIVER_CLASS_NAME "CAM_CALdrv1"
/*******************************************************************************
*
********************************************************************************/
static struct i2c_board_info kd_cam_cal_dev  __initdata ={ 
        I2C_BOARD_INFO(CAM_CAL_DRVNAME, I2C_REGISTER_ID)};

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

#define imx258_I2C_SPEED 400
#define imx258_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048

BYTE imx258_DCC_data[1356]= {0};
BYTE imx258_SPC_data[126]= {0};

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
    if(addr > imx258_MAX_OFFSET)
        return false;
	kdSetI2CSpeed(imx258_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, imx258_EEPROM_READ_ID)<0)
		return false;
    return true;
}


static bool _read_imx258_eeprom(kal_uint16 addr, BYTE* data, int size ){
	int i = 0;
	int offset = addr;
	CAM_CALDB("enter _read_eeprom size = %d\n",size);
	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			return false;
		}
		CAM_CALDB("_read_imx258_eeprom 0x%0x 0x%x \n",offset, data[i]);
		offset++;
	}
//	get_done = true;
//	last_size = size;
//	last_offset = addr;
    return true;
}

static bool byteread_cmos_sensor(unsigned char SLAVEID, unsigned short addr, unsigned char *data)
{
	/* To call your IIC function here*/
	/*char puSendCmd[1] = {(char)(addr & 0xFF) };*/
	/*if(iReadRegI2C(puSendCmd , sizeof(puSendCmd), data, 1, SLAVEID)<0) {*/
	char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	kdSetI2CSpeed(imx258_I2C_SPEED);
	if (iReadRegI2C(puSendCmd , 2, data, 1, SLAVEID) < 0) {
		CAM_CALDB("[CAM_CAL] fail imx258_byteread_cmos_sensor addr =0x%x, data = 0x%x", addr, *data);
		return false;
	}
	/*CAM_CALDB("selective_read_byte addr =0x%x data = 0x%x,page %d, offset 0x%x", addr,
	*data,page,offset);*/
	CAM_CALDB("[CAM_CAL] imx258_cmos_sensor addr =0x%x, data = 0x%x", addr, *data);
	return true;

}



void read_imx258_SPC(BYTE* data){
	int i;
	 kal_uint32 sum = 0;
	int addr = 0x0CDF;//0x791;
	int size = 126;//252;
    unsigned char spc_flag;
    byteread_cmos_sensor(0xA0, 0x0CDE, &spc_flag);
    CAM_CALDB("kdebug spc_flag=%d\n",spc_flag);
	
	CAM_CALDB("read imx258 SPC, size = %d, get_done = %d, last_size = %d, last_offset = %d\n", size, get_done, last_size, last_offset);
	
	if(!get_done || last_size != size || last_offset != addr) {
	//	if(!_read_imx258_eeprom(addr, imx258_SPC_data, size)){
              if(!selective_read_region(addr, imx258_SPC_data, imx258_EEPROM_READ_ID,size)){			
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}

 //      CAM_CALDB("imx258_SPC_data[0] = 0x%x \n",imx258_SPC_data[0]);
//	for(i=1;i<253;i++)
//	{
//	    CAM_CALDB("imx258_SPC_data[%d] = 0x%x \n",i,imx258_SPC_data[i]);
//           sum += imx258_SPC_data[i];
//
//	}

//	CAM_CALDB("imx258_SPC_data[253] = 0x%x  sum = 0x%x \n",imx258_SPC_data[253],sum);
	memcpy(data, imx258_SPC_data , size);
    return true;
}



void read_imx258_DCC( kal_uint16 addr,BYTE* data, kal_uint32 size){
	int i;
	 kal_uint32 sum = 0;
	addr = 0x791;//0x88F;
	size = 1356;//96;

    unsigned char dcc_flag;
    byteread_cmos_sensor(0xA0, 0x0790, &dcc_flag);
    CAM_CALDB("kdebug dcc_flag=%d\n",dcc_flag);
	
	CAM_CALDB("read_imx258_DCC, size = %d, get_done = %d, last_size = %d, last_offset = %d \n", size, get_done, last_size, last_offset);
	
	//if(!get_done || last_size != size || last_offset != addr) {
	if(1) {	
	//	if(!_read_imx258_eeprom(addr, imx258_DCC_data, size)){
              if(!selective_read_region(addr, imx258_DCC_data, imx258_EEPROM_READ_ID,size)){			
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}

      //  CAM_CALDB("imx258_DCC_data[0] = 0x%x \n",imx258_DCC_data[0]);
	for(i=0;i<96;i++)
	{
	    CAM_CALDB("imx258_DCC_data[%d] = 0x%x \n",i,imx258_DCC_data[i]);
        //   sum += imx258_DCC_data[i];

	}

	//CAM_CALDB("imx258_DCC_data[97] = 0x%x  sum = 0x%x \n",imx258_DCC_data[97],sum);
//	CAM_CALDB("imx258_DCC_data[97] = 0x%x  sum = 0x%x \n",imx258_DCC_data[97],sum);

	memcpy(data, imx258_DCC_data , size);
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
	CAM_CALDB("tanyijun Before byteread_cmos_sensor curAddr =%x count=%d buffData=%x\n", curAddr,
	size - size_to_read, *buff);
	while (size_to_read > 0) {
		/*if(selective_read_byte(addr,(u8*)buff,S5K2P8_DEVICE_ID)){*/
		if (byteread_cmos_sensor(0xA0, curAddr, buff)) {
		//	CAM_CALDB("tanyijun after byteread_cmos_sensor curAddr =%x count=%d buffData=%x\n", curAddr,
//			size - size_to_read, *buff);
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
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length,
unsigned char *pinputdata)
{
	CAM_CALDB("[CAM_CAL] not implemented!");
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

static long imx258_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
	stCAM_CAL_INFO_STRUCT __user *data;
	int err;
	CAM_CALDB("[CAMERA SENSOR] imx258_Ioctl_Compat,%p %p %x ioc size %d\n", filp->f_op ,
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
	stCAM_CAL_INFO_STRUCT *ptempbuf;
//	ssize_t writeSize;
       u8 readTryagain = 0;
	
	
#ifdef CAM_CALGETDLT_DEBUG
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
#endif
CAM_CALDB("[CAM_CAL] ioctl\n");

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
		CAM_CALDB("[CAM_CAL] Write CMD\n");
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
		i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
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
		CAM_CALDB("[CAM_CAL] Read CMD\n");

#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
		CAM_CALDB("[CAM_CAL] offset %d\n", ptempbuf->u4Offset);
		CAM_CALDB("[CAM_CAL] length %d\n", ptempbuf->u4Length);
		/**pu1Params = 0;*/
		readTryagain = 3;
		//i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params, IMX258_DEVICE_ID, ptempbuf->u4Length);
#if 0
               while (0 < readTryagain) {
				i4RetValue =  iReadDataFromGT24c64((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
				CAM_CALDB("[S24EEPROM] error (%d) Read retry (%d)\n", i4RetValue, readTryagain);
				if (i4RetValue != 0)
					readTryagain--;
				else
					readTryagain = 0;
			}
#else
		i4RetValue = selective_read_region(ptempbuf->u4Offset, pWorkingBuff, imx258_EEPROM_READ_ID, ptempbuf->u4Length);
#endif
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;

		CAM_CALDB("Read data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
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
	CAM_CALDB("[CAM_CAL] CAM_CAL_Open\n");
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
	.compat_ioctl = imx258_Ioctl_Compat,
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
	g_pstI2Cclient->addr = imx258_EEPROM_READ_ID >> 1;
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

MODULE_DESCRIPTION("S5K2P8 CAM_CAL driver");
MODULE_AUTHOR("LukeHu <luke.hu@Mediatek.com>");
MODULE_LICENSE("GPL");



