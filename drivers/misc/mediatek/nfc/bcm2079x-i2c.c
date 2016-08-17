/*
 * Copyright (C) 2012 Broadcom Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/version.h>

#include "cust_eint.h"
#include "cust_gpio_usage.h"
#include "cust_eint_md1.h"

#include <mach/eint.h>
#include <mach/mt_gpio.h>

#include <linux/nfc/bcm2079x.h>
#include <linux/dma-mapping.h>
#include <linux/syscalls.h>

#include <linux/of.h>
#include <linux/of_irq.h>
//add by slv
#include <mach/mt_pm_ldo.h>


//#define ZTEMT_FOR_NFC_PIN_TEST
//#define TRUE		1
//#define FALSE		0
#define STATE_HIGH	1
#define STATE_LOW	0

/* end of compile options */

/* do not change below */
#define MAX_BUFFER_SIZE		780

	/* Read data */
#define PACKET_HEADER_SIZE_NCI	(4)
#define PACKET_HEADER_SIZE_HCI	(3)
#define PACKET_TYPE_NCI		(16)
#define PACKET_TYPE_HCIEV	(4)
#define MAX_PACKET_SIZE		(PACKET_HEADER_SIZE_NCI + 255)

#define NFC_I2C_BUSNUM  4
#define EINT_NUM       CUST_EINT_IRQ_NFC_NUM/*CUST_EINT_IRQ_NFC_NUM*/

#define  NFC_IRQ 	    GPIO_IRQ_NFC_PIN//GPIO87, i2c_irq
#define  NFC_VENB 	    GPIO_NFC_VENB_PIN//	GPIO86, reg_on
#define  NFC_FIRM 	    GPIO_NFC_FIRM_PIN// GPIO85, nfc_wake
#define  NFC_CLK_REQ 	GPIO_NFC_OSC_EN_PIN// GPIO99, nfc_clk_req

//add by slv
#define VOL_1800 1800000

int bcm_nfc_irq ;
#define BCM2079X_DEBUG 1

#if BCM2079X_DEBUG
        #define NFC_TAG                  "[NFC_BCM] "
        #define NFC_FUN(f)               printk(KERN_INFO NFC_TAG"%s\n", __FUNCTION__)
        #define NFC_ERR(fmt, args...)    printk(KERN_ERR  NFC_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
        #define NFC_LOG(fmt, args...)    printk(KERN_NOTICE NFC_TAG fmt, ##args)
    #else
        #define NFC_TAG
        #define NFC_FUN(f)               do {} while (0)
        #define NFC_ERR(fmt, args...)    do {} while (0)
        #define NFC_LOG(fmt, args...)    do {} while (0)
    #endif


static  char *gpRxDMABuf_va = NULL;
static unsigned int gpRxDMABuf_pa = 0;
static  char *gpTxDMABuf_va = NULL;
static unsigned int gpTxDMABuf_pa = 0;

static struct i2c_board_info __initdata bcm2079x_i2c_nfc = { I2C_BOARD_INFO("bcm2079x-i2c", 0x76) };

static struct bcm2079x_platform_data bcm2079x_pdata = {
	.irq_gpio = NFC_IRQ,
	.en_gpio = NFC_VENB,
	.wake_gpio = NFC_FIRM,
};

struct bcm2079x_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice bcm2079x_device;
	unsigned int wake_gpio;
	unsigned int en_gpio;
	unsigned int irq_gpio;
	bool irq_enabled;
	spinlock_t irq_enabled_lock;
	unsigned int error_write;
	unsigned int error_read;
	unsigned int count_read;
	unsigned int count_irq;
    int original_address;
};

static struct bcm2079x_dev *p_bcm2079x_dev = NULL;

static void bcm2079x_init_stat(struct bcm2079x_dev *bcm2079x_dev)
{
	bcm2079x_dev->error_write = 0;
	bcm2079x_dev->error_read = 0;
	bcm2079x_dev->count_read = 0;
	bcm2079x_dev->count_irq = 0;
}

static void bcm2079x_disable_irq(struct bcm2079x_dev *bcm2079x_dev)
{
	unsigned long flags;
  NFC_LOG("%s : -----begin ----\n", __func__);
	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (bcm2079x_dev->irq_enabled) {
		disable_irq_nosync(bcm2079x_dev->client->irq);
		//mt_eint_mask(EINT_NUM);
		bcm2079x_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
}

static void bcm2079x_enable_irq(struct bcm2079x_dev *bcm2079x_dev)
{
	unsigned long flags;
	NFC_LOG("%s : -----begin ----\n", __func__);
	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (!bcm2079x_dev->irq_enabled) {
		bcm2079x_dev->irq_enabled = true;
		enable_irq(bcm2079x_dev->client->irq);
		//mt_eint_unmask(EINT_NUM);
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
}

/*
 The alias address 0x79, when sent as a 7-bit address from the host processor
 will match the first byte (highest 2 bits) of the default client address
 (0x1FA) that is programmed in bcm20791.
 When used together with the first byte (0xFA) of the byte sequence below,
 it can be used to address the bcm20791 in a system that does not support
 10-bit address and change the default address to 0x38.
 the new address can be changed by changing the CLIENT_ADDRESS below if 0x38
 conflicts with other device on the same i2c bus.
 */
#define ALIAS_ADDRESS	  0x79

static void set_client_addr(struct bcm2079x_dev *bcm2079x_dev, int addr)
{
	struct i2c_client *client = bcm2079x_dev->client;
	client->addr = addr;
	if (addr > 0x7F)
		client->flags |= I2C_CLIENT_TEN;
    else
        client->flags &= ~I2C_CLIENT_TEN;

	NFC_LOG("Set client device changed to (0x%04X) flag = %04x\n",
		client->addr, client->flags);
}

static void change_client_addr(struct bcm2079x_dev *bcm2079x_dev, int addr)
{
#if 0
	struct i2c_client *client;
	int ret;
	int i;
	int offset = 1;
	char addr_data[] = {
		0xFA, 0xF2, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x2A
	};
	
	client = bcm2079x_dev->client;
	if ((client->flags & I2C_CLIENT_TEN) == I2C_CLIENT_TEN) {
		client->addr = ALIAS_ADDRESS;
		client->flags &= ~I2C_CLIENT_TEN;
		offset = 0;
	}

	addr_data[5] = addr & 0xFF;
	ret = 0;
	for (i = 1; i < sizeof(addr_data) - 1; ++i)
		ret += addr_data[i];
	addr_data[sizeof(addr_data) - 1] = (ret & 0xFF);
	dev_info(&client->dev,
		 "Change client device from (0x%04X) flag = "\
		 "%04x, addr_data[%d] = %02x\n",
		 client->addr, client->flags, sizeof(addr_data) - 1,
		 addr_data[sizeof(addr_data) - 1]);
	//ret = i2c_master_send(client, addr_data+offset, sizeof(addr_data)-offset);
	ret = i2c_master_send(bcm2079x_dev->client, (unsigned char *)gpDMABuf_pa, sizeof(addr_data)-offset);
	if (ret != sizeof(addr_data)-offset) {
		client->addr = ALIAS_ADDRESS;
		client->flags &= ~I2C_CLIENT_TEN;
		dev_info(&client->dev,
			 "Change client device from (0x%04X) flag = "\
			 "%04x, addr_data[%d] = %02x\n",
			 client->addr, client->flags, sizeof(addr_data) - 1,
			 addr_data[sizeof(addr_data) - 1]);
		//ret = i2c_master_send(client, addr_data, sizeof(addr_data));
		ret = i2c_master_send(bcm2079x_dev->client, (unsigned char *)gpDMABuf_pa, sizeof(addr_data));
	}
	client->addr = addr_data[5];

	dev_info(&client->dev,
		 "Change client device changed to (0x%04X) flag = %04x, ret = %d\n",
		 client->addr, client->flags, ret);
#endif
}

static irqreturn_t bcm2079x_dev_irq_handler(int irq, void *data)
{
	struct bcm2079x_dev *bcm2079x_dev = p_bcm2079x_dev;
	unsigned long flags;
	//NFC_LOG("%s in  \n", __func__);
//	bcm2079x_disable_irq(bcm2079x_dev);
	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	bcm2079x_dev->count_irq++;
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
	wake_up(&bcm2079x_dev->read_wq);

	return IRQ_HANDLED;  //liaojs del
}

static unsigned int bcm2079x_dev_poll(struct file *filp, poll_table *wait)
{
	struct bcm2079x_dev *bcm2079x_dev = filp->private_data;
	unsigned int mask = 0;
	unsigned long flags;
	NFC_LOG("%s : -----begin ---count_irq = %d -,irq level =%d.\n", 
		__func__,bcm2079x_dev->count_irq,mt_get_gpio_in(bcm2079x_dev->irq_gpio) );
	
	poll_wait(filp, &bcm2079x_dev->read_wq, wait);

	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (bcm2079x_dev->count_irq > 0){
		mask |= POLLIN | POLLRDNORM;
		NFC_LOG("%s : -----middle ---set mask=0x%04x\n", __func__,mask);
		}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
	NFC_LOG("%s : -----end ----mask=0x%08x\n", __func__,mask);

	return mask;
}

static ssize_t bcm2079x_dev_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *offset)
{
	struct bcm2079x_dev *bcm2079x_dev = filp->private_data;
	unsigned char tmp[MAX_BUFFER_SIZE];
	int  len, ret;

	len = 0;
	
	if (bcm2079x_dev->count_irq > 0)
		bcm2079x_dev->count_irq--;

	//bcm2079x_enable_irq(bcm2079x_dev);

	bcm2079x_dev->count_read++;
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&bcm2079x_dev->read_mutex);

	/** Read the first 4 bytes to include the length of the NCI or HCI packet.
	**/
	
	//memset(gpRxDMABuf_va,0,MAX_BUFFER_SIZE);

	
   	//ret = i2c_master_recv(bcm2079x_dev->client, tmp, count);
	ret = i2c_master_recv(bcm2079x_dev->client, (unsigned char *)(uintptr_t)gpRxDMABuf_pa, count);
	if((ret <= count) && (ret > 0))
	{
		memcpy(tmp, gpRxDMABuf_va, ret);
	}
	mutex_unlock(&bcm2079x_dev->read_mutex);
	
    NFC_LOG( "read data ,length =%d ,need to read count = %zu  \n",ret, count);
	//print_hex_dump(KERN_DEBUG, " [NFC_BCM]read: ", DUMP_PREFIX_NONE, 16, 1, tmp, ret, false);

	if (ret >0 ) {
		if (copy_to_user(buf, tmp, ret)) {
			dev_err(&bcm2079x_dev->client->dev,
				"failed to copy to user space, ret = %d\n", ret);
			ret = -EFAULT;
			bcm2079x_dev->error_read++;
		}
	}
//	bcm2079x_enable_irq(bcm2079x_dev);
	return ret;
}

static ssize_t bcm2079x_dev_write(struct file *filp, const char __user *buf,
				   size_t count, loff_t *offset)
{
	struct bcm2079x_dev *bcm2079x_dev = filp->private_data;
	char data[MAX_BUFFER_SIZE];
	int ret = 0;
	memset(data,0,MAX_BUFFER_SIZE);
	char *tmp = gpTxDMABuf_va ? gpTxDMABuf_va: data;
	NFC_LOG("%s() ----begin ----\n", __func__);

	if (count > MAX_BUFFER_SIZE) {
		NFC_LOG("out of memory\n");
		return -ENOMEM;
	}
//	NFC_LOG("%s() ----1 ----writing %zu bytes \n", __func__,count);
    //memset(gpTxDMABuf_va,0,MAX_BUFFER_SIZE);
		
	if (copy_from_user(tmp, buf, count)) {
		NFC_LOG("failed to copy from user space\n");
		return -EFAULT;
	}


	mutex_lock(&bcm2079x_dev->read_mutex);
	/* Write data */
	//ret = i2c_master_send(bcm2079x_dev->client, tmp, count);
	ret = i2c_master_send(bcm2079x_dev->client, (unsigned char *)(uintptr_t) gpTxDMABuf_pa, count);
	NFC_LOG("%s() ----4 , i2c send ret = %d, count=%zu ----\n", __func__,ret, count);

	
	if (ret != count) {
		NFC_LOG("failed to write %d\n", ret);
		ret = -EIO;
		bcm2079x_dev->error_write++;	
	}
	mutex_unlock(&bcm2079x_dev->read_mutex);
	
	//NFC_LOG("[dsc]write data ,length =%d \n",ret);
    //print_hex_dump(KERN_DEBUG, " [NFC_BCM] write: ", DUMP_PREFIX_NONE, 16, 1, tmp, ret, false);

	return ret;
}

static int bcm2079x_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct bcm2079x_dev *bcm2079x_dev = 
		                container_of(filp->private_data,  
		                             struct bcm2079x_dev, bcm2079x_device);

	filp->private_data = bcm2079x_dev;
	bcm2079x_init_stat(bcm2079x_dev);
	//bcm2079x_enable_irq(bcm2079x_dev); 
	NFC_LOG( "device node major=%d, minor=%d\n", imajor(inode), iminor(inode));

	return ret;
}

static long bcm2079x_dev_unlocked_ioctl(struct file *filp,
					 unsigned int cmd, unsigned long arg)
{
	struct bcm2079x_dev *bcm2079x_dev = filp->private_data;
	NFC_LOG("%s() ----begin -- cmd = %x   --\n", __func__, cmd);

	switch (cmd) {
	case BCMNFC_READ_FULL_PACKET:
		break;
	case BCMNFC_READ_MULTI_PACKETS:
		break;
	case BCMNFC_CHANGE_ADDR:
		NFC_LOG( "%s, BCMNFC_CHANGE_ADDR (%x, %lx):\n", __func__, cmd,	 arg);
		change_client_addr(bcm2079x_dev, arg);
		break;
	case BCMNFC_POWER_CTL:
		NFC_LOG("%s, BCMNFC_POWER_CTL (%x, %lx):\n", __func__, cmd,
			 arg);
		if (arg != 1)
			set_client_addr(bcm2079x_dev, bcm2079x_dev->original_address);
		//gpio_set_value(bcm2079x_dev->en_gpio, arg);
		//add by slv
		if (arg == 1){
			hwPowerOn(MT6351_POWER_LDO_VCN18, VOL_1800, "NFC_SWPIN");
		}else{
			hwPowerDown(MT6351_POWER_LDO_VCN18, "NFC_SWPIN");
		}
		
		mt_set_gpio_out(bcm2079x_dev->en_gpio, arg ? GPIO_OUT_ONE: GPIO_OUT_ZERO );
		break;
	case BCMNFC_WAKE_CTL:
		NFC_LOG( "%s, BCMNFC_WAKE_CTL (%x, %lx):\n", __func__, cmd,	 arg);
		//gpio_set_value(bcm2079x_dev->wake_gpio, arg);		
		mt_set_gpio_out(bcm2079x_dev->wake_gpio, arg ?GPIO_OUT_ONE: GPIO_OUT_ZERO );
		break;
	case BCMNFC_READ_MODE:		
		return READ_RAW_PACKET;
	default:
		NFC_LOG("%s, unknown cmd (%x, %lx)\n", __func__, cmd, arg);
		return 0;
	}

	return 0;
}


static const struct file_operations bcm2079x_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.poll = bcm2079x_dev_poll,
	.read = bcm2079x_dev_read,
	.write = bcm2079x_dev_write,
	.open = bcm2079x_dev_open,	
//#ifdef CONFIG_64BIT 
	.unlocked_ioctl = bcm2079x_dev_unlocked_ioctl,
//#else
	.compat_ioctl = bcm2079x_dev_unlocked_ioctl	
//#endif
};

static int bcm2079x_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret;
	struct bcm2079x_platform_data *platform_data;
	struct bcm2079x_dev *bcm2079x_dev;
#ifdef CONFIG_OF
		u32 ints[2] = { 0, 0 };
		//u32 ints1[2] = { 0, 0 };
		struct device_node *node;
#endif

	NFC_LOG("bcm2079x_probe\n");

	client->dev.platform_data = &bcm2079x_pdata;
	platform_data = client->dev.platform_data;

	NFC_LOG("%s, probing bcm2079x driver flags = %x\n", __func__, client->flags);
	if (platform_data == NULL) {
		dev_err(&client->dev, "nfc probe fail\n");
		NFC_LOG("nfc probe fail\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		NFC_LOG("need I2C_FUNC_I2C fail\n");
		return -ENODEV;
	}

	mt_set_gpio_mode(platform_data->en_gpio, GPIO_MODE_00);
	mt_set_gpio_dir(platform_data->en_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(platform_data->en_gpio, 0);
	
	mt_set_gpio_mode(platform_data->wake_gpio, GPIO_MODE_00);
	mt_set_gpio_dir(platform_data->wake_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(platform_data->wake_gpio, 0);
	NFC_LOG(" gpio config complete \n");

	mt_set_gpio_mode(platform_data->irq_gpio, GPIO_IRQ_NFC_PIN_M_EINT);
	mt_set_gpio_dir(platform_data->irq_gpio, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(platform_data->irq_gpio, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(platform_data->irq_gpio, GPIO_PULL_DOWN);	

	bcm2079x_dev = kzalloc(sizeof(*bcm2079x_dev), GFP_KERNEL);
	
	if (bcm2079x_dev == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		NFC_LOG(" failed to allocate memory for module data  \n");
		ret = -ENOMEM;
		goto err_exit;
	}
	
	p_bcm2079x_dev = bcm2079x_dev;
	
	bcm2079x_dev->wake_gpio = platform_data->wake_gpio;
	bcm2079x_dev->irq_gpio = platform_data->irq_gpio;
	bcm2079x_dev->en_gpio = platform_data->en_gpio;
#if 0
	client->ext_flag |= I2C_DMA_FLAG;
	//client->ext_flag |= I2C_DIRECTION_FLAG;
#else
	client->addr = (client->addr & I2C_MASK_FLAG);
	client->addr = (client->addr | I2C_DMA_FLAG);
       //client->addr = (client->addr | I2C_DIRECTION_FLAG);
#endif
	client->timing = 400;

	bcm2079x_dev->client = client;

	/* init mutex and queues */
	init_waitqueue_head(&bcm2079x_dev->read_wq);
	mutex_init(&bcm2079x_dev->read_mutex);
	spin_lock_init(&bcm2079x_dev->irq_enabled_lock);

	bcm2079x_dev->bcm2079x_device.minor = MISC_DYNAMIC_MINOR;
	bcm2079x_dev->bcm2079x_device.name = "bcm2079x";
	bcm2079x_dev->bcm2079x_device.fops = &bcm2079x_dev_fops;

	ret = misc_register(&bcm2079x_dev->bcm2079x_device);
	if (ret) {
		NFC_LOG(" misc_register failed  \n");
		dev_err(&client->dev, "misc_register failed\n");
		goto err_misc_register;
	}

	NFC_LOG(" %s() saving address %x\n",
		 __func__, client->addr);

	dev_info(&client->dev,
		 "%s, saving address %x\n",
		 __func__, client->addr);

	bcm2079x_dev->original_address = client->addr;
#ifdef CONFIG_64BIT 	
	gpRxDMABuf_va = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&gpRxDMABuf_pa, GFP_KERNEL);
#else
	gpRxDMABuf_va = (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&gpRxDMABuf_pa, GFP_KERNEL);
#endif

    //gpRxDMABuf_va = (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE, &gpRxDMABuf_pa, GFP_KERNEL);
	if(!gpRxDMABuf_va){
        NFC_LOG("2079x Allocate Rx DMA I2C Buffer failed!\n");
	return -1;
    }

#ifdef CONFIG_64BIT 	
	gpTxDMABuf_va = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&gpTxDMABuf_pa, GFP_KERNEL);
#else
	gpTxDMABuf_va = (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&gpTxDMABuf_pa, GFP_KERNEL);
#endif

    //gpTxDMABuf_va = (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE, &gpTxDMABuf_pa, GFP_KERNEL);
	if(!gpTxDMABuf_va){
        NFC_LOG("2079x Allocate Tx DMA I2C Buffer failed!\n");
	return -1;
    }

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	dev_info(&client->dev, "requesting IRQ %d with IRQF_NO_SUSPEND\n", client->irq);
	NFC_LOG( "requesting IRQ %d with IRQF_NO_SUSPEND\n", client->irq);
	bcm2079x_dev->irq_enabled = true;

#ifdef CONFIG_OF
	node = of_find_compatible_node(NULL, NULL, "mediatek, IRQ_NFC-eint");
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		//of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));		
		
#if defined(CONFIG_MTK_LEGACY)
//		mt_gpio_set_debounce(ints[0], ints[1]);	//modify by mtk 
#else
		gpio_request(ints[0], "nfc-irq");
//		gpio_set_debounce(ints[0], ints[1]);	//modify by mtk 
#endif
		client->irq = irq_of_parse_and_map(node, 0);

		//ret = request_irq(client->irq, bcm2079x_dev_irq_handler, CUST_EINT_IRQ_NFC_TYPE, "IRQ_NFC-eint", NULL);
		ret = request_irq(client->irq, bcm2079x_dev_irq_handler, IRQF_TRIGGER_RISING, "IRQ_NFC-eint", NULL);
		if (ret != 0) {
			NFC_ERR("[BCM_NFC]EINT IRQ LINE NOT AVAILABLE\n");
		} else {
			NFC_LOG("[BCM_NFC]BCM_NFC set EINT finished, NFC_irq=%d\n",bcm_nfc_irq );			
		}

		i2c_set_clientdata(client, bcm2079x_dev);
	    //bcm2079x_disable_irq(bcm2079x_dev);
	} else {	
		NFC_ERR("[BCM_NFC]%s can't find compatible node\n", __func__);
	}

#endif 

	dev_info(&client->dev, "%s, probing bcm2079x driver exited successfully\n", __func__);
	
	NFC_LOG( "%s, probing bcm2079x driver exited successfully\n", __func__);
	return 0;

//err_request_irq_failed:
	
err_misc_register:
	misc_deregister(&bcm2079x_dev->bcm2079x_device);

	mutex_destroy(&bcm2079x_dev->read_mutex);
	kfree(bcm2079x_dev);
err_exit:
	gpio_free(platform_data->wake_gpio);
//err_firm:
	gpio_free(platform_data->en_gpio);
//err_en:
	gpio_free(platform_data->irq_gpio);
	NFC_LOG( "%s, probing bcm2079x driver exited fail\n", __func__);

	return ret;
}

static int bcm2079x_remove(struct i2c_client *client)
{
	struct bcm2079x_dev *bcm2079x_dev;
	
	if(gpRxDMABuf_va){
#ifdef CONFIG_64BIT 	
	dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, gpRxDMABuf_va, gpRxDMABuf_pa);
#else
	dma_free_coherent(NULL, MAX_BUFFER_SIZE, gpRxDMABuf_va, gpRxDMABuf_pa);
#endif

        //dma_free_coherent(NULL, MAX_BUFFER_SIZE, gpRxDMABuf_va, gpRxDMABuf_pa);
        gpRxDMABuf_va = NULL;
        gpRxDMABuf_pa = 0;
    }

       if(gpTxDMABuf_va){
#ifdef CONFIG_64BIT 	
	dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, gpTxDMABuf_va, gpTxDMABuf_pa);
#else
	dma_free_coherent(NULL, MAX_BUFFER_SIZE, gpTxDMABuf_va, gpTxDMABuf_pa);
#endif

        //dma_free_coherent(NULL, MAX_BUFFER_SIZE, gpTxDMABuf_va, gpTxDMABuf_pa);
        gpTxDMABuf_va = NULL;
        gpTxDMABuf_pa = 0;
    }
	
	bcm2079x_dev = i2c_get_clientdata(client);
	free_irq(client->irq, bcm2079x_dev);
	misc_deregister(&bcm2079x_dev->bcm2079x_device);
	mutex_destroy(&bcm2079x_dev->read_mutex);
	gpio_free(bcm2079x_dev->irq_gpio);
	gpio_free(bcm2079x_dev->en_gpio);
	gpio_free(bcm2079x_dev->wake_gpio);
	kfree(bcm2079x_dev);

	return 0;
}

static struct of_device_id bcm2079x_table[] = {
	{ .compatible = "broadcom,bcm2079x_nfc",},
	{ },
};

static const struct i2c_device_id bcm2079x_id[] = {
	{"bcm2079x-i2c", 0},
	{}
};

static struct i2c_driver bcm2079x_driver = {
	.id_table = bcm2079x_id,
	.probe = bcm2079x_probe,
	.remove = bcm2079x_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "bcm2079x-i2c",
		.of_match_table = bcm2079x_table,

	},
};

/*
 * module load/unload record keeping
 */
static int __init bcm2079x_dev_init(void)
{
	NFC_LOG("bcm2079x_dev_init\n");
	i2c_register_board_info(NFC_I2C_BUSNUM, &bcm2079x_i2c_nfc,1);
	NFC_LOG("bcm2079x_dev_init ---Middle\n");
	i2c_add_driver(&bcm2079x_driver);
	NFC_LOG("bcm2079x_dev_init ---EXIT\n");
	return 0;
	
}
module_init(bcm2079x_dev_init);

static void __exit bcm2079x_dev_exit(void)
{
	i2c_del_driver(&bcm2079x_driver);
}
module_exit(bcm2079x_dev_exit);


//module_i2c_driver(bcm2079x_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("NFC bcm2079x driver");
MODULE_LICENSE("GPL");
