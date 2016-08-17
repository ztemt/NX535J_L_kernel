/* Fingerprint Cards, Hybrid Touch sensor driver
 *
 * Copyright (c) 2014,2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 *
 * Software license : "Dual BSD/GPL"
 * see <linux/module.h> and ./Documentation
 * for  details.
 *
*/

#define DEBUG

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h> 
#include <linux/delay.h>
#include <linux/poll.h> 
#include <linux/sched.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>

#include "fpc_irq_common.h"
#include "fpc_irq_supply.h"
#include <mach/mt_gpio.h>
#include <mach/mt_spi.h>
#include <mach/mt_clkmgr.h>


extern  fpc_spi_data_t* g_fpc_spi_data_t ;
extern void nubia_enable_spi_clk(struct spi_device * spidev, bool en);

static inline void fpc_hw_reset(void)
{
        mt_set_gpio_mode(GPIO_FP_RESET_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_FP_RESET_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_FP_RESET_PIN, GPIO_OUT_ONE);
        msleep(10);
	 mt_set_gpio_out(GPIO_FP_RESET_PIN, GPIO_OUT_ZERO);
	 msleep(10);
	 mt_set_gpio_out(GPIO_FP_RESET_PIN, GPIO_OUT_ONE);
}


static void enable_clk(void)
{
	//#if (!defined(CONFIG_MT_SPI_FPGA_ENABLE))
//		enable_clock(MT_CG_PERI_SPI0, "spi");
            printk( "%s enter\n", __func__);
    
            if(NULL != g_fpc_spi_data_t){
                nubia_enable_spi_clk(g_fpc_spi_data_t->spi,true);                
		   printk("clock enabled !!\n");
             }
	///#endif
		return;
}

static void spi_gpio_set(void)
{
      //spi1
	mt_set_gpio_mode(GPIO_FP_SPICS_PIN, GPIO_FP_SPICS_PIN_M_SPI_CS);
	mt_set_gpio_mode(GPIO_FP_SPICLK_PIN, GPIO_FP_SPICLK_PIN_M_SPI_CK);
	mt_set_gpio_mode(GPIO_FP_SPIMISO_PIN, GPIO_FP_SPIMISO_PIN_M_SPI_MI);
	mt_set_gpio_mode(GPIO_FP_SPIMOSI_PIN, GPIO_FP_SPIMOSI_PIN_M_SPI_MO);
	return;
}

/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration								  */
/* -------------------------------------------------------------------- */
static void fpc_hw_power_enable(u8 bonoff)
{
	
	mt_set_gpio_mode(GPIO_FP_SPI_PWR_PIN, GPIO_FP_SPI_PWR_M_GPIO);
	mt_set_gpio_dir(GPIO_FP_SPI_PWR_PIN, GPIO_DIR_OUT);
	if(bonoff == 0)
		mt_set_gpio_out(GPIO_FP_SPI_PWR_PIN, GPIO_OUT_ZERO);
	else
		mt_set_gpio_out(GPIO_FP_SPI_PWR_PIN, GPIO_OUT_ONE);
	mdelay(15);
	
	return;
}

/* -------------------------------------------------------------------------- */
int fpc_irq_supply_init(fpc_irq_data_t *fpc_irq_data)
{
	int ret = 0;
	dev_dbg(fpc_irq_data->dev, "%s\n", __func__);
	spi_gpio_set();
	enable_clk();
       fpc_hw_power_enable(1);
	//fpc_hw_reset();

	return ret;
}


/* -------------------------------------------------------------------------- */
int fpc_irq_supply_destroy(fpc_irq_data_t *fpc_irq_data)
{
	int ret = 0;

	dev_dbg(fpc_irq_data->dev, "%s\n", __func__);

	// Todo: release used regulators

	return ret;
}


/* -------------------------------------------------------------------------- */
extern int fpc_irq_supply_set(fpc_irq_data_t *fpc_irq_data, bool req_state)
{
	int ret = 0;
	bool curr_state = fpc_irq_data->pm.supply_on;

	dev_dbg(fpc_irq_data->dev, "%s %s => %s\n",
						__func__,
						(curr_state) ? "ON" : "OFF",
						(req_state) ? "ON" : "OFF");

	if (curr_state != req_state) {

		fpc_irq_data->pm.supply_on = req_state;

		// Todo: enable/disable used regulators
		// Todo: If state == off, also set I/O as required fo not sourcing the sensor.
	}

	return ret;
}


/* -------------------------------------------------------------------------- */

