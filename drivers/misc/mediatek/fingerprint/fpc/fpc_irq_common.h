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

#ifndef __FPC_IRQ_COMMON_H
#define __FPC_IRQ_COMMON_H

#include <linux/completion.h>
#include <linux/input.h>
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/version.h>

#include <linux/spi/spi.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <cust_eint.h>

#define FPC_WAKEUP_UEVENT
#ifdef FPC_WAKEUP_UEVENT
#include <linux/wakelock.h>
#endif
/* -------------------------------------------------------------------------- */
/* platform compatibility                                                     */
/* -------------------------------------------------------------------------- */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	#define SLEEP_US(delay) {usleep_range((delay), (delay)); }
#else
	#define SLEEP_US(delay) {usleep((delay)); }
#endif

/* -------------------------------------------------------------------------- */
/* fpc_irq driver constants                                                   */
/* -------------------------------------------------------------------------- */
#define FPC_IRQ_DEV_NAME	"fpc_irq"

#define CUST_EINT_POLARITY_LOW              0
#define CUST_EINT_POLARITY_HIGH             1
#define CUST_EINT_EDGE_SENSITIVE            0
#define CUST_EINT_LEVEL_SENSITIVE           1

#define GPIO_FP_INT_PIN			(GPIO10 | 0x80000000)
#define GPIO_FP_INT_PIN_M_GPIO	GPIO_MODE_00
#define GPIO_FP_INT_PIN_M_EINT	GPIO_FP_INT_PIN_M_GPIO

#define EINT_FP_EINT_NUM			  10

#define GPIO_FP_SPICLK_PIN		   (GPIO28 | 0x80000000)
#define GPIO_FP_SPICLK_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_FP_SPICLK_PIN_M_PWM  GPIO_MODE_03
#define GPIO_FP_SPICLK_PIN_M_SPI_CK   GPIO_MODE_02

#define GPIO_FP_SPIMISO_PIN			(GPIO25 | 0x80000000)
#define GPIO_FP_SPIMISO_PIN_M_GPIO	GPIO_MODE_00
#define GPIO_FP_SPIMISO_PIN_M_PWM  GPIO_MODE_03
#define GPIO_FP_SPIMISO_PIN_M_SPI_MI   GPIO_MODE_02

#define GPIO_FP_SPIMOSI_PIN			(GPIO27 | 0x80000000)
#define GPIO_FP_SPIMOSI_PIN_M_GPIO	GPIO_MODE_00
#define GPIO_FP_SPIMOSI_PIN_M_PWM  GPIO_MODE_03
#define GPIO_FP_SPIMOSI_PIN_M_SPI_MO   GPIO_MODE_02

#define GPIO_FP_SPICS_PIN		  (GPIO26 | 0x80000000)
#define GPIO_FP_SPICS_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_FP_SPICS_PIN_M_PWM  GPIO_MODE_03
#define GPIO_FP_SPICS_PIN_M_SPI_CS	 GPIO_MODE_02

#define GPIO_FP_RESET_PIN		  (GPIO61 | 0x80000000)
#define GPIO_FP_RESET_PIN_M_GPIO  GPIO_MODE_00

#define GPIO_FP_SPI_PWR_PIN		   (GPIO60 | 0x80000000)
#define GPIO_FP_SPI_PWR_M_GPIO  GPIO_MODE_00

/* NOTE: driver and HAL must share same definition for SIGNAL and STATE */
enum {
	FPC_IRQ_SIGNAL_TEST          = 1,
	FPC_IRQ_SIGNAL_INTERRUPT_REQ = 2,
	FPC_IRQ_SIGNAL_SUSPEND_REQ   = 3,
	FPC_IRQ_SIGNAL_RESUME_REQ    = 4,
#ifdef CONFIG_HAS_EARLYSUSPEND
	FPC_IRQ_SIGNAL_SUSPEND_EARLY_REQ = 5,
	FPC_IRQ_SIGNAL_RESUME_LATE_REQ   = 6,
#endif
};

enum {
	FPC_IRQ_STATE_ACTIVE        = 1,
	FPC_IRQ_STATE_SUSPENDED     = 2,
#ifdef CONFIG_HAS_EARLYSUSPEND
	FPC_IRQ_STATE_EARLY_SUSPEND = 3,
	FPC_IRQ_STATE_LATE_RESUME   = 4,
#endif
};

/* -------------------------------------------------------------------------- */
/* fpc data types                                                             */
/* -------------------------------------------------------------------------- */
struct fpc_irq_setup {
	pid_t dst_pid;
	int   dst_signo;
	int   enabled;
	int   intr_enabled;
	int  tac_init;
	int  unlock_enabled;
	int   test_trigger; // Todo: remove ?
};

struct fpc_irq_pm {
	int  state;
	bool supply_on;
	bool hw_reset;
	bool notify_enabled;
	bool notify_ack;
	bool wakeup_req;
};

typedef struct {
	int irq_gpio;
	int irq_no;
	int rst_gpio;
} fpc_irq_pdata_t;

typedef struct {      
	struct device          *device;    
	struct class           *class;
	struct spi_device      *spi;
    
	dev_t                  devno;
	u8                     *huge_buffer;
	size_t                 huge_buffer_size;
	struct input_dev       *input_dev;
} fpc_spi_data_t;

typedef struct {
	struct platform_device  *plat_dev;
	struct device	  	*dev;
	struct input_dev	*input_dev;
	struct class		*class;
	dev_t			devno;
	fpc_irq_pdata_t		pdata;
	struct fpc_irq_setup	setup;
	struct fpc_irq_pm	pm;
	struct task_struct	*worker_thread;
	struct semaphore	mutex;
	struct semaphore	sem_active;
	bool 			idle_request;
	bool 			term_request;
	wait_queue_head_t	wq_enable;
	wait_queue_head_t	wq_irq_return;
	bool			interrupt_done;
	#ifdef FPC_WAKEUP_UEVENT
        struct wake_lock wake_lock;
	struct work_struct irq_workthread;
        bool sleep;
        #endif
	bool                 suspend_index;
	bool                 init_index;
	bool                 unlock_index;
	//bool                 interrupt_sleep_index;
	struct semaphore	pm_mutex;
	struct completion	pm_suspend_completion;
	struct completion	pm_resume_completion;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	pm_early;
	struct completion	pm_suspend_early_completion;
	struct completion	pm_resume_late_completion;
#endif
} fpc_irq_data_t;


/* -------------------------------------------------------------------------- */
/* function prototypes                                                        */
/* -------------------------------------------------------------------------- */
extern int fpc_irq_check_instance(const char *str_name);

extern int fpc_irq_send_signal(struct device *dev,
				pid_t dst_pid,
				int signo,
				int payload);

#endif /* __FPC_IRQ_COMMON_H */

