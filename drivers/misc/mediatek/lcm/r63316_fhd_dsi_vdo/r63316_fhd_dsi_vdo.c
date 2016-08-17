#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "../inc/lcm_drv.h"
#include <cust_gpio_usage.h>
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
	#include <mach/mt_pm_ldo.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  											(1080)
#define FRAME_HEIGHT 											(1920)

#define SET_GPIO_OUT(n, v)	        							(lcm_util.set_gpio_out((n), (v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)			lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)											lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)						lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
 
#define GPIO_LCD_BIAS_ENP_PIN   (GPIO57 | 0x80000000)    //GPIO57   ->LCD_AVDD_P_EN
#define GPIO_LCD_BIAS_ENN_PIN   (GPIO58 | 0x80000000)    //GPIO58   ->LCD_AVDD_N_EN
#define GPIO_LCD_IOVCC_EN       (GPIO104 | 0x80000000)   //GPIO104  ->LCD_IOVCC_EN
#define GPIO_LCD_ID1            (GPIO3 | 0x80000000)     //GPIO3    ->LCD_ID1
#define GPIO_LCD_ID0            (GPIO82 | 0x80000000)    //GPIO82   ->LCD_ID0

#define GPIO_112_LCD_RST_PIN (GPIO158 |0x80000000)
#define SET_RESET_PIN(v) (lcm_util.set_gpio_out((GPIO_112_LCD_RST_PIN), (v)))

static LCM_UTIL_FUNCS   lcm_util = {0};

#define REGFLAG_DELAY             0XFE
#define REGFLAG_END_OF_TABLE      0xFF   // END OF REGISTERS MARKER

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->dsi.mode   					= SYNC_PULSE_VDO_MODE;
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size=256;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count=FRAME_WIDTH*3;

	params->dsi.vertical_sync_active	= 2;
	params->dsi.vertical_backporch		= 7;
	params->dsi.vertical_frontporch		= 7;
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 20;
	params->dsi.horizontal_backporch	= 40;
	params->dsi.horizontal_frontporch	= 60;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;
	params->dsi.PLL_CLOCK=455;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable      = 0;

	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;

}

static struct LCM_setting_table initialization_setting_new[] ={

	{0xB0,	1,	{0x04}},
	{0xD6,  1,  {0x01}},
	{0xB3,	6,	{0x14,0x00,0x00,0x00,0x00,0x00}},
	{0xB4,  2,  {0x0c,0x00}},
	{0xB6,	3,	{0x4B,0xDB,0x16}},
	{0xBE,  2,  {0x00,0x04}},
	{0xc0,	1,	{0x00}},
	{0xC1,	34,	{0x04,0x60,0x00,0x20,
                 0x29,0x41,0x20,0x63,
                 0xF0,0xFF,0xFF,0x9B,
                 0x7B,0xCF,0xB5,0xFF,
                 0xFF,0x87,0x8C,0xC1,
                 0x11,0x54,0x02,0x00,
                 0x00,0x00,0x00,0x00,
                 0x22,0x11,0x02,0x21,
                 0x00,0xFF}},

	{0xC2,	8,	{0x31,0xF7,0x80,0x08,0x08,0x00,0x00,0x08}},
	{0xC3,	3,	{0x00,0x00,0x00}},
	{0xC4,	11,	{0x70,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,
                 0x00,0x00,0x04}},
	{0xCB,	15,	{0xFF,0xE1,0x87,0xFF,0x00,0x00,0x00,0x00,0xFF,0xE1,0x87,0xFF,0xE8,0x00,0x00}},

	{0xC6,	21,	{0xC8,0x3C,0x3C,0x07,
                 0x01,0x07,0x01,0x00,
                 0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,
                 0x00,0x0E,0x1A,0x07,
                 0xC8}},

	{0xC7,	30,	{0x01,0x14,0x1D,0x26,
                 0x35,0x43,0x4D,0x5C,
                 0x40,0x47,0x53,0x5F,
                 0x76,0x7B,0x7C,0x01,
                 0x14,0x1D,0x26,0x35,
                 0x43,0x4D,0x5C,0x40,
                 0x47,0x53,0x5F,0x76,
                 0x7B,0x7C}},
	{0xCC,	1,	{0x32}},

	{0xD0,	10,	{0x11,0x00,0x00,0x57,
                 0x57,0x40,0x19,0x19,
                 0x09,0x00}},
	{0xD1,	4,	{0x00,0x48,0x16,0x0F}},

	{0xD3,	26,	{0x1B,0x33,0xBB,0xBB,
                 0xB3,0x33,0x33,0x33,
                 0x33,0x00,0x01,0x00,
                 0x00,0xD8,0xA0,0x0C,
                 0x33,0x33,0x33,0x33,
                 0x72,0x12,0x8A,0x57,
                 0x3D,0xBC}},
	{0xB8,  7,  {0x57,0x3D,0x19,0x1E,0x0A,0x50,0x50}},
	{0xD8,  3,	{0x00,0x00,0x00}},
	{0xD9,  3,	{0x00,0x00,0x00}},
	{0xCE,  25,	{0x55,0x40,0x49,0x53,
                 0x59,0x5E,0x63,0x68,
                 0x6E,0x74,0x7E,0x8A,
                 0x98,0xA8,0xBB,0xD0,
                 0xFF,0x01,0xD3,0x04,
                 0x04,0x42,0x04,0x69,
                 0x5A}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}

static void init_lcm_registers(void)
{

	unsigned int data_array[16];

	push_table(initialization_setting_new, sizeof(initialization_setting_new) / sizeof(struct LCM_setting_table), 1);

	data_array[0] = 0x00511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x24531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01551500;
	dsi_set_cmdq(data_array, 1, 1);


	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
}

static void lcm_init(void)
{
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);

	init_lcm_registers();
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);

	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);

	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00022902;
	data_array[1] = 0x000004B0;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00022902;//Deep standby
	data_array[1] = 0x000001B1;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(20);

	SET_RESET_PIN(0);

}

static void lcm_resume(void)
{
	lcm_init();
}

static void lcm_init_power(void)
{
	mt_set_gpio_mode(GPIO_LCD_IOVCC_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_IOVCC_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_IOVCC_EN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
	MDELAY(5);
	lcm_util.set_gpio_mode(GPIO_112_LCD_RST_PIN, GPIO_MODE_00);
	lcm_util.set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_DIR_OUT);
	lcm_util.set_gpio_out(GPIO_112_LCD_RST_PIN, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_112_LCD_RST_PIN, GPIO_OUT_ONE);
	MDELAY(5);
}

static void lcm_resume_power(void)
{
    lcm_init_power();
}

static void lcm_suspend_power(void)
{
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_mode(GPIO_LCD_IOVCC_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_IOVCC_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_IOVCC_EN, GPIO_OUT_ZERO);
	MDELAY(1);
}

static void lcm_set_backlight(void* handle,unsigned int level)
{
	if(level<0x03)level=0x03;
	lcm_backlight_level_setting[0].para_list[0] = level;
	push_table(lcm_backlight_level_setting,sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table),1);
}

static unsigned int lcm_compare_id(void)
{
    unsigned int id0 =1;
    unsigned int id1 =1;

    mt_set_gpio_mode(GPIO_LCD_ID0, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_ID0, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_LCD_ID0, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_LCD_ID0, GPIO_PULL_UP);
    mt_set_gpio_mode(GPIO_LCD_ID1, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_ID1, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_LCD_ID1, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_LCD_ID1, GPIO_PULL_UP);

    id0 = mt_get_gpio_in(GPIO_LCD_ID0);
    id1 = mt_get_gpio_in(GPIO_LCD_ID1);
    #ifdef BUILD_LK
            printf("%s,LK r63315 debug: r63315 id0 =%x id1 =%x\n",__func__,id0,id1);
	#else
            printk("%s,kernel r63315 debug: r63315 id0 =%x id1 =%x\n",__func__,id0,id1);
	#endif

    if((id0==1)&&(id1==1))
        return 1;
    else
        return 0;
}

LCM_DRIVER r63316_fhd_dsi_vdo_lcm_drv =
{
    .name			= "r63311_jdi_diabloX",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
	.set_backlight_cmdq		= lcm_set_backlight,
	.compare_id     = lcm_compare_id,
	.init_power    = lcm_init_power,
	.resume_power  = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
};
