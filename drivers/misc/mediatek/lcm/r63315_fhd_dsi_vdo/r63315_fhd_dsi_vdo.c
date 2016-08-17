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
 
#define GPIO_LCD_BIAS_ENP_PIN   (GPIO102 | 0x80000000)    //GPIO57   ->LCD_AVDD_P_EN
#define GPIO_LCD_BIAS_ENN_PIN   (GPIO104 | 0x80000000)    //GPIO58   ->LCD_AVDD_N_EN
#define GPIO_LCD_IOVCC_EN       (GPIO58 | 0x80000000)   //GPIO104  ->LCD_IOVCC_EN
#define GPIO_LCD_ID0            (GPIO57 | 0x80000000)
#define GPIO_LCD_ID1            (GPIO28 | 0x80000000)                   
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
	params->dsi.mode   					= BURST_VDO_MODE;
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
	params->dsi.packet_size=256;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count=720*3;

	params->dsi.vertical_sync_active	= 2;
	params->dsi.vertical_backporch		= 7;
	params->dsi.vertical_frontporch		= 7;
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 20;
	params->dsi.horizontal_backporch	= 60;
	params->dsi.horizontal_frontporch	= 90;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;
	params->dsi.PLL_CLOCK=435;
}

static struct LCM_setting_table initialization_setting_new[] ={
        {0xB0,  1,      {0x04}},
        {0xB3,  6,      {0x14,0x00,0x00,0x00,0x00}},
	{0xB4,	2,	{0x0C,0x00}},
	{0xB6,	3,	{0x4B,0xDB,0x16}},
        {0xB8,  7,      {0x57,0x3D,0x19,0x1E,0x0A,0x50,0x50}},
        {0xB9,  7,      {0x6F,0x3D,0x28,0x3C,0x14,0xC8,0xC8}},
        {0xBA,  7,      {0xB5,0x33,0x41,0x64,0x23,0xA0,0xA0}},
        {0xBB,  2,      {0x14,0x14}},
        {0xBC,  2,      {0x37,0x32}},
        {0xBD,  2,      {0x64,0x32}},
        {0xBE,  2,      {0x00,0x04}},
	{0xC0,	1,	{0x00}},
	{0xC1,	34,	{0x04,0x60,0x00,0x20,
                         0xA9,0x30,0x22,0xFB,
                         0xF0,0xFF,0xFF,0x9B,
                         0x7B,0xCF,0xB5,0xFF,
                         0xFF,0x87,0x9F,0x45,
                         0x22,0x54,0x02,0x00,
                         0x00,0x00,0x00,0x00,
                         0x22,0x33,0x03,0x22,
                         0x00,0xFF}},

	{0xC2,	8,	{0x31,0xF7,0x80,0x08,0x04,0x00,0x00,0x08}},
        {0xC3,  3,      {0x00,0x00,0x00}},
	{0xC4,	11,	{0x70,0x00,0x00,0x00,
			 0x00,0x00,0x00,0x00,
			 0x00,0x00,0x03}},
	{0xC6,	21,	{0xC8,0x3C,0x3C,0x07,
                         0x01,0x07,0x01,0x00,
                         0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,
                         0x00,0x0A,0x1E,0x07,
                         0xC8}},

	{0xC7,	30,	{0x01,0x1E,0x24,0x2D,
                         0x3B,0x48,0x51,0x5F,
                         0x41,0x47,0x52,0x5E,
                         0x67,0x6F,0x76,0x01,
                         0x1E,0x24,0x2D,0x3B,
                         0x48,0x51,0x5F,0x41,
                         0x47,0x52,0x5E,0x67,
                         0x6F,0x76}},
        {0xCB,  15,     {0xFF,0xE1,0x87,0xFF,
                         0x00,0x00,0x00,0x00,
                         0xFF,0xE1,0x87,0xFF,
                         0xE8,0x00,0x00}},
	{0xCC,	1,	{0x34}},
        {0xCE,  25,     {0x55,0x40,0x49,0x53,
                         0x59,0x5E,0x63,0x68,
                         0x6E,0x74,0x7E,0x8A,
                         0x98,0xA8,0xBB,0xD0,
                         0xFF,0x02,0x38,0x04,
                         0x04,0x42,0x00,0x69,
                         0x5A}},   
	{0xD0,	10,	{0x11,0x00,0x00,0x54,
                         0xD2,0x40,0x19,0x19,
                         0x09,0x00}},
        {0xD1,  4,      {0x00,0x48,0x16,0x0F}},
 
	{0xD3,	26,	{0x1B,0x33,0xBB,0xBB,
                         0xB3,0x33,0x33,0x33,
                         0x33,0x00,0x01,0x00,
                         0x00,0xF8,0xA0,0x08,
                         0x2F,0x2F,0x33,0x33,
                         0x72,0x12,0x8A,0x57,
                         0x3D,0xBC}},
        {0xD4,  3,      {0x41,0x04,0x00}},
        {0xD6,  1,      {0x01}},
        {0xD7,  24,     {0xBF,0xF8,0x7F,0xA8,
                         0xCE,0x3E,0xFC,0xC1,
                         0xE1,0xEF,0x83,0x07,
                         0x3F,0x10,0x7F,0xC0,
                         0x01,0xE7,0x40,0x1C,
                         0x00,0xC0,0x00,0x00}},
        {0xD8,   3,     {0x00,0x00,0x00}},
        {0xD9,   3,     {0x00,0x00,0x00}},
        {0xDD,   4,     {0x30,0x06,0x23,0x65}},
        {0xDE,   4,     {0x00,0x3F,0xFF,0x10}},	
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

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(20);
        data_array[0] = 0x00110500;
        dsi_set_cmdq(data_array, 1, 1);
        MDELAY(80);
 
}
static void lcm_init(void)
{
      
       mt_set_gpio_mode(GPIO_LCD_IOVCC_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_IOVCC_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_IOVCC_EN, GPIO_OUT_ONE);
    MDELAY(5);
        mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
    MDELAY(5);
        mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
    MDELAY(5);
       mt_set_gpio_mode(GPIO_LCD_ID0, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_ID0, GPIO_DIR_IN);
    MDELAY(5);
        mt_set_gpio_mode(GPIO_LCD_ID1, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_ID1, GPIO_DIR_IN);
    MDELAY(5);
	lcm_util.set_gpio_mode(GPIO_112_LCD_RST_PIN, GPIO_MODE_00);
	lcm_util.set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_DIR_OUT); 
	lcm_util.set_gpio_out(GPIO_112_LCD_RST_PIN, GPIO_OUT_ZERO); 

        mt_set_gpio_out(GPIO_112_LCD_RST_PIN, GPIO_OUT_ONE);	 
       SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(50);
       SET_RESET_PIN(1);
 
	MDELAY(20);

	init_lcm_registers();
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
	SET_RESET_PIN(0);
        MDELAY(20);
        SET_RESET_PIN(1);
        MDELAY(50);
	data_array[0] = 0x00022902;
        data_array[1] = 0x000004B0;
	dsi_set_cmdq(data_array, 2, 1);
       
	data_array[0] = 0x00022902;//Deep standby
        data_array[1] = 0x000001B1; 
	dsi_set_cmdq(data_array, 2, 1);

	MDELAY(20);
        mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
    MDELAY(5);
        mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
    MDELAY(5);
    mt_set_gpio_mode(GPIO_LCD_IOVCC_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_IOVCC_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_IOVCC_EN, GPIO_OUT_ZERO);
       SET_RESET_PIN(0);
}

static void lcm_resume(void)
{

	lcm_init();

}

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
       unsigned int data_array[16];
       char  buffer[3] = {0};

        data_array[0]= 0x00013700;                    
        dsi_set_cmdq(&data_array, 1, 1);
        read_reg_v2(0x0a, buffer, 1);
        
        if(buffer[0] == 0x1c)
        {
            return FALSE;
        }
        else
        {            
             printk("--test zhjg esd reg 0x0a =0x%x\n",buffer[0]);
             return TRUE;
        }
#endif
}

static unsigned int lcm_esd_recover(void)
{
    
    lcm_resume();
    return TRUE;
}

static void lcm_set_backlight(void* handle,unsigned int level)                                                                             
{	
        lcm_backlight_level_setting[0].para_list[0] = level;
	push_table(lcm_backlight_level_setting,sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table),1);
}
LCM_DRIVER r63315_fhd_dsi_vdo_lcm_drv =
{
    .name			= "r63311_jdi_diabloX",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .esd_check   = lcm_esd_check,
    .esd_recover   = lcm_esd_recover,
    .set_backlight_cmdq		= lcm_set_backlight,


    
};
