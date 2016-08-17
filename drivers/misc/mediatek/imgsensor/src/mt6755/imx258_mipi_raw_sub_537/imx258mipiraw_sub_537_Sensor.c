/*****************************************************************************
 *
 * Filename:
 * ---------
 *     IMX259mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>  
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx258mipiraw_sub_537_Sensor.h"

/****************************Modify following Strings for debug****************************/
#define PFX "IMX258_sub_537_camera_sensor"

#define LOG_1 LOG_INF("IMX258,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 2328*1748@30fps,864Mbps/lane; video 4196*3108@30fps,864Mbps/lane; capture 4656*3496 16M@30fps,864Mbps/lane\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

#define BYTE               unsigned char

static BOOL read_spc_flag = FALSE;

#define imx258_SUB_537_I2C_SPEED 400

static DEFINE_SPINLOCK(imgsensor_drv_lock);

//kal_bool Is_longExp_front=0;  //tanyijun add 

static BYTE imx258_sub_537_SPC_data[252]={0};

extern void read_imx258_sub_537_SPC( BYTE* data );
extern void read_imx258_sub_537_DCC( kal_uint16 addr,BYTE* data, kal_uint32 size);

static imgsensor_info_struct imgsensor_info = {
    .sensor_id = IMX258_SUB_537_SENSOR_ID,        //record sensor id defined in Kd_imgsensor.h

    .checksum_value = 0x16d8abef,        //checksum value for Camera Auto Test
    .pre = {
		.pclk = 259200000,				//record different mode's pclk
		.linelength = 5352,				//record different mode's linelength
		.framelength = 1614,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2100,		//record different mode's width of grabwindow
		.grabwindow_height = 1560,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
    .cap = {/*normal capture*/
		.pclk = 518400000,
		.linelength = 5352,
		.framelength = 3228,//1332,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,//300,
	},
    .cap1 = {/*normal capture*/
		.pclk = 518400000,
		.linelength = 5352,
		.framelength = 3228,//1332,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,//300,
	},

    .normal_video = {   // 30  fps  capture
        .pclk = 302400000,
        .linelength = 5536,
        .framelength = 1796, //3612,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2328, //4656,//4192,
        .grabwindow_height = 1796, //3496,//3104,
        .mipi_data_lp2hs_settle_dc = 30,
        .max_framerate = 300,
    },
    .hs_video = {     // 120 fps
        .pclk = 600000000,
        .linelength = 5536,
        .framelength = 798,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 30,
        .max_framerate = 1200,
    },
    .slim_video = {/*VT Call*/
		.pclk = 259200000,//158400000,
		.linelength = 5352,
		.framelength = 1614,//986,	//1236
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2100,//1400,
		.grabwindow_height = 1560,//752,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
    //ZTEMT:wangkai add for night_mode----Start
    .custom1 = {    // 24 fps  capture
        .pclk = 300000000,
        .linelength = 5536,
        .framelength = 3612,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4656,//4192,
        .grabwindow_height = 3496,//3104,
        .mipi_data_lp2hs_settle_dc = 30,
        .max_framerate = 150,
    },
    .custom2 = {    // 24 fps  capture
        .pclk = 300000000,
        .linelength = 5536,
        .framelength = 3612,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4656,//4192,
        .grabwindow_height = 3496,//3104,
        .mipi_data_lp2hs_settle_dc = 30,
        .max_framerate = 150,
    },
	//ZTEMT:wangkai add for night_mode----End
    .custom3 = {
        .pclk = 300000000,              //record different mode's pclk
        .linelength = 5536,             //record different mode's linelength
        .framelength = 1806,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2328,//2096,        //record different mode's width of grabwindow
        .grabwindow_height = 1748,//1552,       //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 30,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,
    },
    .custom4 = {
        .pclk = 300000000,              //record different mode's pclk
        .linelength = 5536,             //record different mode's linelength
        .framelength = 1806,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2328,//2096,        //record different mode's width of grabwindow
        .grabwindow_height = 1748,//1552,       //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 30,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,
    },
    .custom5 = {
        .pclk = 300000000,              //record different mode's pclk
        .linelength = 5536,             //record different mode's linelength
        .framelength = 1806,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2328,//2096,        //record different mode's width of grabwindow
        .grabwindow_height = 1748,//1552,       //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 30,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,
    },
    .margin = 4,			//sensor framelength & shutter margin
	.min_shutter = 1, //1,		//min shutter
	.max_frame_length = 0x7fff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 1,	  //1, support; 0,not support
	.ihdr_le_firstline = 1,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num

	.cap_delay_frame = 2,		//enter capture delay frame num
	.pre_delay_frame = 2,		//enter preview delay frame num
	.video_delay_frame = 2, 	//enter video delay frame num
	.hs_video_delay_frame = 2,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,//enter slim video delay frame num
    //ZTEMT:wangkai add for night_mode----Start
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2,
    //ZTEMT:wangkai add for night_mode----End

    .isp_driving_current = ISP_DRIVING_4MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
	.i2c_addr_table = {0x34, 0x20, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};




static imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x14d,					//current shutter
	.gain = 0xe000,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
    .current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	//.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,//0x34,//record current sensor's i2c write id
};

#define IMX258_SUB_537_HDR_TYPE (0x00)
#define IMX258_SUB_537_BINNING_TYPE (0x10)
#define IMX258_SUB_537_NOPDAF_TYPE	(0x30)
kal_uint16 imx258_sub_537_type = 0;/*0x00=HDR type, 0x10=binning type*/


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] =
{
 { 4208, 3120,	  4,	0, 4200, 3120, 2100,  1560, 0000, 0000, 2100,  1560,	  0,	0, 2100, 1560}, // Preview
	 { 4208, 3120,	  4,	0, 4200, 3120, 4200,  3120, 0000, 0000, 4200,  3120,	  0,	0, 4200, 3120}, // capture
	 { 4208, 3120,	  4,	0, 4200, 3120, 4200,  3120, 0000, 0000, 4200,  3120,	  0,	0, 4200, 3120}, // video
	 { 4208, 3120,	  8,  	0, 4192, 3120, 1048,  780 , 0000, 0000, 1048,	780,	  0,	0, 1048,  780}, //hight speed video
	 { 4208, 3120,	  4,	0, 4200, 3120, 2100,  1560, 0000, 0000, 2100,  1560,	  0,	0, 2100, 1560},// slim video
 //ZTEMT:wangkai add for night_mode----Start
 { 4656, 3496, 0000, 0000, 4656, 3496, 4656, 3496, 0000, 0000, 4656, 3496, 0000, 0000, 4656, 3496}, // Custom1 (defaultuse preview)
 { 4656, 3496, 0000, 0000, 4656, 3496, 4656, 3496, 0000, 0000, 4656, 3496, 0000, 0000, 4656, 3496}, // Custom2
 //ZTEMT:wangkai add for night_mode----End
 { 4656, 3496, 0000, 0000, 4656, 3496, 2328, 1748, 0000, 0000, 2328, 1748, 0000, 0000, 2328, 1748}, // Custom3
 { 4656, 3496, 0000, 0000, 4656, 3496, 2328, 1748, 0000, 0000, 2328, 1748, 0000, 0000, 2328, 1748}, // Custom4
 { 4656, 3496, 0000, 0000, 4656, 3496, 2328, 1748, 0000, 0000, 2328, 1748, 0000, 0000, 2328, 1748}, // Custom5
 };// slim video

 /*VC1 for HDR(DT=0X35) , VC2 for PDAF(DT=0X36), unit : 8bit*/
 static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=
 {
	/* Preview mode setting */
  	{0x03, 0x0a,	 0x00,	 0x08, 0x40, 0x00,
   	0x00, 0x2b, 0x0834, 0x0618, 0x00, 0x35, 0x0280, 0x0001,
   	0x00, 0x2f, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000},
   	/* Capture mode setting */
   	{0x03, 0x0a,   0x00,   0x08, 0x40, 0x00,
	0x00, 0x2b, 0x1070, 0x0C30, 0x00, 0x35, 0x0280, 0x0001,
   	0x00, 0x2f, 0x00A0, 0x0780, 0x03, 0x00, 0x0000, 0x0000},
	/* Video mode setting */
   	{0x02, 0x0a,   0x00,   0x08, 0x40, 0x00,
	0x00, 0x2b, 0x1070, 0x0C30, 0x01, 0x00, 0x0000, 0x0000,
	0x02, 0x2f, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000}
};

typedef struct
{
    MUINT16 DarkLimit_H;
    MUINT16 DarkLimit_L;
    MUINT16 OverExp_Min_H;
    MUINT16 OverExp_Min_L;
    MUINT16 OverExp_Max_H;
    MUINT16 OverExp_Max_L;
}SENSOR_ATR_INFO, *pSENSOR_ATR_INFO;

static SENSOR_ATR_INFO sensorATR_Info[4]=    
{/* Strength Range Min */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /* Strength Range Std */
    {0x00, 0x32, 0x00, 0x3c, 0x03, 0xff},
    /* Strength Range Max */
    {0x3f, 0xff, 0x3f, 0xff, 0x3f, 0xff},
    /* Strength Range Custom */
    {0x3F, 0xFF, 0x00, 0x0, 0x3F, 0xFF}};


#define IMX258MIPI_SUB_537_MaxGainIndex (154)
kal_uint16 IMX258MIPI_SUB_537_sensorGainMapping[IMX258MIPI_SUB_537_MaxGainIndex][2] ={
	{64 ,0	},
	{65 ,8	},
	{66 ,16 },
	{67 ,25 },
	{68 ,30 },
	{69 ,37 },
	{70 ,45 },
	{71 ,51 },
	{72 ,57 },
	{73 ,63 },
	{74 ,67 },
	{75 ,75 },
	{76 ,81 },
	{77 ,85 },
	{78 ,92 },
	{79 ,96 },
	{80 ,103},
	{81 ,107},
	{82 ,112},
	{83 ,118},
	{84 ,122},
	{86 ,133},
	{88 ,140},
	{89 ,144},
	{90 ,148},
	{93 ,159},
	{96 ,171},
	{97 ,175},
	{99 ,182},
	{101,188},
	{102,192},
	{104,197},
	{106,202},
	{107,206},
	{109,211},
	{112,220},
	{113,222},
	{115,228},
	{118,235},
	{120,239},
	{125,250},
	{126,252},
	{128,256},
	{129,258},
	{130,260},
	{132,264},
	{133,266},
	{135,269},
	{136,271},
	{138,274},
	{139,276},
	{141,279},
	{142,282},
	{144,285},
	{145,286},
	{147,290},
	{149,292},
	{150,294},
	{155,300},
	{157,303},
	{158,305},
	{161,309},
	{163,311},
	{170,319},
	{172,322},
	{174,324},
	{176,326},
	{179,329},
	{181,331},
	{185,335},
	{189,339},
	{193,342},
	{195,344},
	{196,345},
	{200,348},
	{202,350},
	{205,352},
	{207,354},
	{210,356},
	{211,357},
	{214,359},
	{217,361},
	{218,362},
	{221,364},
	{224,366},
	{231,370},
	{237,374},
	{246,379},
	{250,381},
	{252,382},
	{256,384},
	{260,386},
	{262,387},
	{273,392},
	{275,393},
	{280,395},
	{290,399},
	{306,405},
	{312,407},
	{321,410},
	{331,413},
	{345,417},
	{352,419},
	{360,421},
	{364,422},
	{372,424},
	{386,427},
	{400,430},
	{410,432},
	{420,434},
	{431,436},
	{437,437},
	{449,439},
	{455,440},
	{461,441},
	{468,442},
	{475,443},
	{482,444},
	{489,445},
	{496,446},
	{504,447},
	{512,448},
	{520,449},
	{529,450},
	{537,451},
	{546,452},
	{555,453},
	{565,454},
	{575,455},
	{585,456},
	{596,457},
	{607,458},
	{618,459},
	{630,460},
	{642,461},
	{655,462},
	{669,463},
	{683,464},
	{697,465},
	{713,466},
	{728,467},
	{745,468},
	{762,469},
	{780,470},
	{799,471},
	{819,472},
	{840,473},
	{862,474},
	{886,475},
	{910,476},
	{936,477},
	{964,478},
	{993,479},
	{1024,480}
};


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;

    kdSetI2CSpeed(imx258_SUB_537_I2C_SPEED);
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    kdSetI2CSpeed(imx258_SUB_537_I2C_SPEED);
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

// tanyijun add for set actuator to sleep end
static void set_actuator_to_sleep(void)
{
    kal_uint32 addr02h= 0x2;
    kal_uint32 para02h = 0x20;
    kal_uint32 addr32h= 0x32;
    kal_uint32 para32h = 0xA0;	
	
    char pu_send_cmd0[2] = { (char)(addr02h & 0xFF), (char)(para02h & 0xFF)};
    char pu_send_cmd1[2] = { (char)(addr32h & 0xFF), (char)(para32h & 0xFF)};	
    iWriteRegI2C(pu_send_cmd0, 2, 0x18);
    mdelay(1);
    iWriteRegI2C(pu_send_cmd1, 2, 0x18);
    LOG_INF("set_actuator_to_sleep \n");	
}
// tanyijun add for set actuator to sleep end

#if 0
static kal_uint32 imx259_ATR(UINT16 DarkLimit, UINT16 OverExp)
{
/*
    write_cmos_sensor(0x6e50,sensorATR_Info[DarkLimit].DarkLimit_H);
    write_cmos_sensor(0x6e51,sensorATR_Info[DarkLimit].DarkLimit_L);
    write_cmos_sensor(0x9340,sensorATR_Info[OverExp].OverExp_Min_H);
    write_cmos_sensor(03x941,sensorATR_Info[OverExp].OverExp_Min_L);
    write_cmos_sensor(0x9342,sensorATR_Info[OverExp].OverExp_Max_H);
    write_cmos_sensor(0x9343,sensorATR_Info[OverExp].OverExp_Max_L);
    write_cmos_sensor(0x9706,0x10);
    write_cmos_sensor(0x9707,0x03);
    write_cmos_sensor(0x9708,0x03);
    write_cmos_sensor(0x9e24,0x00);
    write_cmos_sensor(0x9e25,0x8c);
    write_cmos_sensor(0x9e26,0x00);
    write_cmos_sensor(0x9e27,0x94);
    write_cmos_sensor(0x9e28,0x00);
    write_cmos_sensor(0x9e29,0x96);
    LOG_INF("DarkLimit 0x6e50(0x%x), 0x6e51(0x%x)\n",sensorATR_Info[DarkLimit].DarkLimit_H,
                                                     sensorATR_Info[DarkLimit].DarkLimit_L);
    LOG_INF("OverExpMin 0x9340(0x%x), 0x9341(0x%x)\n",sensorATR_Info[OverExp].OverExp_Min_H,
                                                     sensorATR_Info[OverExp].OverExp_Min_L);
    LOG_INF("OverExpMin 0x9342(0x%x), 0x9343(0x%x)\n",sensorATR_Info[OverExp].OverExp_Max_H,
                                                     sensorATR_Info[OverExp].OverExp_Max_L);
                                                     */
    return ERROR_NONE; 
}
#endif

static void imx258_sub_537_apply_SPC(void)
{
	int i;

	if(read_spc_flag == FALSE)
	{
		read_imx258_sub_537_SPC(imx258_sub_537_SPC_data);
		read_spc_flag = TRUE;
		return;
	}

	for(i=0; i<63; i++)
	{
		write_cmos_sensor(0xD04C+i, imx258_sub_537_SPC_data[i]);
		LOG_INF("SPC_Data[%d] = %d\n", i, imx258_sub_537_SPC_data[i]);
		//LOG_INF("read_SPC_Data[%d] = %d\n", i, read_cmos_sensor(0xD04C+i));
	}
	for(i=0; i<63; i++)
	{
		write_cmos_sensor(0xD08C+i, imx258_sub_537_SPC_data[i+63]);
		LOG_INF("SPC_Data[%d] = %d\n", i+63, imx258_sub_537_SPC_data[i+63]);
		//LOG_INF("read_SPC_Data[%d] = %d\n", i, read_cmos_sensor(0xD08C+i));
	}
		
}

static void set_dummy(void)
{
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    /* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0104, 0x01);

	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor(0x0104, 0x00);
}    /*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
kal_uint8 tmph,tmpl;
int retry = 10;
kal_uint16 sensor_id = 0;
   // return ((write_cmos_sensor(0x0A29)&0xf0) << 8) | write_cmos_sensor(0x0A2A));
   LOG_INF("Enter\n");
   
   write_cmos_sensor(0x0A02,0x0F);
   write_cmos_sensor(0x0A00,0x01);
   mdelay(10);
   while(retry--)
	{
		 if(read_cmos_sensor(0x0A01) == 0x01)
		 {
		 	 imx258_sub_537_type = read_cmos_sensor(0x0A2E);
             imx258_sub_537_type = IMX258_SUB_537_NOPDAF_TYPE;//ZTEMT:wangkai disable front cam PDAF
			 LOG_INF("imx258_sub_537 type = 0x%x(0x00=HDR,0x10=binning,0x30=No PDAF)", imx258_sub_537_type);
			 sensor_id = (kal_uint16)((read_cmos_sensor(0x0A26) << 4) | (read_cmos_sensor(0x0A27) >> 4));
			 if (sensor_id == 0x258)
           return 0x259;
		 }
	}
   	return 0x00;

}

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    //dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
	//	imgsensor.dummy_line = 0;
	//else
	//	imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}    /*    set_max_framerate  */



/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    //write_shutter(shutter);
    /* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
    /* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

    // OV Recommend Solution
    // if shutter bigger than frame_length, should extend frame length first
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        // Extend frame length
 		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
        }
    } else {
        // Extend frame length
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
    }

    // Update Shutter
	write_cmos_sensor(0x0104, 0x01);
    write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor(0x0203, shutter  & 0xFF);
    write_cmos_sensor(0x0104, 0x00);
    LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}
static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint8 i;

    for (i = 0; i < IMX258MIPI_SUB_537_MaxGainIndex; i++) {
        if(gain <= IMX258MIPI_SUB_537_sensorGainMapping[i][0]){
            break;
        }
    }
    if(gain != IMX258MIPI_SUB_537_sensorGainMapping[i][0])
            LOG_INF("Gain mapping don't correctly:%d %d \n", gain, IMX258MIPI_SUB_537_sensorGainMapping[i][0]);
    return IMX258MIPI_SUB_537_sensorGainMapping[i][1];
}

/*************************************************************************
* FUNCTION
*    set_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    iGain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X	  */
	/* [4:9] = M meams M X		   */
	/* Total gain = M + N /16 X   */

	//
	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");

	if (gain < BASEGAIN)
		gain = BASEGAIN;
	else if (gain > 16 * BASEGAIN)
		gain = 16 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);

	//write_cmos_sensor(0x0216, (reg_gain>>8)& 0xFF);
	//write_cmos_sensor(0x0217, reg_gain & 0xFF);
	write_cmos_sensor(0x0104, 0x00);

	return gain;
}    /*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	kal_uint8 iRation;
	kal_uint16 realtime_fps = 0;
	kal_uint16 reg_gain;
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	spin_lock(&imgsensor_drv_lock);
	if (le > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = le + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
		}
	} else {
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}
	write_cmos_sensor(0x0104, 0x01);
	/* Long exposure */
	write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
	write_cmos_sensor(0x0203, le  & 0xFF);
	/* Short exposure */
	write_cmos_sensor(0x0224, (se >> 8) & 0xFF);
	write_cmos_sensor(0x0225, se  & 0xFF);

	#if 0
	iRation = ((10 * le / se) + 5) / 10;
    if(iRation < 2)
        iRation = 1;
    else if(iRation < 4)
        iRation = 2;
    else if(iRation < 8)
        iRation = 4;
    else if(iRation < 16)
        iRation = 8;
    else if(iRation < 32)
        iRation = 16;
    else
        iRation = 1;
    write_cmos_sensor(0x0222,iRation);
	#endif
	
	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	/* Global analog Gain for Long expo*/
	write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
	/* Global analog Gain for Short expo*/
	write_cmos_sensor(0x0216, (reg_gain>>8)& 0xFF);
	write_cmos_sensor(0x0217, reg_gain & 0xFF);
	write_cmos_sensor(0x0104, 0x00);
}



static void set_mirror_flip(kal_uint8 image_mirror)
{
    LOG_INF("image_mirror = %d\n", image_mirror);
	kal_uint8  iTemp;
	iTemp = read_cmos_sensor(0x0101);
	iTemp&= ~0x03; //Clear the mirror and flip bits.

    switch (image_mirror) {
        case IMAGE_NORMAL:
			write_cmos_sensor(0x0101, iTemp|0x00);
			break;
        case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0101, iTemp|0x01);
            break;
        case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0101, iTemp|0x02);
            break;
        case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0101, iTemp|0x03);
            break;
        default:
            LOG_INF("Error image_mirror setting\n");
    }

}

/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}    /*    night_mode    */

static void imx258_sub_537_ImageQuality_Setting(void)
{
	if((imx258_sub_537_type == IMX258_SUB_537_HDR_TYPE)||(imx258_sub_537_type == IMX258_SUB_537_NOPDAF_TYPE))
	{
		write_cmos_sensor(0x94C7,0xFF);
		write_cmos_sensor(0x94C8,0xFF);
		write_cmos_sensor(0x94C9,0xFF);
		write_cmos_sensor(0x95C7,0xFF);
		write_cmos_sensor(0x95C8,0xFF);
		write_cmos_sensor(0x95C9,0xFF);
		write_cmos_sensor(0x94C4,0x3F);
		write_cmos_sensor(0x94C5,0x3F);
		write_cmos_sensor(0x94C6,0x3F);
		write_cmos_sensor(0x95C4,0x3F);
		write_cmos_sensor(0x95C5,0x3F);
		write_cmos_sensor(0x95C6,0x3F);
		write_cmos_sensor(0x94C1,0x02);
		write_cmos_sensor(0x94C2,0x02);
		write_cmos_sensor(0x94C3,0x02);
		write_cmos_sensor(0x95C1,0x02);
		write_cmos_sensor(0x95C2,0x02);
		write_cmos_sensor(0x95C3,0x02);
		write_cmos_sensor(0x94BE,0x0C);
		write_cmos_sensor(0x94BF,0x0C);
		write_cmos_sensor(0x94C0,0x0C);
		write_cmos_sensor(0x95BE,0x0C);
		write_cmos_sensor(0x95BF,0x0C);
		write_cmos_sensor(0x95C0,0x0C);
		write_cmos_sensor(0x94D0,0x74);
		write_cmos_sensor(0x94D1,0x74);
		write_cmos_sensor(0x94D2,0x74);
		write_cmos_sensor(0x95D0,0x74);
		write_cmos_sensor(0x95D1,0x74);
		write_cmos_sensor(0x95D2,0x74);
		write_cmos_sensor(0x94CD,0x2E);
		write_cmos_sensor(0x94CE,0x2E);
		write_cmos_sensor(0x94CF,0x2E);
		write_cmos_sensor(0x95CD,0x2E);
		write_cmos_sensor(0x95CE,0x2E);
		write_cmos_sensor(0x95CF,0x2E);
		write_cmos_sensor(0x94CA,0x4C);
		write_cmos_sensor(0x94CB,0x4C);
		write_cmos_sensor(0x94CC,0x4C);
		write_cmos_sensor(0x95CA,0x4C);
		write_cmos_sensor(0x95CB,0x4C);
		write_cmos_sensor(0x95CC,0x4C);
		write_cmos_sensor(0x900E,0x32);
		write_cmos_sensor(0x94E2,0xFF);
		write_cmos_sensor(0x94E3,0xFF);
		write_cmos_sensor(0x94E4,0xFF);
		write_cmos_sensor(0x95E2,0xFF);
		write_cmos_sensor(0x95E3,0xFF);
		write_cmos_sensor(0x95E4,0xFF);
		write_cmos_sensor(0x94DF,0x6E);
		write_cmos_sensor(0x94E0,0x6E);
		write_cmos_sensor(0x94E1,0x6E);
		write_cmos_sensor(0x95DF,0x6E);
		write_cmos_sensor(0x95E0,0x6E);
		write_cmos_sensor(0x95E1,0x6E);
		write_cmos_sensor(0x7FCC,0x01);
		write_cmos_sensor(0x7B78,0x00);

		//modify at 2015/10/21		
		write_cmos_sensor(0x9401,0x35);
		write_cmos_sensor(0x9403,0x23);
		write_cmos_sensor(0x9405,0x23);
		write_cmos_sensor(0x9406,0x00);
		write_cmos_sensor(0x9407,0x31);
		write_cmos_sensor(0x9408,0x00);
		write_cmos_sensor(0x9409,0x1B);
		write_cmos_sensor(0x940A,0x00);
		write_cmos_sensor(0x940B,0x15);
		write_cmos_sensor(0x940D,0x3F);
		write_cmos_sensor(0x940F,0x3F);
		write_cmos_sensor(0x9411,0x3F);
		write_cmos_sensor(0x9413,0x64);
		write_cmos_sensor(0x9415,0x64);
		write_cmos_sensor(0x9417,0x64);
		write_cmos_sensor(0x941D,0x34);
		write_cmos_sensor(0x941F,0x01);
		write_cmos_sensor(0x9421,0x01);
		write_cmos_sensor(0x9423,0x01);
		write_cmos_sensor(0x9425,0x23);
		write_cmos_sensor(0x9427,0x23);
		write_cmos_sensor(0x9429,0x23);
		write_cmos_sensor(0x942B,0x2F);
		write_cmos_sensor(0x942D,0x1A);
		write_cmos_sensor(0x942F,0x14);
		write_cmos_sensor(0x9431,0x3F);
		write_cmos_sensor(0x9433,0x3F);
		write_cmos_sensor(0x9435,0x3F);
		write_cmos_sensor(0x9437,0x6B);
		write_cmos_sensor(0x9439,0x7C);
		write_cmos_sensor(0x943B,0x81);
		write_cmos_sensor(0x9443,0x0F);
		write_cmos_sensor(0x9445,0x0F);
		write_cmos_sensor(0x9447,0x0F);
		write_cmos_sensor(0x9449,0x0F);
		write_cmos_sensor(0x944B,0x0F);
		write_cmos_sensor(0x944D,0x0F);
		write_cmos_sensor(0x944F,0x1E);
		write_cmos_sensor(0x9451,0x0F);
		write_cmos_sensor(0x9453,0x0B);
		write_cmos_sensor(0x9455,0x28);
		write_cmos_sensor(0x9457,0x13);
		write_cmos_sensor(0x9459,0x0C);
		write_cmos_sensor(0x945D,0x00);
		write_cmos_sensor(0x945E,0x00);
		write_cmos_sensor(0x945F,0x00);
		write_cmos_sensor(0x946D,0x00);
		write_cmos_sensor(0x946F,0x10);
		write_cmos_sensor(0x9471,0x10);
		write_cmos_sensor(0x9473,0x40);
		write_cmos_sensor(0x9475,0x2E);
		write_cmos_sensor(0x9477,0x10);
		write_cmos_sensor(0x9478,0x0A);
		write_cmos_sensor(0x947B,0xE0);
		write_cmos_sensor(0x947C,0xE0);
		write_cmos_sensor(0x947D,0xE0);
		write_cmos_sensor(0x947E,0xE0);
		write_cmos_sensor(0x947F,0xE0);
		write_cmos_sensor(0x9480,0xE0);
		write_cmos_sensor(0x9483,0x14);
		write_cmos_sensor(0x9485,0x14);
		write_cmos_sensor(0x9487,0x14);
		write_cmos_sensor(0x9501,0x35);
		write_cmos_sensor(0x9503,0x14);
		write_cmos_sensor(0x9505,0x14);
		write_cmos_sensor(0x9507,0x31);
		write_cmos_sensor(0x9509,0x1B);
		write_cmos_sensor(0x950B,0x15);
		write_cmos_sensor(0x950D,0x1E);
		write_cmos_sensor(0x950F,0x1E);
		write_cmos_sensor(0x9511,0x1E);
		write_cmos_sensor(0x9513,0x64);
		write_cmos_sensor(0x9515,0x64);
		write_cmos_sensor(0x9517,0x64);
		write_cmos_sensor(0x951D,0x34);
		write_cmos_sensor(0x951F,0x01);
		write_cmos_sensor(0x9521,0x01);
		write_cmos_sensor(0x9523,0x01);
		write_cmos_sensor(0x9525,0x14);
		write_cmos_sensor(0x9527,0x14);
		write_cmos_sensor(0x9529,0x14);
		write_cmos_sensor(0x952B,0x2F);
		write_cmos_sensor(0x952D,0x1A);
		write_cmos_sensor(0x952F,0x14);
		write_cmos_sensor(0x9531,0x1E);
		write_cmos_sensor(0x9533,0x1E);
		write_cmos_sensor(0x9535,0x1E);
		write_cmos_sensor(0x9537,0x6B);
		write_cmos_sensor(0x9539,0x7C);
		write_cmos_sensor(0x953B,0x81);
		write_cmos_sensor(0x9543,0x0F);
		write_cmos_sensor(0x9545,0x0F);
		write_cmos_sensor(0x9547,0x0F);
		write_cmos_sensor(0x9549,0x0F);
		write_cmos_sensor(0x954B,0x0F);
		write_cmos_sensor(0x954D,0x0F);
		write_cmos_sensor(0x954F,0x15);
		write_cmos_sensor(0x9551,0x0B);
		write_cmos_sensor(0x9553,0x08);
		write_cmos_sensor(0x9555,0x1C);
		write_cmos_sensor(0x9557,0x0D);
		write_cmos_sensor(0x9559,0x08);
		write_cmos_sensor(0x955D,0x00);
		write_cmos_sensor(0x955E,0x00);
		write_cmos_sensor(0x955F,0x00);
		write_cmos_sensor(0x956D,0x00);
		write_cmos_sensor(0x956F,0x10);
		write_cmos_sensor(0x9571,0x10);
		write_cmos_sensor(0x9573,0x40);
		write_cmos_sensor(0x9575,0x2E);
		write_cmos_sensor(0x9577,0x10);
		write_cmos_sensor(0x9578,0x0A);
		write_cmos_sensor(0x957B,0xE0);
		write_cmos_sensor(0x957C,0xE0);
		write_cmos_sensor(0x957D,0xE0);
		write_cmos_sensor(0x957E,0xE0);
		write_cmos_sensor(0x957F,0xE0);
		write_cmos_sensor(0x9580,0xE0);
		write_cmos_sensor(0x9583,0x14);
		write_cmos_sensor(0x9585,0x14);
		write_cmos_sensor(0x9587,0x14);
		write_cmos_sensor(0x7F78,0x00);
		write_cmos_sensor(0x7F89,0x00);
		write_cmos_sensor(0x7F93,0x00);
		write_cmos_sensor(0x924B,0x1B);
		write_cmos_sensor(0x924C,0x0A);
		write_cmos_sensor(0x9304,0x04);
		write_cmos_sensor(0x9315,0x04);
		write_cmos_sensor(0x9250,0x50);
		write_cmos_sensor(0x9251,0x3C);
		write_cmos_sensor(0x9252,0x14);
	}
	else if(imx258_sub_537_type == IMX258_SUB_537_BINNING_TYPE)
	{
		write_cmos_sensor(0x94C7,0xFF);
		write_cmos_sensor(0x94C8,0xFF);
		write_cmos_sensor(0x94C9,0xFF);
		write_cmos_sensor(0x95C7,0xFF);
		write_cmos_sensor(0x95C8,0xFF);
		write_cmos_sensor(0x95C9,0xFF);
		write_cmos_sensor(0x94C4,0x3F);
		write_cmos_sensor(0x94C5,0x3F);
		write_cmos_sensor(0x94C6,0x3F);
		write_cmos_sensor(0x95C4,0x3F);

		write_cmos_sensor(0x95C5,0x3F);
		write_cmos_sensor(0x95C6,0x3F);
		write_cmos_sensor(0x94C1,0x02);
		write_cmos_sensor(0x94C2,0x02);
		write_cmos_sensor(0x94C3,0x02);
		write_cmos_sensor(0x95C1,0x02);
		write_cmos_sensor(0x95C2,0x02);
		write_cmos_sensor(0x95C3,0x02);
		write_cmos_sensor(0x94BE,0x0C);
		write_cmos_sensor(0x94BF,0x0C);
		write_cmos_sensor(0x94C0,0x0C);
		write_cmos_sensor(0x95BE,0x0C);
		write_cmos_sensor(0x95BF,0x0C);
		write_cmos_sensor(0x95C0,0x0C);
		write_cmos_sensor(0x94D0,0x74);
		write_cmos_sensor(0x94D1,0x74);
		write_cmos_sensor(0x94D2,0x74);
		write_cmos_sensor(0x95D0,0x74);
		write_cmos_sensor(0x95D1,0x74);
		write_cmos_sensor(0x95D2,0x74);
		write_cmos_sensor(0x94CD,0x16);
		write_cmos_sensor(0x94CE,0x16);
		write_cmos_sensor(0x94CF,0x16);
		write_cmos_sensor(0x95CD,0x16);
		write_cmos_sensor(0x95CE,0x16);
		write_cmos_sensor(0x95CF,0x16);
		write_cmos_sensor(0x94CA,0x28);
		write_cmos_sensor(0x94CB,0x28);
		write_cmos_sensor(0x94CC,0x28);
		write_cmos_sensor(0x95CA,0x28);
		write_cmos_sensor(0x95CB,0x28);
		write_cmos_sensor(0x95CC,0x28);
		write_cmos_sensor(0x900E,0x32);
		write_cmos_sensor(0x94E2,0xFF);
		write_cmos_sensor(0x94E3,0xFF);
		write_cmos_sensor(0x94E4,0xFF);
		write_cmos_sensor(0x95E2,0xFF);
		write_cmos_sensor(0x95E3,0xFF);
		write_cmos_sensor(0x95E4,0xFF);
		write_cmos_sensor(0x94DF,0x6E);
		write_cmos_sensor(0x94E0,0x6E);
		write_cmos_sensor(0x94E1,0x6E);
		write_cmos_sensor(0x95DF,0x6E);
		write_cmos_sensor(0x95E0,0x6E);
		write_cmos_sensor(0x95E1,0x6E);
		write_cmos_sensor(0x7FCC,0x01);
		write_cmos_sensor(0x7B78,0x00);
		write_cmos_sensor(0x9401,0x35);
		write_cmos_sensor(0x9403,0x23);
		write_cmos_sensor(0x9405,0x23);
		write_cmos_sensor(0x9406,0x00);
		write_cmos_sensor(0x9407,0x31);
		write_cmos_sensor(0x9408,0x00);
		write_cmos_sensor(0x9409,0x1B);
		write_cmos_sensor(0x940A,0x00);
		write_cmos_sensor(0x940B,0x15);
		write_cmos_sensor(0x940D,0x3F);
		write_cmos_sensor(0x940F,0x3F);
		write_cmos_sensor(0x9411,0x3F);
		write_cmos_sensor(0x9413,0x64);
		write_cmos_sensor(0x9415,0x64);
		write_cmos_sensor(0x9417,0x64);
		write_cmos_sensor(0x941D,0x34);
		write_cmos_sensor(0x941F,0x01);
		write_cmos_sensor(0x9421,0x01);
		write_cmos_sensor(0x9423,0x01);
		write_cmos_sensor(0x9425,0x23);
		write_cmos_sensor(0x9427,0x23);
		write_cmos_sensor(0x9429,0x23);
		write_cmos_sensor(0x942B,0x2F);
		write_cmos_sensor(0x942D,0x1A);
		write_cmos_sensor(0x942F,0x14);
		write_cmos_sensor(0x9431,0x3F);
		write_cmos_sensor(0x9433,0x3F);
		write_cmos_sensor(0x9435,0x3F);
		write_cmos_sensor(0x9437,0x6B);
		write_cmos_sensor(0x9439,0x7C);
		write_cmos_sensor(0x943B,0x81);
		write_cmos_sensor(0x9443,0x0F);
		write_cmos_sensor(0x9445,0x0F);
		write_cmos_sensor(0x9447,0x0F);
		write_cmos_sensor(0x9449,0x0F);
		write_cmos_sensor(0x944B,0x0F);
		write_cmos_sensor(0x944D,0x0F);
		write_cmos_sensor(0x944F,0x1E);
		write_cmos_sensor(0x9451,0x0F);
		write_cmos_sensor(0x9453,0x0B);
		write_cmos_sensor(0x9455,0x28);
		write_cmos_sensor(0x9457,0x13);
		write_cmos_sensor(0x9459,0x0C);
		write_cmos_sensor(0x945D,0x00);
		write_cmos_sensor(0x945E,0x00);
		write_cmos_sensor(0x945F,0x00);
		write_cmos_sensor(0x946D,0x00);
		write_cmos_sensor(0x946F,0x10);
		write_cmos_sensor(0x9471,0x10);
		write_cmos_sensor(0x9473,0x40);
		write_cmos_sensor(0x9475,0x2E);
		write_cmos_sensor(0x9477,0x10);
		write_cmos_sensor(0x9478,0x0A);
		write_cmos_sensor(0x947B,0xE0);
		write_cmos_sensor(0x947C,0xE0);
		write_cmos_sensor(0x947D,0xE0);
		write_cmos_sensor(0x947E,0xE0);
		write_cmos_sensor(0x947F,0xE0);
		write_cmos_sensor(0x9480,0xE0);
		write_cmos_sensor(0x9483,0x14);
		write_cmos_sensor(0x9485,0x14);
		write_cmos_sensor(0x9487,0x14);
		write_cmos_sensor(0x9501,0x35);
		write_cmos_sensor(0x9503,0x14);
		write_cmos_sensor(0x9505,0x14);
		write_cmos_sensor(0x9507,0x31);
		write_cmos_sensor(0x9509,0x1B);
		write_cmos_sensor(0x950B,0x15);
		write_cmos_sensor(0x950D,0x1E);
		write_cmos_sensor(0x950F,0x1E);
		write_cmos_sensor(0x9511,0x1E);
		write_cmos_sensor(0x9513,0x64);
		write_cmos_sensor(0x9515,0x64);
		write_cmos_sensor(0x9517,0x64);
		write_cmos_sensor(0x951D,0x34);
		write_cmos_sensor(0x951F,0x01);
		write_cmos_sensor(0x9521,0x01);
		write_cmos_sensor(0x9523,0x01);
		write_cmos_sensor(0x9525,0x14);
		write_cmos_sensor(0x9527,0x14);
		write_cmos_sensor(0x9529,0x14);
		write_cmos_sensor(0x952B,0x2F);
		write_cmos_sensor(0x952D,0x1A);
		write_cmos_sensor(0x952F,0x14);
		write_cmos_sensor(0x9531,0x1E);
		write_cmos_sensor(0x9533,0x1E);
		write_cmos_sensor(0x9535,0x1E);
		write_cmos_sensor(0x9537,0x6B);
		write_cmos_sensor(0x9539,0x7C);
		write_cmos_sensor(0x953B,0x81);
		write_cmos_sensor(0x9543,0x0F);
		write_cmos_sensor(0x9545,0x0F);
		write_cmos_sensor(0x9547,0x0F);
		write_cmos_sensor(0x9549,0x0F);
		write_cmos_sensor(0x954B,0x0F);
		write_cmos_sensor(0x954D,0x0F);
		write_cmos_sensor(0x954F,0x15);
		write_cmos_sensor(0x9551,0x0B);
		write_cmos_sensor(0x9553,0x08);
		write_cmos_sensor(0x9555,0x1C);
		write_cmos_sensor(0x9557,0x0D);
		write_cmos_sensor(0x9559,0x08);
		write_cmos_sensor(0x955D,0x00);
		write_cmos_sensor(0x955E,0x00);
		write_cmos_sensor(0x955F,0x00);
		write_cmos_sensor(0x956D,0x00);
		write_cmos_sensor(0x956F,0x10);
		write_cmos_sensor(0x9571,0x10);
		write_cmos_sensor(0x9573,0x40);
		write_cmos_sensor(0x9575,0x2E);
		write_cmos_sensor(0x9577,0x10);
		write_cmos_sensor(0x9578,0x0A);
		write_cmos_sensor(0x957B,0xE0);
		write_cmos_sensor(0x957C,0xE0);
		write_cmos_sensor(0x957D,0xE0);
		write_cmos_sensor(0x957E,0xE0);
		write_cmos_sensor(0x957F,0xE0);
		write_cmos_sensor(0x9580,0xE0);
		write_cmos_sensor(0x9583,0x14);
		write_cmos_sensor(0x9585,0x14);
		write_cmos_sensor(0x9587,0x14);
		write_cmos_sensor(0x7F78,0x00);
		write_cmos_sensor(0x7F89,0x00);
		write_cmos_sensor(0x7F93,0x00);
		write_cmos_sensor(0x924B,0x1B);
		write_cmos_sensor(0x924C,0x0A);
		write_cmos_sensor(0x9304,0x04);
		write_cmos_sensor(0x9315,0x04);
		write_cmos_sensor(0x9250,0x50);
		write_cmos_sensor(0x9251,0x3C);
		write_cmos_sensor(0x9252,0x14);
		write_cmos_sensor(0x7B5F,0x01);
		write_cmos_sensor(0x94DC,0x20);
		write_cmos_sensor(0x94DD,0x20);
		write_cmos_sensor(0x94DE,0x20);
		write_cmos_sensor(0x95DC,0x20);
		write_cmos_sensor(0x95DD,0x20);
		write_cmos_sensor(0x95DE,0x20);
		write_cmos_sensor(0x7FB0,0x01);
		write_cmos_sensor(0x9010,0x48);
		write_cmos_sensor(0x9419,0x50);
		write_cmos_sensor(0x941B,0x50);
		write_cmos_sensor(0x9519,0x50);
		write_cmos_sensor(0x951B,0x50);

			
		write_cmos_sensor(0xD000,0x00);
		write_cmos_sensor(0xD001,0x18);
		write_cmos_sensor(0xD002,0x00);
		write_cmos_sensor(0xD003,0x18);
		write_cmos_sensor(0xD004,0x10);
		write_cmos_sensor(0xD005,0x57);
		write_cmos_sensor(0xD006,0x0C);
		write_cmos_sensor(0xD007,0x17);
		write_cmos_sensor(0xD00A,0x01);
		write_cmos_sensor(0xD00B,0x02);
		write_cmos_sensor(0xD00D,0x06);
		write_cmos_sensor(0xD00E,0x05);
		write_cmos_sensor(0xD00F,0x01);
		write_cmos_sensor(0xD011,0x06);
		write_cmos_sensor(0xD012,0x07);
		write_cmos_sensor(0xD013,0x01);
		write_cmos_sensor(0xD015,0x05);
		write_cmos_sensor(0xD016,0x0C);
		write_cmos_sensor(0xD017,0x02);
		write_cmos_sensor(0xD019,0x05);
		write_cmos_sensor(0xD01A,0x0E);
		write_cmos_sensor(0xD01B,0x02);
		write_cmos_sensor(0xD01D,0x0E);
		write_cmos_sensor(0xD01E,0x15);
		write_cmos_sensor(0xD01F,0x02);
		write_cmos_sensor(0xD021,0x0E);
		write_cmos_sensor(0xD022,0x17);
		write_cmos_sensor(0xD023,0x02);
		write_cmos_sensor(0xD025,0x0D);
		write_cmos_sensor(0xD026,0x1C);
		write_cmos_sensor(0xD027,0x01);
		write_cmos_sensor(0xD029,0x0D);
		write_cmos_sensor(0xD02A,0x1E);
		write_cmos_sensor(0xD02B,0x01);
	}
}


static void sensor_init(void)
{
	LOG_INF("E\n");
	//init setting
	//imx259
	write_cmos_sensor(0x0136,0x18);
	write_cmos_sensor(0x0137,0x00);

	write_cmos_sensor(0x3051,0x00);
	write_cmos_sensor(0x6B11,0xCF);
	write_cmos_sensor(0x7FF0,0x08);
	write_cmos_sensor(0x7FF1,0x0F);
	write_cmos_sensor(0x7FF2,0x08);
	write_cmos_sensor(0x7FF3,0x1B);
	write_cmos_sensor(0x7FF4,0x23);
	write_cmos_sensor(0x7FF5,0x60);
	write_cmos_sensor(0x7FF6,0x00);
	write_cmos_sensor(0x7FF7,0x01);
	write_cmos_sensor(0x7FF8,0x00);
	write_cmos_sensor(0x7FF9,0x78);
	write_cmos_sensor(0x7FFA,0x01);
	write_cmos_sensor(0x7FFB,0x00);
	write_cmos_sensor(0x7FFC,0x00);
	write_cmos_sensor(0x7FFD,0x00);
	write_cmos_sensor(0x7FFE,0x00);
	write_cmos_sensor(0x7FFF,0x03);
	write_cmos_sensor(0x7F76,0x03);
	write_cmos_sensor(0x7F77,0xFE);
	write_cmos_sensor(0x7FA8,0x03);
	write_cmos_sensor(0x7FA9,0xFE);
	write_cmos_sensor(0x7B24,0x81);
	write_cmos_sensor(0x7B25,0x01);
	write_cmos_sensor(0x6564,0x07);
	write_cmos_sensor(0x6B0D,0x41);
	write_cmos_sensor(0x653D,0x04);
	write_cmos_sensor(0x6B05,0x8C);
	write_cmos_sensor(0x6B06,0xF9);
	write_cmos_sensor(0x6B08,0x65);
	write_cmos_sensor(0x6B09,0xFC);
	write_cmos_sensor(0x6B0A,0xCF);
	write_cmos_sensor(0x6B0B,0xD2);
	write_cmos_sensor(0x6700,0x0E);
	write_cmos_sensor(0x6707,0x0E);
	write_cmos_sensor(0x9104,0x00);
	write_cmos_sensor(0x7421,0x1C);
	write_cmos_sensor(0x7423,0xD7);
	write_cmos_sensor(0x5F04,0x00);
	write_cmos_sensor(0x5F05,0xED);

	imx258_sub_537_ImageQuality_Setting();
	if(imx258_sub_537_type != IMX258_SUB_537_NOPDAF_TYPE)
	{
		imx258_sub_537_apply_SPC();
		write_cmos_sensor(0x7BC8,0x01);//SPC
		write_cmos_sensor(0x7BC9,0x01);
	}
	write_cmos_sensor(0x0B05,0x01);//BPC
	write_cmos_sensor(0x0B06,0x01);
	write_cmos_sensor(0x0100,0x00);
}    /*    sensor_init  */


static void preview_setting(void)
{
	LOG_INF("preview E\n");
    write_cmos_sensor(0x0100,0x00);
	mdelay(10);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x0114,0x03);

	write_cmos_sensor(0x0301,0x05);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x6C);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x02);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xD8);
	write_cmos_sensor(0x0310,0x00);
	write_cmos_sensor(0x0820,0x0A);
	write_cmos_sensor(0x0821,0x20);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0342,0x14);
	write_cmos_sensor(0x0343,0xE8);

	write_cmos_sensor(0x0340,0x06);
	write_cmos_sensor(0x0341,0x4E);

	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x10);
	write_cmos_sensor(0x0349,0x6F);
	write_cmos_sensor(0x034A,0x0C);
	write_cmos_sensor(0x034B,0x2F);

	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x12);
	write_cmos_sensor(0x0902,0x02);

	write_cmos_sensor(0x0401,0x01);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x20);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x10);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x06);
	write_cmos_sensor(0x040F,0x18);
	write_cmos_sensor(0x3038,0x00);
	write_cmos_sensor(0x303A,0x00);
	write_cmos_sensor(0x303B,0x10);
	write_cmos_sensor(0x300D,0x00);

	write_cmos_sensor(0x034C,0x08);
	write_cmos_sensor(0x034D,0x34);
	write_cmos_sensor(0x034E,0x06);
	write_cmos_sensor(0x034F,0x18);

	write_cmos_sensor(0x0202,0x06);
	write_cmos_sensor(0x0203,0x44);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);

	if((imx258_sub_537_type == IMX258_SUB_537_HDR_TYPE)||(imx258_sub_537_type == IMX258_SUB_537_NOPDAF_TYPE))
	{
		write_cmos_sensor(0x7BCD,0x00);
	}
	else if(imx258_sub_537_type == IMX258_SUB_537_BINNING_TYPE)
	{
		write_cmos_sensor(0x7BCD,0x01);
	}
	write_cmos_sensor(0x3030,0x00);
	LOG_INF("0x3030=%d",read_cmos_sensor(0x3030));
	write_cmos_sensor(0x3032,0x00);
	write_cmos_sensor(0x0220,0x00);

    write_cmos_sensor(0x0100,0x01);
	mdelay(10);

}    /*    preview_setting  */
static void preview_setting_HDR_ES2(void)
{
	LOG_INF("Enter\n");
	write_cmos_sensor(0x0100,0x00);
	/*************mode setting***************/														
	write_cmos_sensor(0x0114,0x03); 																
	write_cmos_sensor(0x0220,0x03); 																
	write_cmos_sensor(0x0221,0x22); 																
	write_cmos_sensor(0x0222,0x10);
	write_cmos_sensor(0x302b,0x01);
	write_cmos_sensor(0x0340,0x07); 																
	write_cmos_sensor(0x0341,0x0E); 																
	write_cmos_sensor(0x0342,0x15); 																
	write_cmos_sensor(0x0343,0xA0); 																
	write_cmos_sensor(0x0344,0x00); 																
	write_cmos_sensor(0x0345,0x00); 																
	write_cmos_sensor(0x0346,0x00); 																
	write_cmos_sensor(0x0347,0x00); 																
	write_cmos_sensor(0x0348,0x12); 																
	write_cmos_sensor(0x0349,0x2F); 																
	write_cmos_sensor(0x034A,0x0D); 																
	write_cmos_sensor(0x034B,0xA7); 																
	write_cmos_sensor(0x0381,0x01); 																
	write_cmos_sensor(0x0383,0x01); 																
	write_cmos_sensor(0x0385,0x01); 																
	write_cmos_sensor(0x0387,0x01); 																
	write_cmos_sensor(0x0900,0x00); 																
	write_cmos_sensor(0x0901,0x11); 																
	write_cmos_sensor(0x0902,0x00); 																
	write_cmos_sensor(0x3010,0x75); 																
	write_cmos_sensor(0x3011,0x01); 																
	write_cmos_sensor(0x30C0,0x11); 																
	write_cmos_sensor(0x300D,0x00); 																
	write_cmos_sensor(0x30FD,0x01); 																
	write_cmos_sensor(0x8493,0x00); 																
	write_cmos_sensor(0x8863,0x02); 																
	write_cmos_sensor(0x90D7,0x1E); 																
	/*************output size setting***************/												  
	write_cmos_sensor(0x0112,0x0A); 																
	write_cmos_sensor(0x0113,0x0A); 																
	write_cmos_sensor(0x034C,0x09); 																
	write_cmos_sensor(0x034D,0x18); 																
	write_cmos_sensor(0x034E,0x06); 																
	write_cmos_sensor(0x034F,0xD4); 																
	write_cmos_sensor(0x0401,0x00); 																
	write_cmos_sensor(0x0404,0x00); 																
	write_cmos_sensor(0x0405,0x10); 																
	write_cmos_sensor(0x0408,0x00); 																
	write_cmos_sensor(0x0409,0x00); 																
	write_cmos_sensor(0x040A,0x00); 																
	write_cmos_sensor(0x040B,0x00); 																
	write_cmos_sensor(0x040C,0x09); 																
	write_cmos_sensor(0x040D,0x18); 																
	write_cmos_sensor(0x040E,0x06); 																
	write_cmos_sensor(0x040F,0xD4); 																
	/************clock setting**************/														  
	write_cmos_sensor(0x0301,0x05); 																
	write_cmos_sensor(0x0303,0x02); 																
	write_cmos_sensor(0x0305,0x04); 																
	write_cmos_sensor(0x0306,0x00); 																
	write_cmos_sensor(0x0307,0x7D); 																
	write_cmos_sensor(0x0309,0x0A); 																
	write_cmos_sensor(0x030B,0x01); 																
	write_cmos_sensor(0x030D,0x0F); 																
	write_cmos_sensor(0x030E,0x03); 																
	write_cmos_sensor(0x030F,0x41); 																
	write_cmos_sensor(0x0310,0x00); 																
	/************data rate setting*************/													  
	write_cmos_sensor(0x0820,0x0B); 																
	write_cmos_sensor(0x0821,0xB8); 																
	write_cmos_sensor(0x0822,0x00); 																
	write_cmos_sensor(0x0823,0x00); 																  
	/************integration time setting*******/													
	write_cmos_sensor(0x0202,0x07); 																
	write_cmos_sensor(0x0203,0x04); 																
	write_cmos_sensor(0x0224,0x00); 																
	write_cmos_sensor(0x0225,0x70); 																
	/************integration time setting*******/													  
	write_cmos_sensor(0x0204,0x00); 																
	write_cmos_sensor(0x0205,0x00); 																
	write_cmos_sensor(0x0216,0x00); 																
	write_cmos_sensor(0x0217,0x00); 																
	write_cmos_sensor(0x020E,0x01); 																
	write_cmos_sensor(0x020F,0x00); 																
	write_cmos_sensor(0x0210,0x01); 																
	write_cmos_sensor(0x0211,0x00); 																
	write_cmos_sensor(0x0212,0x01); 																
	write_cmos_sensor(0x0213,0x00); 																
	write_cmos_sensor(0x0214,0x01); 																
	write_cmos_sensor(0x0215,0x00); 																
	/************NR setting*******/ 																
	write_cmos_sensor(0x3058,0x00); 																
	/************pd setting*******/ 																
	write_cmos_sensor(0x3103,0x01);//PD_OUT_EN
   write_cmos_sensor(0x3165,0x01);//0:16*12 window,1:8*6window;2:flexible
   write_cmos_sensor(0x3166,0x01);//flexible window mode 
   /************flexible window*************/
   //write_cmos_sensor(0x3100,0x01);//x_start
   //write_cmos_sensor(0x3101,0x01);
   //write_cmos_sensor(0x3102,0x01);//y_start
   //write_cmos_sensor(0x3103,0x01);
   //write_cmos_sensor(0x3104,0x01);//x_end
   //write_cmos_sensor(0x3105,0x01);
   //write_cmos_sensor(0x3106,0x01);//x_end
   //write_cmos_sensor(0x3107,0x01);
   /************PD 8*6 window*************/
   write_cmos_sensor(0x3108,0x00);//x_start
   write_cmos_sensor(0x3109,0x2C);
   write_cmos_sensor(0x310a,0x00);//y_start
   write_cmos_sensor(0x310b,0x24);
   write_cmos_sensor(0x310c,0x01);//x_end
   write_cmos_sensor(0x310d,0x18);
   write_cmos_sensor(0x310e,0x01);//x_end
   write_cmos_sensor(0x310f,0x16);
   write_cmos_sensor(0x0100,0x01);		
  // mdelay(50);
}    /*    preview_setting  */

static void capture_setting(kal_uint16 curretfps, kal_uint8  pdaf_mode)
{
	LOG_INF("capture E\n");
	LOG_INF("E! currefps:%d\n",curretfps);
	if(curretfps == 150)
	{
		LOG_INF("PIP15fps capture E\n");
		write_cmos_sensor(0x0100,0x00);
		mdelay(10);

		write_cmos_sensor(0x0112,0x0A);
		write_cmos_sensor(0x0113,0x0A);
		write_cmos_sensor(0x0114,0x03);

		write_cmos_sensor(0x0301,0x05);
		write_cmos_sensor(0x0303,0x02);
		write_cmos_sensor(0x0305,0x04);
		write_cmos_sensor(0x0306,0x00);
		write_cmos_sensor(0x0307,0x6C);
		write_cmos_sensor(0x0309,0x0A);
		write_cmos_sensor(0x030B,0x01);
		write_cmos_sensor(0x030D,0x02);
		write_cmos_sensor(0x030E,0x00);
		write_cmos_sensor(0x030F,0xD8);
		write_cmos_sensor(0x0310,0x00);
		write_cmos_sensor(0x0820,0x0A);
		write_cmos_sensor(0x0821,0x20);
		write_cmos_sensor(0x0822,0x00);
		write_cmos_sensor(0x0823,0x00);

		write_cmos_sensor(0x0342,0x14);
		write_cmos_sensor(0x0343,0xE8);

		write_cmos_sensor(0x0340,0x0C);
		write_cmos_sensor(0x0341,0x9C);

		write_cmos_sensor(0x0344,0x00);
		write_cmos_sensor(0x0345,0x00);
		write_cmos_sensor(0x0346,0x00);
		write_cmos_sensor(0x0347,0x00);
		write_cmos_sensor(0x0348,0x10);
		write_cmos_sensor(0x0349,0x6F);
		write_cmos_sensor(0x034A,0x0C);
		write_cmos_sensor(0x034B,0x2F);

		write_cmos_sensor(0x0381,0x01);
		write_cmos_sensor(0x0383,0x01);
		write_cmos_sensor(0x0385,0x01);
		write_cmos_sensor(0x0387,0x01);
		write_cmos_sensor(0x0900,0x00);
		write_cmos_sensor(0x0901,0x11);
		write_cmos_sensor(0x0902,0x00);

		write_cmos_sensor(0x0401,0x00);
		write_cmos_sensor(0x0404,0x00);
		write_cmos_sensor(0x0405,0x10);
		write_cmos_sensor(0x0408,0x00);
		write_cmos_sensor(0x0409,0x00);
		write_cmos_sensor(0x040A,0x00);
		write_cmos_sensor(0x040B,0x00);
		write_cmos_sensor(0x040C,0x10);
		write_cmos_sensor(0x040D,0x70);
		write_cmos_sensor(0x040E,0x0C);
		write_cmos_sensor(0x040F,0x30);
		write_cmos_sensor(0x3038,0x00);
		write_cmos_sensor(0x303A,0x00);
		write_cmos_sensor(0x303B,0x10);
		write_cmos_sensor(0x300D,0x00);

		write_cmos_sensor(0x034C,0x10);
		write_cmos_sensor(0x034D,0x70);
		write_cmos_sensor(0x034E,0x0C);
		write_cmos_sensor(0x034F,0x30);

		write_cmos_sensor(0x0202,0x0C);
		write_cmos_sensor(0x0203,0x92);

		write_cmos_sensor(0x0204,0x00);
		write_cmos_sensor(0x0205,0x00);
		write_cmos_sensor(0x020E,0x01);
		write_cmos_sensor(0x020F,0x00);

		write_cmos_sensor(0x7BCD,0x00);


		if(pdaf_mode == 1) {
			LOG_INF("read 0x3030\n");
			write_cmos_sensor(0x3030,0x01);//pdaf enable
			LOG_INF("0x3030=%d\n",read_cmos_sensor(0x3030));
			write_cmos_sensor(0x3032,0x01); /* 0:raw10, 1:BYTE2 */
			LOG_INF("0x3032=%d\n",read_cmos_sensor(0x3032));

		} else {
			write_cmos_sensor(0x3030,0x00);
			LOG_INF("0x3030=%d\n",read_cmos_sensor(0x3030));
			write_cmos_sensor(0x3032,0x00);
			LOG_INF("0x3032=%d\n",read_cmos_sensor(0x3032));
		}
		write_cmos_sensor(0x0220,0x00);
		write_cmos_sensor(0x0100,0x01);
		mdelay(10);
	}
	else if(curretfps == 240)
	{
		LOG_INF("PIP24fps capture E\n");
		write_cmos_sensor(0x0100,0x00);
		mdelay(10);

		write_cmos_sensor(0x0112,0x0A);
		write_cmos_sensor(0x0113,0x0A);
		write_cmos_sensor(0x0114,0x03);

		write_cmos_sensor(0x0301,0x05);
		write_cmos_sensor(0x0303,0x02);
		write_cmos_sensor(0x0305,0x04);
		write_cmos_sensor(0x0306,0x00);
		write_cmos_sensor(0x0307,0xAD);
		write_cmos_sensor(0x0309,0x0A);
		write_cmos_sensor(0x030B,0x01);
		write_cmos_sensor(0x030D,0x02);
		write_cmos_sensor(0x030E,0x00);
		write_cmos_sensor(0x030F,0xD8);
		write_cmos_sensor(0x0310,0x00);
		write_cmos_sensor(0x0820,0x10);
		write_cmos_sensor(0x0821,0x38);
		write_cmos_sensor(0x0822,0x00);
		write_cmos_sensor(0x0823,0x00);

		write_cmos_sensor(0x0342,0x14);
		write_cmos_sensor(0x0343,0xE8);

		write_cmos_sensor(0x0340,0x0C);
		write_cmos_sensor(0x0341,0x9C);

		write_cmos_sensor(0x0344,0x00);
		write_cmos_sensor(0x0345,0x00);
		write_cmos_sensor(0x0346,0x00);
		write_cmos_sensor(0x0347,0x00);
		write_cmos_sensor(0x0348,0x10);
		write_cmos_sensor(0x0349,0x6F);
		write_cmos_sensor(0x034A,0x0C);
		write_cmos_sensor(0x034B,0x2F);

		write_cmos_sensor(0x0381,0x01);
		write_cmos_sensor(0x0383,0x01);
		write_cmos_sensor(0x0385,0x01);
		write_cmos_sensor(0x0387,0x01);
		write_cmos_sensor(0x0900,0x00);
		write_cmos_sensor(0x0901,0x11);
		write_cmos_sensor(0x0902,0x00);

		write_cmos_sensor(0x0401,0x00);
		write_cmos_sensor(0x0404,0x00);
		write_cmos_sensor(0x0405,0x10);
		write_cmos_sensor(0x0408,0x00);
		write_cmos_sensor(0x0409,0x00);
		write_cmos_sensor(0x040A,0x00);
		write_cmos_sensor(0x040B,0x00);
		write_cmos_sensor(0x040C,0x10);
		write_cmos_sensor(0x040D,0x70);
		write_cmos_sensor(0x040E,0x0C);
		write_cmos_sensor(0x040F,0x30);
		write_cmos_sensor(0x3038,0x00);
		write_cmos_sensor(0x303A,0x00);
		write_cmos_sensor(0x303B,0x10);
		write_cmos_sensor(0x300D,0x00);

		write_cmos_sensor(0x034C,0x10);
		write_cmos_sensor(0x034D,0x70);
		write_cmos_sensor(0x034E,0x0C);
		write_cmos_sensor(0x034F,0x30);

		write_cmos_sensor(0x0202,0x0C);
		write_cmos_sensor(0x0203,0x92);

		write_cmos_sensor(0x0204,0x00);
		write_cmos_sensor(0x0205,0x00);
		write_cmos_sensor(0x020E,0x01);
		write_cmos_sensor(0x020F,0x00);

		write_cmos_sensor(0x7BCD,0x00);


		if(pdaf_mode == 1) {
			LOG_INF("read 0x3030\n");
			write_cmos_sensor(0x3030,0x01);//pdaf enable
			LOG_INF("0x3030=%d\n",read_cmos_sensor(0x3030));
			write_cmos_sensor(0x3032,0x01); /* 0:raw10, 1:BYTE2 */
			LOG_INF("0x3032=%d\n",read_cmos_sensor(0x3032));

		} else {
			write_cmos_sensor(0x3030,0x00);
			LOG_INF("0x3030=%d",read_cmos_sensor(0x3030));
			write_cmos_sensor(0x3032,0x00);
			LOG_INF("0x3032=%d\n",read_cmos_sensor(0x3032));
		}

		write_cmos_sensor(0x0220,0x00);
		write_cmos_sensor(0x0100,0x01);
		mdelay(10);

	}
	else
	{
		write_cmos_sensor(0x0100,0x00);
		mdelay(10);

		write_cmos_sensor(0x0112,0x0A);
		write_cmos_sensor(0x0113,0x0A);
		write_cmos_sensor(0x0114,0x03);

		write_cmos_sensor(0x0301,0x05);
		write_cmos_sensor(0x0303,0x02);
		write_cmos_sensor(0x0305,0x04);
		write_cmos_sensor(0x0306,0x00);
		write_cmos_sensor(0x0307,0xD8);
		write_cmos_sensor(0x0309,0x0A);
		write_cmos_sensor(0x030B,0x01);
		write_cmos_sensor(0x030D,0x02);
		write_cmos_sensor(0x030E,0x00);
		write_cmos_sensor(0x030F,0xD8);
		write_cmos_sensor(0x0310,0x00);
		write_cmos_sensor(0x0820,0x14);
		write_cmos_sensor(0x0821,0x40);
		write_cmos_sensor(0x0822,0x00);
		write_cmos_sensor(0x0823,0x00);

		write_cmos_sensor(0x0342,0x14);
		write_cmos_sensor(0x0343,0xE8);

		write_cmos_sensor(0x0340,0x0C);
		write_cmos_sensor(0x0341,0x9C);

		write_cmos_sensor(0x0344,0x00);
		write_cmos_sensor(0x0345,0x00);
		write_cmos_sensor(0x0346,0x00);
		write_cmos_sensor(0x0347,0x00);
		write_cmos_sensor(0x0348,0x10);
		write_cmos_sensor(0x0349,0x6F);
		write_cmos_sensor(0x034A,0x0C);
		write_cmos_sensor(0x034B,0x2F);

		write_cmos_sensor(0x0381,0x01);
		write_cmos_sensor(0x0383,0x01);
		write_cmos_sensor(0x0385,0x01);
		write_cmos_sensor(0x0387,0x01);
		write_cmos_sensor(0x0900,0x00);
		write_cmos_sensor(0x0901,0x11);
		write_cmos_sensor(0x0902,0x00);

		write_cmos_sensor(0x0401,0x00);
		write_cmos_sensor(0x0404,0x00);
		write_cmos_sensor(0x0405,0x10);
		write_cmos_sensor(0x0408,0x00);
		write_cmos_sensor(0x0409,0x00);
		write_cmos_sensor(0x040A,0x00);
		write_cmos_sensor(0x040B,0x00);
		write_cmos_sensor(0x040C,0x10);
		write_cmos_sensor(0x040D,0x70);
		write_cmos_sensor(0x040E,0x0C);
		write_cmos_sensor(0x040F,0x30);
		write_cmos_sensor(0x3038,0x00);
		write_cmos_sensor(0x303A,0x00);
		write_cmos_sensor(0x303B,0x10);
		write_cmos_sensor(0x300D,0x00);

		write_cmos_sensor(0x034C,0x10);
		write_cmos_sensor(0x034D,0x70);
		write_cmos_sensor(0x034E,0x0C);
		write_cmos_sensor(0x034F,0x30);

		write_cmos_sensor(0x0202,0x0C);
		write_cmos_sensor(0x0203,0x92);

		write_cmos_sensor(0x0204,0x00);
		write_cmos_sensor(0x0205,0x00);
		write_cmos_sensor(0x020E,0x01);
		write_cmos_sensor(0x020F,0x00);

		write_cmos_sensor(0x7BCD,0x00);


		if(pdaf_mode == 1) {
			LOG_INF("read 0x3030\n");
			write_cmos_sensor(0x3030,0x01);//pdaf enable
			LOG_INF("0x3030=%d\n",read_cmos_sensor(0x3030));
			write_cmos_sensor(0x3032,0x01); /* 0:raw10, 1:BYTE2 */
			LOG_INF("0x3032=%d\n",read_cmos_sensor(0x3032));

		} else {
			write_cmos_sensor(0x3030,0x00);
			LOG_INF("0x3030=%d",read_cmos_sensor(0x3030));
			write_cmos_sensor(0x3032,0x00);
			LOG_INF("0x3032=%d\n",read_cmos_sensor(0x3032));
		}

		write_cmos_sensor(0x0220,0x00);

		write_cmos_sensor(0x0100,0x01);
		mdelay(10);
	}
}

static void capture_setting_pdaf(kal_uint16 currefps)
{
	LOG_INF("capture_setting_pdaf E\n");
	write_cmos_sensor(0x0100,0x00);
	mdelay(10);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x0114,0x03);

	write_cmos_sensor(0x0301,0x05);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xD8);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x02);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xD8);
	write_cmos_sensor(0x0310,0x00);
	write_cmos_sensor(0x0820,0x14);
	write_cmos_sensor(0x0821,0x40);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0342,0x14);
	write_cmos_sensor(0x0343,0xE8);

	write_cmos_sensor(0x0340,0x0C);
	write_cmos_sensor(0x0341,0x9C);

	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x10);
	write_cmos_sensor(0x0349,0x6F);
	write_cmos_sensor(0x034A,0x0C);
	write_cmos_sensor(0x034B,0x2F);

	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x00);
	write_cmos_sensor(0x0901,0x11);
	write_cmos_sensor(0x0902,0x00);

	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x10);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x0C);
	write_cmos_sensor(0x040F,0x30);
	write_cmos_sensor(0x3038,0x00);
	write_cmos_sensor(0x303A,0x00);
	write_cmos_sensor(0x303B,0x10);
	write_cmos_sensor(0x300D,0x00);

	write_cmos_sensor(0x034C,0x10);
	write_cmos_sensor(0x034D,0x70);
	write_cmos_sensor(0x034E,0x0C);
	write_cmos_sensor(0x034F,0x30);

	write_cmos_sensor(0x0202,0x0C);
	write_cmos_sensor(0x0203,0x92);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);

	write_cmos_sensor(0x94DC,0x20);
	write_cmos_sensor(0x94DD,0x20);
	write_cmos_sensor(0x94DE,0x20);
	write_cmos_sensor(0x95DC,0x20);
	write_cmos_sensor(0x95DD,0x20);
	write_cmos_sensor(0x95DE,0x20);
	write_cmos_sensor(0x7FB0,0x00);
	write_cmos_sensor(0x9010,0x3E);

	LOG_INF("read 0x3030\n");
	write_cmos_sensor(0x3030,0x01);//pdaf enable
	LOG_INF("0x3030=%d\n",read_cmos_sensor(0x3030));
	write_cmos_sensor(0x3032,0x01); /* 0:raw10, 1:BYTE2 */
	write_cmos_sensor(0x0220,0x00);

	write_cmos_sensor(0x0100,0x01);
	mdelay(10);
}

#if 1
static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal video E\n");
	LOG_INF("E! currefps:%d\n",currefps);
	write_cmos_sensor(0x0100,0x00);
	mdelay(10);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x0114,0x03);

	write_cmos_sensor(0x0301,0x05);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xD8);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x02);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xD8);
	write_cmos_sensor(0x0310,0x00);
	write_cmos_sensor(0x0820,0x14);
	write_cmos_sensor(0x0821,0x40);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0342,0x14);
	write_cmos_sensor(0x0343,0xE8);

	write_cmos_sensor(0x0340,0x0C);
	if((imx258_sub_537_type == IMX258_SUB_537_HDR_TYPE)||(imx258_sub_537_type == IMX258_SUB_537_NOPDAF_TYPE))
		write_cmos_sensor(0x0341,0x9C);
	else if(imx258_sub_537_type == IMX258_SUB_537_BINNING_TYPE)
		write_cmos_sensor(0x0341,0x98);

	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x10);
	write_cmos_sensor(0x0349,0x6F);
	write_cmos_sensor(0x034A,0x0C);
	write_cmos_sensor(0x034B,0x2F);

	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x00);
	write_cmos_sensor(0x0901,0x11);
	write_cmos_sensor(0x0902,0x00);

	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x10);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x0C);
	write_cmos_sensor(0x040F,0x30);
	write_cmos_sensor(0x3038,0x00);
	write_cmos_sensor(0x303A,0x00);
	write_cmos_sensor(0x303B,0x10);
	write_cmos_sensor(0x300D,0x00);

	write_cmos_sensor(0x034C,0x10);
	write_cmos_sensor(0x034D,0x70);
	write_cmos_sensor(0x034E,0x0C);
	write_cmos_sensor(0x034F,0x30);

	write_cmos_sensor(0x0202,0x0C);
	if((imx258_sub_537_type == IMX258_SUB_537_HDR_TYPE)||(imx258_sub_537_type == IMX258_SUB_537_NOPDAF_TYPE))
		write_cmos_sensor(0x0203,0x92);
	else if(imx258_sub_537_type == IMX258_SUB_537_BINNING_TYPE)
		write_cmos_sensor(0x0203,0x8E);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);

	write_cmos_sensor(0x7BCD,0x00);

	write_cmos_sensor(0x3030,0x00);
	write_cmos_sensor(0x3032,0x00);

	//if(imgsensor.ihdr_en == 1)
	//{
		//write_cmos_sensor(0x0220,0x23);	//0x03
		//write_cmos_sensor(0x0222,0x00);
	//}
	//else
	//{
		write_cmos_sensor(0x0220,0x00);
	//}
	//LOG_INF("imgsensor.ihdr_en in video mode:%d\n",imgsensor.ihdr_en);


	write_cmos_sensor(0x0100,0x01);
	mdelay(10);

}

#else
static void normal_video_setting(kal_uint16 currefps)
{
	write_cmos_sensor(0x0100,0x00);
	/*************mode setting************/
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0220,0x00);
	write_cmos_sensor(0x0221,0x11);
	write_cmos_sensor(0x0222,0x10);
	write_cmos_sensor(0x302b,0x00);
	write_cmos_sensor(0x0340,0x0E);
	write_cmos_sensor(0x0341,0x1C);
	write_cmos_sensor(0x0342,0x15);
	write_cmos_sensor(0x0343,0xA0);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x12);
	write_cmos_sensor(0x0349,0x2F);
	write_cmos_sensor(0x034A,0x0D);
	write_cmos_sensor(0x034B,0xA7);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x00);
	write_cmos_sensor(0x0901,0x11);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3010,0x65);
	write_cmos_sensor(0x3011,0x01);
	write_cmos_sensor(0x30C0,0x11);
	write_cmos_sensor(0x300D,0x00);
	write_cmos_sensor(0x30FD,0x00);
	write_cmos_sensor(0x8493,0x00);
	write_cmos_sensor(0x8863,0x00);
	write_cmos_sensor(0x90D7,0x19);
	/************output size setting**********/
	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x12);
	write_cmos_sensor(0x034D,0x30);
	write_cmos_sensor(0x034E,0x0D);
	write_cmos_sensor(0x034F,0xA8);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x12);
	write_cmos_sensor(0x040D,0x30);
	write_cmos_sensor(0x040E,0x0D);
	write_cmos_sensor(0x040F,0xA8);
	/***********clock setting************/
	write_cmos_sensor(0x0301,0x05);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xFA);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0F);
	write_cmos_sensor(0x030E,0x03);
	write_cmos_sensor(0x030F,0x41);
	write_cmos_sensor(0x0310,0x00);
	/*************data rateing setting*********/
	write_cmos_sensor(0x0820,0x17);
	write_cmos_sensor(0x0821,0x70);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);
	/*************integration time setting******/
	write_cmos_sensor(0x0202,0x0E);
	write_cmos_sensor(0x0203,0x12);
	write_cmos_sensor(0x0224,0x01);
	write_cmos_sensor(0x0225,0xF4);
	/************gain setting****************/
	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x0216,0x00);
	write_cmos_sensor(0x0217,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);
	/*******NR Setting*****************/
	write_cmos_sensor(0x3058,0x00);
	/***********PD setting************/
	write_cmos_sensor(0x3103,0x01);//PD_OUT_EN
   write_cmos_sensor(0x3165,0x01);//0:16*12 window,1:8*6window;2:flexible
   write_cmos_sensor(0x3166,0x01);//flexible window mode 
   /************flexible window*************/
   //write_cmos_sensor(0x3100,0x01);//x_start
   //write_cmos_sensor(0x3101,0x01);
   //write_cmos_sensor(0x3102,0x01);//y_start
   //write_cmos_sensor(0x3103,0x01);
   //write_cmos_sensor(0x3104,0x01);//x_end
   //write_cmos_sensor(0x3105,0x01);
   //write_cmos_sensor(0x3106,0x01);//x_end
   //write_cmos_sensor(0x3107,0x01);
   /************PD 8*6 window*************/
   write_cmos_sensor(0x3108,0x00);//x_start
   write_cmos_sensor(0x3109,0x58);
   write_cmos_sensor(0x310a,0x00);//y_start
   write_cmos_sensor(0x310b,0x48);
   write_cmos_sensor(0x310c,0x02);//x_end
   write_cmos_sensor(0x310d,0x30);
   write_cmos_sensor(0x310e,0x02);//x_end
   write_cmos_sensor(0x310f,0x2c);
   write_cmos_sensor(0x0100,0x01);
}

#endif
static void hs_video_setting(void)
{
#if 0
	LOG_INF("hs_video E\n");

	write_cmos_sensor(0x0100,0x00);
	mdelay(10);


	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x0114,0x03);

	write_cmos_sensor(0x0301,0x05);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xD8);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x02);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xD8);
	write_cmos_sensor(0x0310,0x00);
	write_cmos_sensor(0x0820,0x14);
	write_cmos_sensor(0x0821,0x40);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0342,0x14);
	write_cmos_sensor(0x0343,0xE8);

	write_cmos_sensor(0x0340,0x03);
	write_cmos_sensor(0x0341,0x26);

	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x01);
	write_cmos_sensor(0x0347,0xB0);
	write_cmos_sensor(0x0348,0x10);
	write_cmos_sensor(0x0349,0x6F);
	write_cmos_sensor(0x034A,0x0A);
	write_cmos_sensor(0x034B,0x7F);

	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x03);
	write_cmos_sensor(0x0387,0x03);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x12);
	write_cmos_sensor(0x0902,0x02);

	write_cmos_sensor(0x0401,0x01);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x30);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x10);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x02);
	write_cmos_sensor(0x040F,0xF0);
	write_cmos_sensor(0x3038,0x00);
	write_cmos_sensor(0x303A,0x00);
	write_cmos_sensor(0x303B,0x10);
	write_cmos_sensor(0x300D,0x00);

	write_cmos_sensor(0x034C,0x05);
	write_cmos_sensor(0x034D,0x78);
	write_cmos_sensor(0x034E,0x02);
	write_cmos_sensor(0x034F,0xF0);

	write_cmos_sensor(0x0202,0x03);
	write_cmos_sensor(0x0203,0x1C);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);

	if(imx259_type == IMX259_HDR_TYPE)
	{
		write_cmos_sensor(0x7BCD,0x00);
	}
	else if(imx259_type == IMX259_BINNING_TYPE)
	{
		write_cmos_sensor(0x7BCD,0x01);
	}


	write_cmos_sensor(0x3030,0x00);
	write_cmos_sensor(0x3032,0x00);
	write_cmos_sensor(0x0220,0x00);

	write_cmos_sensor(0x0100,0x01);
	//mdelay(10);
#elif 0
	LOG_INF("hs_video E\n");

	write_cmos_sensor(0x0100,0x00);
	mdelay(10);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x0114,0x03);

	write_cmos_sensor(0x0301,0x05);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xC8);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x02);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xD8);
	write_cmos_sensor(0x0310,0x00);
	write_cmos_sensor(0x0820,0x12);
	write_cmos_sensor(0x0821,0xC0);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0342,0x14);
	write_cmos_sensor(0x0343,0xE8);

	write_cmos_sensor(0x0340,0x02);
	write_cmos_sensor(0x0341,0xEA);

	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x02);
	write_cmos_sensor(0x0347,0x50);
	write_cmos_sensor(0x0348,0x10);
	write_cmos_sensor(0x0349,0x6F);
	write_cmos_sensor(0x034A,0x09);
	write_cmos_sensor(0x034B,0xCF);

	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x03);
	write_cmos_sensor(0x0387,0x03);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x12);
	write_cmos_sensor(0x0902,0x02);

	write_cmos_sensor(0x0401,0x01);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x40);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x10);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x02);
	write_cmos_sensor(0x040F,0x80);
	write_cmos_sensor(0x3038,0x00);
	write_cmos_sensor(0x303A,0x00);
	write_cmos_sensor(0x303B,0x10);
	write_cmos_sensor(0x300D,0x01);

	write_cmos_sensor(0x034C,0x04);
	write_cmos_sensor(0x034D,0x18);
	write_cmos_sensor(0x034E,0x01);
	write_cmos_sensor(0x034F,0xE0);

	write_cmos_sensor(0x0202,0x02);
	write_cmos_sensor(0x0203,0xE0);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);

	if(imx259_type == IMX259_HDR_TYPE)
	{
		write_cmos_sensor(0x7BCD,0x00);
	}
	else if(imx259_type == IMX259_BINNING_TYPE)
	{
		write_cmos_sensor(0x7BCD,0x01);
	}

	write_cmos_sensor(0x3030,0x00);
	LOG_INF("0x3030=%d\n",read_cmos_sensor(0x3030));
	write_cmos_sensor(0x3032,0x00);
	LOG_INF("0x3032=%d\n",read_cmos_sensor(0x3032));
	write_cmos_sensor(0x0220,0x00);

	write_cmos_sensor(0x0100,0x01);
	mdelay(10);
#else
	LOG_INF("hs_video E\n");
	
	write_cmos_sensor(0x0100,0x00);
	mdelay(10);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x0114,0x03);

	write_cmos_sensor(0x0301,0x05);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xD8);//0xc8  //PLL_IVT_MPY[7:0]
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x02);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xD8);
	write_cmos_sensor(0x0310,0x00);
	write_cmos_sensor(0x0820,0x14);//0x12   //Output Date rate[31:24]
	write_cmos_sensor(0x0821,0x40);//0xc0  //Output Date rate[23:16]
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0342,0x14);
	write_cmos_sensor(0x0343,0xE8);

	write_cmos_sensor(0x0340,0x03);//0x02
	write_cmos_sensor(0x0341,0x2C);//0xEA

	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);//0x02
	write_cmos_sensor(0x0347,0x00);//0x50
	write_cmos_sensor(0x0348,0x10);
	write_cmos_sensor(0x0349,0x6F);
	write_cmos_sensor(0x034A,0x0C);//0x09
	write_cmos_sensor(0x034B,0x2F);//0xCF

	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);//0x03
	write_cmos_sensor(0x0387,0x01);//0x03
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x14);//0x12
	//write_cmos_sensor(0x0902,0x02);

	write_cmos_sensor(0x0401,0x01);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x40);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x10);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x03);//0x02
	write_cmos_sensor(0x040F,0x0C);//0x80
	write_cmos_sensor(0x3038,0x00);
	write_cmos_sensor(0x303A,0x00);
	write_cmos_sensor(0x303B,0x10);
	write_cmos_sensor(0x300D,0x00);//0x01

	write_cmos_sensor(0x034C,0x04);
	write_cmos_sensor(0x034D,0x18);
	write_cmos_sensor(0x034E,0x03);//0x01
	write_cmos_sensor(0x034F,0x0C);//0xE0

	write_cmos_sensor(0x0202,0x03);//0x02
	write_cmos_sensor(0x0203,0x22);//0xE0

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);

	if((imx258_sub_537_type == IMX258_SUB_537_HDR_TYPE)||(imx258_sub_537_type == IMX258_SUB_537_NOPDAF_TYPE))
	{
		write_cmos_sensor(0x7BCD,0x00);
	}
	else if(imx258_sub_537_type == IMX258_SUB_537_BINNING_TYPE)
	{
		write_cmos_sensor(0x7BCD,0x01);
	}

	write_cmos_sensor(0x3030,0x00);
	LOG_INF("0x3030=%d\n",read_cmos_sensor(0x3030));
	write_cmos_sensor(0x3032,0x01);//0x00
	LOG_INF("0x3032=%d\n",read_cmos_sensor(0x3032));
	write_cmos_sensor(0x0220,0x00);

	write_cmos_sensor(0x0100,0x01);
	mdelay(10);	
#endif

}
static void slim_video_setting(void)
{
#if 0
	LOG_INF("slim_video E\n");

	write_cmos_sensor(0x0100,0x00);
	mdelay(10);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x0114,0x03);

	write_cmos_sensor(0x0301,0x05);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x42);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x02);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xD8);
	write_cmos_sensor(0x0310,0x00);
	write_cmos_sensor(0x0820,0x06);
	write_cmos_sensor(0x0821,0x30);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0342,0x14);
	write_cmos_sensor(0x0343,0xE8);

	write_cmos_sensor(0x0340,0x03);
	write_cmos_sensor(0x0341,0xDA);

	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x01);
	write_cmos_sensor(0x0347,0xB0);
	write_cmos_sensor(0x0348,0x10);
	write_cmos_sensor(0x0349,0x6F);
	write_cmos_sensor(0x034A,0x0A);
	write_cmos_sensor(0x034B,0x7F);

	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x03);
	write_cmos_sensor(0x0387,0x03);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x12);
	write_cmos_sensor(0x0902,0x02);

	write_cmos_sensor(0x0401,0x01);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x30);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x10);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x02);
	write_cmos_sensor(0x040F,0xF0);
	write_cmos_sensor(0x3038,0x00);
	write_cmos_sensor(0x303A,0x00);
	write_cmos_sensor(0x303B,0x10);
	write_cmos_sensor(0x300D,0x00);

	write_cmos_sensor(0x034C,0x05);
	write_cmos_sensor(0x034D,0x78);
	write_cmos_sensor(0x034E,0x02);
	write_cmos_sensor(0x034F,0xF0);

	write_cmos_sensor(0x0202,0x03);
	write_cmos_sensor(0x0203,0xD0);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);

	if(imx259_type == IMX259_HDR_TYPE)
	{
		write_cmos_sensor(0x7BCD,0x00);
	}
	else if(imx259_type == IMX259_BINNING_TYPE)
	{
		write_cmos_sensor(0x7BCD,0x01);
	}

	write_cmos_sensor(0x3030,0x00);
	write_cmos_sensor(0x3032,0x00);
	write_cmos_sensor(0x0220,0x00);

	write_cmos_sensor(0x0100,0x01);
	mdelay(10);
#else
	LOG_INF("slim video E\n");
    write_cmos_sensor(0x0100,0x00);
	mdelay(10);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x0114,0x03);

	write_cmos_sensor(0x0301,0x05);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x6C);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x02);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xD8);
	write_cmos_sensor(0x0310,0x00);
	write_cmos_sensor(0x0820,0x0A);
	write_cmos_sensor(0x0821,0x20);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0342,0x14);
	write_cmos_sensor(0x0343,0xE8);

	write_cmos_sensor(0x0340,0x06);
	write_cmos_sensor(0x0341,0x4E);

	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x10);
	write_cmos_sensor(0x0349,0x6F);
	write_cmos_sensor(0x034A,0x0C);
	write_cmos_sensor(0x034B,0x2F);

	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x12);
	write_cmos_sensor(0x0902,0x02);

	write_cmos_sensor(0x0401,0x01);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x20);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x10);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x06);
	write_cmos_sensor(0x040F,0x18);
	write_cmos_sensor(0x3038,0x00);
	write_cmos_sensor(0x303A,0x00);
	write_cmos_sensor(0x303B,0x10);
	write_cmos_sensor(0x300D,0x00);

	write_cmos_sensor(0x034C,0x08);
	write_cmos_sensor(0x034D,0x34);
	write_cmos_sensor(0x034E,0x06);
	write_cmos_sensor(0x034F,0x18);

	write_cmos_sensor(0x0202,0x06);
	write_cmos_sensor(0x0203,0x44);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);

	if((imx258_sub_537_type == IMX258_SUB_537_HDR_TYPE)||(imx258_sub_537_type == IMX258_SUB_537_NOPDAF_TYPE))
	{
		write_cmos_sensor(0x7BCD,0x00);
	}
	else if(imx258_sub_537_type == IMX258_SUB_537_BINNING_TYPE)
	{
		write_cmos_sensor(0x7BCD,0x01);
	}

	write_cmos_sensor(0x3030,0x00);
	LOG_INF("0x3030=%d\n",read_cmos_sensor(0x3030));
	write_cmos_sensor(0x3032,0x00);
	LOG_INF("0x3032=%d\n",read_cmos_sensor(0x3032));
	write_cmos_sensor(0x0220,0x00);

    write_cmos_sensor(0x0100,0x01);
	mdelay(10);
#endif

}
static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);
    if (enable) {
        write_cmos_sensor(0x0601, 0x02);
    } else {
        write_cmos_sensor(0x0601, 0x00);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
		//开机之后马达上电默认到standby模式，功耗4mA左右，待机漏电
		///在此将马达设置到sleep模式，功耗uA级。tanyijun add 
		  // set_actuator_to_sleep(); 
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }	
 
	
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
    LOG_1;
    LOG_2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;
    
    /* initail sequence write in  */
    sensor_init();
	//imx258_sub_537_apply_SPC();
	//write_cmos_sensor(0x7BC8,0x01);//SPC
    //write_cmos_sensor(0x7BC9,0x01);
    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_mode = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}    /*    open  */



/*************************************************************************
* FUNCTION
*    close
*
* DESCRIPTION
*
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E\n");

    /*No Need to implement this function*/

    return ERROR_NONE;
}    /*    close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    
    int tmp;

   LOG_INF("E\n");	
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	if(imgsensor.ihdr_mode == 2) 
	{
        preview_setting_HDR_ES2(); /*HDR + PDAF*/ 
	}
    else
    {
		preview_setting(); /*PDAF only*/
    }
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    preview   */

/*************************************************************************
* FUNCTION
*    capture
*
* DESCRIPTION
*    This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
	
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps,imgsensor.pdaf_mode);
	#if 0
	if(imgsensor.pdaf_mode == 1)
		capture_setting_pdaf(imgsensor.current_fps);/*PDAF only*/
    else
		capture_setting(imgsensor.current_fps);/*Full mode*/
    #endif
    return ERROR_NONE;
}    /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);	
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();	
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);

    return ERROR_NONE;
}    /*    slim_video     */

//ZTEMT:wangkai add for night_mode----Start
/*************************************************************************
* FUNCTION
* Custom1
*
* DESCRIPTION
*   This function start the sensor Custom1.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	imgsensor.current_fps = 150;
	capture_setting(imgsensor.current_fps,imgsensor.pdaf_mode);
	#if 0
    if(imgsensor.pdaf_mode == 1)
		capture_setting_pdaf(imgsensor.current_fps);/*PDAF only*/
    else
		capture_setting(imgsensor.current_fps);/*Full mode*/
	#endif
    return ERROR_NONE;
}   /*  Custom1   */

/*************************************************************************
* FUNCTION
* Custom2
*
* DESCRIPTION
*   This function start the sensor Custom2.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	imgsensor.current_fps = 150;
	capture_setting(imgsensor.current_fps,imgsensor.pdaf_mode);
	#if 0
    if(imgsensor.pdaf_mode == 1)
		capture_setting_pdaf(imgsensor.current_fps);/*PDAF only*/
    else
		capture_setting(imgsensor.current_fps);/*Full mode*/
	#endif
    return ERROR_NONE;
}   /*  Custom2   */
//ZTEMT:wangkai add for night_mode----End
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    //ZTEMT:wangkai add for night_mode----Start
	sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;
	//ZTEMT:wangkai add for night_mode----End
    return ERROR_NONE;
}    /*    get_resolution    */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);


    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	//ZTEMT:wangkai add for night_mode----Start
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	//ZTEMT:wangkai add for night_mode----End


    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
    //sensor_info->PDAF_Support = 0; /*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode*/

	if(imx258_sub_537_type == IMX258_SUB_537_HDR_TYPE)
		sensor_info->PDAF_Support = 2; /*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode, 3:PDAF Binning mode*/
	else if(imx258_sub_537_type == IMX258_SUB_537_BINNING_TYPE)
		sensor_info->PDAF_Support = 3; 
	else
		sensor_info->PDAF_Support = 0;


    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

            break;
		//ZTEMT:wangkai add for night_mode----Start
		case MSDK_SCENARIO_ID_CUSTOM1:
            sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
            break;
		case MSDK_SCENARIO_ID_CUSTOM2:
            sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
            break;
		//ZTEMT:wangkai add for night_mode----End
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("kdebug scenario_id = %d\n", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            preview(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            capture(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            normal_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            slim_video(image_window, sensor_config_data);
            break;
		//ZTEMT:wangkai add for night_mode----Start
		case MSDK_SCENARIO_ID_CUSTOM1:
            Custom1(image_window, sensor_config_data); // Custom1
            break;
		case MSDK_SCENARIO_ID_CUSTOM2:
            Custom2(image_window, sensor_config_data); // Custom2
            break;
		//ZTEMT:wangkai add for night_mode----End
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}    /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{//This Function not used after ROME
    LOG_INF("framerate = %d\n ", framerate);
    // SetVideoMode Function should fix framerate
    if (framerate == 0)
        // Dynamic frame rate
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);
    set_max_framerate(imgsensor.current_fps,1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable) //enable auto flicker
        imgsensor.autoflicker_en = KAL_TRUE;
    else //Cancel Auto flick
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
		//ZTEMT:wangkai add for night_mode----Start
	    case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
		case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
		//ZTEMT:wangkai add for night_mode----End
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
		//ZTEMT:wangkai add for night_mode----Start
		case MSDK_SCENARIO_ID_CUSTOM1:
            *framerate = imgsensor_info.custom1.max_framerate;
            break;
		case MSDK_SCENARIO_ID_CUSTOM2:
            *framerate = imgsensor_info.custom2.max_framerate;
            break;
		//ZTEMT:wangkai add for night_mode----End
        default:
            break;
    }

    return ERROR_NONE;
}


static kal_uint32 imx258_sub_537_awb_gain(SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
    
    UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

LOG_INF("imx258_sub_537_awb_gain\n");
    grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
    rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
    bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
    gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

    LOG_INF("[imx258_sub_537_awb_gain] ABS_GAIN_GR:%d, grgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GR, grgain_32);
    LOG_INF("[imx258_sub_537_awb_gain] ABS_GAIN_R:%d, rgain_32:%d\n", pSetSensorAWB->ABS_GAIN_R, rgain_32);
    LOG_INF("[imx258_sub_537_awb_gain] ABS_GAIN_B:%d, bgain_32:%d\n", pSetSensorAWB->ABS_GAIN_B, bgain_32);
    LOG_INF("[imx258_sub_537_awb_gain] ABS_GAIN_GB:%d, gbgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GB, gbgain_32);

    write_cmos_sensor(0x0b8e, (grgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b8f, grgain_32 & 0xFF);
    write_cmos_sensor(0x0b90, (rgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b91, rgain_32 & 0xFF);
    write_cmos_sensor(0x0b92, (bgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b93, bgain_32 & 0xFF);
    write_cmos_sensor(0x0b94, (gbgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b95, gbgain_32 & 0xFF);
    return ERROR_NONE;
}


static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long*)feature_para;
	unsigned long long *feature_return_data = (unsigned long long*)feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    SENSOR_VC_INFO_STRUCT *pvcinfo;
    SET_SENSOR_AWB_GAIN *pSetSensorAWB=(SET_SENSOR_AWB_GAIN *)feature_para;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
			night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data_16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:	
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			//ZTEMT:wangkai disable front cam PDAF----Start
			//read_imx258_sub_537_DCC((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
            //ZTEMT:wangkai disable front cam PDAF----End
			break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", *feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data);
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
            break;
		/*HDR CMD*/
		case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", *feature_data);
            spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_mode = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",*feature_data,*(feature_data+1),*(feature_data+2));
            ihdr_write_shutter_gain(*feature_data,*(feature_data+1),*(feature_data+2));
            break;
		case SENSOR_FEATURE_GET_VC_INFO:
            LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
            pvcinfo = (SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
            switch (*feature_data_32) {
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            }
            break;
		case SENSOR_FEATURE_SET_AWB_GAIN:
            imx258_sub_537_awb_gain(pSetSensorAWB);
            break;
		/*END OF HDR CMD*/
		/*PDAF CMD*/
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", *feature_data);
			//PDAF capacity enable or not, 2p8 only full size support PDAF
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0; 
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_PDAF:
			LOG_INF("PDAF mode :%d\n", *feature_data_16);
			*feature_data_16 = 0;  //ZTEMT:wangkai disable front cam PDAF
			imgsensor.pdaf_mode= *feature_data_16;
			break;
		/*End of PDAF*/
        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 IMX258_MIPI_RAW_SUB_537_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    /*    IMX258_MIPI_RAW_SUB_537_SensorInit    */
