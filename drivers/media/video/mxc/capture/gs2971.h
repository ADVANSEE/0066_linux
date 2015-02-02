#ifndef _GS2971_H
#define _GS2971_H


#ifdef __KERNEL__
#include <linux/spi/spi.h>
#include <linux/device.h>
//#include <linux/videodev.h>
#include <linux/gpio.h>
#include <linux/videodev2.h>
#endif				/* __KERNEL__ */


//#define USE_CEA861	//use hysnc/vsync/de not h:v:f eav mode

//#include <media/davinci/videohd.h> // For HD std (V4L2_STD_1080I, etc)

typedef enum {
     GS2971_VID_STD_1920_1080_60FPS = 0x2B,
     GS2971_VID_STD_1920_1080_50FPS = 0x2D,
     GS2971_VID_STD_1920_1080_60FPS_DBLSAMPLED = 0x2A,
     GS2971_VID_STD_1920_1080_50FPS_DBLSAMPLED = 0x2C,
     GS2971_VID_STD_1280_720_60FPS = 0x20,
     GS2971_VID_STD_1280_720_50FPS = 0x24,

     GS2971_VID_STD_1920_1080_30FPS = 0x0B,
     GS2971_VID_STD_1920_1080_25FPS = 0x0D,
} gs2971_vid_std_t;


#define GS2971_NUM_CHANNELS  1

#define GS2971_MAX_NO_CONTROLS  0
#define GS2971_MAX_NO_INPUTS    1
#define GS2971_MAX_NO_STANDARDS 11


/* Video core registers */

#define GS2971_VCFG1		0x00

#define GS_VCFG1_TRS_REMAP_DISABLE_MASK       0x4000
#define GS_VCFG1_EDH_FLAG_DISABLE_MASK        0x1000
#define GS_VCFG1_EDH_CRC_DISABLE_MASK         0x0800
#define GS_VCFG1_H_CONFIG_MASK                0x0400 /* 1=TRS-based blanking, 0=active-line blanking */

#define GS_VCFG1_ANC_DATA_DISABLE_MASK        0x0200
#define GS_VCFG1_AUDIO_DISABLE_MASK           0x0100
#define GS_VCFG1_861_PIN_DISABLE_MASK         0x0080
#define GS_VCFG1_TIMING_861_MASK              0x0040 /* 1=CEA-861 timing, 0=F/V/H timing */
#define GS_VCFG1_ILLEGAL_REMAP_DISABLE_MASK   0x0010
#define GS_VCFG1_ANC_CRC_DISABLE_MASK         0x0008
#define GS_VCFG1_CRC_INS_DISABLE_MASK         0x0004
#define GS_VCFG1_LINENUM_INS_DISABLE_MASK     0x0002
#define GS_VCFG1_TRS_INS_DISABLE_MASK         0x0001

#define GS2971_VCFG2		0x01
#define GS_VCFG2_TRS_WORD_REMAP_DS2_DISABLE	(1 << 12)
#define GS_VCFG2_REGEN_352M_MASK		(1 << 10)
#define GS_VCFG2_DS_SWAP_3G			(1 << 9)
#define GS_VCFG2_LEVEL_B2A_CONV_DISABLE_MASK	(1 << 8)
#define GS_VCFG2_ANC_EXT_SEL_DS2_nDS1		(1 << 7)
#define GS_VCFG2_AUDIO_SEL_DS2_nDS1		(1 << 6)
#define GS_VCFG2_ILLEGAL_WORD_REMAP_DS2_MASK	(1 << 4)
#define GS_VCFG2_ANC_CHECKSUM_INSERTION_DS2_MASK	(1 << 3)
#define GS_VCFG2_CRC_INS_DS2_MASK		(1 << 2)
#define GS_VCFG2_LNUM_INS_DS2_MASK		(1 << 1)
#define GS_VCFG2_TRS_INS_DS2_MASK		(1 << 0)


#define GS2971_ERROR_STAT_1		0x02
#define GS2971_ERROR_STAT_2		0x03
#define GS2971_EDH_FLAG_IN		0x04
#define GS2971_EDH_FLAG_OUT		0x05


#define GS2971_DATA_FORMAT_DS1		0x06
#define GS2971_DATA_FORMAT_DS2		0x07
#define GS_DF_VD_STD	    0x3f00

#define GS2971_IO_CONFIG		0x08
#define GS2971_IO_CONFIG2		0x09
#define GS_IOCFG_HSYNC		0x00 // horiz blank signal or HSYNC
#define GS_IOCFG_VSYNC		0x01 // vert blank signal or VSYNC
#define GS_IOCFG_DE		0x02 // field signal or DE
#define GS_IOCFG_LOCKED		0x03 // locked indication
#define GS_IOCFG_Y_ANC		0x04 // Luma anciallary indication
#define GS_IOCFG_C_ANC		0x05 // Chroma anciallary indication
#define GS_IOCFG_DATA_ERROR	0x06 // Data error
#define GS_IOCFG_VIDEO_ERROR	0x07 // Video error
#define GS_IOCFG_AUDIO_ERROR	0x08 // Audio error
#define GS_IOCFG_EDH_DETECT	0x09 // EDH detected
#define GS_IOCFG_CARRIER_DETECT	0x0A // Carrier detected
#define GS_IOCFG_RATE_DET0	0x0B // Rate detect bit 0
#define GS_IOCFG_RATE_DET1	0x0C // Rate detect bit 1

#define GS2971_ANC_CONTROL		0x0a
#define ANCCTL_ANC_DATA_SWITCH		(1 << 3)
#define ANCCTL_ANC_DATA_DEL		(1 << 2)

#define GS2971_ANC_LINE_A		0x0b
#define GS2971_ANC_LINE_B		0x0c

#define GS2971_ANC_TYPE_1_AP1		0x0f
#define GS2971_ANC_TYPE_2_AP1		0x10
#define GS2971_ANC_TYPE_3_AP1		0x11
#define GS2971_ANC_TYPE_4_AP1		0x12
#define GS2971_ANC_TYPE_5_AP1		0x13
#define GS2971_ANC_TYPE_1_AP2		0x14
#define GS2971_ANC_TYPE_2_AP2		0x15
#define GS2971_ANC_TYPE_3_AP2		0x16
#define GS2971_ANC_TYPE_4_AP2		0x17
#define GS2971_ANC_TYPE_5_AP2		0x18

#define GS2971_VIDEO_FORMAT_352_A_1	0x19
#define GS2971_VIDEO_FORMAT_352_B_1	0x1a
#define GS2971_VIDEO_FORMAT_352_A_2	0x1b
#define GS2971_VIDEO_FORMAT_352_B_2	0x1c
#define GS2971_VIDEO_FORMAT_352_INS_A	0x1d
#define GS2971_VIDEO_FORMAT_352_INS_B	0x1e

#define GS2971_RASTER_STRUCT1		0x1f // words per active line
#define GS_RS1_WORDS_PER_ACTLINE	0x3fff

#define GS2971_RASTER_STRUCT2		0x20 // words per line (including blanking)
#define GS_RS2_WORDS_PER_LINE		0x3fff

#define GS2971_RASTER_STRUCT3		0x21 // lines per frame (including blanking)
#define GS_RS3_LINES_PER_FRAME		0x7ff

#define GS2971_RASTER_STRUCT4		0x22
#define GS_RS4_RATE_SEL_READBACK	(3 << 14)
#define GS_RS4_RATE_SEL_HD		(0 << 14)
#define GS_RS4_RATE_SEL_SD1		(1 << 14)
#define GS_RS4_RATE_SEL_3G		(2 << 14)
#define GS_RS4_RATE_SEL_SD2		(3 << 14)

#define GS_RS4_M		        (1 << 13)
#define GS_RS4_STD_LOCK			(1 << 12)
#define GS_RS4_INT_nPROG		(1 << 11)
#define GS_RS4_ACTLINES_PER_FIELD	0x7ff

#define GS2971_FLYWHEEL_STATUS		0x23
#define GS_FLY_V_LOCK_DS2		(1 << 4)
#define GS_FLY_H_LOCK_DS2		(1 << 3)
#define GS_FLY_V_LOCK_DS1		(1 << 1)
#define GS_FLY_H_LOCK_DS1		(1 << 0)

#define GS2971_RATE_SEL			0x24
#define GS_RS_AUTO_nMAN			(1 << 2)
#define GS_RS_RATE_SEL_TOP		(3 << 0)

#define GS2971_TIM_861_FORMAT		0x25
#define GS2971_TIM_861_CFG		0x26
#define GS_TIMCFG_VSYNC_INVERT		(1 << 2)
#define GS_TIMCFG_HSYNC_INVERT		(1 << 1)
#define GS_TIMCFG_TRS_861		(1 << 0)	/* BT.656 timing (7 extra lines of blanks) for 525i */


#define GS2971_ERROR_MASK_1		0x37
#define GS2971_ERROR_MASK_2		0x38
#define GS2971_ACGEN_CTRL		0x39
#define GS_ACGEN_SCLK_INV		(1 << 4)
#define GS_ACGEN_AMCLK_INV		(1 << 3)
#define GS_ACGEN_AMCLK_SEL		(3 << 0)
#define GS_ACGEN_AMCLK_SEL_128FS	(0 << 0)
#define GS_ACGEN_AMCLK_SEL_256FS	(1 << 0)
#define GS_ACGEN_AMCLK_SEL_512FS	(2 << 0)

#define GS2971_CLK_GEN			0x6c
#define GS2971_IO_DRIVE_STRENGTH	0x6d
#define GS_IODS_DOUT_MSB		(3 << 4)
#define GS_IODS_STAT			(3 << 2)
#define GS_IODS_DOUT_LSB		(3 << 0)
#define GS_IODS_DOUT_LSB_4MA		(0 << 0)
#define GS_IODS_DOUT_LSB_6MA		(1 << 0)
#define GS_IODS_DOUT_LSB_8MA		(2 << 0)
#define GS_IODS_DOUT_LSB_10MA		(3 << 0)

#define GS2971_EQ_BYPASS		0x73

/* HD Audio registers - regs 0x200-0x297*/

#define GS2971_HD_CFG_AUD		0x200
#define GS2971_HD_ACS_DET		0x201
#define GS2971_HD_AUD_DET1		0x202
#define GS2971_HD_AUD_DET2		0x203
#define GS2971_HD_REGEN			0x204
#define GS2971_HD_CH_MUTE		0x205
#define GS2971_HD_CH_VALID		0x206


/* SD Audio registers - regs 0x400-0x496*/

#define GS2971_CFG_AUD			0x400
#define GS2971_DBN_ERR			0x401
#define GS2971_REGEN			0x402
#define GS2971_AUD_DET			0x403
#define GS2971_CSUM_ERR_DET		0x404
#define GS2971_CH_MUTE			0x405
#define GS2971_CH_VALID			0x406
#define GS2971_SD_AUDIO_ERROR_MASK	0x407

#define GS2971_CFG_OUTPUT		0x408
#define GS2971_OUTPUT_SEL_1		0x409
#define GS2971_OUTPUT_SEL_2		0x40a

#define GS2971_AFNA12			0x420
#define GS2971_AFNA34			0x421
#define GS2971_RATEA			0x422
#define GS2971_ACT_A			0x423
#define GS2971_PRIM_AUD_DELAY_1		0x424
#define GS2971_PRIM_AUD_DELAY_2		0x425
#define GS2971_PRIM_AUD_DELAY_3		0x426
#define GS2971_PRIM_AUD_DELAY_4		0x427
#define GS2971_PRIM_AUD_DELAY_5		0x428
#define GS2971_PRIM_AUD_DELAY_6		0x429
#define GS2971_PRIM_AUD_DELAY_7		0x42a
#define GS2971_PRIM_AUD_DELAY_8		0x42b
#define GS2971_PRIM_AUD_DELAY_9		0x42c
#define GS2971_PRIM_AUD_DELAY_10	0x42d
#define GS2971_PRIM_AUD_DELAY_11	0x42e
#define GS2971_PRIM_AUD_DELAY_12	0x42f

#define GS2971_AFNB12			0x430
#define GS2971_AFNB34			0x431
#define GS2971_RATEB			0x432
#define GS2971_ACT_B			0x433
#define GS2971_SEC_AUD_DELAY_1		0x434
#define GS2971_SEC_AUD_DELAY_2		0x435
#define GS2971_SEC_AUD_DELAY_3		0x436
#define GS2971_SEC_AUD_DELAY_4		0x437
#define GS2971_SEC_AUD_DELAY_5		0x438
#define GS2971_SEC_AUD_DELAY_6		0x439
#define GS2971_SEC_AUD_DELAY_7		0x43a
#define GS2971_SEC_AUD_DELAY_8		0x43b
#define GS2971_SEC_AUD_DELAY_9		0x43c
#define GS2971_SEC_AUD_DELAY_10		0x43d
#define GS2971_SEC_AUD_DELAY_11		0x43e
#define GS2971_SEC_AUD_DELAY_12		0x43f

#define GS2971_ACSR1_2A_BYTE(byte)	(0x440 + ((byte) >> 1))	/* byte 0 - 22 */
#define GS2971_ACSR3_4A_BYTE(byte)	(0x450 + ((byte) >> 1))	/* byte 0 - 22 */
#define GS2971_ACSR1_2B_BYTE(byte)	(0x460 + ((byte) >> 1))	/* byte 0 - 22 */
#define GS2971_ACSR3_4B_BYTE(byte)	(0x470 + ((byte) >> 1))	/* byte 0 - 22 */
#define GS2971_ACSR_BYTE(byte)		(0x480 + (byte))	/* byte 0 - 22 */


/* ANC data extraction - regs 0x800-0xBFF */
#define GS2971_ANC_BANK_REG                      0x800

#define GS2971_ANC_BANK_SIZE                     0x400

// Control pins
#define SMPTE_BYPASS		IMX_GPIO_NR(5, 8)
#define DVB_ASI		IMX_GPIO_NR(5, 9)

#define GS2971_SPI_TRANSFER_MAX 1024

struct gs2971_spidata {
	struct spi_device	*spi;

	struct mutex		buf_lock;
	unsigned		users;
	u8			*buffer;
	u16 txbuf[GS2971_SPI_TRANSFER_MAX+1];
	u16 rxbuf[GS2971_SPI_TRANSFER_MAX+1];
};

struct gs2971_params {
	v4l2_std_id std;
	int         inputidx;
	struct v4l2_format fmt;
};

struct gs2971_channel {
    // struct v4l2_subdev    sd;
     struct gs2971_spidata *spidata;
     struct gs2971_params   params;
     u16    anc_buf[GS2971_ANC_BANK_SIZE];
};

#endif
