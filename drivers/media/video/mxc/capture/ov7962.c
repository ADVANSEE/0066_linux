/*
 * Copyright (C) 2014 ADVANSEE. All Rights Reserved.
 * Benoît Thébaudeau <benoit.thebaudeau@advansee.com>
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"
#include <linux/proc_fs.h>

#define OV7962_VOLTAGE_ANALOG               3300000
#define OV7962_VOLTAGE_DIGITAL_CORE         1500000
#define OV7962_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 30
#define MAX_FPS 60
#define DEFAULT_FPS 30

#define OV7962_XCLK_MIN 6000000
#define OV7962_XCLK_MAX 29500000

/*
 * System Control Registers
 */
#define REG_GAIN	0x3000
#define REG_BGAIN	0x3001
#define REG_RGAIN	0x3002
#define REG_GGAIN	0x3003
#define REG_SCTRL00	0x3004
#define REG_SCTRL01	0x3005
#define   SCTRL01_REGTVCOL	0x80
#define   SCTRL01_NTSCBAND_OFS	6
#define   SCTRL01_NTSCBAND_SZ	1
#define   SCTRL01_NTSCBAND_MSK	0x40
#define   SCTRL01_NTSCBAND_60HZ	0x00
#define   SCTRL01_NTSCBAND_50HZ	0x01
#define   SCTRL01_REGBANDFILT	0x20
#define   SCTRL01_MANLAEC	0x10
#define   SCTRL01_PALWINADJ	0x08
#define   SCTRL01_REGMIRROR	0x04
#define   SCTRL01_ANACTRL_OFS	0
#define   SCTRL01_ANACTRL_SZ	2
#define   SCTRL01_ANACTRL_MSK	0x03
#define REG_BRGAINH	0x3006
#define REG_GGAINH	0x3007
#define REG_BLC		0x3008
#define REG_SCTRL03	0x3009
#define REG_PIDH	0x300a
#define REG_PIDL	0x300b
#define REG_SCTRL04	0x300c
#define   SCTRL04_VFLIP		0x80
#define   SCTRL04_MIRROR	0x40
#define   SCTRL04_MAXEXPTIM_OFS	1
#define   SCTRL04_MAXEXPTIM_SZ	2
#define   SCTRL04_MAXEXPTIM_MSK	0x06
#define REG_VSOPT	0x300d
#define REG_SCTRL05	0x300e
#define   SCTRL05_BLCDBGCTRL	0x80
#define   SCTRL05_OPTBROW_OFS	5
#define   SCTRL05_OPTBROW_SZ	2
#define   SCTRL05_OPTBROW_MSK	0x60
#define   SCTRL05_OPTBROW_R	0x01
#define   SCTRL05_OPTBROW_B	0x02
#define   SCTRL05_OPTBROW_BR	0x03
#define   SCTRL05_ANAPWDN	0x10
#define   SCTRL05_SLEEP		0x08
#define   SCTRL05_DBLEXP	0x04
#define   SCTRL05_PADDRV_OFS	0
#define   SCTRL05_PADDRV_SZ	2
#define   SCTRL05_PADDRV_MSK	0x03
#define   SCTRL05_PADDRV_1X	0x00
#define   SCTRL05_PADDRV_2X	0x01
#define   SCTRL05_PADDRV_3X	0x02
#define   SCTRL05_PADDRV_4X	0x03
#define REG_AECH	0x300f
#define REG_AECL	0x3010
#define REG_CLK		0x3011
#define   CLK_DBGCTRL		0x80
#define   CLK_PXLINCLK		0x40
#define   CLK_SYSCLKDIV_OFS	0
#define   CLK_SYSCLKDIV_SZ	6
#define   CLK_SYSCLKDIV_MSK	0x3f
#define REG_SCTRL06	0x3012
#define   SCTRL06_SRST		0x80
#define   SCTRL06_TV_OFS	6
#define   SCTRL06_TV_SZ		1
#define   SCTRL06_TV_MSK	0x40
#define   SCTRL06_TV_NTSC	0x00
#define   SCTRL06_TV_PAL	0x01
#define   SCTRL06_BW		0x20
#define   SCTRL06_DBGCTRL	0x10
#define   SCTRL06_VGA		0x08
#define   SCTRL06_QVGA		0x04
#define   SCTRL06_WVGA		0x02
#define   SCTRL06_ANAOUT	0x01
#define REG_SCTRL07	0x3013
#define   SCTRL07_FASTAEC	0x80
#define   SCTRL07_AGCAECOPT	0x40
#define   SCTRL07_BANDFILT	0x20
#define   SCTRL07_BANDLAEC	0x10
#define   SCTRL07_LAEC		0x08
#define   SCTRL07_AGC		0x04
#define   SCTRL07_AWB		0x02
#define   SCTRL07_AEC		0x01
#define REG_SCTRL08	0x3014
#define REG_SCTRL09	0x3015
#define   SCTRL09_NGHTMOD	0x80
#define   SCTRL09_NGHTMAXFR_OFS	4
#define   SCTRL09_NGHTMAXFR_SZ	3
#define   SCTRL09_NGHTMAXFR_MSK	0x70
#define   SCTRL09_NGHTMAXFR_0	0x00
#define   SCTRL09_NGHTMAXFR_1	0x01
#define   SCTRL09_NGHTMAXFR_2	0x02
#define   SCTRL09_NGHTMAXFR_3	0x03
#define   SCTRL09_NGHTMAXFR_7	0x04
#define   SCTRL09_NGHTGTH_OFS	2
#define   SCTRL09_NGHTGTH_SZ	2
#define   SCTRL09_NGHTGTH_MSK	0x0c
#define   SCTRL09_NGHTGTH_2X	0x00
#define   SCTRL09_NGHTGTH_4X	0x01
#define   SCTRL09_NGHTGTH_8X	0x02
#define   SCTRL09_NGHTGTH_16X	0x03
#define   SCTRL09_GAINH_OFS	0
#define   SCTRL09_GAINH_SZ	2
#define   SCTRL09_GAINH_MSK	0x03
#define REG_SCTRL10	0x3016
#define   SCTRL10_AVSIZEL_OFS	6
#define   SCTRL10_AVSIZEL_SZ	2
#define   SCTRL10_AVSIZEL_MSK	0xc0
#define   SCTRL10_AHSIZEL_OFS	4
#define   SCTRL10_AHSIZEL_SZ	2
#define   SCTRL10_AHSIZEL_MSK	0x30
#define   SCTRL10_VSTARTL_OFS	2
#define   SCTRL10_VSTARTL_SZ	2
#define   SCTRL10_VSTARTL_MSK	0x0c
#define   SCTRL10_HSTARTL_OFS	0
#define   SCTRL10_HSTARTL_SZ	2
#define   SCTRL10_HSTARTL_MSK	0x03
#define REG_HSTARTH	0x3017
#define REG_AHSIZEH	0x3018
#define REG_VSTARTH	0x3019
#define REG_AVSIZEH	0x301a
#define REG_PSHFT	0x301b
#define REG_MIDH	0x301c
#define REG_MIDL	0x301d
#define REG_SCTRL11	0x301e
#define   SCTRL11_NGHTLTCH	0x80
#define   SCTRL11_DBGMOD0	0x40
#define   SCTRL11_ANADBGMOD	0x20
#define   SCTRL11_LGHTAECAGCTRG	0x10
#define   SCTRL11_AECAGCALG_OFS	3
#define   SCTRL11_AECAGCALG_SZ	1
#define   SCTRL11_AECAGCALG_MSK	0x08
#define   SCTRL11_AECAGCALG_AVG	0x00
#define   SCTRL11_AECAGCALG_HST	0x01
#define   SCTRL11_DBGMOD1	0x04
#define   SCTRL11_NGHTDBLEXP	0x02
#define   SCTRL11_TIMCTRL	0x01
#define REG_LAECL	0x301f
#define REG_SCTRL12	0x3020
#define REG_BDNUM	0x3021
#define REG_SCTRL14	0x3022
#define REG_SCTRL15	0x3023
#define REG_WPT		0x3024
#define REG_BPT		0x3025
#define REG_VPT		0x3026
#define REG_SCTRL16	0x3027
#define   SCTRL16_BLKSUNCNCL	0x80
#define   SCTRL16_DBGMOD	0x40
#define   SCTRL16_BLCDBGMOD_OFS	4
#define   SCTRL16_BLCDBGMOD_SZ	2
#define   SCTRL16_BLCDBGMOD_MSK	0x30
#define   SCTRL16_FSIGCTRL_OFS	0
#define   SCTRL16_FSIGCTRL_SZ	4
#define   SCTRL16_FSIGCTRL_MSK	0x0f
#define REG_SCTRL17	0x3028
#define REG_CSENDL	0x3029
#define REG_CSENDH	0x302a
#define REG_RENDL	0x302b
#define REG_RENDH	0x302c
#define REG_ADDVSL	0x302d
#define REG_ADDVSH	0x302e
#define REG_YAVG	0x302f
#define REG_LAECH	0x3030
#define REG_HSIZEOH	0x3031
#define REG_VSIZEOH	0x3032
#define REG_HVOFFS	0x3033
#define   HVOFFS_HOFFS_OFS	4
#define   HVOFFS_HOFFS_SZ	4
#define   HVOFFS_HOFFS_MSK	0xf0
#define   HVOFFS_VOFFS_OFS	0
#define   HVOFFS_VOFFS_SZ	4
#define   HVOFFS_VOFFS_MSK	0x0f
#define REG_SCTRL18	0x3034
#define   SCTRL18_HSIZEOL_OFS	2
#define   SCTRL18_HSIZEOL_SZ	2
#define   SCTRL18_HSIZEOL_MSK	0x0c
#define   SCTRL18_VSIZEOL_OFS	0
#define   SCTRL18_VSIZEOL_SZ	2
#define   SCTRL18_VSIZEOL_MSK	0x03
#define REG_SCTRL19	0x3035
#define REG_SCTRL20	0x3036
#define REG_SCTRL21	0x3037
#define REG_ACTRL1	0x3038
#define   ACTRL1_MANHRSTART	0x80
#define   ACTRL1_ANACTRL_OFS	4
#define   ACTRL1_ANACTRL_SZ	3
#define   ACTRL1_ANACTRL_MSK	0x70
#define REG_ACTRL2	0x3039
#define REG_ACTRL3	0x303a
#define REG_ACTRL4	0x303b
#define REG_ACTRL5	0x303c
#define REG_ACTRL6	0x303d
#define REG_ACTRL7	0x303e
#define REG_ACTRL8	0x303f
#define REG_ACTRL9	0x3040
#define REG_ACTRL10	0x3041
#define   ACTRL10_ANACTRL_OFS	0
#define   ACTRL10_ANACTRL_SZ	3
#define   ACTRL10_ANACTRL_MSK	0x07
#define REG_ACTRL11	0x3042
#define REG_ACTRL12	0x3043
#define REG_ACTRL13	0x3044
#define REG_ACTRL14	0x3045
#define REG_ACTRL15	0x3046
#define REG_ACTRL16	0x3047
#define REG_ACTRL17	0x3048
#define   ACTRL17_ANACTRL	0x80
#define   ACTRL17_HRSTART_OFS	5
#define   ACTRL17_HRSTART_SZ	2
#define   ACTRL17_HRSTART_MSK	0x60
#define REG_ACTRL18	0x3049
#define REG_SCTRL22	0x304a
#define REG_ACTRL19	0x304b
#define REG_ACTRL20	0x304c
#define   ACTRL20_ANACTRL_OFS	1
#define   ACTRL20_ANACTRL_SZ	7
#define   ACTRL20_ANACTRL_MSK	0xfe
#define REG_ACTRL21	0x304d
#define REG_ACTRL22	0x304e
#define REG_ACTRL23	0x304f
#define REG_ACTRL24	0x3050
#define REG_ACTRL25	0x3051
#define REG_SCTRL24	0x3052
#define REG_SCTRL25	0x3053
#define REG_BD50ST	0x3054
#define REG_BD60ST	0x3055
#define REG_BLCCTRL0	0x305a
#define   BLCCTRL0_ANATIM_OFS	6
#define   BLCCTRL0_ANATIM_SZ	2
#define   BLCCTRL0_ANATIM_MSK	0xc0
#define   BLCCTRL0_YAVG_OFS	2
#define   BLCCTRL0_YAVG_SZ	1
#define   BLCCTRL0_YAVG_MSK	0x04
#define   BLCCTRL0_YAVG_FRAME	0x00
#define   BLCCTRL0_YAVG_FIELD	0x01
#define   BLCCTRL0_BLCDBG_OFS	0
#define   BLCCTRL0_BLCDBG_SZ	2
#define   BLCCTRL0_BLCDBG_MSK	0x03
#define REG_ERNG	0x305b
#define REG_BLCCTRL1	0x305d
#define REG_BLCCTRL2	0x305e
#define REG_MLEN	0x305f
#define   MLEN_LINEADJ		0x80
#define   MLEN_FRAMEADJ		0x40
#define   MLEN_DATPATH_OFS	4
#define   MLEN_DATPATH_SZ	2
#define   MLEN_DATPATH_MSK	0x30
#define   MLEN_GAINCMP_OFS	0
#define   MLEN_GAINCMP_SZ	4
#define   MLEN_GAINCMP_MSK	0x0f
#define REG_SCTRL38	0x3062
#define REG_AVGZN	0x3063
#define REG_SCTRL39	0x3064
#define   SCTRL39_ANACTRL_OFS	5
#define   SCTRL39_ANACTRL_SZ	3
#define   SCTRL39_ANACTRL_MSK	0xe0
#define   SCTRL39_QVGAHSUBSAMP	0x08
#define   SCTRL39_QVGAVSUBSAMP	0x04
#define   SCTRL39_DBGMOD_OFS	0
#define   SCTRL39_DBGMOD_SZ	2
#define   SCTRL39_DBGMOD_MSK	0x03
#define REG_SCTRL40	0x3065
#define REG_AWBBIAS	0x3066
#define REG_BLC0	0x3067
#define   BLC0_BLC		0x80
#define   BLC0_BLCCTRL0		0x40
#define   BLC0_CLRBLKLVLCMP	0x20
#define   BLC0_BLCCTRL1		0x10
#define   BLC0_TGTBLKLVL_OFS	0
#define   BLC0_TGTBLKLVL_SZ	4
#define   BLC0_TGTBLKLVL_MSK	0x0f
#define REG_BLC1	0x3068
#define REG_BLC2	0x3069
#define REG_HFIELD	0x306c
#define REG_VFIELD	0x306d
#define REG_SCTRL41	0x306e
#define REG_SCTRL42	0x306f
#define REG_5060DET0	0x3070
#define REG_5060DET1	0x3071
#define REG_5060DET2	0x3072
#define REG_5060DET3	0x3073
#define REG_5060DET4	0x3074
#define REG_5060DET5	0x3075
#define REG_5060DET6	0x3076
#define REG_5060DET7	0x3077
#define REG_5060DET8	0x3078
#define REG_5060DET9	0x3079
#define REG_5060DET10	0x307a
#define REG_5060DET11	0x307b
#define REG_5060DET12	0x307c
#define REG_5060DET13	0x307d
#define REG_5060DET14	0x307e
#define REG_SCTRL43	0x307f
#define REG_AVGRH	0x3080
#define REG_AVGBH	0x3081
#define REG_AVGGRH	0x3082
#define REG_AVGGBH	0x3083
#define REG_AVGL	0x3084
#define REG_BLCHTMNTH	0x3085
#define REG_BLCHTMNTL	0x3086
#define REG_HVPAD	0x3087
#define REG_HOFFOUT	0x3088
#define REG_VOFFOUT	0x3089
#define REG_SCTRL45	0x308a
#define REG_BPLTH1	0x308b
#define REG_BPLTH2	0x308c
#define REG_WPTH1	0x308d
#define REG_WPTH2	0x308e
#define REG_BPCNT1	0x308f
#define REG_BPCNT2	0x3090
#define REG_WPCNT1	0x3091
#define REG_WPCNT2	0x3092
#define REG_YEN0	0x3093
#define REG_SCTRL46	0x3094
#define   SCTRL46_MANHVFIELD	0x40
#define   SCTRL46_GRPREGWR	0x20
#define   SCTRL46_HREFOUT	0x10
#define   SCTRL46_PCLKOUT	0x08
#define   SCTRL46_VSYNCOUT	0x04
#define   SCTRL46_Y10OUT_OFS	0
#define   SCTRL46_Y10OUT_SZ	2
#define   SCTRL46_Y10OUT_MSK	0x03
#define REG_SCTRL47	0x3095
#define REG_SCTRL48	0x3096
#define   SCTRL48_BYPASSPLL	0x80
#define   SCTRL48_PLLOUTDIV_OFS	5
#define   SCTRL48_PLLOUTDIV_SZ	2
#define   SCTRL48_PLLOUTDIV_MSK	0x60
#define   SCTRL48_ANACTRL_OFS	3
#define   SCTRL48_ANACTRL_SZ	2
#define   SCTRL48_ANACTRL_MSK	0x18
#define   SCTRL48_PLLRST	0x04
#define   SCTRL48_DBGCTRL	0x02
#define   SCTRL48_SYSPLLRST	0x01
#define REG_VSFT	0x3097
#define REG_H2SFT	0x3098
#define REG_BLCGBH	0x3099
#define REG_BLCGRH	0x309a
#define REG_BLCBH	0x309b
#define REG_BLCRH	0x309c
#define REG_BLCL	0x309d
#define REG_BLCULMT	0x309e
#define REG_BLCLLMT	0x309f
#define REG_SCTRL50	0x30ec
#define   SCTRL50_AUTOBANDFILT	0x80
#define   SCTRL50_LGHTFRQ_OFS	6
#define   SCTRL50_LGHTFRQ_SZ	1
#define   SCTRL50_LGHTFRQ_MSK	0x40
#define   SCTRL50_LGHTFRQ_60HZ	0x00
#define   SCTRL50_LGHTFRQ_50HZ	0x01
#define   SCTRL50_ANATIM_OFS	0
#define   SCTRL50_ANATIM_SZ	3
#define   SCTRL50_ANATIM_MSK	0x07
#define REG_SCTRL51	0x30ee
#define REG_OTPCTRL	0x30ef
#define REG_OTPBUF0	0x30f0
#define REG_OTPBUF1	0x30f1
#define REG_OTPBUF2	0x30f2
#define REG_OTPBUF3	0x30f3
#define REG_OTPBUF4	0x30f4
#define REG_OTPBUF5	0x30f5
#define REG_OTPBUF6	0x30f6
#define REG_OTPBUF7	0x30f7
#define REG_OTPBUF8	0x30f8
#define REG_OTPBUF9	0x30f9
#define REG_OTPBUF10	0x30fa
#define REG_OTPBUF11	0x30fb
#define REG_OTPBUF12	0x30fc
#define REG_OTPBUF13	0x30fd
#define REG_OTPBUF14	0x30fe
#define REG_OTPBUF15	0x30ff

/*
 * Format Control Registers
 */
#define REG_FMTCTRL	0x4300
#define   FMTCTRL_FMT_OFS	0
#define   FMTCTRL_FMT_SZ	7
#define   FMTCTRL_FMT_MSK	0x7f
#define   FMTCTRL_FMT_BGGR	0x00
#define   FMTCTRL_FMT_GBRG	0x01
#define   FMTCTRL_FMT_GRBG	0x02
#define   FMTCTRL_FMT_RGGB	0x03
#define   FMTCTRL_FMT_BGR565	0x10
#define   FMTCTRL_FMT_RGB565	0x11
#define   FMTCTRL_FMT_RGB666	0x20
#define   FMTCTRL_FMT_BGR666	0x21
#define   FMTCTRL_FMT_YUYV	0x30
#define   FMTCTRL_FMT_YVYU	0x31
#define   FMTCTRL_FMT_UYVY	0x32
#define   FMTCTRL_FMT_VYUY	0x33
#define   FMTCTRL_FMT_YUV	0x40
#define   FMTCTRL_FMT_YVU	0x41
#define   FMTCTRL_FMT_UYV	0x42
#define   FMTCTRL_FMT_UVY	0x43
#define   FMTCTRL_FMT_VYU	0x44
#define   FMTCTRL_FMT_VUY	0x45
#define REG_VFIFOPCALL	0x4600
#define REG_VFIFOCTRL1	0x4601
#define   VFIFOCTRL1_SSZH_OFS	4
#define   VFIFOCTRL1_SSZH_SZ	1
#define   VFIFOCTRL1_SSZH_MSK	0x10
#define   VFIFOCTRL1_PCALH_OFS	0
#define   VFIFOCTRL1_PCALH_SZ	3
#define   VFIFOCTRL1_PCALH_MSK	0x07
#define REG_VFIFOCTRL2	0x4602
#define   VFIFOCTRL2_VFULLCLR	0x80
#define   VFIFOCTRL2_LCAL_OFS	0
#define   VFIFOCTRL2_LCAL_SZ	7
#define   VFIFOCTRL2_LCAL_MSK	0x7f
#define REG_VFIFOSSZL	0x4603
#define REG_VSUB2CTRL	0x46c0
#define   VSUB2CTRL_EVENPOL	0x08
#define   VSUB2CTRL_ALLODD	0x04
#define   VSUB2CTRL_ALLEVEN	0x02
#define   VSUB2CTRL_FIELDSEL	0x01

/*
 * DVP Control Registers
 */
#define REG_SHPMANH	0x4700
#define REG_SHPMANL	0x4701
#define REG_SHLMAN	0x4702
#define REG_SVLMAN	0x4703
#define REG_SVPMANH	0x4704
#define REG_SVPMANL	0x4705
#define REG_HVPMANH	0x4706
#define REG_HVPMANL	0x4707
#define REG_HVLMAN	0x4708
#define REG_VSLMAN	0x4709
#define REG_LLVSPMAN	0x470a
#define REG_VSPMAN	0x470b
#define REG_LLMANH	0x470c
#define REG_LLMANL	0x470d
#define REG_CCIRCTRL	0x470e
#define REG_VREFOFFSET	0x470f
#define REG_VREFADJ	0x4710
#define REG_PADLCNT	0x4711
#define REG_PADRCNT	0x4712
#define REG_VREFST	0x4713
#define REG_VREFEND	0x4714
#define REG_VREFENDST	0x4715
#define REG_FREFSTL	0x4716
#define REG_FREFSTH	0x4717
#define REG_FIELDVSTL	0x4718
#define REG_FIELDHSTL	0x4719
#define REG_FIELDHSTH	0x471a
#define REG_DVPCTRL00	0x471b
#define REG_DVPCTRL01	0x471c
#define REG_DVPCTRL02	0x471d
#define   DVPCTRL02_VSMAN	0x20
#define   DVPCTRL02_HVMAN	0x10
#define   DVPCTRL02_SVMAN	0x08
#define   DVPCTRL02_VSSEL1CLR	0x04
#define   DVPCTRL02_VSSEL2	0x02
#define   DVPCTRL02_VSSEL1	0x01
#define REG_DVPCTRL03	0x471e
#define REG_PCLKDIVMAN	0x471f
#define REG_DVPCTRL04	0x4720
#define REG_FS		0x4721
#define REG_FE		0x4722
#define REG_LS		0x4723
#define REG_LE		0x4724
#define REG_TOG01	0x4725
#define REG_TOG0	0x4726
#define REG_TOG1	0x4727
#define REG_DVPCTRL05	0x4728
#define REG_DVPCTRL06	0x4729
#define REG_TOGGLEDATA0	0x472a
#define REG_DVPCTRL07	0x472b
#define REG_TOGGLEDATA1	0x472c
#define REG_DVPCTRL08	0x472d
#define REG_CLIPMAX	0x472e
#define REG_CLIPMIN	0x472f

/*
 * ISP Control Registers
 */
#define REG_ISPCTRL00	0x5000
#define   ISPCTRL00_LENCCORR	0x80
#define   ISPCTRL00_YUVGAM	0x40
#define   ISPCTRL00_RAWGAM	0x20
#define   ISPCTRL00_DBGCTRL	0x10
#define   ISPCTRL00_DNS		0x08
#define   ISPCTRL00_BLKPIXCNCL	0x04
#define   ISPCTRL00_WHTPIXCNCL	0x02
#define   ISPCTRL00_CIP		0x01
#define REG_ISPCTRL01	0x5001
#define   ISPCTRL01_SDE		0x80
#define   ISPCTRL01_AUTOSATADJ	0x40
#define   ISPCTRL01_VSCAL	0x20
#define   ISPCTRL01_HSCAL	0x10
#define   ISPCTRL01_AUTOCNTRST	0x08
#define   ISPCTRL01_UVAVG	0x04
#define   ISPCTRL01_CMX		0x02
#define   ISPCTRL01_AWB		0x01
#define REG_ISPCTRL02	0x5002
#define REG_ISPCTRL03	0x5003
#define   ISPCTRL03_BUFPADINT	0x80
#define   ISPCTRL03_YCBCR	0x08
#define   ISPCTRL03_COLBAR	0x01
#define REG_ISPCTRL04	0x5004
#define   ISPCTRL04_BARMOVE	0x04
#define   ISPCTRL04_COLBAR_OFS	0
#define   ISPCTRL04_COLBAR_SZ	2
#define   ISPCTRL04_COLBAR_MSK	0x03
#define REG_ISPCTRL05	0x5005
#define   ISPCTRL05_RAWGAMBBLC	0x80
#define   ISPCTRL05_RAWGAMBLVL	0x40
#define   ISPCTRL05_OLDUVAVG	0x20
#define   ISPCTRL05_AWBBLKLVL	0x10
#define   ISPCTRL05_LCORRBBLC	0x08
#define   ISPCTRL05_LCORRBLVL	0x04

/*
 * LENC Control Registers
 */
#define REG_LENCRX0H	0x5100
#define REG_LENCRX0L	0x5101
#define REG_LENCRY0H	0x5102
#define REG_LENCRY0L	0x5103
#define REG_LENCRA1	0x5104
#define REG_LENCRA2	0x5105
#define REG_LENCRB1	0x5106
#define REG_LENCRB2	0x5107
#define REG_LENCGX0H	0x5108
#define REG_LENCGX0L	0x5109
#define REG_LENCGY0H	0x510a
#define REG_LENCGY0L	0x510b
#define REG_LENCGA1	0x510c
#define REG_LENCGA2	0x510d
#define REG_LENCGB1	0x510e
#define REG_LENCGB2	0x510f
#define REG_LENCBX0H	0x5110
#define REG_LENCBX0L	0x5111
#define REG_LENCBY0H	0x5112
#define REG_LENCBY0L	0x5113
#define REG_LENCBA1	0x5114
#define REG_LENCBA2	0x5115
#define REG_LENCBB1	0x5116
#define REG_LENCBB2	0x5117
#define REG_LENCXSTARTH	0x5118
#define REG_LENCXSTARTL	0x5119
#define REG_LENCYSTARTH	0x511a
#define REG_LENCYSTARTL	0x511b
#define REG_LENCCTRL24	0x511c

/*
 * WBC Control Registers
 */
#define REG_DNSCTRL00	0x5290
#define REG_DNSCTRL01	0x5291
#define REG_DNSCTRL02	0x5292
#define REG_NSY		0x5293
#define REG_NSUH	0x5294
#define REG_NSUL	0x5295
#define REG_NSVH	0x5296
#define REG_NSVL	0x5297
#define REG_DNSEDGETH	0x5298
#define REG_DNSCBCRTH	0x5299
#define REG_NSYLST0	0x529a
#define REG_NSYLST1	0x529b
#define REG_NSYLST2	0x529c
#define REG_NSYLST3	0x529d
#define REG_NSYLST4	0x529e
#define REG_NSYLST5	0x529f
#define REG_NSYLST6	0x52a0
#define REG_NSYLST7	0x52a1
#define REG_NSUVLST0H	0x52a2
#define REG_NSUVLST0L	0x52a3
#define REG_NSUVLST1H	0x52a4
#define REG_NSUVLST1L	0x52a5
#define REG_NSUVLST2H	0x52a6
#define REG_NSUVLST2L	0x52a7
#define REG_NSUVLST3H	0x52a8
#define REG_NSUVLST3L	0x52a9
#define REG_NSUVLST4H	0x52aa
#define REG_NSUVLST4L	0x52ab
#define REG_NSUVLST5H	0x52ac
#define REG_NSUVLST5L	0x52ad
#define REG_NSUVLST6H	0x52ae
#define REG_NSUVLST6L	0x52af
#define REG_NSUVLST7H	0x52b0
#define REG_NSUVLST7L	0x52b1

/*
 * CIP Control Registers
 */
#define REG_CIPMINGAINH	0x5300
#define REG_CIPMINGAINL	0x5301
#define REG_CIPMAXGAINH	0x5302
#define REG_CIPMAXGAINL	0x5303
#define REG_CIPMINDNSH	0x5304
#define REG_CIPMINDNSL	0x5305
#define REG_CIPMAXDNSH	0x5306
#define REG_CIPMAXDNSL	0x5307
#define REG_CIPSHRPMSK0	0x5308
#define REG_CIPSHRPMSK1	0x5309
#define REG_CIPCTRL10	0x530a
#define REG_CIPCTRL11	0x530b
#define   CIPCTRL11_AALIAS_OFS	0
#define   CIPCTRL11_AALIAS_SZ	4
#define   CIPCTRL11_AALIAS_MSK	0x0f
#define REG_CIPCTRL12	0x530c
#define REG_CIPCTRL13	0x530d
#define   CIPCTRL13_MAXSHRP_OFS	0
#define   CIPCTRL13_MAXSHRP_SZ	6
#define   CIPCTRL13_MAXSHRP_MSK	0x3f
#define REG_CIPCTRL14	0x530e
#define REG_CIPCTRL15	0x530f
#define REG_CIPCTRL16	0x5310
#define REG_CIPCTRL17	0x5311
#define REG_CIPCTRL18	0x5312
#define REG_CIPCTRL19	0x5313
#define REG_CIPCTRL20	0x5314
#define REG_CIPCTRL21	0x5315
#define REG_CIPCTRL22	0x5316
#define REG_CIPCTRL23	0x5317
#define REG_CIPCTRL24	0x5318
#define REG_CIPCTRL25	0x5319
#define REG_CIPCTRL26	0x531a
#define REG_CIPCTRL27	0x531b
#define REG_CIPCTRL28	0x531c
#define REG_CIPCTRL29	0x531d
#define REG_CIPCTRL30	0x531e
#define REG_CIPCTRL31	0x531f
#define REG_CIPCTRL32	0x5320
#define REG_CIPCTRL33	0x5321

/*
 * Contrast Control Registers
 */
#define REG_CTMAXHLVLH	0x5400
#define REG_CTMAXHLVLL	0x5401
#define REG_CTMINHLVLH	0x5402
#define REG_CTMINHLVLL	0x5403
#define REG_CTMAXLLVLH	0x5404
#define REG_CTMAXLLVLL	0x5405
#define REG_CTMINLLVLH	0x5406
#define REG_CTMINLLVLL	0x5407
#define REG_CTCURLLVLH	0x5408
#define REG_CTCURLLVLL	0x5409
#define REG_CTCURHLVLH	0x540a
#define REG_CTCURHLVLL	0x540b
#define REG_CTTH1H	0x540e
#define REG_CTTH1M	0x540f
#define REG_CTTH1L	0x5410
#define REG_CTTH2H	0x5412
#define REG_CTTH2M	0x5413
#define REG_CTTH2L	0x5414
#define REG_DEBUGMODE0	0x5415
#define REG_DEBUGMODE1	0x5416
#define REG_DEBUGMODE2	0x5417
#define REG_DEBUGMODE3	0x5418
#define REG_DEBUGMODE4	0x5419
#define REG_DEBUGMODE5	0x541a
#define REG_DEBUGMODE6	0x541b
#define REG_DEBUGMODE7	0x541c
#define REG_CTSTEP	0x541d
#define REG_DEBUGMODE8	0x541e
#define REG_DEBUGMODE9	0x541f
#define REG_DEBUGMODE10	0x5420
#define REG_DEBUGMODE11	0x5421
#define REG_DEBUGMODE12	0x5422
#define REG_DEBUGMODE13	0x5423
#define REG_DEBUGMODE14	0x5424
#define REG_DEBUGMODE15	0x5425
#define REG_DEBUGMODE16	0x5426

/*
 * Gamma Control Registers
 */
#define REG_GAMCTRL00	0x5480
#define REG_GAMCTRL01	0x5481
#define REG_GAMCTRL02	0x5482
#define REG_GAMCTRL03	0x5483
#define REG_GAMCTRL04	0x5484
#define REG_GAMCTRL05	0x5485
#define REG_GAMCTRL06	0x5486
#define REG_GAMCTRL07	0x5487
#define REG_GAMCTRL08	0x5488
#define REG_GAMCTRL09	0x5489
#define REG_GAMCTRL0A	0x548a
#define REG_GAMCTRL0B	0x548b
#define REG_GAMCTRL0C	0x548c
#define REG_GAMCTRL0D	0x548d
#define REG_GAMCTRL0E	0x548e
#define REG_GAMCTRL0F	0x548f
#define REG_GAMGAINL00H	0x5490
#define REG_GAMGAINL00L	0x5491
#define REG_GAMGAINL01H	0x5492
#define REG_GAMGAINL01L	0x5493
#define REG_GAMGAINL02H	0x5494
#define REG_GAMGAINL02L	0x5495
#define REG_GAMGAINL03H	0x5496
#define REG_GAMGAINL03L	0x5497
#define REG_GAMGAINL04H	0x5498
#define REG_GAMGAINL04L	0x5499
#define REG_GAMGAINL05H	0x549a
#define REG_GAMGAINL05L	0x549b
#define REG_GAMGAINL06H	0x549c
#define REG_GAMGAINL06L	0x549d
#define REG_GAMGAINL07H	0x549e
#define REG_GAMGAINL07L	0x549f
#define REG_GAMGAINL08H	0x54a0
#define REG_GAMGAINL08L	0x54a1
#define REG_GAMGAINL09H	0x54a2
#define REG_GAMGAINL09L	0x54a3
#define REG_GAMGAINL10H	0x54a4
#define REG_GAMGAINL10L	0x54a5
#define REG_GAMGAINL11H	0x54a6
#define REG_GAMGAINL11L	0x54a7
#define REG_GAMGAINL12H	0x54a8
#define REG_GAMGAINL12L	0x54a9
#define REG_GAMGAINL13H	0x54aa
#define REG_GAMGAINL13L	0x54ab
#define REG_GAMGAINL14H	0x54ac
#define REG_GAMGAINL14L	0x54ad
#define REG_GAMGAINL15H	0x54ae
#define REG_GAMGAINL15L	0x54af
#define REG_GAMCTRL30	0x54b0
#define REG_GAMCTRL31	0x54b1
#define REG_GAMCTRL32	0x54b2
#define   GAMCTRL32_DRKNSRD	0x01
#define REG_GAMCTRL33	0x54b3
#define REG_GAMCTRL34	0x54b4
#define REG_GAMCTRL35	0x54b5
#define REG_GAMCTRL36	0x54b6
#define   GAMCTRL36_BRTNSRD	0x01
#define REG_GAMCTRL37	0x54b7

/*
 * Autocolor Saturation Registers
 */
#define REG_UVADJCTRL0	0x5500
#define REG_UVADJCTRL3	0x5501
#define REG_UVADJTH1H	0x5502
#define REG_UVADJTH1L	0x5503
#define REG_UVADJTH2H	0x5504
#define REG_UVADJTH2L	0x5505

/*
 * SDE Control Registers
 */
#define REG_SDECTRL0	0x5580
#define   SDECTRL0_FIXY		0x80
#define   SDECTRL0_NEGY		0x40
#define   SDECTRL0_GRAY		0x20
#define   SDECTRL0_FIXV		0x10
#define   SDECTRL0_FIXU		0x08
#define   SDECTRL0_CNTRST	0x04
#define   SDECTRL0_SAT		0x02
#define   SDECTRL0_HUE		0x01
#define REG_HUECOS	0x5581
#define REG_HUEANGL	0x5581
#define REG_HUESIN	0x5582
#define REG_SDECTRL2	0x5582
#define   SDECTRL2_HUEANGH_OFS	0
#define   SDECTRL2_HUEANGH_SZ	1
#define   SDECTRL2_HUEANGH_MSK	0x01
#define REG_SATU	0x5583
#define REG_SATV	0x5584
#define REG_FIXU	0x5585
#define REG_FIXV	0x5586
#define REG_YOFFS	0x5587
#define REG_FIXY	0x5587
#define REG_YGAIN	0x5588
#define REG_YBRGHT	0x5589
#define REG_SDECTRL10	0x558a
#define   SDECTRL10_MANYOFFS	0x80
#define   SDECTRL10_HUEANG	0x40
#define   SDECTRL10_CRCOSSGN	0x20
#define   SDECTRL10_CBCOSSGN	0x10
#define   SDECTRL10_YOFFSSGN	0x08
#define   SDECTRL10_YBRGHTSGN	0x04
#define   SDECTRL10_CBSINSGN	0x02
#define   SDECTRL10_CRSINSGN	0x01

/*
 * AVG Control Registers
 */
#define REG_AVGXSTARTH	0x5680
#define REG_AVGXSTARTL	0x5681
#define REG_AVGXENDH	0x5682
#define REG_AVGXENDL	0x5683
#define REG_AVGYSTARTH	0x5684
#define REG_AVGYSTARTL	0x5685
#define REG_AVGYENDH	0x5686
#define REG_AVGYENDL	0x5687
#define REG_AVGR8	0x5688
#define REG_AVGR9	0x5689
#define REG_AVGRA	0x568a
#define REG_AVGRB	0x568b
#define REG_AVGRC	0x568c
#define REG_AVGRD	0x568d
#define REG_AVGRE	0x568e
#define REG_AVGRF	0x568f

/*
 * ID
 */

#define OMNIVISION	0x7fa2
#define MID(midh, midl)		(((midh) << 8) | (midl))

#define OV7962		0x7960
#define PID_MODEL_MSK	0xfff0
#define PID_REV_MSK	0x000f
#define PID(pidh, pidl)		(((pidh) << 8) | (pidl))

enum ov7962_mode {
	ov7962_mode_MIN = 0,
	ov7962_mode_VGA_640_480 = 0,
	ov7962_mode_MAX = 0
};

enum ov7962_frame_rate {
	ov7962_30_fps,
	ov7962_60_fps
};

static const int ov7962_framerates[] = {
	[ov7962_30_fps] = 30,
	[ov7962_60_fps] = 60,
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov7962_mode_info {
	enum ov7962_mode mode;
	u32 width;
	u32 height;
	const struct reg_value *init_data_ptr;
	u32 init_data_size;
};

/*!
 * Maintains the information on the current state of the sensor.
 */
struct ov7962_priv {
	struct fsl_mxc_camera_platform_data *platform_data;
	struct sensor_data sensor_data;
	struct regulator *io_regulator;
	struct regulator *core_regulator;
	struct regulator *analog_regulator;
	struct regulator *gpo_regulator;
	char pde_name[32];
};

static const struct reg_value ov7962_setting_30fps_VGA_640_480[] = {
	{0x3012, 0x80, 0, 10}, {0x3012, 0x08, 0, 0}, {0x3011, 0x01, 0, 0},
	{0x3005, 0xa4, 0, 0}, {0x300c, 0x02, 0, 0}, {0x3022, 0x0f, 0, 0},
	{0x3027, 0x83, 0, 0}, {0x3038, 0x80, 0, 0}, {0x3039, 0x10, 0, 0},
	{0x3041, 0x04, 0, 0}, {0x3044, 0x74, 0, 0}, {0x3048, 0xc0, 0, 0},
	{0x304b, 0x68, 0, 0}, {0x304c, 0x1a, 0, 0}, {0x305a, 0x30, 0, 0},
	{0x305f, 0xc4, 0, 0}, {0x3067, 0xd4, 0, 0}, {0x3069, 0x60, 0, 0},
	{0x3029, 0xfc, 0, 0}, {0x302a, 0x02, 0, 0}, {0x302b, 0x0c, 0, 0},
	{0x302c, 0x02, 0, 0}, {0x3033, 0x48, 0, 0}, {0x3064, 0x20, 0, 0},
	{0x3097, 0x18, 0, 0}, {0x3093, 0xff, 0, 0}, {0x3094, 0x1f, 0, 0},
	{0x3096, 0x00, 0, 0}, {0x3604, 0x0b, 0, 0}, {0x4300, 0x32, 0, 0},
	{0x4600, 0x0b, 0, 0}, {0x4601, 0x01, 0, 0}, {0x4602, 0x03, 0, 0},
	{0x46c0, 0x02, 0, 0}, {0x4708, 0x2f, 0, 0}, {0x471d, 0x80, 0, 0},
	{0x529a, 0x00, 0, 0}, {0x529b, 0x01, 0, 0}, {0x529c, 0x02, 0, 0},
	{0x529d, 0x04, 0, 0}, {0x529e, 0x05, 0, 0}, {0x529f, 0x06, 0, 0},
	{0x52a0, 0x06, 0, 0}, {0x52a1, 0x08, 0, 0}, {0x52a3, 0x00, 0, 0},
	{0x52a5, 0x01, 0, 0}, {0x52a7, 0x03, 0, 0}, {0x52a9, 0x06, 0, 0},
	{0x52ab, 0x08, 0, 0}, {0x52ad, 0x0a, 0, 0}, {0x52af, 0x0c, 0, 0},
	{0x52b1, 0x10, 0, 0}, {0x530b, 0x08, 0, 0}, {0x530d, 0x08, 0, 0},
	{0x5315, 0x10, 0, 0}, {0x5317, 0x0f, 0, 0}, {0x5318, 0x03, 0, 0},
	{0x5319, 0x10, 0, 0}, {0x54b2, 0x00, 0, 0}, {0x54b6, 0x00, 0, 0},
	{0x5380, 0x01, 0, 0}, {0x5381, 0x00, 0, 0}, {0x5382, 0x00, 0, 0},
	{0x5383, 0x44, 0, 0}, {0x5384, 0x00, 0, 0}, {0x5385, 0x0b, 0, 0},
	{0x5386, 0x00, 0, 0}, {0x5387, 0x00, 0, 0}, {0x5388, 0x01, 0, 0},
	{0x5389, 0x06, 0, 0}, {0x538a, 0x00, 0, 0}, {0x538b, 0x25, 0, 0},
	{0x538c, 0x00, 0, 0}, {0x538d, 0x00, 0, 0}, {0x538e, 0x00, 0, 0},
	{0x538f, 0x02, 0, 0}, {0x5390, 0x00, 0, 0}, {0x5391, 0xd2, 0, 0},
	{0x5380, 0x01, 0, 0}, {0x5381, 0x00, 0, 0}, {0x5382, 0x00, 0, 0},
	{0x5383, 0x44, 0, 0}, {0x5384, 0x00, 0, 0}, {0x5385, 0x0b, 0, 0},
	{0x5386, 0x00, 0, 0}, {0x5387, 0x00, 0, 0}, {0x5388, 0x01, 0, 0},
	{0x5389, 0x3a, 0, 0}, {0x538a, 0x00, 0, 0}, {0x538b, 0x2c, 0, 0},
	{0x538c, 0x00, 0, 0}, {0x538d, 0x00, 0, 0}, {0x538e, 0x00, 0, 0},
	{0x538f, 0x02, 0, 0}, {0x5390, 0x00, 0, 0}, {0x5391, 0xfc, 0, 0},
	{0x5380, 0x01, 0, 0}, {0x5381, 0x00, 0, 0}, {0x5382, 0x00, 0, 0},
	{0x5383, 0x44, 0, 0}, {0x5384, 0x00, 0, 0}, {0x5385, 0x0b, 0, 0},
	{0x5386, 0x00, 0, 0}, {0x5387, 0x00, 0, 0}, {0x5388, 0x01, 0, 0},
	{0x5389, 0x55, 0, 0}, {0x538a, 0x00, 0, 0}, {0x538b, 0x30, 0, 0},
	{0x538c, 0x00, 0, 0}, {0x538d, 0x00, 0, 0}, {0x538e, 0x00, 0, 0},
	{0x538f, 0x03, 0, 0}, {0x5390, 0x01, 0, 0}, {0x5391, 0x11, 0, 0},
	{0x5003, 0x08, 0, 0}, {0x5004, 0x00, 0, 0}, {0x300e, 0xec, 0, 0},
	{0x3012, 0x08, 0x0e, 0}, {0x3017, 0x25, 0, 0}, {0x3018, 0xa0, 0, 0},
	{0x3019, 0x02, 0, 0}, {0x301a, 0x7a, 0, 0}, {0x3016, 0x06, 0, 0},
	{0x3031, 0xa0, 0, 0}, {0x3032, 0x78, 0, 0}, {0x3034, 0x00, 0x0f, 0},
	{0x4300, 0x32, 0x7f, 0}, {0x5001, 0x80, 0x80, 0}, {0x5580, 0x04, 0x04, 0},
	{0x558a, 0x00, 0x84, 0}, {0x5589, 0x00, 0, 0}, {0x5001, 0x80, 0x80, 0},
	{0x5580, 0x04, 0x04, 0}, {0x558a, 0x00, 0x80, 0}, {0x5588, 0x20, 0, 0},
	{0x5001, 0x80, 0x80, 0}, {0x5580, 0x02, 0x02, 0}, {0x5583, 0x40, 0, 0},
	{0x5584, 0x40, 0, 0}, {0x5001, 0x80, 0x80, 0}, {0x5580, 0x01, 0x01, 0},
	{0x5581, 0x80, 0, 0}, {0x5582, 0x00, 0, 0}, {0x558a, 0x01, 0x73, 0},
	{0x5001, 0x01, 0x01, 0}, {0x3013, 0x04, 0x04, 0}, {0x300c, 0x00, 0x40, 0},
	{0x300c, 0x00, 0x80, 0}, {0x3013, 0x30, 0x30, 0}, {0x3005, 0x20, 0x20, 0},
	{0x301e, 0x10, 0x10, 0}, {0x30ec, 0x80, 0xc0, 0}, {0x5001, 0x80, 0x80, 0},
	{0x5580, 0x00, 0x60, 0}, {0x3013, 0x09, 0x09, 0}, {0x300e, 0x00, 0x08, 10},
};

static const struct ov7962_mode_info
ov7962_mode_info_data[2][ov7962_mode_MAX + 1] = {
	{
		{ov7962_mode_VGA_640_480,    640,  480,
		ov7962_setting_30fps_VGA_640_480,
		ARRAY_SIZE(ov7962_setting_30fps_VGA_640_480)},
	}, {
		{ov7962_mode_VGA_640_480, 0, 0, NULL, 0},
	},
};

static int ov7962_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov7962_remove(struct i2c_client *client);

static s32 ov7962_read_reg(struct i2c_client *client, u16 reg, u8 *val);
static s32 ov7962_write_reg(struct i2c_client *client, u16 reg, u8 val);

static const struct i2c_device_id ov7962_id[] = {
	{"ov7962", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov7962_id);

static struct i2c_driver ov7962_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov7962",
		  },
	.probe  = ov7962_probe,
	.remove = ov7962_remove,
	.id_table = ov7962_id,
};

static s32 ov7962_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(client, au8Buf, 3) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}

	return 0;
}

static s32 ov7962_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}

static s32 power_control(struct sensor_data *sensor, int on)
{
	struct ov7962_priv *priv = container_of(sensor, struct ov7962_priv,
						sensor_data);
	struct fsl_mxc_camera_platform_data *plat_data = priv->platform_data;

	if (sensor->on != on) {
		if (plat_data->pwdn)
			plat_data->pwdn(on ? 0 : 1);
		sensor->on = on;
	}
	return 0;
}

static int ov7962_init_mode(struct sensor_data *sensor,
			    enum ov7962_frame_rate frame_rate,
			    enum ov7962_mode mode)
{
	struct i2c_client *client = sensor->i2c_client;
	const struct reg_value *pModeSetting = NULL;
	s32 i = 0;
	s32 iModeSettingArySize = 0;
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int retval = 0;

	if (mode > ov7962_mode_MAX || mode < ov7962_mode_MIN) {
		pr_err("Wrong ov7962 mode detected!\n");
		return -1;
	}

	pModeSetting = ov7962_mode_info_data[frame_rate][mode].init_data_ptr;
	iModeSettingArySize =
		ov7962_mode_info_data[frame_rate][mode].init_data_size;

	sensor->pix.width = ov7962_mode_info_data[frame_rate][mode].width;
	sensor->pix.height = ov7962_mode_info_data[frame_rate][mode].height;

	if (sensor->pix.width == 0 || sensor->pix.height == 0 ||
	    pModeSetting == NULL || iModeSettingArySize == 0)
		return -EINVAL;

	for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov7962_read_reg(client, RegAddr, &RegVal);
			if (retval < 0) {
				pr_err("read reg error addr=0x%x", RegAddr);
				goto err;
			}

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov7962_write_reg(client, RegAddr, Val);
		if (retval < 0) {
			pr_err("write reg error addr=0x%x", RegAddr);
			goto err;
		}

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
	return retval;
}


/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct sensor_data *sensor;

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	sensor = s->priv;

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = sensor->mclk;
	pr_debug("   clock_curr=mclk=%d\n", sensor->mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = OV7962_XCLK_MIN;
	p->u.bt656.clock_max = OV7962_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;
	struct ov7962_priv *priv = container_of(sensor, struct ov7962_priv,
						sensor_data);

	if (on && !sensor->on) {
		if (priv->io_regulator)
			if (regulator_enable(priv->io_regulator) != 0)
				return -EIO;
		if (priv->core_regulator)
			if (regulator_enable(priv->core_regulator) != 0)
				return -EIO;
		if (priv->gpo_regulator)
			if (regulator_enable(priv->gpo_regulator) != 0)
				return -EIO;
		if (priv->analog_regulator)
			if (regulator_enable(priv->analog_regulator) != 0)
				return -EIO;
		/* Make sure power on */
		power_control(sensor, 1);

	} else if (!on && sensor->on) {
		if (priv->analog_regulator)
			regulator_disable(priv->analog_regulator);
		if (priv->core_regulator)
			regulator_disable(priv->core_regulator);
		if (priv->io_regulator)
			regulator_disable(priv->io_regulator);
		if (priv->gpo_regulator)
			regulator_disable(priv->gpo_regulator);

		power_control(sensor, 0);
	}
	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ov7962_frame_rate frame_rate;
	int ret = 0;

	/* Make sure power on */
	power_control(sensor, 1);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0))
		{
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 30)
			frame_rate = ov7962_30_fps;
		else if (tgt_fps == 60)
			frame_rate = ov7962_60_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		ret = ov7962_init_mode(sensor,
				       frame_rate, a->parm.capture.capturemode);
		if (ret < 0)
			return ret;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	struct sensor_data *sensor = s->priv;
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = sensor->brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = sensor->hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = sensor->contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = sensor->saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = sensor->red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = sensor->blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = sensor->ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;

	pr_debug("In ov7962:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_MXC_ROT:
	case V4L2_CID_MXC_VF_ROT:
		switch (vc->value) {
		case V4L2_MXC_ROTATE_NONE:
			break;
		case V4L2_MXC_ROTATE_VERT_FLIP:
			break;
		case V4L2_MXC_ROTATE_HORIZ_FLIP:
			break;
		case V4L2_MXC_ROTATE_180:
			break;
		default:
			retval = -EPERM;
			break;
		}
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	struct sensor_data *sensor = s->priv;

	if (fsize->index > ov7962_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = sensor->pix.pixelformat;
	fsize->discrete.width =
			max(ov7962_mode_info_data[0][fsize->index].width,
			    ov7962_mode_info_data[1][fsize->index].width);
	fsize->discrete.height =
			max(ov7962_mode_info_data[0][fsize->index].height,
			    ov7962_mode_info_data[1][fsize->index].height);
	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	struct sensor_data *sensor = s->priv;
	int i, j, count;

	if (fival->index < 0 || fival->index > ov7962_mode_MAX)
		return -EINVAL;

	if (fival->pixel_format == 0 || fival->width == 0 || fival->height == 0) {
		pr_warning("Please assign pixelformat, width and height.\n");
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(ov7962_mode_info_data); i++) {
		for (j = 0; j < (ov7962_mode_MAX + 1); j++) {
			if (fival->pixel_format == sensor->pix.pixelformat
			 && fival->width == ov7962_mode_info_data[i][j].width
			 && fival->height == ov7962_mode_info_data[i][j].height
			 && ov7962_mode_info_data[i][j].init_data_ptr != NULL) {
				count++;
			}
			if (fival->index == (count - 1)) {
				fival->discrete.denominator =
						ov7962_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "ov7962_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	struct sensor_data *sensor = s->priv;

	if (fmt->index > 0)	/* only 1 pixelformat support so far */
		return -EINVAL;

	fmt->pixelformat = sensor->pix.pixelformat;

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor_data *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum ov7962_frame_rate frame_rate;

	sensor->on = true;

	/* mclk */
	tgt_xclk = sensor->mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV7962_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV7962_XCLK_MIN);
	sensor->mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	set_mclk_rate(&sensor->mclk, sensor->csi);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps == 30)
		frame_rate = ov7962_30_fps;
	else if (tgt_fps == 60)
		frame_rate = ov7962_60_fps;
	else
		return -EINVAL; /* Only support 30fps or 60fps now. */

	return ov7962_init_mode(sensor,
				frame_rate, sensor->streamcap.capturemode);
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ov7962_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm},
	{vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
				(v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static int write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	struct ov7962_priv *priv = data;
	struct sensor_data *sensor = &priv->sensor_data;
	struct i2c_client *client = sensor->i2c_client;
	char localbuf[256];

	if (count < sizeof(localbuf)) {
		if (copy_from_user(localbuf, buffer, count)) {
			printk(KERN_ERR "Error reading user buf\n");
		} else {
			int addr;
			int value;
			int numScanned;

			numScanned = sscanf(localbuf, "%04x %02x", &addr, &value);
			if (numScanned == 2) {
				if (addr <= 0xffff && value <= 0xff) {
					s32 rval;

					rval = ov7962_write_reg(client, addr, value);
                                        if (rval < 0)
						pr_err("%s, write reg 0x%x failed: %d\n", __func__, addr, rval);
					else
                                                pr_err("ov7962[%04x] = %02x\n", addr, value);
				} else {
					printk(KERN_ERR "Invalid data: %s\n", localbuf);
				}
			} else if (numScanned == 1) {
				if (addr <= 0xffff) {
					s32 rval;
					u8 value;

					rval = ov7962_read_reg(client, addr, &value);
					if (rval)
						pr_err("%s, read reg 0x%x failed: %d\n", __func__, addr, rval);
					else
						pr_err("ov7962[%04x] == 0x%02x\n", addr, value);
				}
			} else {
				printk(KERN_ERR "Invalid data: %s\n", localbuf);
			}
		}
	}

	return count;
}

/*!
 * ov7962 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov7962_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int retval;
	struct ov7962_priv *priv;
	struct sensor_data *sensor;
	struct v4l2_int_device *v4l2_int_device;
	struct v4l2_int_slave *v4l2_int_slave;
	struct fsl_mxc_camera_platform_data *plat_data =
			client->dev.platform_data;
	u8 manufacturer_id_high, manufacturer_id_low, chip_id_high, chip_id_low;
	struct proc_dir_entry *pde;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Set initial values for the sensor struct. */
	sensor = &priv->sensor_data;
	sensor->mclk = 24000000; /* 6 - 29.5 MHz, typical 12.27MHz */
	sensor->mclk = plat_data->mclk;
	sensor->mclk_source = plat_data->mclk_source;
	sensor->csi = plat_data->csi;
	sensor->ipu = plat_data->ipu;
	sensor->io_init = plat_data->io_init;

	sensor->i2c_client = client;
	i2c_set_clientdata(client, priv);
	sensor->pix.pixelformat = V4L2_PIX_FMT_UYVY;
	sensor->pix.width = 640;
	sensor->pix.height = 480;
	sensor->streamcap.capability = V4L2_MODE_HIGHQUALITY |
					V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.capturemode = 0;
	sensor->streamcap.timeperframe.denominator = DEFAULT_FPS;
	sensor->streamcap.timeperframe.numerator = 1;
	priv->platform_data = plat_data;

	if (plat_data->io_regulator) {
		priv->io_regulator = regulator_get(&client->dev,
						   plat_data->io_regulator);
		if (!IS_ERR(priv->io_regulator)) {
			regulator_set_voltage(priv->io_regulator,
					      OV7962_VOLTAGE_DIGITAL_IO,
					      OV7962_VOLTAGE_DIGITAL_IO);
			retval = regulator_enable(priv->io_regulator);
			if (retval) {
				pr_err("%s:io set voltage error\n", __func__);
				goto err0;
			} else {
				dev_dbg(&client->dev,
					"%s:io set voltage ok\n", __func__);
			}
		} else
			priv->io_regulator = NULL;
	}

	if (plat_data->core_regulator) {
		priv->core_regulator = regulator_get(&client->dev,
						     plat_data->core_regulator);
		if (!IS_ERR(priv->core_regulator)) {
			regulator_set_voltage(priv->core_regulator,
					      OV7962_VOLTAGE_DIGITAL_CORE,
					      OV7962_VOLTAGE_DIGITAL_CORE);
			retval = regulator_enable(priv->core_regulator);
			if (retval) {
				pr_err("%s:core set voltage error\n", __func__);
				goto err1;
			} else {
				dev_dbg(&client->dev,
					"%s:core set voltage ok\n", __func__);
			}
		} else
			priv->core_regulator = NULL;
	}

	if (plat_data->analog_regulator) {
		priv->analog_regulator = regulator_get(&client->dev,
						plat_data->analog_regulator);
		if (!IS_ERR(priv->analog_regulator)) {
			regulator_set_voltage(priv->analog_regulator,
					      OV7962_VOLTAGE_ANALOG,
					      OV7962_VOLTAGE_ANALOG);
			retval = regulator_enable(priv->analog_regulator);
			if (retval) {
				pr_err("%s:analog set voltage error\n",
					__func__);
				goto err2;
			} else {
				dev_dbg(&client->dev,
					"%s:analog set voltage ok\n", __func__);
			}
		} else
			priv->analog_regulator = NULL;
	}

	if (plat_data->io_init)
		plat_data->io_init();

	power_control(sensor, 1);

	if (ov7962_read_reg(client, REG_MIDH, &manufacturer_id_high) < 0 ||
			ov7962_read_reg(client, REG_MIDL,
					&manufacturer_id_low) < 0 ||
			MID(manufacturer_id_high, manufacturer_id_low) !=
					OMNIVISION ||
			ov7962_read_reg(client, REG_PIDH, &chip_id_high) < 0 ||
			ov7962_read_reg(client, REG_PIDL, &chip_id_low) < 0 ||
			(PID(chip_id_high, chip_id_low) & PID_MODEL_MSK) !=
					OV7962) {
		pr_warning("camera ov7962 is not found\n");
		retval = -ENODEV;
	} else {
		retval = 0;
	}

	power_control(sensor, 0);

	if (retval)
		goto err3;

	v4l2_int_slave = kzalloc(sizeof(*v4l2_int_slave), GFP_KERNEL);
	if (!v4l2_int_slave) {
		retval = -ENOMEM;
		goto err3;
	}

	v4l2_int_slave->ioctls = ov7962_ioctl_desc;
	v4l2_int_slave->num_ioctls = ARRAY_SIZE(ov7962_ioctl_desc);
	sprintf(v4l2_int_slave->attach_to, "mxc_v4l2_cap%d", sensor->csi);

	v4l2_int_device = kzalloc(sizeof(*v4l2_int_device), GFP_KERNEL);
	if (!v4l2_int_device) {
		retval = -ENOMEM;
		goto err4;
	}

	v4l2_int_device->module = THIS_MODULE;
	sprintf(v4l2_int_device->name, "ov7962-csi%d", sensor->csi);
	v4l2_int_device->type = v4l2_int_type_slave;
	v4l2_int_device->u.slave = v4l2_int_slave;

	sensor->v4l2_int_device = v4l2_int_device;
	v4l2_int_device->priv = sensor;

	sprintf(priv->pde_name, "driver/%s", v4l2_int_device->name);
	pde = create_proc_entry(priv->pde_name, 0, 0);
	if (pde) {
		pde->write_proc = write_proc;
		pde->data = priv;
	} else {
		printk(KERN_ERR "Error creating ov7962 proc entry\n");
	}

	retval = v4l2_int_device_register(v4l2_int_device);

	pr_info("camera ov7962 is found\n");
	return retval;

err4:
	kfree(v4l2_int_slave);
err3:
	if (priv->analog_regulator) {
		regulator_disable(priv->analog_regulator);
		regulator_put(priv->analog_regulator);
	}
err2:
	if (priv->core_regulator) {
		regulator_disable(priv->core_regulator);
		regulator_put(priv->core_regulator);
	}
err1:
	if (priv->io_regulator) {
		regulator_disable(priv->io_regulator);
		regulator_put(priv->io_regulator);
	}
err0:
	kfree(priv);
	return retval;
}

/*!
 * ov7962 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov7962_remove(struct i2c_client *client)
{
	struct ov7962_priv *priv = i2c_get_clientdata(client);
	struct sensor_data *sensor = &priv->sensor_data;
	struct v4l2_int_device *v4l2_int_device = sensor->v4l2_int_device;
	struct v4l2_int_slave *v4l2_int_slave = v4l2_int_device->u.slave;

	remove_proc_entry(priv->pde_name, NULL);

	v4l2_int_device_unregister(v4l2_int_device);

	if (priv->gpo_regulator) {
		regulator_disable(priv->gpo_regulator);
		regulator_put(priv->gpo_regulator);
	}

	if (priv->analog_regulator) {
		regulator_disable(priv->analog_regulator);
		regulator_put(priv->analog_regulator);
	}

	if (priv->core_regulator) {
		regulator_disable(priv->core_regulator);
		regulator_put(priv->core_regulator);
	}

	if (priv->io_regulator) {
		regulator_disable(priv->io_regulator);
		regulator_put(priv->io_regulator);
	}

	kfree(v4l2_int_device);
	kfree(v4l2_int_slave);
	kfree(priv);
	return 0;
}

/*!
 * ov7962 init function
 * Called by insmod ov7962_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov7962_init(void)
{
	u8 err;

	err = i2c_add_driver(&ov7962_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

	return err;
}

/*!
 * OV7962 cleanup function
 * Called on rmmod ov7962_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov7962_clean(void)
{
	i2c_del_driver(&ov7962_i2c_driver);
}

module_init(ov7962_init);
module_exit(ov7962_clean);

MODULE_AUTHOR("Benoît Thébaudeau <benoit.thebaudeau@advansee.com>");
MODULE_DESCRIPTION("OV7962 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
