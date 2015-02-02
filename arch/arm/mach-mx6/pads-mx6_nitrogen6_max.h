#undef MX6PAD
#undef MX6NAME
#undef MX6

//#define ONE_WIRE

#ifdef FOR_DL_SOLO
#define MX6(a) MX6DL_##a
#define MX6PAD(a) MX6DL_PAD_##a
#define MX6NAME(a) mx6dl_solo_##a
#else
#define MX6(a) MX6Q_##a
#define MX6PAD(a) MX6Q_PAD_##a
#define MX6NAME(a) mx6q_##a
#endif

#define PADCFG_INPUT	(PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
#define PADCFG_INPUT_DN	(PADCFG_INPUT | PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_DOWN)
#define PADCFG_INPUT_UP	(PADCFG_INPUT | PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)

#define UART_PAD_CTRL	(PADCFG_INPUT_UP | PAD_CTL_SRE_FAST)
#define WEAK_PULLDN	PADCFG_INPUT_DN

#define PADCFG_FLOAT_IRQ (PADCFG_INPUT | PAD_CTL_PUE | PAD_CTL_PUS_100K_DOWN)
#define WEAK_PULLUP	(PADCFG_INPUT | PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define OUTPUT_40OHM	(PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define PAD_CTL_PIS(pull, impede, speed)	(PAD_CTL_PKE | PAD_CTL_PUE |	\
		PAD_CTL_PUS_##pull  | PAD_CTL_SPEED_##speed |		\
		PAD_CTL_DSE_##impede   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define MX6Q_USDHC_PAD_CTRL_22KPU_34OHM_50MHZ	PAD_CTL_PIS(22K_UP, 34ohm, LOW)
#define MX6Q_USDHC_PAD_CTRL_22KPU_40OHM_50MHZ	PAD_CTL_PIS(22K_UP, 40ohm, LOW)
#define MX6Q_USDHC_PAD_CTRL_22KPU_48OHM_50MHZ	PAD_CTL_PIS(22K_UP, 48ohm, LOW)
#define MX6Q_USDHC_PAD_CTRL_22KPU_60OHM_50MHZ	PAD_CTL_PIS(22K_UP, 60ohm, LOW)
#define MX6Q_USDHC_PAD_CTRL_22KPU_80OHM_50MHZ	PAD_CTL_PIS(22K_UP, 80ohm, LOW)
#define MX6Q_USDHC_PAD_CTRL_22KPU_120OHM_50MHZ	PAD_CTL_PIS(22K_UP, 120ohm, LOW)
#define MX6Q_USDHC_PAD_CTRL_47KPU_34OHM_50MHZ	PAD_CTL_PIS(47K_UP, 34ohm, LOW)
#define MX6Q_USDHC_PAD_CTRL_47KPU_40OHM_50MHZ	PAD_CTL_PIS(47K_UP, 40ohm, LOW)
#define MX6Q_USDHC_PAD_CTRL_47KPU_48OHM_50MHZ	PAD_CTL_PIS(47K_UP, 48ohm, LOW)
#define MX6Q_USDHC_PAD_CTRL_47KPU_60OHM_50MHZ	PAD_CTL_PIS(47K_UP, 60ohm, LOW)
#define MX6Q_USDHC_PAD_CTRL_47KPU_80OHM_50MHZ	PAD_CTL_PIS(47K_UP, 80ohm, LOW)
#define MX6Q_USDHC_PAD_CTRL_47KPU_120OHM_50MHZ	PAD_CTL_PIS(47K_UP, 120ohm, LOW)
#define MX6Q_USDHC_PAD_CTRL_50MHZ		MX6Q_USDHC_PAD_CTRL


#define MX6DL_USDHC_PAD_CTRL_22KPU_34OHM_50MHZ	PAD_CTL_PIS(22K_UP, 34ohm, LOW)
#define MX6DL_USDHC_PAD_CTRL_22KPU_40OHM_50MHZ	PAD_CTL_PIS(22K_UP, 40ohm, LOW)
#define MX6DL_USDHC_PAD_CTRL_22KPU_48OHM_50MHZ	PAD_CTL_PIS(22K_UP, 48ohm, LOW)
#define MX6DL_USDHC_PAD_CTRL_22KPU_60OHM_50MHZ	PAD_CTL_PIS(22K_UP, 60ohm, LOW)
#define MX6DL_USDHC_PAD_CTRL_22KPU_80OHM_50MHZ	PAD_CTL_PIS(22K_UP, 80ohm, LOW)
#define MX6DL_USDHC_PAD_CTRL_22KPU_120OHM_50MHZ	PAD_CTL_PIS(22K_UP, 120ohm, LOW)
#define MX6DL_USDHC_PAD_CTRL_47KPU_34OHM_50MHZ	PAD_CTL_PIS(47K_UP, 34ohm, LOW)
#define MX6DL_USDHC_PAD_CTRL_47KPU_40OHM_50MHZ	PAD_CTL_PIS(47K_UP, 40ohm, LOW)
#define MX6DL_USDHC_PAD_CTRL_47KPU_48OHM_50MHZ	PAD_CTL_PIS(47K_UP, 48ohm, LOW)
#define MX6DL_USDHC_PAD_CTRL_47KPU_60OHM_50MHZ	PAD_CTL_PIS(47K_UP, 60ohm, LOW)
#define MX6DL_USDHC_PAD_CTRL_47KPU_80OHM_50MHZ	PAD_CTL_PIS(47K_UP, 80ohm, LOW)
#define MX6DL_USDHC_PAD_CTRL_47KPU_120OHM_50MHZ	PAD_CTL_PIS(47K_UP, 120ohm, LOW)
#define MX6DL_USDHC_PAD_CTRL_50MHZ		MX6DL_USDHC_PAD_CTRL

#define MX6Q_PAD_SD3_CLK__USDHC3_CLK	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ
#define MX6Q_PAD_SD3_CMD__USDHC3_CMD	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ
#define MX6Q_PAD_SD3_DAT0__USDHC3_DAT0	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ
#define MX6Q_PAD_SD3_DAT1__USDHC3_DAT1	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ
#define MX6Q_PAD_SD3_DAT2__USDHC3_DAT2	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ
#define MX6Q_PAD_SD3_DAT3__USDHC3_DAT3	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ
#define MX6Q_PAD_SD4_CLK__USDHC4_CLK	MX6Q_PAD_SD4_CLK__USDHC4_CLK_50MHZ
#define MX6Q_PAD_SD4_CMD__USDHC4_CMD	MX6Q_PAD_SD4_CMD__USDHC4_CMD_50MHZ
#define MX6Q_PAD_SD4_DAT0__USDHC4_DAT0	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ
#define MX6Q_PAD_SD4_DAT1__USDHC4_DAT1	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ
#define MX6Q_PAD_SD4_DAT2__USDHC4_DAT2	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ
#define MX6Q_PAD_SD4_DAT3__USDHC4_DAT3	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ
#define MX6Q_PAD_SD4_DAT4__USDHC4_DAT4	MX6Q_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ
#define MX6Q_PAD_SD4_DAT5__USDHC4_DAT5	MX6Q_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ
#define MX6Q_PAD_SD4_DAT6__USDHC4_DAT6	MX6Q_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ
#define MX6Q_PAD_SD4_DAT7__USDHC4_DAT7	MX6Q_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ

#define MX6DL_PAD_SD3_CLK__USDHC3_CLK	MX6DL_PAD_SD3_CLK__USDHC3_CLK_50MHZ
#define MX6DL_PAD_SD3_CMD__USDHC3_CMD	MX6DL_PAD_SD3_CMD__USDHC3_CMD_50MHZ
#define MX6DL_PAD_SD3_DAT0__USDHC3_DAT0	MX6DL_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ
#define MX6DL_PAD_SD3_DAT1__USDHC3_DAT1	MX6DL_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ
#define MX6DL_PAD_SD3_DAT2__USDHC3_DAT2	MX6DL_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ
#define MX6DL_PAD_SD3_DAT3__USDHC3_DAT3	MX6DL_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ
#define MX6DL_PAD_SD4_CLK__USDHC4_CLK	MX6DL_PAD_SD4_CLK__USDHC4_CLK_50MHZ
#define MX6DL_PAD_SD4_CMD__USDHC4_CMD	MX6DL_PAD_SD4_CMD__USDHC4_CMD_50MHZ
#define MX6DL_PAD_SD4_DAT0__USDHC4_DAT0	MX6DL_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ
#define MX6DL_PAD_SD4_DAT1__USDHC4_DAT1	MX6DL_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ
#define MX6DL_PAD_SD4_DAT2__USDHC4_DAT2	MX6DL_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ
#define MX6DL_PAD_SD4_DAT3__USDHC4_DAT3	MX6DL_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ
#define MX6DL_PAD_SD4_DAT4__USDHC4_DAT4	MX6DL_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ
#define MX6DL_PAD_SD4_DAT5__USDHC4_DAT5	MX6DL_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ
#define MX6DL_PAD_SD4_DAT6__USDHC4_DAT6	MX6DL_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ
#define MX6DL_PAD_SD4_DAT7__USDHC4_DAT7	MX6DL_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ

#define NP(id, pin, pad_ctl) \
	NEW_PAD_CTRL(MX6PAD(SD##id##_##pin##__USDHC##id##_##pin), MX6(pad_ctl))

#define SD_PINS(id, pad_ctl) \
	NP(id, CLK, pad_ctl),	\
	NP(id, CMD, pad_ctl),	\
	NP(id, DAT0, pad_ctl),	\
	NP(id, DAT1, pad_ctl),	\
	NP(id, DAT2, pad_ctl),	\
	NP(id, DAT3, pad_ctl)

#define SD_PINS8(id, pad_ctl) \
	SD_PINS(id, pad_ctl), \
	NP(id, DAT4, pad_ctl),	\
	NP(id, DAT5, pad_ctl),	\
	NP(id, DAT6, pad_ctl),	\
	NP(id, DAT7, pad_ctl)

static iomux_v3_cfg_t MX6NAME(board_pads)[] = {
	/* AUDMUX */
	MX6PAD(CSI0_DAT7__AUDMUX_AUD3_RXD),
	MX6PAD(CSI0_DAT4__AUDMUX_AUD3_TXC),
	MX6PAD(CSI0_DAT5__AUDMUX_AUD3_TXD),
	MX6PAD(CSI0_DAT6__AUDMUX_AUD3_TXFS),

	/* CAN1  */
	MX6PAD(KEY_ROW2__CAN1_RXCAN),
	MX6PAD(KEY_COL2__CAN1_TXCAN),
#define GP_CAN1_STBY		IMX_GPIO_NR(1, 2)
	MX6PAD(GPIO_2__GPIO_1_2),		/* STNDBY */

	/* CCM  */
	MX6PAD(GPIO_0__CCM_CLKO),		/* SGTL500 sys_mclk */
	MX6PAD(GPIO_3__CCM_CLKO2),		/* J5, pin 17 - CSI0 Camera MCLK */

	/* ECSPI1 */
	MX6PAD(EIM_D17__ECSPI1_MISO),
	MX6PAD(EIM_D18__ECSPI1_MOSI),
	MX6PAD(EIM_D16__ECSPI1_SCLK),
#define GP_ECSPI1_CS1		IMX_GPIO_NR(3, 19)
	MX6PAD(EIM_D19__GPIO_3_19),	/*SS1*/

	/* ENET */
	MX6PAD(ENET_MDIO__ENET_MDIO),
	MX6PAD(ENET_MDC__ENET_MDC),
	MX6PAD(RGMII_TXC__ENET_RGMII_TXC),
	MX6PAD(RGMII_TD0__ENET_RGMII_TD0),
	MX6PAD(RGMII_TD1__ENET_RGMII_TD1),
	MX6PAD(RGMII_TD2__ENET_RGMII_TD2),
	MX6PAD(RGMII_TD3__ENET_RGMII_TD3),
	MX6PAD(RGMII_TX_CTL__ENET_RGMII_TX_CTL),
	MX6PAD(ENET_REF_CLK__ENET_TX_CLK),
	MX6PAD(RGMII_RXC__ENET_RGMII_RXC),
	MX6PAD(RGMII_RD0__ENET_RGMII_RD0),
	MX6PAD(RGMII_RD1__ENET_RGMII_RD1),
	MX6PAD(RGMII_RD2__ENET_RGMII_RD2),
	MX6PAD(RGMII_RD3__ENET_RGMII_RD3),
	MX6PAD(RGMII_RX_CTL__ENET_RGMII_RX_CTL),
#define GP_ENET_PHY_RESET	IMX_GPIO_NR(1, 27)
	MX6PAD(ENET_RXD0__GPIO_1_27),		/* Micrel RGMII Phy Reset */
#define GP_ENET_PHY_INT		IMX_GPIO_NR(1, 28)
	MX6PAD(ENET_TX_EN__GPIO_1_28),		/* Micrel RGMII Phy Interrupt */

	/* GPIO1 */

	/* gpio_keys - J14  */
#define GP_VOL_UP_KEY		IMX_GPIO_NR(7, 13)
	MX6PAD(GPIO_18__GPIO_7_13),	/* pin 2 - Volume Up */
#define GP_HOME_KEY		IMX_GPIO_NR(2, 4)
	MX6PAD(NANDF_D4__GPIO_2_4),	/* pin 3 - Home Button */
#define GP_ONOFF_KEY		IMX_GPIO_NR(2, 3)
	MX6PAD(NANDF_D3__GPIO_2_3),	/* pin 4 - Search Button */
#define GP_BACK_KEY		IMX_GPIO_NR(2, 2)
	MX6PAD(NANDF_D2__GPIO_2_2),	/* pin 5 - Back Button */
#define GP_MENU_KEY		IMX_GPIO_NR(2, 1)
	MX6PAD(NANDF_D1__GPIO_2_1),	/* pin 6 - Menu Button */
#define GP_VOL_DOWN_KEY		IMX_GPIO_NR(7, 1)
	MX6PAD(SD3_DAT4__GPIO_7_1),	/* pin 7 - Volume Down */


	/* Camera - J5(CSI0) */
#define GP_OV5642_CSI0_PWRDN	IMX_GPIO_NR(3, 29)
	MX6PAD(EIM_D29__GPIO_3_29),
#define GP_OV5642_CSI0_RESET	IMX_GPIO_NR(1, 4)
	MX6PAD(GPIO_4__GPIO_1_4),

	/* Camera - J12 - IPU - CSI1/Bootmode pins */
#ifdef FOR_DL_SOLO
	/* Dualite/Solo doesn't have IPU2 */
	MX6PAD(EIM_EB2__IPU1_CSI1_D_19),	/* GPIO2[30] */
	MX6PAD(EIM_A23__IPU1_CSI1_D_18),	/* GPIO6[6] */
	MX6PAD(EIM_A22__IPU1_CSI1_D_17),	/* GPIO2[16] */
	MX6PAD(EIM_A21__IPU1_CSI1_D_16),	/* GPIO2[17] */
	MX6PAD(EIM_A20__IPU1_CSI1_D_15),	/* GPIO2[18] */
	MX6PAD(EIM_A19__IPU1_CSI1_D_14),	/* GPIO2[19] */
	MX6PAD(EIM_A18__IPU1_CSI1_D_13),	/* GPIO2[20] */
	MX6PAD(EIM_A17__IPU1_CSI1_D_12),	/* GPIO2[21] */
	MX6PAD(EIM_EB0__IPU1_CSI1_D_11),	/* GPIO2[28] */
	MX6PAD(EIM_EB1__IPU1_CSI1_D_10),	/* GPIO2[29] */
	MX6PAD(EIM_DA0__IPU1_CSI1_D_9),		/* GPIO3[0] */
	MX6PAD(EIM_DA1__IPU1_CSI1_D_8),		/* GPIO3[1] */
	MX6PAD(EIM_DA2__IPU1_CSI1_D_7),		/* GPIO3[2] */
	MX6PAD(EIM_DA3__IPU1_CSI1_D_6),		/* GPIO3[3] */
	MX6PAD(EIM_DA4__IPU1_CSI1_D_5),		/* GPIO3[4] */
	MX6PAD(EIM_DA5__IPU1_CSI1_D_4),		/* GPIO3[5] */
	MX6PAD(EIM_DA6__IPU1_CSI1_D_3),		/* GPIO3[6] */
	MX6PAD(EIM_DA7__IPU1_CSI1_D_2),		/* GPIO3[7] */
	MX6PAD(EIM_DA8__IPU1_CSI1_D_1),		/* GPIO3[8] */
	MX6PAD(EIM_DA9__IPU1_CSI1_D_0),		/* GPIO3[9] */
	MX6PAD(EIM_DA10__IPU1_CSI1_DATA_EN),	/* GPIO3[10] */
	MX6PAD(EIM_DA11__IPU1_CSI1_HSYNC),	/* GPIO3[11] */
	MX6PAD(EIM_DA12__IPU1_CSI1_VSYNC),	/* GPIO3[12] */
	MX6PAD(EIM_A16__IPU1_CSI1_PIXCLK),	/* GPIO2[22] */
#else
	MX6PAD(EIM_EB2__IPU2_CSI1_D_19),	/* GPIO2[30] */
	MX6PAD(EIM_A23__IPU2_CSI1_D_18),	/* GPIO6[6] */
	MX6PAD(EIM_A22__IPU2_CSI1_D_17),	/* GPIO2[16] */
	MX6PAD(EIM_A21__IPU2_CSI1_D_16),	/* GPIO2[17] */
	MX6PAD(EIM_A20__IPU2_CSI1_D_15),	/* GPIO2[18] */
	MX6PAD(EIM_A19__IPU2_CSI1_D_14),	/* GPIO2[19] */
	MX6PAD(EIM_A18__IPU2_CSI1_D_13),	/* GPIO2[20] */
	MX6PAD(EIM_A17__IPU2_CSI1_D_12),	/* GPIO2[21] */
	MX6PAD(EIM_EB0__IPU2_CSI1_D_11),	/* GPIO2[28] */
	MX6PAD(EIM_EB1__IPU2_CSI1_D_10),	/* GPIO2[29] */
	MX6PAD(EIM_DA0__IPU2_CSI1_D_9),		/* GPIO3[0] */
	MX6PAD(EIM_DA1__IPU2_CSI1_D_8),		/* GPIO3[1] */
	MX6PAD(EIM_DA2__IPU2_CSI1_D_7),		/* GPIO3[2] */
	MX6PAD(EIM_DA3__IPU2_CSI1_D_6),		/* GPIO3[3] */
	MX6PAD(EIM_DA4__IPU2_CSI1_D_5),		/* GPIO3[4] */
	MX6PAD(EIM_DA5__IPU2_CSI1_D_4),		/* GPIO3[5] */
	MX6PAD(EIM_DA6__IPU2_CSI1_D_3),		/* GPIO3[6] */
	MX6PAD(EIM_DA7__IPU2_CSI1_D_2),		/* GPIO3[7] */
	MX6PAD(EIM_DA8__IPU2_CSI1_D_1),		/* GPIO3[8] */
	MX6PAD(EIM_DA9__IPU2_CSI1_D_0),		/* GPIO3[9] */
	MX6PAD(EIM_DA10__IPU2_CSI1_DATA_EN),	/* GPIO3[10] */
	MX6PAD(EIM_DA11__IPU2_CSI1_HSYNC),	/* GPIO3[11] */
	MX6PAD(EIM_DA12__IPU2_CSI1_VSYNC),	/* GPIO3[12] */
	MX6PAD(EIM_A16__IPU2_CSI1_PIXCLK),	/* GPIO2[22] */
#endif
#define GP_OV5640_CSI1_PWRDN	IMX_GPIO_NR(3, 13)
#define GP_ADV7180_CSI1_PWR	IMX_GPIO_NR(3, 13)
	MX6PAD(EIM_DA13__GPIO_3_13),		/* pin 32 - Power */
#define GP_OV5640_CSI1_RESET	IMX_GPIO_NR(3, 14)
#define GP_ADV7180_CSI1_RESET	IMX_GPIO_NR(3, 14)
	MX6PAD(EIM_DA14__GPIO_3_14),		/* pin 36 - Reset */
	MX6PAD(EIM_WAIT__GPIO_5_0),		/* pin 31 - Irq */
	MX6PAD(EIM_A24__GPIO_5_4),		/* pin 35 - Field */
	MX6PAD(EIM_RW__GPIO_2_26),		/* pin 21 - unused */
	MX6PAD(EIM_LBA__GPIO_2_27),		/* pin 27 - unused */
	MX6PAD(EIM_EB3__GPIO_2_31),		/* pin 17 - unused */
	MX6PAD(EIM_DA15__GPIO_3_15),		/* pin 40 - unused */

	/* Camera(Mipi) - J16 */
	MX6PAD(SD1_DAT1__PWM3_PWMO),		/* pin 10 */
#define GP_OV5640_MIPI_PWRDN	IMX_GPIO_NR(6, 9)
	MX6PAD(NANDF_WP_B__GPIO_6_9),
#define GP_OV5640_MIPI_RESET	IMX_GPIO_NR(2, 5)
	MX6PAD(NANDF_D5__GPIO_2_5),


	/* I2C1, SGTL5000 */
	MX6PAD(EIM_D21__I2C1_SCL),	/* GPIO3[21] */
	MX6PAD(EIM_D28__I2C1_SDA),	/* GPIO3[28] */

	/* I2C2 Camera, MIPI */
	MX6PAD(KEY_COL3__I2C2_SCL),	/* GPIO4[12] */
	MX6PAD(KEY_ROW3__I2C2_SDA),	/* GPIO4[13] */
#define GP_OV5642_CSI0_I2C_EN	IMX_GPIO_NR(3, 20)
	MX6PAD(EIM_D20__GPIO_3_20),	/* ov5642 CSI0 I2C enable */
#define GP_OV5640_MIPI_I2C_EN	IMX_GPIO_NR(4, 15)
	MX6PAD(KEY_ROW4__GPIO_4_15),	/* ov5640 MIPI CSI I2C enable */

	/* I2C3 */
	MX6PAD(GPIO_5__I2C3_SCL),	/* GPIO1[5] - J7 - Display card */
	MX6PAD(GPIO_16__I2C3_SDA),	/* GPIO7[11] - J15 - RGB connector */
	/* I2C Touchscreen IRQ on RGB display connector J15 pin4, contrast*/
#define GP_DRGB_IRQGPIO		IMX_GPIO_NR(4, 20)
	NEW_PAD_CTRL(MX6PAD(DI0_PIN4__GPIO_4_20), WEAK_PULLUP),
#define GP_CAP_TCH_INT1		IMX_GPIO_NR(1, 9)
	MX6PAD(GPIO_9__GPIO_1_9),		/* J7, Pin 4 */


	/* LVDS0 - J6 */
	MX6PAD(NANDF_D0__GPIO_2_0),		/* pin 19 - contrast */
	MX6PAD(SD1_CMD__PWM4_PWMO),		/* pin 20 - backlight */

	/* LVDS1 - J11 */
	NEW_PAD_CTRL(MX6PAD(EIM_CS0__GPIO_2_23), WEAK_PULLDN),	/* pin 19 - contrast */
	MX6PAD(SD1_DAT2__PWM2_PWMO),		/* pin 20 - backlight */


	/* LCD - J15 RGB Display */
	MX6PAD(SD1_DAT3__PWM1_PWMO),		/* J15, pin 37 - backlight */

	/* PCIe */
#define GP_PCIE_RESET		IMX_GPIO_NR(6, 31)
	NEW_PAD_CTRL(MX6PAD(EIM_BCLK__GPIO_6_31), OUTPUT_40OHM),

	/* rtc */
#define GP_RTC_RV4162_IRQ	IMX_GPIO_NR(4, 6)
	NEW_PAD_CTRL(MX6PAD(KEY_COL0__GPIO_4_6), WEAK_PULLUP),

	/* SPDIF */
	MX6PAD(GPIO_7__SPDIF_PLOCK),
	MX6PAD(GPIO_8__SPDIF_SRCLK),
	MX6PAD(GPIO_19__SPDIF_OUT1),
	MX6PAD(ENET_CRS_DV__SPDIF_SPDIF_EXTCLK),
	MX6PAD(ENET_RX_ER__SPDIF_IN1),

	/* UART1  */
#ifdef ONE_WIRE
	NEW_PAD_CTRL(MX6PAD(SD3_DAT7__UART1_TXD), 0x0001f8b1),
	NEW_PAD_CTRL(MX6PAD(SD3_DAT6__UART1_RXD), 0x0001f0b1),
#else
	MX6PAD(SD3_DAT7__UART1_TXD),
	MX6PAD(SD3_DAT6__UART1_RXD),
#endif

	/* UART2 for debug */
	MX6PAD(EIM_D26__UART2_TXD),
	MX6PAD(EIM_D27__UART2_RXD),

	/* UART3 for wl1271: TiWi bluetooth */
	MX6PAD(EIM_D24__UART3_TXD),
	MX6PAD(EIM_D25__UART3_RXD),
	MX6PAD(EIM_D23__UART3_CTS),
	MX6PAD(EIM_D31__UART3_RTS),

#if !(defined(CSI0_CAMERA))
	/* UART4 */
	MX6PAD(CSI0_DAT12__UART4_TXD),
	MX6PAD(CSI0_DAT13__UART4_RXD),
	MX6PAD(CSI0_DAT17__UART4_CTS),
	MX6PAD(CSI0_DAT16__UART4_RTS),
#endif

	/* UART5 - ISL3330IAZ rs485/rs232 selection */
	NEW_PAD_CTRL(MX6PAD(KEY_COL1__UART5_TXD), UART_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(KEY_ROW1__UART5_RXD), UART_PAD_CTRL),
#define GP_UART5_RX_EN		IMX_GPIO_NR(6, 10)
	NEW_PAD_CTRL(MX6PAD(NANDF_RB0__GPIO_6_10), WEAK_PULLDN),	/* RS485 RX Enable */
#define GP_UART5_TX_EN		IMX_GPIO_NR(6, 7)
	NEW_PAD_CTRL(MX6PAD(NANDF_CLE__GPIO_6_7), WEAK_PULLDN),		/* RS485 TX Enable */
#define GP_UART5_RS485_EN	IMX_GPIO_NR(2, 24)
	NEW_PAD_CTRL(MX6PAD(EIM_CS1__GPIO_2_24), WEAK_PULLDN),		/* RS485/RS232 Select 2.5V */
#define GP_UART5_AON		IMX_GPIO_NR(6, 8)
	NEW_PAD_CTRL(MX6PAD(NANDF_ALE__GPIO_6_8), WEAK_PULLDN),		/* ON - meaning depends on others */

	/* USBH1 */
	MX6PAD(EIM_D30__USBOH3_USBH1_OC),
#define GP_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
	MX6PAD(GPIO_17__GPIO_7_12),	/* USB Hub Reset */

	/* USBOTG  */
	MX6PAD(GPIO_1__USBOTG_ID),
	MX6PAD(KEY_COL4__USBOH3_USBOTG_OC),
#define GP_USB_OTG_PWR		IMX_GPIO_NR(3, 22)
	NEW_PAD_CTRL(MX6PAD(EIM_D22__GPIO_3_22), OUTPUT_40OHM),

	/* USDHC2: wl1271: TiWi wlan/bluetooth(UART3) */
	SD_PINS(2, USDHC_PAD_CTRL_22KPU_40OHM_50MHZ),
        MX6PAD(SD1_CLK__OSC32K_32K_OUT), /* wl1271 clock */
#define GP_WL1271_WL_IRQ	IMX_GPIO_NR(6, 14)
	NEW_PAD_CTRL(MX6PAD(NANDF_CS1__GPIO_6_14), PADCFG_FLOAT_IRQ),	/* wl1271 wl_irq */
#define GP_WL1271_WL_EN		IMX_GPIO_NR(6, 15)
	NEW_PAD_CTRL(MX6PAD(NANDF_CS2__GPIO_6_15), OUTPUT_40OHM),	/* wl1271 wl_en */
#define GP_WL1271_BT_EN		IMX_GPIO_NR(6, 16)
	NEW_PAD_CTRL(MX6PAD(NANDF_CS3__GPIO_6_16), OUTPUT_40OHM),	/* wl1271 bt_en */

	/* USDHC3 */
	SD_PINS(3, USDHC_PAD_CTRL_47KPU_80OHM_50MHZ),
#define GP_SD3_CD		IMX_GPIO_NR(7, 0)
	NEW_PAD_CTRL(MX6PAD(SD3_DAT5__GPIO_7_0), WEAK_PULLUP),		/* J18 - SD3_CD */
#define GP_SD3_POWER_SEL	IMX_GPIO_NR(6, 11)
	NEW_PAD_CTRL(MX6PAD(NANDF_CS0__GPIO_6_11), OUTPUT_40OHM),

	/* USDHC4 */
	SD_PINS8(4, USDHC_PAD_CTRL_50MHZ),
#define GP_EMMC_RESET	IMX_GPIO_NR(2, 6)
	MX6PAD(NANDF_D6__GPIO_2_6),		/* eMMC reset */
	0
};

static iomux_v3_cfg_t MX6NAME(lcd_pads_enable)[] = {
	MX6PAD(DI0_DISP_CLK__IPU1_DI0_DISP_CLK),
	MX6PAD(DI0_PIN15__IPU1_DI0_PIN15),		/* DE */
	MX6PAD(DI0_PIN2__IPU1_DI0_PIN2),		/* HSync */
	MX6PAD(DI0_PIN3__IPU1_DI0_PIN3),		/* VSync */
	MX6PAD(DISP0_DAT0__IPU1_DISP0_DAT_0),
	MX6PAD(DISP0_DAT1__IPU1_DISP0_DAT_1),
	MX6PAD(DISP0_DAT2__IPU1_DISP0_DAT_2),
	MX6PAD(DISP0_DAT3__IPU1_DISP0_DAT_3),
	MX6PAD(DISP0_DAT4__IPU1_DISP0_DAT_4),
	MX6PAD(DISP0_DAT5__IPU1_DISP0_DAT_5),
	MX6PAD(DISP0_DAT6__IPU1_DISP0_DAT_6),
	MX6PAD(DISP0_DAT7__IPU1_DISP0_DAT_7),
	MX6PAD(DISP0_DAT8__IPU1_DISP0_DAT_8),
	MX6PAD(DISP0_DAT9__IPU1_DISP0_DAT_9),
	MX6PAD(DISP0_DAT10__IPU1_DISP0_DAT_10),
	MX6PAD(DISP0_DAT11__IPU1_DISP0_DAT_11),
	MX6PAD(DISP0_DAT12__IPU1_DISP0_DAT_12),
	MX6PAD(DISP0_DAT13__IPU1_DISP0_DAT_13),
	MX6PAD(DISP0_DAT14__IPU1_DISP0_DAT_14),
	MX6PAD(DISP0_DAT15__IPU1_DISP0_DAT_15),
	MX6PAD(DISP0_DAT16__IPU1_DISP0_DAT_16),
	MX6PAD(DISP0_DAT17__IPU1_DISP0_DAT_17),
	MX6PAD(DISP0_DAT18__IPU1_DISP0_DAT_18),
	MX6PAD(DISP0_DAT19__IPU1_DISP0_DAT_19),
	MX6PAD(DISP0_DAT20__IPU1_DISP0_DAT_20),
	MX6PAD(DISP0_DAT21__IPU1_DISP0_DAT_21),
	MX6PAD(DISP0_DAT22__IPU1_DISP0_DAT_22),
	MX6PAD(DISP0_DAT23__IPU1_DISP0_DAT_23),
	0
};

static iomux_v3_cfg_t MX6NAME(lcd_pads_disable)[] = {
	MX6PAD(DI0_DISP_CLK__GPIO_4_16),
	MX6PAD(DI0_PIN15__GPIO_4_17),			/* DE */
	MX6PAD(DI0_PIN2__GPIO_4_18),			/* HSync */
	MX6PAD(DI0_PIN3__GPIO_4_19),			/* VSync */
	MX6PAD(DISP0_DAT0__GPIO_4_21),
	MX6PAD(DISP0_DAT1__GPIO_4_22),
	MX6PAD(DISP0_DAT2__GPIO_4_23),
	MX6PAD(DISP0_DAT3__GPIO_4_24),
	MX6PAD(DISP0_DAT4__GPIO_4_25),
	MX6PAD(DISP0_DAT5__GPIO_4_26),
	MX6PAD(DISP0_DAT6__GPIO_4_27),
	MX6PAD(DISP0_DAT7__GPIO_4_28),
	MX6PAD(DISP0_DAT8__GPIO_4_29),
	MX6PAD(DISP0_DAT9__GPIO_4_30),
	MX6PAD(DISP0_DAT10__GPIO_4_31),
	MX6PAD(DISP0_DAT11__GPIO_5_5),
	MX6PAD(DISP0_DAT12__GPIO_5_6),
	MX6PAD(DISP0_DAT13__GPIO_5_7),
	MX6PAD(DISP0_DAT14__GPIO_5_8),
	MX6PAD(DISP0_DAT15__GPIO_5_9),
#ifdef ONE_WIRE
	MX6PAD(DISP0_DAT16__ECSPI2_MOSI),
	MX6PAD(DISP0_DAT17__ECSPI2_MISO),
	MX6PAD(DISP0_DAT18__GPIO_5_12),		/* SS0 */
	MX6PAD(DISP0_DAT19__ECSPI2_SCLK),
#else
	MX6PAD(DISP0_DAT16__GPIO_5_10),
	MX6PAD(DISP0_DAT17__GPIO_5_11),
	MX6PAD(DISP0_DAT18__GPIO_5_12),
	MX6PAD(DISP0_DAT19__GPIO_5_13),
#endif
	MX6PAD(DISP0_DAT20__GPIO_5_14),
	MX6PAD(DISP0_DAT21__GPIO_5_15),
	MX6PAD(DISP0_DAT22__GPIO_5_16),
	MX6PAD(DISP0_DAT23__GPIO_5_17),
	0
};

#if defined(CSI0_CAMERA)
/* J5 Camera */
static iomux_v3_cfg_t MX6NAME(csi0_sensor_pads)[] = {
	/* IPU1 Camera */
	MX6PAD(CSI0_DAT8__IPU1_CSI0_D_8),
	MX6PAD(CSI0_DAT9__IPU1_CSI0_D_9),
	MX6PAD(CSI0_DAT10__IPU1_CSI0_D_10),
	MX6PAD(CSI0_DAT11__IPU1_CSI0_D_11),
	MX6PAD(CSI0_DAT12__IPU1_CSI0_D_12),
	MX6PAD(CSI0_DAT13__IPU1_CSI0_D_13),
	MX6PAD(CSI0_DAT14__IPU1_CSI0_D_14),
	MX6PAD(CSI0_DAT15__IPU1_CSI0_D_15),
	MX6PAD(CSI0_DAT16__IPU1_CSI0_D_16),
	MX6PAD(CSI0_DAT17__IPU1_CSI0_D_17),
	MX6PAD(CSI0_DAT18__IPU1_CSI0_D_18),
	MX6PAD(CSI0_DAT19__IPU1_CSI0_D_19),
	MX6PAD(CSI0_DATA_EN__IPU1_CSI0_DATA_EN),
	MX6PAD(CSI0_MCLK__IPU1_CSI0_HSYNC),
	MX6PAD(CSI0_PIXCLK__IPU1_CSI0_PIXCLK),
	MX6PAD(CSI0_VSYNC__IPU1_CSI0_VSYNC),
	MX6PAD(SD1_DAT0__GPIO_1_16),		/* Camera GP */
	0
};
#endif


static iomux_v3_cfg_t MX6NAME(hdmi_ddc_pads)[] = {
	MX6PAD(KEY_COL3__HDMI_TX_DDC_SCL), /* HDMI DDC SCL */
	MX6PAD(KEY_ROW3__HDMI_TX_DDC_SDA), /* HDMI DDC SDA */
	0
};

static iomux_v3_cfg_t MX6NAME(i2c2_pads)[] = {
	MX6PAD(KEY_COL3__I2C2_SCL),	/* I2C2 SCL */
	MX6PAD(KEY_ROW3__I2C2_SDA),	/* I2C2 SDA */
	0
};

#define MX6_USDHC_PAD_SETTING(id, speed, pad_ctl)	\
		MX6NAME(sd##id##_##speed##mhz)[] = { SD_PINS(id, pad_ctl), 0 }

#define MX6_USDHC_PAD_SETTING8(id, speed, pad_ctl)	\
		MX6NAME(sd##id##_##speed##mhz)[] = { SD_PINS8(id, pad_ctl), 0 }

static iomux_v3_cfg_t MX6_USDHC_PAD_SETTING(2, 50, USDHC_PAD_CTRL_22KPU_40OHM_50MHZ);
static iomux_v3_cfg_t MX6_USDHC_PAD_SETTING(2, 100, USDHC_PAD_CTRL_100MHZ);
static iomux_v3_cfg_t MX6_USDHC_PAD_SETTING(2, 200, USDHC_PAD_CTRL_200MHZ);
static iomux_v3_cfg_t MX6_USDHC_PAD_SETTING(3, 50, USDHC_PAD_CTRL_47KPU_80OHM_50MHZ);
static iomux_v3_cfg_t MX6_USDHC_PAD_SETTING(3, 100, USDHC_PAD_CTRL_100MHZ);
static iomux_v3_cfg_t MX6_USDHC_PAD_SETTING(3, 200, USDHC_PAD_CTRL_200MHZ);
static iomux_v3_cfg_t MX6_USDHC_PAD_SETTING8(4, 50, USDHC_PAD_CTRL_50MHZ);
static iomux_v3_cfg_t MX6_USDHC_PAD_SETTING8(4, 100, USDHC_PAD_CTRL_100MHZ);
static iomux_v3_cfg_t MX6_USDHC_PAD_SETTING8(4, 200, USDHC_PAD_CTRL_200MHZ);

#define _50MHZ 0
#define _100MHZ 1
#define _200MHZ 2
#define SD_SPEED_CNT 3
static iomux_v3_cfg_t * MX6NAME(sd_pads)[] =
{
	MX6NAME(sd2_50mhz),
	MX6NAME(sd2_100mhz),
	MX6NAME(sd2_200mhz),
	MX6NAME(sd3_50mhz),
	MX6NAME(sd3_100mhz),
	MX6NAME(sd3_200mhz),
	MX6NAME(sd4_50mhz),
	MX6NAME(sd4_100mhz),
	MX6NAME(sd4_200mhz),
};