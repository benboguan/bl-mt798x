// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Maxlinear Corporation
 * Copyright (C) 2020 Intel Corporation
 * Modify Intel GPY211 Driver Support For Uboot By nanchuci
 */
#include <common.h>
#include <dm.h>
#include <dm/devres.h>
#include <miiphy.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <phy.h>
#include <linux/netdevice.h>

#define PHY_MIISTAT		0x18	/* MII state */
#define PHY_LED			0x1B	/* LEDs */
#define PHY_FWV			0x1E	/* firmware version */

#define PHY_FWV_REL_MASK	BIT(15)
#define PHY_FWV_MAJOR_MASK	GENMASK(11, 8)
#define PHY_FWV_MINOR_MASK	GENMASK(7, 0)

#define PHY_MIISTAT_SPD_MASK	GENMASK(2, 0)
#define PHY_MIISTAT_DPX		BIT(3)
#define PHY_MIISTAT_LS		BIT(10)

#define PHY_MIISTAT_SPD_10	0
#define PHY_MIISTAT_SPD_100	1
#define PHY_MIISTAT_SPD_1000	2
#define PHY_MIISTAT_SPD_2500	4

#define	ETHTOOL_LINK_MODE_10baseT_Half_BIT	 0
#define	ETHTOOL_LINK_MODE_10baseT_Full_BIT	 1
#define	ETHTOOL_LINK_MODE_100baseT_Half_BIT	 2
#define	ETHTOOL_LINK_MODE_100baseT_Full_BIT	 3
#define	ETHTOOL_LINK_MODE_1000baseT_Half_BIT	 4
#define	ETHTOOL_LINK_MODE_1000baseT_Full_BIT	 5
#define ETHTOOL_LINK_MODE_Autoneg_BIT			 6
#define	ETHTOOL_LINK_MODE_10000baseT_Full_BIT	 12
#define	ETHTOOL_LINK_MODE_2500baseX_Full_BIT	 15
#define	ETHTOOL_LINK_MODE_1000baseKX_Full_BIT	 17
#define ETHTOOL_LINK_MODE_1000baseX_Full_BIT	 41
#define	ETHTOOL_LINK_MODE_2500baseT_Full_BIT	 47
#define	ETHTOOL_LINK_MODE_5000baseT_Full_BIT	 48
#define	ETHTOOL_LINK_MODE_100baseT1_Full_BIT	 67
#define	ETHTOOL_LINK_MODE_1000baseT1_Full_BIT	 68

#define __ETHTOOL_LINK_MODE_MASK_NBITS 51

/* PHY Link State */
#define PHY_LINK_DOWN	0
#define PHY_LINK_UP	1

#define PHY_LINK_UNKNOWN	-1

/* PHY Interface Modes */
#define PHY_INTERFACE_MODE_NONE	0
#define PHY_INTERFACE_MODE_INTERNAL	1
#define PHY_INTERFACE_MODE_MII	2
#define PHY_INTERFACE_MODE_GMII	3
#define PHY_INTERFACE_MODE_SGMII	4
#define PHY_INTERFACE_MODE_TBI	5
#define PHY_INTERFACE_MODE_REVMII	6
#define PHY_INTERFACE_MODE_RMII	7
#define PHY_INTERFACE_MODE_RGMII	8
#define PHY_INTERFACE_MODE_RGMII_ID	9
#define PHY_INTERFACE_MODE_RGMII_RXID	10
#define PHY_INTERFACE_MODE_RGMII_TXID	11
#define PHY_INTERFACE_MODE_RTBI	12
#define PHY_INTERFACE_MODE_SMII	13
#define PHY_INTERFACE_MODE_XGMII	14
#define PHY_INTERFACE_MODE_MOCA	15
#define PHY_INTERFACE_MODE_QSGMII	16
#define PHY_INTERFACE_MODE_TRGMII	17
#define PHY_INTERFACE_MODE_1000BASEX	18
#define PHY_INTERFACE_MODE_2500BASEX	19
#define PHY_INTERFACE_MODE_RXAUI	20
#define PHY_INTERFACE_MODE_XAUI	21
#define PHY_INTERFACE_MODE_10GKR	22 // 10GBASE-KR, XFI, SFI - single lane 10G Serdes
#define PHY_INTERFACE_MODE_USXGMII	23

/* PHY Link Speeds */
#define SPEED_UNKNOWN	-1

/* PHY Duplex Modes */
#define DUPLEX_UNKNOWN	-1

/* SGMII */
#define VSPEC1_SGMII_CTRL	0x08
#define VSPEC1_SGMII_CTRL_ANEN	BIT(12)		/* Aneg enable */
#define VSPEC1_SGMII_CTRL_ANRS	BIT(9)		/* Restart Aneg */
#define VSPEC1_SGMII_ANEN_ANRS	(VSPEC1_SGMII_CTRL_ANEN | \
				 VSPEC1_SGMII_CTRL_ANRS)

/* LED */
#define VSPEC1_LED(idx)		(1 + (idx))
/* #define VSPEC1_LED_BLINKS	GENMASK(15, 12)
 * #define VSPEC1_LED_PULSE	GENMASK(11, 8)
 * #define VSPEC1_LED_CON		GENMASK(7, 4)
 * #define VSPEC1_LED_BLINKF	GENMASK(3, 0)
 * 
 * #define VSPEC1_LED_LINK10	BIT(0)
 * #define VSPEC1_LED_LINK100	BIT(1)
 * #define VSPEC1_LED_LINK1000	BIT(2)
 * #define VSPEC1_LED_LINK2500	BIT(3)
 * 
 * #define VSPEC1_LED_TXACT	BIT(0)
 * #define VSPEC1_LED_RXACT	BIT(1)
 * #define VSPEC1_LED_COL		BIT(2)
 * #define VSPEC1_LED_NO_CON	BIT(3)
 */
// Starder Magament Registers
#define MDIO_MMD_STD              0x0
#define VSPEC1_NBT_DS_CTRL        0xA
#define DOWNSHIFT_THR_MASK    GENMASK(6, 2)
#define DOWNSHIFT_EN          BIT(1)

#define DEFAULT_INTEL_GPY211_PHYID1_VALUE	0x67c9

#define MAXLINEAR_MAX_LED_INDEX 4
#define MAX_RETRY_TIMES	80
#define RETRY_INTERVAL	10 // unit is ms

struct gpy_priv {
	u8 fw_major;
	u8 fw_minor;
};

static inline void linkmode_set_bit(int nr, int addr)
{
	__set_bit(nr, addr);
}

static inline void linkmode_clear_bit(int nr, int addr)
{
	__clear_bit(nr, addr);
}

static inline void linkmode_mod_bit(int nr, int addr,
				    int set)
{
	if (set)
		linkmode_set_bit(nr, addr);
	else
		linkmode_clear_bit(nr, addr);
}

/**
 * mii_stat1000_mod_linkmode_lpa_t
 * @advertising: target the linkmode advertisement settings
 * @adv: value of the MII_STAT1000 register
 *
 * A small helper function that translates MII_STAT1000 bits, when in
 * 1000Base-T mode, to linkmode advertisement settings. Other bits in
 * advertising are not changes.
 */
static inline void mii_stat1000_mod_linkmode_lpa_t(u32 advertising,
						   u32 lpa)
{
	linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
			 advertising, lpa & LPA_1000HALF);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
			 advertising, lpa & LPA_1000FULL);
}

static int gpy211_led_write(struct phy_device *phydev)
{
	u32 led_regs[MAXLINEAR_MAX_LED_INDEX] = {0};
	int i = 0, ret;
	u16 val = 0xff00;
	u32 phyid1;
	ofnode node;

	i = MAX_RETRY_TIMES;
	while (i) {
		phyid1 = phy_read_mmd(phydev, MDIO_MMD_STD, MDIO_DEVID1);
		if (phyid1 == DEFAULT_INTEL_GPY211_PHYID1_VALUE)
			break;

		/* msleep(RETRY_INTERVAL); */
		i--;
	}
	if (!i) {
		printf("phy is not ready over %d ms!\n", (MAX_RETRY_TIMES-i)*10);
	} else {
		printf("driver wait %d ms for phy ready!\n", (MAX_RETRY_TIMES-i)*10);
	}

	node = phy_get_ofnode(phydev);
	if (!ofnode_valid(node))
		return -EINVAL;

	if (ofnode_read_u32_array(node, "maxlinear,led-reg", led_regs, MAXLINEAR_MAX_LED_INDEX))
		return 0;

	if (ofnode_read_bool(node, "maxlinear,led-drive-vdd"))
		val &= 0x0fff;

	/* Enable LED function handling on all ports*/
	phy_write(phydev, MDIO_MMD_VEND1, PHY_LED, val);

	/* Write LED register values */
	for (i = 0; i < MAXLINEAR_MAX_LED_INDEX; i++) {
		ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, VSPEC1_LED(i), (u16)led_regs[i]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int gpy211_phy_config(struct phy_device *phydev)
{
	genphy_config_aneg(phydev);
	genphy_restart_aneg(phydev);

	phy_reset(phydev);

	return gpy211_led_write(phydev);
}

static int gpy211_probe(struct phy_device *phydev)
{
	int sgmii_reg = phy_read_mmd(phydev, MDIO_MMD_VEND1, 8);
	struct gpy_priv *priv;
	int fw_version;
	int buf = 0;

	priv = devm_kzalloc(phydev->priv, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	phydev->priv = priv;

	fw_version = phy_read(phydev, MDIO_MMD_VEND1, PHY_FWV);
	if (fw_version < 0)
		return fw_version;
	priv->fw_major = FIELD_GET(PHY_FWV_MAJOR_MASK, fw_version);
	priv->fw_minor = FIELD_GET(PHY_FWV_MINOR_MASK, fw_version);

	/* Show GPY PHY FW version in dmesg */
	printf("Firmware Version: %d.%d (0x%04X%s)\n",
		    priv->fw_major, priv->fw_minor, fw_version,
		    fw_version & PHY_FWV_REL_MASK ? "" : " test version");

	/* enable 2.5G SGMII rate adaption */
	phy_write_mmd(phydev, MDIO_MMD_VEND1, 8, 0x24e2);

	buf = phy_read_mmd(phydev, MDIO_MMD_VEND1, VSPEC1_NBT_DS_CTRL);
	//enable downshift and set training counter threshold to 3
	phy_write_mmd(phydev, MDIO_MMD_VEND1, VSPEC1_NBT_DS_CTRL, buf | FIELD_PREP(DOWNSHIFT_THR_MASK, 0x3) | DOWNSHIFT_EN);

	return 0;
}

static int genphy_read_abilities(struct phy_device *phydev)
{
	int val;

	val = phy_read(phydev, MDIO_MMD_VEND1, MII_BMSR);
	if (val < 0)
		return val;

	linkmode_mod_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, phydev->supported,
			 val & BMSR_ANEGCAPABLE);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, phydev->supported,
			 val & BMSR_100FULL);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT, phydev->supported,
			 val & BMSR_100HALF);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT, phydev->supported,
			 val & BMSR_10FULL);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT, phydev->supported,
			 val & BMSR_10HALF);

	if (val & BMSR_ESTATEN) {
		val = phy_read(phydev, MDIO_MMD_VEND1, MII_ESTATUS);
		if (val < 0)
			return val;

		linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
				 phydev->supported, val & ESTATUS_1000_TFULL);
		linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
				 phydev->supported, val & ESTATUS_1000_THALF);
		linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseX_Full_BIT,
				 phydev->supported, val & ESTATUS_1000_XFULL);
	}

	return 0;
}

static int gpy211_get_features(struct phy_device *phydev)
{
	int ret;

	ret = genphy_read_abilities(phydev);
	if (ret)
		return ret;

	/* GPY211 with rate adaption supports 100M/1G/2.5G speed. */
	linkmode_clear_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT,
			   phydev->supported);
	linkmode_clear_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT,
			   phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_2500baseX_Full_BIT,
			 phydev->supported);

	return 0;
}

static bool gpy211_sgmii_aneg_en(struct phy_device *phydev)
{
	int ret;

	ret = phy_read_mmd(phydev, MDIO_MMD_VEND1, VSPEC1_SGMII_CTRL);
	if (ret < 0) {
		printf("Error: MMD register access failed: %d\n", ret);
		return true;
	}

	return (ret & VSPEC1_SGMII_CTRL_ANEN) ? true : false;
}

static int gpy211_update_interface(struct phy_device *phydev)
{
	int ret;

	/* Interface mode is fixed for USXGMII and integrated PHY */
	if (phydev->interface == PHY_INTERFACE_MODE_USXGMII ||
	    phydev->interface == PHY_INTERFACE_MODE_INTERNAL)
		return -EINVAL;

	/* Automatically switch SERDES interface between SGMII and 2500-BaseX
	 * according to speed. Disable ANEG in 2500-BaseX mode.
	 */
	switch (phydev->speed) {
	case SPEED_2500:
		phydev->interface = PHY_INTERFACE_MODE_2500BASEX;
		ret = phy_modify_mmd(phydev, MDIO_MMD_VEND1, VSPEC1_SGMII_CTRL,
				     VSPEC1_SGMII_CTRL_ANEN, 0);
		if (ret < 0) {
			printf("Error: Disable of SGMII ANEG failed: %d\n", ret);
			return ret;
		}
		break;
	case SPEED_1000:
	case SPEED_100:
	case SPEED_10:
		phydev->interface = PHY_INTERFACE_MODE_SGMII;
		if (gpy211_sgmii_aneg_en(phydev))
			break;
		/* Enable and restart SGMII ANEG for 10/100/1000Mbps link speed
		 * if ANEG is disabled (in 2500-BaseX mode).
		 */
		ret = phy_modify_mmd(phydev, MDIO_MMD_VEND1, VSPEC1_SGMII_CTRL,
				     VSPEC1_SGMII_ANEN_ANRS,
				     VSPEC1_SGMII_ANEN_ANRS);
		if (ret < 0) {
			printf("Error: Enable of SGMII ANEG failed: %d\n", ret);
			return ret;
		}
		break;
	}

	//return gpy211_get_features(phydev);
	return ret;
}

static int gpy211_read_status(struct phy_device *phydev)
{
	const char *speed;
	const char *duplex;
	int ret;

	ret = genphy_update_link(phydev);
	if (ret)
		return ret;

	phydev->speed = SPEED_UNKNOWN;
	phydev->duplex = DUPLEX_UNKNOWN;
	phydev->pause = 0;
	phydev->asym_pause = 0;

	if (phydev->autoneg == AUTONEG_ENABLE) {
		/* Read the link partner's 1G advertisement */
		ret = phy_read(phydev, MDIO_MMD_STD, MII_STAT1000);
		if (ret < 0)
			return ret;
		mii_stat1000_mod_linkmode_lpa_t(phydev->advertising, ret);
	} else {
		phydev->autoneg = AUTONEG_DISABLE;
	}

	ret = phy_read(phydev, MDIO_MMD_STD, PHY_MIISTAT);
	if (ret < 0)
		return ret;

	phydev->link = (ret & PHY_MIISTAT_LS) ? 1 : 0;
	phydev->duplex = (ret & PHY_MIISTAT_DPX) ? DUPLEX_FULL : DUPLEX_HALF;
	duplex = (phydev->duplex == DUPLEX_FULL) ? "F" : "H";
	switch (FIELD_GET(PHY_MIISTAT_SPD_MASK, ret)) {
	case PHY_MIISTAT_SPD_10:
		phydev->speed = SPEED_10;
		speed = "10";
		break;
	case PHY_MIISTAT_SPD_100:
		phydev->speed = SPEED_100;
		speed = "100";
		break;
	case PHY_MIISTAT_SPD_1000:
		phydev->speed = SPEED_1000;
		speed = "1000";
		break;
	case PHY_MIISTAT_SPD_2500:
		phydev->speed = SPEED_2500;
		speed = "2500";
		break;
	}

	if (phydev->link) {
		ret = gpy211_update_interface(phydev);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int gpy211_startup(struct phy_device *phydev)
{
	int ret;

	ret = genphy_update_link(phydev);
	if (ret)
		return ret;

	return gpy211_read_status(phydev);
}

static int gpy211_shutdown(struct phy_device *phydev)
{
	return 0;
}

U_BOOT_PHY_DRIVER(gpy211) = {
	.name = "Intel GPY211 PHY",
	.uid = 0x67c9de0a,
	.mask = 0x0ffffff0,
	.features = PHY_GBIT_FEATURES,
	.probe = &gpy211_probe,
	.config = &gpy211_phy_config,
	.startup = &gpy211_startup,
	.shutdown = &gpy211_shutdown,
	//.get_features	= &gpy211_get_features,
};
