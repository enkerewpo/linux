// SPDX-License-Identifier: GPL-2.0+
/* Copyright (c) 2022 - 2024 Motorcomm Electronic Technology Co.,Ltd. */

#include <linux/inetdevice.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <net/addrconf.h>
#include <linux/inet.h>
#include <linux/tcp.h>

#include "yt6801.h"

#define PHY_WR_CONFIG(reg_offset)	(0x8000205 + ((reg_offset) * 0x10000))
static int fxgmac_phy_write_reg(struct fxgmac_pdata *priv, u32 reg_id, u32 data)
{
	u32 val;
	int ret;

	FXGMAC_MAC_IO_WR(priv, MAC_MDIO_DATA, data);
	FXGMAC_MAC_IO_WR(priv, MAC_MDIO_ADDRESS, PHY_WR_CONFIG(reg_id));
	ret = read_poll_timeout_atomic(FXGMAC_MAC_IO_RD, val,
				       !FXGMAC_GET_BITS(val, MAC_MDIO_ADDR, BUSY),
				       10, 250, false, priv, MAC_MDIO_ADDRESS);
	if (ret == -ETIMEDOUT) {
		yt_err(priv, "%s err, id:%x ctrl:0x%08x, data:0x%08x\n",
		       __func__, reg_id, PHY_WR_CONFIG(reg_id), data);
		return ret;
	}

	return ret;
}

#define PHY_RD_CONFIG(reg_offset)	(0x800020d + ((reg_offset) * 0x10000))
static int fxgmac_phy_read_reg(struct fxgmac_pdata *priv, u32 reg_id)
{
	u32 val;
	int ret;

	FXGMAC_MAC_IO_WR(priv, MAC_MDIO_ADDRESS, PHY_RD_CONFIG(reg_id));
	ret = read_poll_timeout_atomic(FXGMAC_MAC_IO_RD, val,
				       !FXGMAC_GET_BITS(val, MAC_MDIO_ADDR, BUSY),
				       10, 250, false, priv, MAC_MDIO_ADDRESS);
	if (ret == -ETIMEDOUT) {
		yt_err(priv, "%s err, id:%x, ctrl:0x%08x, val:0x%08x.\n",
		       __func__, reg_id, PHY_RD_CONFIG(reg_id), val);
		return ret;
	}

	return FXGMAC_MAC_IO_RD(priv, MAC_MDIO_DATA); /* Read data */
}

static int fxgmac_mdio_write_reg(struct mii_bus *mii_bus, int phyaddr,
				 int phyreg, u16 val)
{
	if (phyaddr > 0)
		return -ENODEV;

	return fxgmac_phy_write_reg(mii_bus->priv, phyreg, val);
}

static int fxgmac_mdio_read_reg(struct mii_bus *mii_bus, int phyaddr,
				int phyreg)
{
	if (phyaddr > 0)
		return -ENODEV;

	return fxgmac_phy_read_reg(mii_bus->priv, phyreg);
}

static int fxgmac_mdio_register(struct fxgmac_pdata *priv)
{
	struct pci_dev *pdev = to_pci_dev(priv->dev);
	struct phy_device *phydev;
	struct mii_bus *new_bus;
	int ret;

	new_bus = devm_mdiobus_alloc(&pdev->dev);
	if (!new_bus)
		return -ENOMEM;

	new_bus->name = "yt6801";
	new_bus->priv = priv;
	new_bus->parent = &pdev->dev;
	new_bus->read = fxgmac_mdio_read_reg;
	new_bus->write = fxgmac_mdio_write_reg;
	snprintf(new_bus->id, MII_BUS_ID_SIZE, "yt6801-%x-%x",
		 pci_domain_nr(pdev->bus), pci_dev_id(pdev));

	ret = devm_mdiobus_register(&pdev->dev, new_bus);
	if (ret < 0)
		return ret;

	phydev = mdiobus_get_phy(new_bus, 0);
	if (!phydev)
		return -ENODEV;

	priv->phydev = phydev;
	return 0;
}
