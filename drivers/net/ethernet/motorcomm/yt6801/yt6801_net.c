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

static void fxgmac_phy_release(struct fxgmac_pdata *priv)
{
	FXGMAC_IO_WR_BITS(priv, EPHY_CTRL, RESET, 1);
	fsleep(100);
}

void fxgmac_phy_reset(struct fxgmac_pdata *priv)
{
	FXGMAC_IO_WR_BITS(priv, EPHY_CTRL, RESET, 0);
	fsleep(1500);
}

#ifdef CONFIG_PCI_MSI
static void fxgmac_init_interrupt_scheme(struct fxgmac_pdata *priv)
{
	struct pci_dev *pdev = to_pci_dev(priv->dev);
	int req_vectors = FXGMAC_MAX_DMA_CHANNELS;

	/* Since we have FXGMAC_MAX_DMA_CHANNELS channels, we must
	 *  ensure the number of cpu core is ok. otherwise, just roll back to legacy.
	 */
	if (num_online_cpus() < FXGMAC_MAX_DMA_CHANNELS - 1)
		goto enable_msi_interrupt;

	priv->msix_entries =
		kcalloc(req_vectors, sizeof(struct msix_entry), GFP_KERNEL);
	if (!priv->msix_entries)
		goto enable_msi_interrupt;

	for (u32 i = 0; i < req_vectors; i++)
		priv->msix_entries[i].entry = i;

	if (pci_enable_msix_exact(pdev, priv->msix_entries, req_vectors) < 0) {
		/* Roll back to msi */
		kfree(priv->msix_entries);
		priv->msix_entries = NULL;
		yt_err(priv, "enable MSIx err, clear msix entries.\n");
		goto enable_msi_interrupt;
	}

	FXGMAC_SET_BITS(priv->int_flag, INT_FLAG, INTERRUPT, BIT(INT_FLAG_MSIX_POS));
	priv->per_channel_irq = 1;
	return;

enable_msi_interrupt:
	if (pci_enable_msi(pdev) < 0) {
		FXGMAC_SET_BITS(priv->int_flag, INT_FLAG, INTERRUPT, BIT(INT_FLAG_LEGACY_POS));
		yt_err(priv, "MSI err, rollback to LEGACY.\n");
	} else {
		FXGMAC_SET_BITS(priv->int_flag, INT_FLAG, INTERRUPT, BIT(INT_FLAG_MSI_POS));
		priv->dev_irq = pdev->irq;
	}
}
#endif

int fxgmac_drv_probe(struct device *dev, struct fxgmac_resources *res)
{
	struct fxgmac_pdata *priv;
	struct net_device *netdev;
	int ret;

	netdev = alloc_etherdev_mq(sizeof(struct fxgmac_pdata),
				   FXGMAC_MAX_DMA_RX_CHANNELS);
	if (!netdev)
		return -ENOMEM;

	SET_NETDEV_DEV(netdev, dev);
	priv = netdev_priv(netdev);

	priv->dev = dev;
	priv->netdev = netdev;
	priv->dev_irq = res->irq;
	priv->hw_addr = res->addr;
	priv->msg_enable = NETIF_MSG_DRV;
	priv->dev_state = FXGMAC_DEV_PROBE;

	/* Default to legacy interrupt */
	FXGMAC_SET_BITS(priv->int_flag, INT_FLAG, INTERRUPT, BIT(INT_FLAG_LEGACY_POS));
	pci_set_drvdata(to_pci_dev(priv->dev), priv);

	if (IS_ENABLED(CONFIG_PCI_MSI))
		fxgmac_init_interrupt_scheme(priv);

	ret = fxgmac_init(priv, true);
	if (ret < 0) {
		yt_err(priv, "fxgmac_init err:%d\n", ret);
		goto err_free_netdev;
	}

	fxgmac_phy_reset(priv);
	fxgmac_phy_release(priv);
	ret = fxgmac_mdio_register(priv);
	if (ret < 0) {
		yt_err(priv, "fxgmac_mdio_register err:%d\n", ret);
		goto err_free_netdev;
	}

	netif_carrier_off(netdev);
	ret = register_netdev(netdev);
	if (ret) {
		yt_err(priv, "register_netdev err:%d\n", ret);
		goto err_free_netdev;
	}

	return 0;

err_free_netdev:
	free_netdev(netdev);
	return ret;
}
