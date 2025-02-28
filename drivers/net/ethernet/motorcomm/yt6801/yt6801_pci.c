// SPDX-License-Identifier: GPL-2.0+
/* Copyright (c) 2022 - 2024 Motorcomm Electronic Technology Co.,Ltd.
 *
 * Below is a simplified block diagram of YT6801 chip and its relevant
 * interfaces.
 *                      ||
 *  ********************++**********************
 *  *            | PCIE Endpoint |             *
 *  *            +---------------+             *
 *  *                | GMAC |                  *
 *  *                +--++--+                  *
 *  *                  |**|                    *
 *  *         GMII --> |**| <-- MDIO           *
 *  *                 +-++--+                  *
 *  *            | Integrated PHY |  YT8531S   *
 *  *                 +-++-+                   *
 *  ********************||******************* **
 */

#include <linux/kernel.h>
#include <linux/module.h>

#ifdef CONFIG_PCI_MSI
#include <linux/pci.h>
#endif

#include "yt6801.h"

static int fxgmac_probe(struct pci_dev *pcidev, const struct pci_device_id *id)
{
	struct device *dev = &pcidev->dev;
	struct fxgmac_resources res;
	int i, ret;

	ret = pcim_enable_device(pcidev);
	if (ret) {
		dev_err(dev, "%s pcim_enable_device err:%d\n", __func__, ret);
		return ret;
	}

	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		if (pci_resource_len(pcidev, i) == 0)
			continue;

		ret = pcim_iomap_regions(pcidev, BIT(i), FXGMAC_DRV_NAME);
		if (ret) {
			dev_err(dev, "%s, pcim_iomap_regions err:%d\n",
				__func__, ret);
			return ret;
		}
		break;
	}

	pci_set_master(pcidev);

	memset(&res, 0, sizeof(res));
	res.irq = pcidev->irq;
	res.addr = pcim_iomap_table(pcidev)[i];

	return fxgmac_drv_probe(&pcidev->dev, &res);
}

static void fxgmac_remove(struct pci_dev *pcidev)
{
	struct fxgmac_pdata *priv = dev_get_drvdata(&pcidev->dev);
	struct net_device *netdev = priv->netdev;
	struct device *dev = &pcidev->dev;

	unregister_netdev(netdev);
	fxgmac_phy_reset(priv);
	free_netdev(netdev);

	if (IS_ENABLED(CONFIG_PCI_MSI) &&
	    FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, MSIX)) {
		pci_disable_msix(pcidev);
		kfree(priv->msix_entries);
		priv->msix_entries = NULL;
	}

	dev_dbg(dev, "%s has been removed\n", netdev->name);
}

static void __fxgmac_shutdown(struct pci_dev *pcidev)
{
	struct fxgmac_pdata *priv = dev_get_drvdata(&pcidev->dev);
	struct net_device *netdev = priv->netdev;

	rtnl_lock();
	fxgmac_net_powerdown(priv);
	netif_device_detach(netdev);
	rtnl_unlock();
}

static void fxgmac_shutdown(struct pci_dev *pcidev)
{
	struct fxgmac_pdata *priv = dev_get_drvdata(&pcidev->dev);

	mutex_lock(&priv->mutex);
	 __fxgmac_shutdown(pcidev);
	if (system_state == SYSTEM_POWER_OFF) {
		pci_wake_from_d3(pcidev, false);
		pci_set_power_state(pcidev, PCI_D3hot);
	}
	mutex_unlock(&priv->mutex);
}

static int fxgmac_suspend(struct device *device)
{
	struct fxgmac_pdata *priv = dev_get_drvdata(device);
	struct net_device *netdev = priv->netdev;
	int ret = 0;

	mutex_lock(&priv->mutex);
	if (priv->dev_state != FXGMAC_DEV_START)
		goto unlock;

	if (netif_running(netdev))
		__fxgmac_shutdown(to_pci_dev(device));

	priv->dev_state = FXGMAC_DEV_SUSPEND;
unlock:
	mutex_unlock(&priv->mutex);

	return ret;
}

static int fxgmac_resume(struct device *device)
{
	struct fxgmac_pdata *priv = dev_get_drvdata(device);
	struct net_device *netdev = priv->netdev;
	int ret = 0;

	mutex_lock(&priv->mutex);
	if (priv->dev_state != FXGMAC_DEV_SUSPEND)
		goto unlock;

	priv->dev_state = FXGMAC_DEV_RESUME;
	__clear_bit(FXGMAC_POWER_STATE_DOWN, &priv->powerstate);

	rtnl_lock();
	if (netif_running(netdev)) {
		ret = fxgmac_net_powerup(priv);
		if (ret < 0) {
			dev_err(device, "%s, fxgmac_net_powerup err:%d\n",
				__func__, ret);
			goto unlock;
		}
	}

	netif_device_attach(netdev);
	rtnl_unlock();

unlock:
	mutex_unlock(&priv->mutex);

	return ret;
}

#define MOTORCOMM_PCI_ID			0x1f0a
#define YT6801_PCI_DEVICE_ID			0x6801

static const struct pci_device_id fxgmac_pci_tbl[] = {
	{ PCI_DEVICE(MOTORCOMM_PCI_ID, YT6801_PCI_DEVICE_ID) },
	{ 0 }
};

MODULE_DEVICE_TABLE(pci, fxgmac_pci_tbl);

static const struct dev_pm_ops fxgmac_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(fxgmac_suspend, fxgmac_resume)
};

static struct pci_driver fxgmac_pci_driver = {
	.name		= FXGMAC_DRV_NAME,
	.id_table	= fxgmac_pci_tbl,
	.probe		= fxgmac_probe,
	.remove		= fxgmac_remove,
	.driver.pm	= pm_ptr(&fxgmac_pm_ops),
	.shutdown	= fxgmac_shutdown,
};

module_pci_driver(fxgmac_pci_driver);

MODULE_AUTHOR("Motorcomm Electronic Tech. Co., Ltd.");
MODULE_DESCRIPTION(FXGMAC_DRV_DESC);
MODULE_LICENSE("GPL");
