// SPDX-License-Identifier: GPL-2.0+
/* Copyright (c) 2022 - 2024 Motorcomm Electronic Technology Co.,Ltd. */

#include <linux/inetdevice.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <net/addrconf.h>
#include <linux/inet.h>
#include <linux/tcp.h>

#include "yt6801.h"
#include "yt6801_desc.h"

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

static void fxgmac_disable_mgm_irq(struct fxgmac_pdata *priv)
{
	FXGMAC_IO_WR_BITS(priv, MGMT_INT_CTRL0, INT_MASK,
			  MGMT_INT_CTRL0_INT_MASK_MASK);
}

static void napi_disable_del(struct fxgmac_pdata *priv, struct napi_struct *n,
			     u32 flag_pos)
{
	napi_disable(n);
	netif_napi_del(n);
	SET_BITS(priv->int_flag, flag_pos, 1, 0); /* set flag_pos bit to 0 */
}

static void fxgmac_napi_disable(struct fxgmac_pdata *priv)
{
	u32 rx = FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, RX_NAPI);
	struct fxgmac_channel *channel = priv->channel_head;

	if (!priv->per_channel_irq) {
		if (!FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, LEGACY_NAPI))
			return;

		napi_disable_del(priv, &priv->napi,
				 INT_FLAG_LEGACY_NAPI_POS);
		return;
	}

	if (FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, TX_NAPI))
		napi_disable_del(priv, &channel->napi_tx,
				 INT_FLAG_TX_NAPI_POS);

	for (u32 i = 0; i < priv->channel_count; i++, channel++)
		if (GET_BITS(rx, i, INT_FLAG_PER_RX_NAPI_LEN))
			napi_disable_del(priv, &channel->napi_rx,
					 INT_FLAG_RX_NAPI_POS + i);
}

static void fxgmac_free_irqs(struct fxgmac_pdata *priv)
{
	u32 i, rx = FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, RX_IRQ);
	struct fxgmac_channel *channel = priv->channel_head;

	if (!FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, MSIX) &&
	    FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, LEGACY_IRQ)) {
		devm_free_irq(priv->dev, priv->dev_irq, priv);
		FXGMAC_SET_BITS(priv->int_flag, INT_FLAG, LEGACY_IRQ, 0);
	}

	if (!priv->per_channel_irq)
		return;

	if (FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, TX_IRQ)) {
		FXGMAC_SET_BITS(priv->int_flag, INT_FLAG, TX_IRQ, 0);
		devm_free_irq(priv->dev, channel->dma_irq_tx, channel);
	}

	for (i = 0; i < priv->channel_count; i++, channel++) {
		if (GET_BITS(rx, i, INT_FLAG_PER_RX_IRQ_LEN)) {
			SET_BITS(priv->int_flag, INT_FLAG_RX_IRQ_POS + i,
				 INT_FLAG_PER_RX_IRQ_LEN, 0);
			devm_free_irq(priv->dev, channel->dma_irq_rx, channel);
		}
	}
}

static void fxgmac_free_tx_data(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;
	struct fxgmac_ring *ring;

	for (u32 i = 0; i < priv->channel_count; i++, channel++) {
		ring = channel->tx_ring;
		if (!ring)
			break;

		for (u32 j = 0; j < ring->dma_desc_count; j++)
			fxgmac_desc_data_unmap(priv,
					       FXGMAC_GET_DESC_DATA(ring, j));
	}
}

static void fxgmac_free_rx_data(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;
	struct fxgmac_ring *ring;

	for (u32 i = 0; i < priv->channel_count; i++, channel++) {
		ring = channel->rx_ring;
		if (!ring)
			break;

		for (u32 j = 0; j < ring->dma_desc_count; j++)
			fxgmac_desc_data_unmap(priv,
					       FXGMAC_GET_DESC_DATA(ring, j));
	}
}

static void fxgmac_prepare_tx_stop(struct fxgmac_pdata *priv,
				   struct fxgmac_channel *channel)
{
	unsigned int tx_q_idx, tx_status;
	unsigned int tx_dsr, tx_pos;
	unsigned long tx_timeout;

	/* Calculate the status register to read and the position within */
	if (channel->queue_index < DMA_DSRX_FIRST_QUEUE) {
		tx_dsr = DMA_DSR0;
		tx_pos = (channel->queue_index * DMA_DSR_Q_LEN) +
			 DMA_DSR0_TPS_START;
	} else {
		tx_q_idx = channel->queue_index - DMA_DSRX_FIRST_QUEUE;

		tx_dsr = DMA_DSR1 + ((tx_q_idx / DMA_DSRX_QPR) * DMA_DSRX_INC);
		tx_pos = ((tx_q_idx % DMA_DSRX_QPR) * DMA_DSR_Q_LEN) +
			 DMA_DSRX_TPS_START;
	}

	/* The Tx engine cannot be stopped if it is actively processing
	 * descriptors. Wait for the Tx engine to enter the stopped or
	 * suspended state.
	 */
	tx_timeout = jiffies + (FXGMAC_DMA_STOP_TIMEOUT * HZ);

	while (time_before(jiffies, tx_timeout)) {
		tx_status = FXGMAC_MAC_IO_RD(priv, tx_dsr);
		tx_status = GET_BITS(tx_status, tx_pos, DMA_DSR_TPS_LEN);
		if (tx_status == DMA_TPS_STOPPED ||
		    tx_status == DMA_TPS_SUSPENDED)
			break;

		fsleep(500);
	}

	if (!time_before(jiffies, tx_timeout))
		yt_err(priv,
		       "timed out waiting for Tx DMA channel %u to stop\n",
		       channel->queue_index);
}

static void fxgmac_disable_tx(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;

	/* Prepare for Tx DMA channel stop */
	fxgmac_prepare_tx_stop(priv, channel);

	FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, TE, 0);/* Disable MAC Tx */

	/* Disable Tx queue */
	FXGMAC_MTL_IO_WR_BITS(priv, 0, MTL_Q_TQOMR, TXQEN, MTL_Q_DISABLED);

	/* Disable Tx DMA channel */
	FXGMAC_DMA_IO_WR_BITS(channel, DMA_CH_TCR, ST, 0);
}

static void fxgmac_prepare_rx_stop(struct fxgmac_pdata *priv,
				   unsigned int queue)
{
	unsigned int rx_status, rx_q, rx_q_sts;
	unsigned long rx_timeout;

	/* The Rx engine cannot be stopped if it is actively processing
	 * packets. Wait for the Rx queue to empty the Rx fifo.
	 */
	rx_timeout = jiffies + (FXGMAC_DMA_STOP_TIMEOUT * HZ);

	while (time_before(jiffies, rx_timeout)) {
		rx_status = FXGMAC_MTL_IO_RD(priv, queue, MTL_Q_RQDR);
		rx_q = FXGMAC_GET_BITS(rx_status, MTL_Q_RQDR, PRXQ);
		rx_q_sts = FXGMAC_GET_BITS(rx_status, MTL_Q_RQDR, RXQSTS);
		if (rx_q == 0 && rx_q_sts == 0)
			break;

		fsleep(500);
	}

	if (!time_before(jiffies, rx_timeout))
		yt_err(priv, "timed out waiting for Rx queue %u to empty\n",
		       queue);
}

static void fxgmac_disable_rx(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;
	u32 i;

	/* Disable MAC Rx */
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, CST, 0);
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, ACS, 0);
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, RE, 0);

	/* Prepare for Rx DMA channel stop */
	for (i = 0; i < priv->rx_q_count; i++)
		fxgmac_prepare_rx_stop(priv, i);

	FXGMAC_MAC_IO_WR(priv, MAC_RQC0R, 0); /* Disable each Rx queue */

	/* Disable each Rx DMA channel */
	for (i = 0; i < priv->channel_count; i++, channel++)
		FXGMAC_DMA_IO_WR_BITS(channel, DMA_CH_RCR, SR, 0);
}

/**
 * fxgmac_set_oob_wol - disable or enable oob wol crtl function
 * @priv: driver private struct
 * @enable: 1 or 0
 *
 * Description:  After enable OOB_WOL from efuse, mac will loopcheck phy status,
 *   and lead to panic sometimes. So we should disable it from powerup,
 *   enable it from power down.
 */
static void fxgmac_set_oob_wol(struct fxgmac_pdata *priv, unsigned int en)
{
	FXGMAC_IO_WR_BITS(priv, OOB_WOL_CTRL, DIS, !en);/* en = 1 is disable */
}

static void fxgmac_pre_powerdown(struct fxgmac_pdata *priv)
{
	fxgmac_set_oob_wol(priv, 1);
	fsleep(2000);
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

static void fxgmac_disable_msix_irqs(struct fxgmac_pdata *priv)
{
	for (u32 intid = 0; intid < MSIX_TBL_MAX_NUM; intid++)
		fxgmac_disable_msix_one_irq(priv, intid);
}

static void fxgmac_stop(struct fxgmac_pdata *priv)
{
	struct net_device *netdev = priv->netdev;
	struct netdev_queue *txq;

	if (priv->dev_state != FXGMAC_DEV_START)
		return;

	priv->dev_state = FXGMAC_DEV_STOP;

	if (priv->per_channel_irq)
		fxgmac_disable_msix_irqs(priv);
	else
		fxgmac_disable_mgm_irq(priv);

	netif_carrier_off(netdev);
	netif_tx_stop_all_queues(netdev);
	fxgmac_disable_tx(priv);
	fxgmac_disable_rx(priv);
	fxgmac_free_irqs(priv);
	fxgmac_napi_disable(priv);
	phy_stop(priv->phydev);

	txq = netdev_get_tx_queue(netdev, priv->channel_head->queue_index);
	netdev_tx_reset_queue(txq);
}

static void fxgmac_config_powerdown(struct fxgmac_pdata *priv)
{
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, RE, 1); /* Enable MAC Rx */
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, TE, 1); /* Enable MAC TX */

	/* Set GAMC power down */
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_PMT_STA, PWRDWN, 1);
}

int fxgmac_net_powerdown(struct fxgmac_pdata *priv)
{
	struct net_device *netdev = priv->netdev;

	/* Signal that we are down to the interrupt handler */
	if (__test_and_set_bit(FXGMAC_POWER_STATE_DOWN, &priv->powerstate))
		return 0; /* do nothing if already down */

	__clear_bit(FXGMAC_POWER_STATE_UP, &priv->powerstate);
	netif_tx_stop_all_queues(netdev); /* Shut off incoming Tx traffic */

	/* Call carrier off first to avoid false dev_watchdog timeouts */
	netif_carrier_off(netdev);
	netif_tx_disable(netdev);
	fxgmac_disable_rx(priv);

	/* Synchronize_rcu() needed for pending XDP buffers to drain */
	synchronize_rcu();

	fxgmac_stop(priv);
	fxgmac_pre_powerdown(priv);

	if (!test_bit(FXGMAC_POWER_STATE_DOWN, &priv->powerstate))
		yt_err(priv,
		       "fxgmac powerstate is %lu when config powe down.\n",
		       priv->powerstate);

	/* Set mac to lowpower mode */
	fxgmac_config_powerdown(priv);
	fxgmac_free_tx_data(priv);
	fxgmac_free_rx_data(priv);

	return 0;
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
