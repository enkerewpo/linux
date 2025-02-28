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

const struct net_device_ops *fxgmac_get_netdev_ops(void);
static void fxgmac_napi_enable(struct fxgmac_pdata *priv);

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

static void fxgmac_enable_msix_one_irq(struct fxgmac_pdata *priv, u32 int_id)
{
	FXGMAC_IO_WR(priv, MSIX_TBL_MASK + int_id * 16, 0);
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

static int fxgmac_request_irqs(struct fxgmac_pdata *priv)
{
	u32 rx, i = 0, msi = FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, MSI);
	struct fxgmac_channel *channel = priv->channel_head;
	struct net_device *netdev = priv->netdev;
	int ret;

	if (!FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, MSIX) &&
	    !FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, LEGACY_IRQ)) {
		FXGMAC_SET_BITS(priv->int_flag, INT_FLAG, LEGACY_IRQ, 1);
		ret = devm_request_irq(priv->dev, priv->dev_irq, fxgmac_isr,
				       msi ? 0 : IRQF_SHARED, netdev->name,
				       priv);
		if (ret) {
			yt_err(priv, "requesting irq:%d ,err:%d\n",
			       priv->dev_irq, ret);
			return ret;
		}
	}

	if (!priv->per_channel_irq)
		return 0;

	if (!FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, TX_IRQ)) {
		snprintf(channel->dma_irq_tx_name,
			 sizeof(channel->dma_irq_tx_name) - 1,
			 "%s-ch%d-Tx-%u", netdev_name(netdev), 0,
			 channel->queue_index);
		FXGMAC_SET_BITS(priv->int_flag, INT_FLAG, TX_IRQ, 1);
		ret = devm_request_irq(priv->dev, channel->dma_irq_tx,
				       fxgmac_dma_isr, 0,
				       channel->dma_irq_tx_name, channel);
		if (ret) {
			yt_err(priv, "requesting tx irq:%d ,err:%d\n",
			       channel->dma_irq_tx, ret);
			goto err_irq;
		}
	}

	rx = FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, RX_IRQ);
	for (i = 0; i < priv->channel_count; i++, channel++) {
		snprintf(channel->dma_irq_rx_name,
			 sizeof(channel->dma_irq_rx_name) - 1, "%s-ch%d-Rx-%u",
			 netdev_name(netdev), i, channel->queue_index);

		if (!GET_BITS(rx, i, INT_FLAG_PER_RX_IRQ_LEN)) {
			SET_BITS(priv->int_flag, INT_FLAG_RX_IRQ_POS + i,
				 INT_FLAG_PER_RX_IRQ_LEN, 1);
			ret = devm_request_irq(priv->dev, channel->dma_irq_rx,
					       fxgmac_dma_isr, 0,
					       channel->dma_irq_rx_name,
					       channel);
			if (ret) {
				yt_err(priv, "requesting rx irq:%d ,err:%d\n",
				       channel->dma_irq_rx, ret);
				goto err_irq;
			}
		}
	}

	return 0;

err_irq:
	fxgmac_free_irqs(priv);
	return ret;
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

static void fxgmac_enable_tx(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;

	/* Enable Tx DMA channel */
	FXGMAC_DMA_IO_WR_BITS(channel, DMA_CH_TCR, ST, 1);

	/* Enable Tx queue */
	FXGMAC_MTL_IO_WR_BITS(priv, 0, MTL_Q_TQOMR, TXQEN, MTL_Q_ENABLED);
	/* Enable MAC Tx */
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, TE, 1);
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

static void fxgmac_enable_rx(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;
	u32 val = 0, i;

	/* Enable each Rx DMA channel */
	for (i = 0; i < priv->channel_count; i++, channel++)
		FXGMAC_DMA_IO_WR_BITS(channel, DMA_CH_RCR, SR, 1);

	/* Enable each Rx queue */
	for (i = 0; i < priv->rx_q_count; i++)
		val |= (0x02 << (i << 1));

	FXGMAC_MAC_IO_WR(priv, MAC_RQC0R, val);

	/* Enable MAC Rx */
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, CST, 1);
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, ACS, 1);
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, RE, 1);
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

static void fxgmac_default_speed_duplex_config(struct fxgmac_pdata *priv)
{
	priv->mac_duplex = DUPLEX_FULL;
	priv->mac_speed = SPEED_1000;
}

static void fxgmac_config_mac_speed(struct fxgmac_pdata *priv)
{
	if (priv->mac_duplex == DUPLEX_UNKNOWN &&
	    priv->mac_speed == SPEED_UNKNOWN)
		fxgmac_default_speed_duplex_config(priv);

	FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, DM, priv->mac_duplex);

	switch (priv->mac_speed) {
	case SPEED_1000:
		FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, PS, 0);
		FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, FES, 0);
		break;
	case SPEED_100:
		FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, PS, 1);
		FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, FES, 1);
		break;
	case SPEED_10:
		FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, PS, 1);
		FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, FES, 0);
		break;
	default:
		WARN_ON(1);
		break;
	}
}

static void fxgmac_phylink_handler(struct net_device *ndev)
{
	struct fxgmac_pdata *priv = netdev_priv(ndev);

	priv->mac_speed = priv->phydev->speed;
	priv->mac_duplex = priv->phydev->duplex;

	if (priv->phydev->link) {
		fxgmac_config_mac_speed(priv);
		fxgmac_enable_rx(priv);
		fxgmac_enable_tx(priv);
		if (netif_running(priv->netdev))
			netif_tx_wake_all_queues(priv->netdev);
	} else {
		netif_tx_stop_all_queues(priv->netdev);
		fxgmac_disable_rx(priv);
		fxgmac_disable_tx(priv);
	}

	phy_print_status(priv->phydev);
}

static int fxgmac_phy_connect(struct fxgmac_pdata *priv)
{
	struct phy_device *phydev = priv->phydev;
	int ret;

	priv->phydev->irq = PHY_POLL;
	ret = phy_connect_direct(priv->netdev, phydev, fxgmac_phylink_handler,
				 PHY_INTERFACE_MODE_INTERNAL);
	if (ret)
		return ret;

	phy_support_asym_pause(phydev);
	priv->phydev->mac_managed_pm = 1;
	phy_attached_info(phydev);

	return 0;
}

static void fxgmac_enable_msix_irqs(struct fxgmac_pdata *priv)
{
	for (u32 intid = 0; intid < MSIX_TBL_MAX_NUM; intid++)
		fxgmac_enable_msix_one_irq(priv, intid);
}

static void fxgmac_enable_dma_interrupts(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;
	u32 ch_sr;

	for (u32 i = 0; i < priv->channel_count; i++, channel++) {
		/* Clear all the interrupts which are set */
		ch_sr = FXGMAC_DMA_IO_RD(channel, DMA_CH_SR);
		FXGMAC_DMA_IO_WR(channel, DMA_CH_SR, ch_sr);

		ch_sr = 0;
		/* Enable Normal Interrupt Summary Enable and Fatal Bus Error
		 * Enable interrupts.
		 */
		FXGMAC_SET_BITS(ch_sr, DMA_CH_IER, NIE, 1);
		FXGMAC_SET_BITS(ch_sr, DMA_CH_IER, FBEE, 1);

		/* only one tx, enable Transmit Interrupt Enable interrupts */
		if (i == 0 && channel->tx_ring)
			FXGMAC_SET_BITS(ch_sr, DMA_CH_IER, TIE, 1);

		if (channel->rx_ring) {
			/* Enable Receive Buffer Unavailable Enable and Receive
			 * Interrupt Enable interrupts.
			 */
			FXGMAC_SET_BITS(ch_sr, DMA_CH_IER, RBUE, 1);
			FXGMAC_SET_BITS(ch_sr, DMA_CH_IER, RIE, 1);
		}

		FXGMAC_DMA_IO_WR(channel, DMA_CH_IER, ch_sr);
	}
}

static void fxgmac_dismiss_all_int(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;
	u32 i;

	/* Clear all the interrupts which are set */
	for (i = 0; i < priv->channel_count; i++, channel++)
		FXGMAC_DMA_IO_WR(channel, DMA_CH_SR,
				 FXGMAC_DMA_IO_RD(channel, DMA_CH_SR));

	for (i = 0; i < priv->hw_feat.rx_q_cnt; i++)
		FXGMAC_MTL_IO_WR(priv, i, MTL_Q_ISR,
				 FXGMAC_MTL_IO_RD(priv, i, MTL_Q_ISR));

	FXGMAC_MAC_IO_RD(priv, MAC_ISR);      /* Clear all MAC interrupts */
	FXGMAC_MAC_IO_RD(priv, MAC_TX_RX_STA);/* Clear tx/rx error interrupts */
	FXGMAC_MAC_IO_RD(priv, MAC_PMT_STA);
	FXGMAC_MAC_IO_RD(priv, MAC_LPI_STA);

	FXGMAC_MAC_IO_WR(priv, MAC_DBG_STA,
			 FXGMAC_MAC_IO_RD(priv, MAC_DBG_STA));
}

static void fxgmac_set_interrupt_moderation(struct fxgmac_pdata *priv)
{
	FXGMAC_IO_WR_BITS(priv, INT_MOD, TX, priv->tx_usecs);
	FXGMAC_IO_WR_BITS(priv, INT_MOD, RX, priv->rx_usecs);
}

static void fxgmac_enable_mgm_irq(struct fxgmac_pdata *priv)
{
	FXGMAC_IO_WR_BITS(priv, MGMT_INT_CTRL0, INT_MASK,
			  MGMT_INT_CTRL0_INT_MASK_DISABLE);
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

static void fxgmac_config_powerup(struct fxgmac_pdata *priv)
{
	fxgmac_set_oob_wol(priv, 0);
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_PMT_STA, PWRDWN, 0); /* GAMC power up */
}

static void fxgmac_pre_powerdown(struct fxgmac_pdata *priv)
{
	fxgmac_set_oob_wol(priv, 1);
	fsleep(2000);
}

static void fxgmac_restore_nonstick_reg(struct fxgmac_pdata *priv)
{
	for (u32 i = GLOBAL_CTRL0; i < MSI_PBA; i += 4)
		FXGMAC_IO_WR(priv, i,
			     priv->reg_nonstick[(i - GLOBAL_CTRL0) >> 2]);
}

static void fxgmac_phy_release(struct fxgmac_pdata *priv)
{
	FXGMAC_IO_WR_BITS(priv, EPHY_CTRL, RESET, 1);
	fsleep(100);
}

static void fxgmac_hw_exit(struct fxgmac_pdata *priv)
{
	/* Reset CHIP, it will reset trigger circuit and reload efuse patch */
	FXGMAC_IO_WR_BITS(priv, SYS_RESET, RESET, 1);
	fsleep(9000);

	fxgmac_phy_release(priv);

	/* Reset will clear nonstick registers. */
	fxgmac_restore_nonstick_reg(priv);
}

static void fxgmac_pcie_init(struct fxgmac_pdata *priv)
{
	/* snoopy + non-snoopy */
	FXGMAC_IO_WR_BITS(priv, LTR_IDLE_ENTER, REQUIRE,
			  LTR_IDLE_ENTER_REQUIRE);
	FXGMAC_IO_WR_BITS(priv, LTR_IDLE_ENTER, SCALE,
			  LTR_IDLE_ENTER_SCALE_1024_NS);
	FXGMAC_IO_WR_BITS(priv, LTR_IDLE_ENTER, ENTER, LTR_IDLE_ENTER_900_US);

	/* snoopy + non-snoopy */
	FXGMAC_IO_WR_BITS(priv, LTR_IDLE_EXIT, REQUIRE, LTR_IDLE_EXIT_REQUIRE);
	FXGMAC_IO_WR_BITS(priv, LTR_IDLE_EXIT, SCALE, LTR_IDLE_EXIT_SCALE);
	FXGMAC_IO_WR_BITS(priv, LTR_IDLE_EXIT, EXIT, LTR_IDLE_EXIT_171_US);

	FXGMAC_IO_WR_BITS(priv, PCIE_SERDES_PLL, AUTOOFF, 1);
}

void fxgmac_phy_reset(struct fxgmac_pdata *priv)
{
	FXGMAC_IO_WR_BITS(priv, EPHY_CTRL, RESET, 0);
	fsleep(1500);
}

static int fxgmac_start(struct fxgmac_pdata *priv)
{
	int ret;

	if (priv->dev_state != FXGMAC_DEV_OPEN &&
	    priv->dev_state != FXGMAC_DEV_STOP &&
	    priv->dev_state != FXGMAC_DEV_RESUME) {
		return 0;
	}

	if (priv->dev_state != FXGMAC_DEV_STOP) {
		fxgmac_phy_reset(priv);
		fxgmac_phy_release(priv);
	}

	if (priv->dev_state == FXGMAC_DEV_OPEN) {
		ret = fxgmac_phy_connect(priv);
		if (ret < 0)
			return ret;
	}

	fxgmac_pcie_init(priv);
	if (test_bit(FXGMAC_POWER_STATE_DOWN, &priv->powerstate)) {
		yt_err(priv, "fxgmac powerstate is %lu when config power up.\n",
		       priv->powerstate);
	}

	fxgmac_config_powerup(priv);
	fxgmac_dismiss_all_int(priv);
	ret = fxgmac_hw_init(priv);
	if (ret < 0) {
		yt_err(priv, "fxgmac hw init error.\n");
		return ret;
	}

	fxgmac_napi_enable(priv);
	ret = fxgmac_request_irqs(priv);
	if (ret < 0)
		return ret;

	/* Config interrupt to level signal */
	FXGMAC_MAC_IO_WR_BITS(priv, DMA_MR, INTM, 2);
	FXGMAC_MAC_IO_WR_BITS(priv, DMA_MR, QUREAD, 1);

	fxgmac_enable_mgm_irq(priv);
	fxgmac_set_interrupt_moderation(priv);

	if (priv->per_channel_irq)
		fxgmac_enable_msix_irqs(priv);

	fxgmac_enable_dma_interrupts(priv);
	priv->dev_state = FXGMAC_DEV_START;
	phy_start(priv->phydev);

	return 0;
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

static void fxgmac_restart(struct fxgmac_pdata *priv)
{
	int ret;

	/* If not running, "restart" will happen on open */
	if (!netif_running(priv->netdev) && priv->dev_state != FXGMAC_DEV_START)
		return;

	mutex_lock(&priv->mutex);
	fxgmac_stop(priv);
	fxgmac_free_tx_data(priv);
	fxgmac_free_rx_data(priv);
	ret = fxgmac_start(priv);
	if (ret < 0)
		yt_err(priv, "%s err, ret = %d.\n", __func__, ret);

	mutex_unlock(&priv->mutex);
}

static void fxgmac_restart_work(struct work_struct *work)
{
	rtnl_lock();
	fxgmac_restart(container_of(work, struct fxgmac_pdata, restart_work));
	rtnl_unlock();
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

static int fxgmac_calc_rx_buf_size(struct fxgmac_pdata *priv, unsigned int mtu)
{
	u32 rx_buf_size, max_mtu = FXGMAC_JUMBO_PACKET_MTU - ETH_HLEN;

	if (mtu > max_mtu) {
		yt_err(priv, "MTU exceeds maximum supported value\n");
		return -EINVAL;
	}

	rx_buf_size = mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;
	rx_buf_size =
		clamp_val(rx_buf_size, FXGMAC_RX_MIN_BUF_SIZE, PAGE_SIZE * 4);

	rx_buf_size = (rx_buf_size + FXGMAC_RX_BUF_ALIGN - 1) &
		      ~(FXGMAC_RX_BUF_ALIGN - 1);

	return rx_buf_size;
}

static int fxgmac_open(struct net_device *netdev)
{
	struct fxgmac_pdata *priv = netdev_priv(netdev);
	int ret;

	mutex_lock(&priv->mutex);
	priv->dev_state = FXGMAC_DEV_OPEN;

	/* Calculate the Rx buffer size before allocating rings */
	ret = fxgmac_calc_rx_buf_size(priv, netdev->mtu);
	if (ret < 0)
		goto unlock;

	priv->rx_buf_size = ret;
	ret = fxgmac_channels_rings_alloc(priv);
	if (ret < 0)
		goto unlock;

	INIT_WORK(&priv->restart_work, fxgmac_restart_work);
	ret = fxgmac_start(priv);
	if (ret < 0)
		goto err_channels_and_rings;

	mutex_unlock(&priv->mutex);
	return 0;

err_channels_and_rings:
	fxgmac_channels_rings_free(priv);
	yt_err(priv, "%s, channel alloc err\n", __func__);
unlock:
	mutex_unlock(&priv->mutex);
	return ret;
}

#define EFUSE_FISRT_UPDATE_ADDR				255
#define EFUSE_SECOND_UPDATE_ADDR			209
#define EFUSE_MAX_ENTRY					39
#define EFUSE_PATCH_ADDR_START				0
#define EFUSE_PATCH_DATA_START				2
#define EFUSE_PATCH_SIZE				6
#define EFUSE_REGION_A_B_LENGTH				18

static bool fxgmac_efuse_read_data(struct fxgmac_pdata *priv, u32 offset,
				   u8 *value)
{
	u32 val = 0, wait = 1000;
	bool ret = false;

	FXGMAC_SET_BITS(val, EFUSE_OP, ADDR, offset);
	FXGMAC_SET_BITS(val, EFUSE_OP, START, 1);
	FXGMAC_SET_BITS(val, EFUSE_OP, MODE, EFUSE_OP_MODE_ROW_READ);
	FXGMAC_IO_WR(priv, EFUSE_OP_CTRL_0, val);

	while (wait--) {
		fsleep(20);
		val = FXGMAC_IO_RD(priv, EFUSE_OP_CTRL_1);
		if (FXGMAC_GET_BITS(val, EFUSE_OP, DONE)) {
			ret = true;
			break;
		}
	}

	if (!ret) {
		yt_err(priv, "Fail to reading efuse Byte%d\n", offset);
		return ret;
	}

	if (value)
		*value = FXGMAC_GET_BITS(val, EFUSE_OP, RD_DATA) & 0xff;

	return ret;
}

static bool fxgmac_efuse_read_index_patch(struct fxgmac_pdata *priv, u8 index,
					  u32 *offset, u32 *value)
{
	u8 tmp[EFUSE_PATCH_SIZE - EFUSE_PATCH_DATA_START];
	u32 addr, i;
	bool ret;

	if (index >= EFUSE_MAX_ENTRY) {
		yt_err(priv, "Reading efuse out of range, index %d\n", index);
		return false;
	}

	for (i = EFUSE_PATCH_ADDR_START; i < EFUSE_PATCH_DATA_START; i++) {
		addr = EFUSE_REGION_A_B_LENGTH + index * EFUSE_PATCH_SIZE + i;
		ret = fxgmac_efuse_read_data(priv, addr,
					     tmp + i - EFUSE_PATCH_ADDR_START);
		if (!ret) {
			yt_err(priv, "Fail to reading efuse Byte%d\n", addr);
			return ret;
		}
	}
	/* tmp[0] is low 8bit date, tmp[1] is high 8bit date */
	if (offset)
		*offset = tmp[0] | (tmp[1] << 8);

	for (i = EFUSE_PATCH_DATA_START; i < EFUSE_PATCH_SIZE; i++) {
		addr = EFUSE_REGION_A_B_LENGTH + index * EFUSE_PATCH_SIZE + i;
		ret = fxgmac_efuse_read_data(priv, addr,
					     tmp + i - EFUSE_PATCH_DATA_START);
		if (!ret) {
			yt_err(priv, "Fail to reading efuse Byte%d\n", addr);
			return ret;
		}
	}
	/* tmp[0] is low 8bit date, tmp[1] is low 8bit date
	 * ...  tmp[3] is highest 8bit date
	 */
	if (value)
		*value = tmp[0] | (tmp[1] << 8) | (tmp[2] << 16) |
			 (tmp[3] << 24);

	return ret;
}

static bool fxgmac_efuse_read_mac_subsys(struct fxgmac_pdata *priv,
					 u8 *mac_addr, u32 *subsys, u32 *revid)
{
	u32 machr = 0, maclr = 0, offset = 0, val = 0;

	for (u8 index = 0; index < EFUSE_MAX_ENTRY; index++) {
		if (!fxgmac_efuse_read_index_patch(priv, index, &offset, &val))
			return false;

		if (offset == 0x00)
			break; /* Reach the blank. */
		if (offset == MACA0LR_FROM_EFUSE)
			maclr = val;
		if (offset == MACA0HR_FROM_EFUSE)
			machr = val;
		if (offset == PCI_REVISION_ID && revid)
			*revid = val;
		if (offset == PCI_SUBSYSTEM_VENDOR_ID && subsys)
			*subsys = val;
	}

	if (mac_addr) {
		mac_addr[5] = (u8)(maclr & 0xFF);
		mac_addr[4] = (u8)((maclr >> 8) & 0xFF);
		mac_addr[3] = (u8)((maclr >> 16) & 0xFF);
		mac_addr[2] = (u8)((maclr >> 24) & 0xFF);
		mac_addr[1] = (u8)(machr & 0xFF);
		mac_addr[0] = (u8)((machr >> 8) & 0xFF);
	}

	return true;
}

static int fxgmac_read_mac_addr(struct fxgmac_pdata *priv)
{
	u8 default_addr[ETH_ALEN] = { 0, 0x55, 0x7b, 0xb5, 0x7d, 0xf7 };
	struct net_device *netdev = priv->netdev;
	int ret;

	/* If efuse have mac addr, use it. if not, use static mac address. */
	ret = fxgmac_efuse_read_mac_subsys(priv, priv->mac_addr, NULL, NULL);
	if (!ret)
		return -1;

	if (is_zero_ether_addr(priv->mac_addr))
		/* Use a static mac address for test */
		memcpy(priv->mac_addr, default_addr, netdev->addr_len);

	return 0;
}

static void fxgmac_default_config(struct fxgmac_pdata *priv)
{
	priv->sysclk_rate = 125000000; /* System clock is 125 MHz */
	priv->tx_threshold = MTL_TX_THRESHOLD_128;
	priv->rx_threshold = MTL_RX_THRESHOLD_128;
	priv->tx_osp_mode = DMA_OSP_ENABLE;
	priv->tx_sf_mode = MTL_TSF_ENABLE;
	priv->rx_sf_mode = MTL_RSF_ENABLE;
	priv->pblx8 = DMA_PBL_X8_ENABLE;
	priv->tx_pbl = DMA_PBL_16;
	priv->rx_pbl = DMA_PBL_4;
	priv->tx_pause = 1;	/* Enable tx pause */
	priv->rx_pause = 1;	/* Enable rx pause */

	fxgmac_default_speed_duplex_config(priv);
}

static void fxgmac_get_all_hw_features(struct fxgmac_pdata *priv)
{
	struct fxgmac_hw_features *hw_feat = &priv->hw_feat;
	unsigned int mac_hfr0, mac_hfr1, mac_hfr2, mac_hfr3;

	mac_hfr0 = FXGMAC_MAC_IO_RD(priv, MAC_HWF0R);
	mac_hfr1 = FXGMAC_MAC_IO_RD(priv, MAC_HWF1R);
	mac_hfr2 = FXGMAC_MAC_IO_RD(priv, MAC_HWF2R);
	mac_hfr3 = FXGMAC_MAC_IO_RD(priv, MAC_HWF3R);
	memset(hw_feat, 0, sizeof(*hw_feat));
	hw_feat->version = FXGMAC_MAC_IO_RD(priv, MAC_VR);

	/* Hardware feature register 0 */
	hw_feat->phyifsel = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, ACTPHYIFSEL);
	hw_feat->vlhash = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, VLHASH);
	hw_feat->sma = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, SMASEL);
	hw_feat->rwk = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, RWKSEL);
	hw_feat->mgk = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, MGKSEL);
	hw_feat->mmc = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, MMCSEL);
	hw_feat->aoe = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, ARPOFFSEL);
	hw_feat->ts = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, TSSEL);
	hw_feat->eee = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, EEESEL);
	hw_feat->tx_coe = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, TXCOESEL);
	hw_feat->rx_coe = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, RXCOESEL);
	hw_feat->addn_mac = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, ADDMACADRSEL);
	hw_feat->ts_src = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, TSSTSSEL);
	hw_feat->sa_vlan_ins = FXGMAC_GET_BITS(mac_hfr0, MAC_HWF0R, SAVLANINS);

	/* Hardware feature register 1 */
	hw_feat->rx_fifo_size =
		FXGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, RXFIFOSIZE);
	hw_feat->tx_fifo_size =
		FXGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, TXFIFOSIZE);
	hw_feat->adv_ts_hi = FXGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, ADVTHWORD);
	hw_feat->dma_width = FXGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, ADDR64);
	hw_feat->dcb = FXGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, DCBEN);
	hw_feat->sph = FXGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, SPHEN);
	hw_feat->tso = FXGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, TSOEN);
	hw_feat->dma_debug = FXGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, DBGMEMA);
	hw_feat->avsel = FXGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, RAVSEL);
	hw_feat->ravsel = FXGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, RAVSEL);
	hw_feat->hash_table_size =
		FXGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, HASHTBLSZ);
	hw_feat->l3l4_filter_num =
		FXGMAC_GET_BITS(mac_hfr1, MAC_HWF1R, L3L4FNUM);
	hw_feat->tx_q_cnt = FXGMAC_GET_BITS(mac_hfr2, MAC_HWF2R, TXQCNT);
	hw_feat->rx_ch_cnt = FXGMAC_GET_BITS(mac_hfr2, MAC_HWF2R, RXCHCNT);
	hw_feat->tx_ch_cnt = FXGMAC_GET_BITS(mac_hfr2, MAC_HWF2R, TXCHCNT);
	hw_feat->pps_out_num = FXGMAC_GET_BITS(mac_hfr2, MAC_HWF2R, PPSOUTNUM);
	hw_feat->aux_snap_num =
		FXGMAC_GET_BITS(mac_hfr2, MAC_HWF2R, AUXSNAPNUM);

	/* Translate the Hash Table size into actual number */
	switch (hw_feat->hash_table_size) {
	case 0:
		break;
	case 1:
		hw_feat->hash_table_size = 64;
		break;
	case 2:
		hw_feat->hash_table_size = 128;
		break;
	case 3:
		hw_feat->hash_table_size = 256;
		break;
	}

	/* Translate the address width setting into actual number */
	switch (hw_feat->dma_width) {
	case 0:
		hw_feat->dma_width = 32;
		break;
	case 1:
		hw_feat->dma_width = 40;
		break;
	case 2:
		hw_feat->dma_width = 48;
		break;
	default:
		hw_feat->dma_width = 32;
	}

	/* The Queue, Channel are zero based so increment them
	 * to get the actual number
	 */
	hw_feat->tx_q_cnt++;
	hw_feat->rx_ch_cnt++;
	hw_feat->tx_ch_cnt++;

	/* HW implement 1 rx fifo, 4 dma channel.  but from software
	 * we see 4 logical queues. hardcode to 4 queues.
	 */
	hw_feat->rx_q_cnt = 4;

	hw_feat->hwfr3 = mac_hfr3;
}

static unsigned int fxgmac_usec_to_riwt(struct fxgmac_pdata *priv,
					unsigned int usec)
{
	/* Convert the input usec value to the watchdog timer value. Each
	 * watchdog timer value is equivalent to 256 clock cycles.
	 * Calculate the required value as:
	 *  ( usec * ( system_clock_mhz / 10^6) / 256
	 */
	return (usec * (priv->sysclk_rate / 1000000)) / 256;
}

static void fxgmac_save_nonstick_reg(struct fxgmac_pdata *priv)
{
	for (u32 i = GLOBAL_CTRL0; i < MSI_PBA; i += 4) {
		priv->reg_nonstick[(i - GLOBAL_CTRL0) >> 2] =
			FXGMAC_IO_RD(priv, i);
	}
}

static int fxgmac_init(struct fxgmac_pdata *priv, bool save_private_reg)
{
	struct net_device *netdev = priv->netdev;
	int ret;

	fxgmac_default_config(priv);	/* Set default configuration data */
	netdev->irq = priv->dev_irq;
	netdev->base_addr = (unsigned long)priv->hw_addr;

	ret = fxgmac_read_mac_addr(priv);
	if (ret) {
		yt_err(priv, "fxgmac_read_mac_addr err:%d\n", ret);
		return ret;
	}
	eth_hw_addr_set(netdev, priv->mac_addr);

	if (save_private_reg)
		fxgmac_save_nonstick_reg(priv);

	fxgmac_hw_exit(priv);	/* Reset here to get hw features correctly */
	fxgmac_get_all_hw_features(priv);

	/* Set the DMA mask */
	ret = dma_set_mask_and_coherent(priv->dev,
					DMA_BIT_MASK(priv->hw_feat.dma_width));
	if (ret) {
		ret = dma_set_mask_and_coherent(priv->dev, DMA_BIT_MASK(32));
		if (ret) {
			yt_err(priv, "No usable DMA configuration, aborting\n");
			return ret;
		}
	}

	if (FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, LEGACY)) {
		/* We should disable msi and msix here when we use legacy
		 * interrupt,for two reasons:
		 * 1. Exit will restore msi and msix config regisiter,
		 * that may enable them.
		 * 2. When the driver that uses the msix interrupt by default
		 * is compiled into the OS, uninstall the driver through rmmod,
		 * and then install the driver that uses the legacy interrupt,
		 * at which time the msix enable will be turned on again by
		 * default after waking up from S4 on some
		 * platform. such as UOS platform.
		 */
		pci_disable_msi(to_pci_dev(priv->dev));
		pci_disable_msix(to_pci_dev(priv->dev));
	}

	BUILD_BUG_ON_NOT_POWER_OF_2(FXGMAC_TX_DESC_CNT);
	priv->tx_desc_count = FXGMAC_TX_DESC_CNT;
	BUILD_BUG_ON_NOT_POWER_OF_2(FXGMAC_RX_DESC_CNT);
	priv->rx_desc_count = FXGMAC_RX_DESC_CNT;

	ret = netif_set_real_num_tx_queues(netdev, FXGMAC_TX_1_Q);
	if (ret) {
		yt_err(priv, "error setting real tx queue count\n");
		return ret;
	}

	priv->rx_ring_count = min_t(unsigned int,
				    netif_get_num_default_rss_queues(),
				    priv->hw_feat.rx_ch_cnt);
	priv->rx_ring_count = min_t(unsigned int, priv->rx_ring_count,
				    priv->hw_feat.rx_q_cnt);
	priv->rx_q_count = priv->rx_ring_count;
	ret = netif_set_real_num_rx_queues(netdev, priv->rx_q_count);
	if (ret) {
		yt_err(priv, "error setting real rx queue count\n");
		return ret;
	}

	priv->channel_count =
		max_t(unsigned int, FXGMAC_TX_1_RING, priv->rx_ring_count);

	netdev->min_mtu = ETH_MIN_MTU;
	netdev->max_mtu =
		FXGMAC_JUMBO_PACKET_MTU + (ETH_HLEN + VLAN_HLEN + ETH_FCS_LEN);

	netdev->netdev_ops = fxgmac_get_netdev_ops();/* Set device operations */

	/* Set device features */
	if (priv->hw_feat.tso) {
		netdev->hw_features = NETIF_F_TSO;
		netdev->hw_features |= NETIF_F_TSO6;
		netdev->hw_features |= NETIF_F_SG;
		netdev->hw_features |= NETIF_F_IP_CSUM;
		netdev->hw_features |= NETIF_F_IPV6_CSUM;
	} else if (priv->hw_feat.tx_coe) {
		netdev->hw_features = NETIF_F_IP_CSUM;
		netdev->hw_features |= NETIF_F_IPV6_CSUM;
	}

	if (priv->hw_feat.rx_coe) {
		netdev->hw_features |= NETIF_F_RXCSUM;
		netdev->hw_features |= NETIF_F_GRO;
	}

	netdev->hw_features |= NETIF_F_RXHASH;
	netdev->vlan_features |= netdev->hw_features;
	netdev->hw_features |= NETIF_F_HW_VLAN_CTAG_RX;

	if (priv->hw_feat.sa_vlan_ins)
		netdev->hw_features |= NETIF_F_HW_VLAN_CTAG_TX;

	netdev->features |= netdev->hw_features;
	priv->netdev_features = netdev->features;

	netdev->priv_flags |= IFF_UNICAST_FLT;
	netdev->watchdog_timeo = msecs_to_jiffies(5000);

#define NIC_MAX_TCP_OFFLOAD_SIZE 7300
	netif_set_tso_max_size(netdev, NIC_MAX_TCP_OFFLOAD_SIZE);

/* Default coalescing parameters */
#define FXGMAC_INIT_DMA_TX_USECS INT_MOD_200_US
#define FXGMAC_INIT_DMA_TX_FRAMES 25
#define FXGMAC_INIT_DMA_RX_USECS INT_MOD_200_US
#define FXGMAC_INIT_DMA_RX_FRAMES 25

	/* Tx coalesce parameters initialization */
	priv->tx_usecs = FXGMAC_INIT_DMA_TX_USECS;
	priv->tx_frames = FXGMAC_INIT_DMA_TX_FRAMES;

	/* Rx coalesce parameters initialization */
	priv->rx_riwt = fxgmac_usec_to_riwt(priv, FXGMAC_INIT_DMA_RX_USECS);
	priv->rx_usecs = FXGMAC_INIT_DMA_RX_USECS;
	priv->rx_frames = FXGMAC_INIT_DMA_RX_FRAMES;

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

static const struct net_device_ops fxgmac_netdev_ops = {
	.ndo_open		= fxgmac_open,
};

const struct net_device_ops *fxgmac_get_netdev_ops(void)
{
	return &fxgmac_netdev_ops;
}

static void napi_add_enable(struct fxgmac_pdata *priv, struct napi_struct *napi,
			    int (*poll)(struct napi_struct *, int),
			    u32 flag_pos)
{
	netif_napi_add(priv->netdev, napi, poll);
	napi_enable(napi);
	SET_BITS(priv->int_flag, flag_pos, 1, 1); /* set flag_pos bit to 1 */
}

static void fxgmac_napi_enable(struct fxgmac_pdata *priv)
{
	u32 rx = FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, RX_NAPI);
	struct fxgmac_channel *channel = priv->channel_head;

	if (!priv->per_channel_irq) {
		if (FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, LEGACY_NAPI))
			return;

		napi_add_enable(priv, &priv->napi, fxgmac_all_poll,
				INT_FLAG_LEGACY_NAPI_POS);
		return;
	}

	if (!FXGMAC_GET_BITS(priv->int_flag, INT_FLAG, TX_NAPI))
		napi_add_enable(priv, &channel->napi_tx, fxgmac_one_poll_tx,
				INT_FLAG_TX_NAPI_POS);

	for (u32 i = 0; i < priv->channel_count; i++, channel++)
		if (!(GET_BITS(rx, i, INT_FLAG_PER_RX_NAPI_LEN)))
			napi_add_enable(priv, &channel->napi_rx,
					fxgmac_one_poll_rx,
					INT_FLAG_RX_NAPI_POS + i);
}
