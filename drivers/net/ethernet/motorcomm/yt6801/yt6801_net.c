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

static void fxgmac_tx_start_xmit(struct fxgmac_channel *channel,
				 struct fxgmac_ring *ring)
{
	struct fxgmac_desc_data *desc_data;

	wmb();  /* Make sure everything is written before the register write */

	/* Issue a poll command to Tx DMA by writing address
	 * of next immediate free descriptor
	 */
	desc_data = FXGMAC_GET_DESC_DATA(ring, ring->cur);
	FXGMAC_DMA_IO_WR(channel, DMA_CH_TDTR_LO,
			 lower_32_bits(desc_data->dma_desc_addr));

	ring->tx.xmit_more = 0;
}

static unsigned int fxgmac_desc_tx_avail(struct fxgmac_ring *ring)
{
	if (ring->dirty > ring->cur)
		return ring->dirty - ring->cur;
	else
		return ring->dma_desc_count - ring->cur + ring->dirty;
}

static netdev_tx_t fxgmac_maybe_stop_tx_queue(struct fxgmac_channel *channel,
					      struct fxgmac_ring *ring,
					      unsigned int count)
{
	struct fxgmac_pdata *priv = channel->priv;

	if (count > fxgmac_desc_tx_avail(ring)) {
		yt_err(priv, "Tx queue stopped, not enough descriptors available\n");
		netif_stop_subqueue(priv->netdev, channel->queue_index);
		ring->tx.queue_stopped = 1;

		/* If we haven't notified the hardware because of xmit_more
		 * support, tell it now
		 */
		if (ring->tx.xmit_more)
			fxgmac_tx_start_xmit(channel, ring);

		return NETDEV_TX_BUSY;
	}

	return NETDEV_TX_OK;
}

static void fxgmac_enable_msix_one_irq(struct fxgmac_pdata *priv, u32 int_id)
{
	FXGMAC_IO_WR(priv, MSIX_TBL_MASK + int_id * 16, 0);
}

static void fxgmac_disable_msix_one_irq(struct fxgmac_pdata *priv, u32 intid)
{
	FXGMAC_IO_WR(priv, MSIX_TBL_MASK + intid * 16, 1);
}

static void fxgmac_disable_mgm_irq(struct fxgmac_pdata *priv)
{
	FXGMAC_IO_WR_BITS(priv, MGMT_INT_CTRL0, INT_MASK,
			  MGMT_INT_CTRL0_INT_MASK_MASK);
}

static irqreturn_t fxgmac_isr(int irq, void *data)
{
	struct fxgmac_pdata *priv = data;
	u32 val;

	val = FXGMAC_IO_RD(priv, MGMT_INT_CTRL0);
	if (!(val & MGMT_INT_CTRL0_INT_STATUS_RXTXMISC))
		return IRQ_NONE;

	/* Restart the device on a Fatal Bus Error */
	for (u32 i = 0; i < priv->channel_count; i++) {
		val = FXGMAC_DMA_IO_RD(priv->channel_head + i, DMA_CH_SR);
		if (FXGMAC_GET_BITS(val, DMA_CH_SR, FBE))
			schedule_work(&priv->restart_work);
	}

	fxgmac_disable_mgm_irq(priv);
	napi_schedule_irqoff(&priv->napi); /* Turn on polling */
	return IRQ_HANDLED;
}

static irqreturn_t fxgmac_dma_isr(int irq, void *data)
{
	struct fxgmac_channel *channel = data;

	if (irq == channel->dma_irq_tx) {
		fxgmac_disable_msix_one_irq(channel->priv, MSI_ID_TXQ0);
		/* Clear Tx signal */
		FXGMAC_DMA_IO_WR(channel, DMA_CH_SR, BIT(DMA_CH_SR_TI_POS));
		napi_schedule_irqoff(&channel->napi_tx);
		return IRQ_HANDLED;
	}

	fxgmac_disable_msix_one_irq(channel->priv, channel->queue_index);
	/* Clear Rx signal */
	FXGMAC_DMA_IO_WR(channel, DMA_CH_SR, BIT(DMA_CH_SR_RI_POS));
	napi_schedule_irqoff(&channel->napi_rx);
	return IRQ_HANDLED;
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

static void __fxgmac_set_mac_address(struct fxgmac_pdata *priv, u8 *addr)
{
	u32 mac_hi, mac_lo;

	mac_lo = (u32)addr[0] | ((u32)addr[1] << 8) | ((u32)addr[2] << 16) |
		 ((u32)addr[3] << 24);

	mac_hi = (u32)addr[4] | ((u32)addr[5] << 8);

	FXGMAC_MAC_IO_WR(priv, MAC_MACA0LR, mac_lo);
	FXGMAC_MAC_IO_WR(priv, MAC_MACA0HR, mac_hi);
}

static void fxgmac_config_mac_address(struct fxgmac_pdata *priv)
{
	__fxgmac_set_mac_address(priv, priv->mac_addr);
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_PFR, HPF, 1);
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_PFR, HUC, 1);
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_PFR, HMC, 1);
}

static void fxgmac_config_crc_check_en(struct fxgmac_pdata *priv)
{
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_ECR, DCRCC, 1);
}

static void fxgmac_config_checksum_offload(struct fxgmac_pdata *priv)
{
	if (priv->netdev->features & NETIF_F_RXCSUM)
		FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, IPC, 1);
	else
		FXGMAC_MAC_IO_WR_BITS(priv, MAC_CR, IPC, 0);
}

static void fxgmac_set_promiscuous_mode(struct fxgmac_pdata *priv,
					unsigned int enable)
{
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_PFR, PR, enable);
}

static void fxgmac_enable_rx_broadcast(struct fxgmac_pdata *priv,
				       unsigned int enable)
{
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_PFR, DBF, enable);
}

static void fxgmac_set_all_multicast_mode(struct fxgmac_pdata *priv,
					  unsigned int enable)
{
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_PFR, PM, enable);
}

static void fxgmac_config_rx_mode(struct fxgmac_pdata *priv)
{
	u32 pr_mode, am_mode, bd_mode;

	pr_mode = ((priv->netdev->flags & IFF_PROMISC) != 0);
	am_mode = ((priv->netdev->flags & IFF_ALLMULTI) != 0);
	bd_mode = ((priv->netdev->flags & IFF_BROADCAST) != 0);

	fxgmac_enable_rx_broadcast(priv, bd_mode);
	fxgmac_set_promiscuous_mode(priv, pr_mode);
	fxgmac_set_all_multicast_mode(priv, am_mode);
}

static void fxgmac_config_tx_flow_control(struct fxgmac_pdata *priv)
{
	/* Set MTL flow control */
	for (u32 i = 0; i < priv->rx_q_count; i++)
		FXGMAC_MTL_IO_WR_BITS(priv, i, MTL_Q_RQOMR, EHFC,
				      priv->tx_pause);

	/* Set MAC flow control */
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_Q0TFCR, TFE, priv->tx_pause);

	if (priv->tx_pause == 1) /* Set pause time */
		FXGMAC_MAC_IO_WR_BITS(priv, MAC_Q0TFCR, PT, 0xffff);
}

static void fxgmac_config_rx_flow_control(struct fxgmac_pdata *priv)
{
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_RFCR, RFE, priv->rx_pause);
}

static void fxgmac_config_rx_coalesce(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;

	for (u32 i = 0; i < priv->channel_count; i++, channel++) {
		if (!channel->rx_ring)
			break;
		FXGMAC_DMA_IO_WR_BITS(channel, DMA_CH_RIWT, RWT, priv->rx_riwt);
	}
}

static void fxgmac_config_rx_fep_disable(struct fxgmac_pdata *priv)
{
	/* Enable the rx queue forward packet with error status
	 * (crc error,gmii_er, watch dog timeout.or overflow)
	 */
	for (u32 i = 0; i < priv->rx_q_count; i++)
		FXGMAC_MTL_IO_WR_BITS(priv, i, MTL_Q_RQOMR, FEP, 1);
}

static void fxgmac_config_rx_fup_enable(struct fxgmac_pdata *priv)
{
	for (u32 i = 0; i < priv->rx_q_count; i++)
		FXGMAC_MTL_IO_WR_BITS(priv, i, MTL_Q_RQOMR, FUP, 1);
}

static void fxgmac_config_rx_buffer_size(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;

	for (u32 i = 0; i < priv->channel_count; i++, channel++)
		FXGMAC_DMA_IO_WR_BITS(channel, DMA_CH_RCR, RBSZ,
				      priv->rx_buf_size);
}

static void fxgmac_config_tso_mode(struct fxgmac_pdata *priv)
{
	FXGMAC_DMA_IO_WR_BITS(priv->channel_head, DMA_CH_TCR, TSE,
			      priv->hw_feat.tso);
}

static void fxgmac_config_sph_mode(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;

	for (u32 i = 0; i < priv->channel_count; i++, channel++)
		FXGMAC_DMA_IO_WR_BITS(channel, DMA_CH_CR, SPH, 0);

	FXGMAC_MAC_IO_WR_BITS(priv, MAC_ECR, HDSMS, MAC_ECR_HDSMS_512B);
}

static void fxgmac_config_rx_threshold(struct fxgmac_pdata *priv,
				       unsigned int set_val)
{
	for (u32 i = 0; i < priv->rx_q_count; i++)
		FXGMAC_MTL_IO_WR_BITS(priv, i, MTL_Q_RQOMR, RTC, set_val);
}

static void fxgmac_config_mtl_mode(struct fxgmac_pdata *priv)
{
	/* Set Tx to weighted round robin scheduling algorithm */
	FXGMAC_MAC_IO_WR_BITS(priv, MTL_OMR, ETSALG, MTL_ETSALG_WRR);

	/* Set Tx traffic classes to use WRR algorithm with equal weights */
	FXGMAC_MTL_IO_WR_BITS(priv, 0, MTL_TC_QWR, QW, 1);

	/* Set Rx to strict priority algorithm */
	FXGMAC_MAC_IO_WR_BITS(priv, MTL_OMR, RAA, MTL_RAA_SP);
}

static void fxgmac_config_queue_mapping(struct fxgmac_pdata *priv)
{
	unsigned int ppq, ppq_extra, prio_queues;
	unsigned int __maybe_unused prio;
	unsigned int reg, val, mask;

	/* Map the 8 VLAN priority values to available MTL Rx queues */
	prio_queues =
		min_t(unsigned int, IEEE_8021QAZ_MAX_TCS, priv->rx_q_count);
	ppq = IEEE_8021QAZ_MAX_TCS / prio_queues;
	ppq_extra = IEEE_8021QAZ_MAX_TCS % prio_queues;

	reg = MAC_RQC2R;
	for (u32 i = 0, prio = 0; i < prio_queues;) {
		val = 0;
		mask = 0;
		for (u32 j = 0; j < ppq; j++) {
			mask |= (1 << prio);
			prio++;
		}

		if (i < ppq_extra) {
			mask |= (1 << prio);
			prio++;
		}

		val |= (mask << ((i++ % MAC_RQC2_Q_PER_REG) << 3));

		if ((i % MAC_RQC2_Q_PER_REG) && i != prio_queues)
			continue;

		FXGMAC_MAC_IO_WR(priv, reg, val);
		reg += MAC_RQC2_INC;
	}

	/* Configure one to one, MTL Rx queue to DMA Rx channel mapping
	 * ie Q0 <--> CH0, Q1 <--> CH1 ... Q7 <--> CH7
	 */
	val = FXGMAC_MAC_IO_RD(priv, MTL_RQDCM0R);
	val |= (MTL_RQDCM0R_Q0MDMACH | MTL_RQDCM0R_Q1MDMACH |
		MTL_RQDCM0R_Q2MDMACH | MTL_RQDCM0R_Q3MDMACH);
	FXGMAC_MAC_IO_WR(priv, MTL_RQDCM0R, val);

	val = FXGMAC_MAC_IO_RD(priv, MTL_RQDCM0R + MTL_RQDCM_INC);
	val |= (MTL_RQDCM1R_Q4MDMACH | MTL_RQDCM1R_Q5MDMACH |
		MTL_RQDCM1R_Q6MDMACH | MTL_RQDCM1R_Q7MDMACH);
	FXGMAC_MAC_IO_WR(priv, MTL_RQDCM0R + MTL_RQDCM_INC, val);
}

static unsigned int fxgmac_calculate_per_queue_fifo(unsigned int fifo_size,
						    unsigned int queue_count)
{
	u32 q_fifo_size, p_fifo;

	/* Calculate the configured fifo size */
	q_fifo_size = 1 << (fifo_size + 7);

#define FXGMAC_MAX_FIFO 81920
	/* The configured value may not be the actual amount of fifo RAM */
	q_fifo_size = min_t(unsigned int, FXGMAC_MAX_FIFO, q_fifo_size);
	q_fifo_size = q_fifo_size / queue_count;

	/* Each increment in the queue fifo size represents 256 bytes of
	 * fifo, with 0 representing 256 bytes. Distribute the fifo equally
	 * between the queues.
	 */
	p_fifo = q_fifo_size / 256;
	if (p_fifo)
		p_fifo--;

	return p_fifo;
}

static void fxgmac_config_tx_fifo_size(struct fxgmac_pdata *priv)
{
	u32 fifo_size;

	fifo_size = fxgmac_calculate_per_queue_fifo(priv->hw_feat.tx_fifo_size,
						    FXGMAC_TX_1_Q);
	FXGMAC_MTL_IO_WR_BITS(priv, 0, MTL_Q_TQOMR, TQS, fifo_size);
}

static void fxgmac_config_rx_fifo_size(struct fxgmac_pdata *priv)
{
	u32 fifo_size;

	fifo_size = fxgmac_calculate_per_queue_fifo(priv->hw_feat.rx_fifo_size,
						    priv->rx_q_count);

	for (u32 i = 0; i < priv->rx_q_count; i++)
		FXGMAC_MTL_IO_WR_BITS(priv, i, MTL_Q_RQOMR, RQS, fifo_size);
}

static void fxgmac_config_flow_control_threshold(struct fxgmac_pdata *priv)
{
	for (u32 i = 0; i < priv->rx_q_count; i++) {
		/* Activate flow control when less than 4k left in fifo */
		FXGMAC_MTL_IO_WR_BITS(priv, i, MTL_Q_RQOMR, RFA, 6);
		/* De-activate flow control when more than 6k left in fifo */
		FXGMAC_MTL_IO_WR_BITS(priv, i, MTL_Q_RQOMR, RFD, 10);
	}
}

static void fxgmac_config_tx_threshold(struct fxgmac_pdata *priv,
				       unsigned int set_val)
{
	FXGMAC_MTL_IO_WR_BITS(priv, 0, MTL_Q_TQOMR, TTC, set_val);
}

static void fxgmac_config_rsf_mode(struct fxgmac_pdata *priv,
				   unsigned int set_val)
{
	for (u32 i = 0; i < priv->rx_q_count; i++)
		FXGMAC_MTL_IO_WR_BITS(priv, i, MTL_Q_RQOMR, RSF, set_val);
}

static void fxgmac_config_tsf_mode(struct fxgmac_pdata *priv,
				   unsigned int set_val)
{
	FXGMAC_MTL_IO_WR_BITS(priv, 0, MTL_Q_TQOMR, TSF, set_val);
}

static void fxgmac_config_osp_mode(struct fxgmac_pdata *priv)
{
	FXGMAC_DMA_IO_WR_BITS(priv->channel_head, DMA_CH_TCR, OSP,
			      priv->tx_osp_mode);
}

static void fxgmac_config_pblx8(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;

	for (u32 i = 0; i < priv->channel_count; i++, channel++)
		FXGMAC_DMA_IO_WR_BITS(channel, DMA_CH_CR, PBLX8, priv->pblx8);
}

static void fxgmac_config_tx_pbl_val(struct fxgmac_pdata *priv)
{
	FXGMAC_DMA_IO_WR_BITS(priv->channel_head, DMA_CH_TCR, PBL,
			      priv->tx_pbl);
}

static void fxgmac_config_rx_pbl_val(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;

	for (u32 i = 0; i < priv->channel_count; i++, channel++)
		FXGMAC_DMA_IO_WR_BITS(channel, DMA_CH_RCR, PBL, priv->rx_pbl);
}

static void fxgmac_config_mmc(struct fxgmac_pdata *priv)
{
	/* Set counters to reset on read, Reset the counters */
	FXGMAC_MAC_IO_WR_BITS(priv, MMC_CR, ROR, 1);
	FXGMAC_MAC_IO_WR_BITS(priv, MMC_CR, CR, 1);

	FXGMAC_MAC_IO_WR(priv, MMC_IPC_RXINT_MASK, 0xffffffff);
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

static void fxgmac_enable_mtl_interrupts(struct fxgmac_pdata *priv)
{
	unsigned int mtl_q_isr;

	for (u32 i = 0; i < priv->hw_feat.rx_q_cnt; i++) {
		/* Clear all the interrupts which are set */
		mtl_q_isr = FXGMAC_MTL_IO_RD(priv, i, MTL_Q_ISR);
		FXGMAC_MTL_IO_WR(priv, i, MTL_Q_ISR, mtl_q_isr);

		/* No MTL interrupts to be enabled */
		FXGMAC_MTL_IO_WR(priv, i, MTL_Q_IER, 0);
	}
}

static void fxgmac_enable_mac_interrupts(struct fxgmac_pdata *priv)
{
	/* Disable Timestamp interrupt */
	FXGMAC_MAC_IO_WR_BITS(priv, MAC_IER, TSIE, 0);

	FXGMAC_MAC_IO_WR_BITS(priv, MMC_RIER, ALL_INTERRUPTS, 0);
	FXGMAC_MAC_IO_WR_BITS(priv, MMC_TIER, ALL_INTERRUPTS, 0);
}

static int fxgmac_flush_tx_queues(struct fxgmac_pdata *priv)
{
	u32 val, count = 2000;

	FXGMAC_MTL_IO_WR_BITS(priv, 0, MTL_Q_TQOMR, FTQ, 1);
	do {
		fsleep(20);
		val = FXGMAC_MTL_IO_RD(priv, 0, MTL_Q_TQOMR);
		val = FXGMAC_GET_BITS(val, MTL_Q_TQOMR, FTQ);

	} while (--count && val);

	if (val)
		return -EBUSY;

	return 0;
}

static void fxgmac_config_dma_bus(struct fxgmac_pdata *priv)
{
	u32 val = FXGMAC_MAC_IO_RD(priv, DMA_SBMR);

	/* Set enhanced addressing mode */
	FXGMAC_SET_BITS(val, DMA_SBMR, EAME, 1);

	/* Out standing read/write requests */
	FXGMAC_SET_BITS(val, DMA_SBMR, RD_OSR_LMT, 0x7);
	FXGMAC_SET_BITS(val, DMA_SBMR, WR_OSR_LMT, 0x7);

	/* Set the System Bus mode */
	FXGMAC_SET_BITS(val, DMA_SBMR, FB, 0);
	FXGMAC_SET_BITS(val, DMA_SBMR, BLEN_4, 1);
	FXGMAC_SET_BITS(val, DMA_SBMR, BLEN_8, 1);
	FXGMAC_SET_BITS(val, DMA_SBMR, BLEN_16, 1);
	FXGMAC_SET_BITS(val, DMA_SBMR, BLEN_32, 1);

	FXGMAC_MAC_IO_WR(priv, DMA_SBMR, val);
}

static void fxgmac_desc_rx_channel_init(struct fxgmac_channel *channel)
{
	struct fxgmac_ring *ring = channel->rx_ring;
	unsigned int start_index = ring->cur;
	struct fxgmac_desc_data *desc_data;

	/* Initialize all descriptors */
	for (u32 i = 0; i < ring->dma_desc_count; i++) {
		desc_data = FXGMAC_GET_DESC_DATA(ring, i);
		fxgmac_desc_rx_reset(desc_data); /* Initialize Rx descriptor */
	}

	/* Update the total number of Rx descriptors */
	FXGMAC_DMA_IO_WR(channel, DMA_CH_RDRLR, ring->dma_desc_count - 1);

	/* Update the starting address of descriptor ring */
	desc_data = FXGMAC_GET_DESC_DATA(ring, start_index);
	FXGMAC_DMA_IO_WR(channel, DMA_CH_RDLR_HI,
			 upper_32_bits(desc_data->dma_desc_addr));
	FXGMAC_DMA_IO_WR(channel, DMA_CH_RDLR_LO,
			 lower_32_bits(desc_data->dma_desc_addr));

	/* Update the Rx Descriptor Tail Pointer */
	desc_data = FXGMAC_GET_DESC_DATA(ring, start_index +
					 ring->dma_desc_count - 1);
	FXGMAC_DMA_IO_WR(channel, DMA_CH_RDTR_LO,
			 lower_32_bits(desc_data->dma_desc_addr));
}

static void fxgmac_desc_rx_init(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;
	struct fxgmac_desc_data *desc_data;
	struct fxgmac_dma_desc *dma_desc;
	dma_addr_t dma_desc_addr;
	struct fxgmac_ring *ring;

	for (u32 i = 0; i < priv->channel_count; i++, channel++) {
		ring = channel->rx_ring;
		dma_desc = ring->dma_desc_head;
		dma_desc_addr = ring->dma_desc_head_addr;

		for (u32 j = 0; j < ring->dma_desc_count; j++) {
			desc_data = FXGMAC_GET_DESC_DATA(ring, j);
			desc_data->dma_desc = dma_desc;
			desc_data->dma_desc_addr = dma_desc_addr;
			if (fxgmac_rx_buffe_map(priv, ring, desc_data))
				break;

			dma_desc++;
			dma_desc_addr += sizeof(struct fxgmac_dma_desc);
		}

		ring->cur = 0;
		ring->dirty = 0;

		fxgmac_desc_rx_channel_init(channel);
	}
}

static void fxgmac_desc_tx_channel_init(struct fxgmac_channel *channel)
{
	struct fxgmac_ring *ring = channel->tx_ring;
	struct fxgmac_desc_data *desc_data;
	int start_index = ring->cur;

	/* Initialize all descriptors */
	for (u32 i = 0; i < ring->dma_desc_count; i++) {
		desc_data = FXGMAC_GET_DESC_DATA(ring, i);
		fxgmac_desc_tx_reset(desc_data); /* Initialize Tx descriptor */
	}

	/* Update the total number of Tx descriptors */
	FXGMAC_DMA_IO_WR(channel, DMA_CH_TDRLR,
			 channel->priv->tx_desc_count - 1);

	/* Update the starting address of descriptor ring */
	desc_data = FXGMAC_GET_DESC_DATA(ring, start_index);
	FXGMAC_DMA_IO_WR(channel, DMA_CH_TDLR_HI,
			 upper_32_bits(desc_data->dma_desc_addr));
	FXGMAC_DMA_IO_WR(channel, DMA_CH_TDLR_LO,
			 lower_32_bits(desc_data->dma_desc_addr));
}

static void fxgmac_desc_tx_init(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;
	struct fxgmac_ring *ring = channel->tx_ring;
	struct fxgmac_desc_data *desc_data;
	struct fxgmac_dma_desc *dma_desc;
	dma_addr_t dma_desc_addr;

	dma_desc = ring->dma_desc_head;
	dma_desc_addr = ring->dma_desc_head_addr;

	for (u32 j = 0; j < ring->dma_desc_count; j++) {
		desc_data = FXGMAC_GET_DESC_DATA(ring, j);
		desc_data->dma_desc = dma_desc;
		desc_data->dma_desc_addr = dma_desc_addr;

		dma_desc++;
		dma_desc_addr += sizeof(struct fxgmac_dma_desc);
	}

	ring->cur = 0;
	ring->dirty = 0;
	memset(&ring->tx, 0, sizeof(ring->tx));
	fxgmac_desc_tx_channel_init(priv->channel_head);
}

static int fxgmac_hw_init(struct fxgmac_pdata *priv)
{
	int ret;

	ret = fxgmac_flush_tx_queues(priv); /* Flush Tx queues */
	if (ret < 0) {
		yt_err(priv, "%s, flush tx queue err:%d\n", __func__, ret);
		return ret;
	}

	/* Initialize DMA related features */
	fxgmac_config_dma_bus(priv);
	fxgmac_config_osp_mode(priv);
	fxgmac_config_pblx8(priv);
	fxgmac_config_tx_pbl_val(priv);
	fxgmac_config_rx_pbl_val(priv);
	fxgmac_config_rx_coalesce(priv);
	fxgmac_config_rx_buffer_size(priv);
	fxgmac_config_tso_mode(priv);
	fxgmac_config_sph_mode(priv);
	fxgmac_desc_tx_init(priv);
	fxgmac_desc_rx_init(priv);
	fxgmac_enable_dma_interrupts(priv);

	/* Initialize MTL related features */
	fxgmac_config_mtl_mode(priv);
	fxgmac_config_queue_mapping(priv);
	fxgmac_config_tsf_mode(priv, priv->tx_sf_mode);
	fxgmac_config_rsf_mode(priv, priv->rx_sf_mode);
	fxgmac_config_tx_threshold(priv, priv->tx_threshold);
	fxgmac_config_rx_threshold(priv, priv->rx_threshold);
	fxgmac_config_tx_fifo_size(priv);
	fxgmac_config_rx_fifo_size(priv);
	fxgmac_config_flow_control_threshold(priv);
	fxgmac_config_rx_fep_disable(priv);
	fxgmac_config_rx_fup_enable(priv);
	fxgmac_enable_mtl_interrupts(priv);

	/* Initialize MAC related features */
	fxgmac_config_mac_address(priv);
	fxgmac_config_crc_check_en(priv);
	fxgmac_config_rx_mode(priv);
	fxgmac_config_tx_flow_control(priv);
	fxgmac_config_rx_flow_control(priv);
	fxgmac_config_mac_speed(priv);
	fxgmac_config_checksum_offload(priv);
	fxgmac_config_mmc(priv);
	fxgmac_enable_mac_interrupts(priv);

	return 0;
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

int fxgmac_net_powerup(struct fxgmac_pdata *priv)
{
	int ret;

	priv->powerstate = 0;/* clear all bits as normal now */
	ret = fxgmac_start(priv);
	if (ret < 0) {
		yt_err(priv, "%s: fxgmac_start ret: %d\n", __func__, ret);
		return ret;
	}

	return 0;
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

static int fxgmac_close(struct net_device *netdev)
{
	struct fxgmac_pdata *priv = netdev_priv(netdev);

	mutex_lock(&priv->mutex);
	fxgmac_stop(priv); /* Stop the device */
	priv->dev_state = FXGMAC_DEV_CLOSE;
	fxgmac_channels_rings_free(priv); /* Free the channels and rings */
	fxgmac_phy_reset(priv);
	phy_disconnect(priv->phydev);
	mutex_unlock(&priv->mutex);
	return 0;
}

static void fxgmac_dump_state(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;
	struct fxgmac_ring *ring = &channel->tx_ring[0];

	yt_err(priv, "Tx descriptor info:\n");
	yt_err(priv, "Tx cur = 0x%x\n", ring->cur);
	yt_err(priv, "Tx dirty = 0x%x\n", ring->dirty);
	yt_err(priv, "Tx dma_desc_head = %pad\n", &ring->dma_desc_head);
	yt_err(priv, "Tx desc_data_head = %pad\n", &ring->desc_data_head);

	for (u32 i = 0; i < priv->channel_count; i++, channel++) {
		ring = &channel->rx_ring[0];
		yt_err(priv, "Rx[%d] descriptor info:\n", i);
		yt_err(priv, "Rx cur = 0x%x\n", ring->cur);
		yt_err(priv, "Rx dirty = 0x%x\n", ring->dirty);
		yt_err(priv, "Rx dma_desc_head = %pad\n", &ring->dma_desc_head);
		yt_err(priv, "Rx desc_data_head = %pad\n",
		       &ring->desc_data_head);
	}

	yt_err(priv, "Device Registers:\n");
	yt_err(priv, "MAC_ISR = %08x\n", FXGMAC_MAC_IO_RD(priv, MAC_ISR));
	yt_err(priv, "MAC_IER = %08x\n", FXGMAC_MAC_IO_RD(priv, MAC_IER));
	yt_err(priv, "MMC_RISR = %08x\n", FXGMAC_MAC_IO_RD(priv, MMC_RISR));
	yt_err(priv, "MMC_RIER = %08x\n", FXGMAC_MAC_IO_RD(priv, MMC_RIER));
	yt_err(priv, "MMC_TISR = %08x\n", FXGMAC_MAC_IO_RD(priv, MMC_TISR));
	yt_err(priv, "MMC_TIER = %08x\n", FXGMAC_MAC_IO_RD(priv, MMC_TIER));

	yt_err(priv, "EPHY_CTRL = %04x\n", FXGMAC_IO_RD(priv, EPHY_CTRL));
	yt_err(priv, "MGMT_INT_CTRL0 = %04x\n",
	       FXGMAC_IO_RD(priv, MGMT_INT_CTRL0));
	yt_err(priv, "MSIX_TBL_MASK = %04x\n",
	       FXGMAC_IO_RD(priv, MSIX_TBL_MASK));

	yt_err(priv, "Dump nonstick regs:\n");
	for (u32 i = GLOBAL_CTRL0; i < MSI_PBA; i += 4)
		yt_err(priv, "[%d] = %04x\n", i / 4, FXGMAC_IO_RD(priv, i));
}

static void fxgmac_tx_timeout(struct net_device *netdev, unsigned int unused)
{
	struct fxgmac_pdata *priv = netdev_priv(netdev);

	fxgmac_dump_state(priv);
	schedule_work(&priv->restart_work);
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

static void fxgmac_dbg_pkt(struct fxgmac_pdata *priv, struct sk_buff *skb,
			   bool tx_rx)
{
	struct ethhdr *eth = (struct ethhdr *)skb->data;
	unsigned char buffer[128];

	yt_dbg(priv, "\n************** SKB dump ****************\n");
	yt_dbg(priv, "%s, packet of %d bytes\n", (tx_rx ? "TX" : "RX"),
	       skb->len);
	yt_dbg(priv, "Dst MAC addr: %pM\n", eth->h_dest);
	yt_dbg(priv, "Src MAC addr: %pM\n", eth->h_source);
	yt_dbg(priv, "Protocol: %#06x\n", ntohs(eth->h_proto));

	for (u32 i = 0; i < skb->len; i += 32) {
		unsigned int len = min(skb->len - i, 32U);

		hex_dump_to_buffer(&skb->data[i], len, 32, 1, buffer,
				   sizeof(buffer), false);
		yt_dbg(priv, "  %#06x: %s\n", i, buffer);
	}

	yt_dbg(priv, "\n************** SKB dump ****************\n");
}

static void fxgmac_dev_xmit(struct fxgmac_channel *channel)
{
	struct fxgmac_pdata *priv = channel->priv;
	struct fxgmac_ring *ring = channel->tx_ring;
	unsigned int tso_context, vlan_context;
	struct fxgmac_desc_data *desc_data;
	struct fxgmac_dma_desc *dma_desc;
	struct fxgmac_pkt_info *pkt_info;
	unsigned int csum, tso, vlan;
	int i, start_index = ring->cur;
	int cur_index = ring->cur;

	pkt_info = &ring->pkt_info;
	csum = FXGMAC_GET_BITS(pkt_info->attr, ATTR_TX, CSUM_ENABLE);
	tso = FXGMAC_GET_BITS(pkt_info->attr, ATTR_TX, TSO_ENABLE);
	vlan = FXGMAC_GET_BITS(pkt_info->attr, ATTR_TX, VLAN_CTAG);

	if (tso && pkt_info->mss != ring->tx.cur_mss)
		tso_context = 1;
	else
		tso_context = 0;

	if (vlan && pkt_info->vlan_ctag != ring->tx.cur_vlan_ctag)
		vlan_context = 1;
	else
		vlan_context = 0;

	desc_data = FXGMAC_GET_DESC_DATA(ring, cur_index);
	dma_desc = desc_data->dma_desc;

	/* Create a context descriptor if this is a TSO pkt_info */
	if (tso_context) {
		/* Set the MSS size */
		FXGMAC_SET_BITS_LE(dma_desc->desc2, TX_CONTEXT_DESC2, MSS,
				   pkt_info->mss);

		/* Mark it as a CONTEXT descriptor */
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_CONTEXT_DESC3, CTXT, 1);

		/* Indicate this descriptor contains the MSS */
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_CONTEXT_DESC3, TCMSSV,
				   1);

		ring->tx.cur_mss = pkt_info->mss;
	}

	if (vlan_context) {
		/* Mark it as a CONTEXT descriptor */
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_CONTEXT_DESC3, CTXT, 1);

		/* Set the VLAN tag */
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_CONTEXT_DESC3, VT,
				   pkt_info->vlan_ctag);

		/* Indicate this descriptor contains the VLAN tag */
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_CONTEXT_DESC3, VLTV, 1);

		ring->tx.cur_vlan_ctag = pkt_info->vlan_ctag;
	}
	if (tso_context || vlan_context) {
		cur_index = FXGMAC_GET_ENTRY(cur_index, ring->dma_desc_count);
		desc_data = FXGMAC_GET_DESC_DATA(ring, cur_index);
		dma_desc = desc_data->dma_desc;
	}

	/* Update buffer address (for TSO this is the header) */
	dma_desc->desc0 = cpu_to_le32(lower_32_bits(desc_data->skb_dma));
	dma_desc->desc1 = cpu_to_le32(upper_32_bits(desc_data->skb_dma));

	/* Update the buffer length */
	FXGMAC_SET_BITS_LE(dma_desc->desc2, TX_NORMAL_DESC2, HL_B1L,
			   desc_data->skb_dma_len);

	/* VLAN tag insertion check */
	if (vlan)
		FXGMAC_SET_BITS_LE(dma_desc->desc2, TX_NORMAL_DESC2, VTIR,
				   TX_NORMAL_DESC2_VLAN_INSERT);

	/* Timestamp enablement check */
	if (FXGMAC_GET_BITS(pkt_info->attr, ATTR_TX, PTP))
		FXGMAC_SET_BITS_LE(dma_desc->desc2, TX_NORMAL_DESC2, TTSE, 1);

	/* Mark it as First Descriptor */
	FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3, FD, 1);

	/* Mark it as a NORMAL descriptor */
	FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3, CTXT, 0);

	/* Set OWN bit if not the first descriptor */
	if (cur_index != start_index)
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3, OWN, 1);

	if (tso) {
		/* Enable TSO */
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3, TSE, 1);
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3, TCPPL,
				   pkt_info->tcp_payload_len);
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3, TCPHDRLEN,
				   pkt_info->tcp_header_len / 4);
	} else {
		/* Enable CRC and Pad Insertion */
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3, CPC, 0);

		/* Enable HW CSUM */
		if (csum)
			FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3,
					   CIC, 0x3);

		/* Set the total length to be transmitted */
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3, FL,
				   pkt_info->length);
	}

	if (start_index <= cur_index)
		i = cur_index - start_index + 1;
	else
		i = ring->dma_desc_count - start_index + cur_index;

	for (; i < pkt_info->desc_count; i++) {
		cur_index = FXGMAC_GET_ENTRY(cur_index, ring->dma_desc_count);
		desc_data = FXGMAC_GET_DESC_DATA(ring, cur_index);
		dma_desc = desc_data->dma_desc;

		/* Update buffer address */
		dma_desc->desc0 =
			cpu_to_le32(lower_32_bits(desc_data->skb_dma));
		dma_desc->desc1 =
			cpu_to_le32(upper_32_bits(desc_data->skb_dma));

		/* Update the buffer length */
		FXGMAC_SET_BITS_LE(dma_desc->desc2, TX_NORMAL_DESC2, HL_B1L,
				   desc_data->skb_dma_len);

		/* Set OWN bit */
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3, OWN, 1);

		/* Mark it as NORMAL descriptor */
		FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3, CTXT, 0);

		/* Enable HW CSUM */
		if (csum)
			FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3,
					   CIC, 0x3);
	}

	/* Set LAST bit for the last descriptor */
	FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3, LD, 1);

	FXGMAC_SET_BITS_LE(dma_desc->desc2, TX_NORMAL_DESC2, IC, 1);

	/* Save the Tx info to report back during cleanup */
	desc_data->tx.packets = pkt_info->tx_packets;
	desc_data->tx.bytes = pkt_info->tx_bytes;

	/* In case the Tx DMA engine is running, make sure everything
	 * is written to the descriptor(s) before setting the OWN bit
	 * for the first descriptor
	 */
	dma_wmb();

	/* Set OWN bit for the first descriptor */
	desc_data = FXGMAC_GET_DESC_DATA(ring, start_index);
	dma_desc = desc_data->dma_desc;
	FXGMAC_SET_BITS_LE(dma_desc->desc3, TX_NORMAL_DESC3, OWN, 1);

	if (netif_msg_tx_queued(priv))
		fxgmac_dump_tx_desc(priv, ring, start_index,
				    pkt_info->desc_count, 1);

	smp_wmb();  /* Make sure ownership is written to the descriptor */

	ring->cur = FXGMAC_GET_ENTRY(cur_index, ring->dma_desc_count);
	fxgmac_tx_start_xmit(channel, ring);
}

static void fxgmac_prep_vlan(struct sk_buff *skb,
			     struct fxgmac_pkt_info *pkt_info)
{
	if (skb_vlan_tag_present(skb))
		pkt_info->vlan_ctag = skb_vlan_tag_get(skb);
}

static int fxgmac_prep_tso(struct fxgmac_pdata *priv, struct sk_buff *skb,
			   struct fxgmac_pkt_info *pkt_info)
{
	int ret;

	if (!FXGMAC_GET_BITS(pkt_info->attr, ATTR_TX, TSO_ENABLE))
		return 0;

	ret = skb_cow_head(skb, 0);
	if (ret)
		return ret;

	pkt_info->header_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
	pkt_info->tcp_header_len = tcp_hdrlen(skb);
	pkt_info->tcp_payload_len = skb->len - pkt_info->header_len;
	pkt_info->mss = skb_shinfo(skb)->gso_size;

	/* Update the number of packets that will ultimately be transmitted
	 * along with the extra bytes for each extra packet
	 */
	pkt_info->tx_packets = skb_shinfo(skb)->gso_segs;
	pkt_info->tx_bytes += (pkt_info->tx_packets - 1) * pkt_info->header_len;

	return 0;
}

static int fxgmac_is_tso(struct sk_buff *skb)
{
	if (skb->ip_summed != CHECKSUM_PARTIAL)
		return 0;

	if (!skb_is_gso(skb))
		return 0;

	return 1;
}

static void fxgmac_prep_tx_pkt(struct fxgmac_pdata *priv,
			       struct fxgmac_ring *ring, struct sk_buff *skb,
			       struct fxgmac_pkt_info *pkt_info)
{
	u32 len, context_desc = 0;

	pkt_info->skb = skb;
	pkt_info->desc_count = 0;
	pkt_info->tx_packets = 1;
	pkt_info->tx_bytes = skb->len;

	if (fxgmac_is_tso(skb)) {
		/* TSO requires an extra descriptor if mss is different */
		if (skb_shinfo(skb)->gso_size != ring->tx.cur_mss) {
			context_desc = 1;
			pkt_info->desc_count++;
		}

		/* TSO requires an extra descriptor for TSO header */
		pkt_info->desc_count++;
		FXGMAC_SET_BITS(pkt_info->attr, ATTR_TX, TSO_ENABLE, 1);
		FXGMAC_SET_BITS(pkt_info->attr, ATTR_TX, CSUM_ENABLE, 1);
	} else if (skb->ip_summed == CHECKSUM_PARTIAL) {
		FXGMAC_SET_BITS(pkt_info->attr, ATTR_TX, CSUM_ENABLE, 1);
	}

	if (skb_vlan_tag_present(skb)) {
		/* VLAN requires an extra descriptor if tag is different */
		if (skb_vlan_tag_get(skb) != ring->tx.cur_vlan_ctag)
			/* We can share with the TSO context descriptor */
			if (!context_desc)
				pkt_info->desc_count++;

		FXGMAC_SET_BITS(pkt_info->attr, ATTR_TX, VLAN_CTAG, 1);
	}

	for (len = skb_headlen(skb); len;) {
		pkt_info->desc_count++;
		len -= min_t(unsigned int, len, FXGMAC_TX_MAX_BUF_SIZE);
	}

	for (u32 i = 0; i < skb_shinfo(skb)->nr_frags; i++)
		for (len = skb_frag_size(&skb_shinfo(skb)->frags[i]); len;) {
			pkt_info->desc_count++;
			len -= min_t(unsigned int, len, FXGMAC_TX_MAX_BUF_SIZE);
		}
}

static netdev_tx_t fxgmac_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct fxgmac_pdata *priv = netdev_priv(netdev);
	struct fxgmac_pkt_info *tx_pkt_info;
	struct fxgmac_channel *channel;
	struct netdev_queue *txq;
	struct fxgmac_ring *ring;
	int ret;

	channel = priv->channel_head + skb->queue_mapping;
	txq = netdev_get_tx_queue(netdev, channel->queue_index);
	ring = channel->tx_ring;
	tx_pkt_info = &ring->pkt_info;

	if (skb->len == 0) {
		yt_err(priv, "empty skb received from stack\n");
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/* Prepare preliminary packet info for TX */
	memset(tx_pkt_info, 0, sizeof(*tx_pkt_info));
	fxgmac_prep_tx_pkt(priv, ring, skb, tx_pkt_info);

	/* Check that there are enough descriptors available */
	ret = fxgmac_maybe_stop_tx_queue(channel, ring,
					 tx_pkt_info->desc_count);
	if (ret == NETDEV_TX_BUSY)
		return ret;

	ret = fxgmac_prep_tso(priv, skb, tx_pkt_info);
	if (ret < 0) {
		yt_err(priv, "error processing TSO packet\n");
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	fxgmac_prep_vlan(skb, tx_pkt_info);

	if (!fxgmac_tx_skb_map(channel, skb)) {
		dev_kfree_skb_any(skb);
		yt_err(priv, "xmit, map tx skb err\n");
		return NETDEV_TX_OK;
	}

	/* Report on the actual number of bytes (to be) sent */
	netdev_tx_sent_queue(txq, tx_pkt_info->tx_bytes);

	/* Configure required descriptor fields for transmission */
	fxgmac_dev_xmit(channel);

	if (netif_msg_pktdata(priv))
		fxgmac_dbg_pkt(priv, skb, true);

	/* Stop the queue in advance if there may not be enough descriptors */
	fxgmac_maybe_stop_tx_queue(channel, ring, FXGMAC_TX_MAX_DESC_NR);

	return NETDEV_TX_OK;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void fxgmac_poll_controller(struct net_device *netdev)
{
	struct fxgmac_pdata *priv = netdev_priv(netdev);
	struct fxgmac_channel *channel;

	if (priv->per_channel_irq) {
		channel = priv->channel_head;
		for (u32 i = 0; i < priv->channel_count; i++, channel++)
			fxgmac_dma_isr(channel->dma_irq_rx, channel);
	} else {
		disable_irq(priv->dev_irq);
		fxgmac_isr(priv->dev_irq, priv);
		enable_irq(priv->dev_irq);
	}
}
#endif /* CONFIG_NET_POLL_CONTROLLER */

static const struct net_device_ops fxgmac_netdev_ops = {
	.ndo_open		= fxgmac_open,
	.ndo_stop		= fxgmac_close,
	.ndo_start_xmit		= fxgmac_xmit,
	.ndo_tx_timeout		= fxgmac_tx_timeout,
	.ndo_validate_addr	= eth_validate_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= fxgmac_poll_controller,
#endif
};

const struct net_device_ops *fxgmac_get_netdev_ops(void)
{
	return &fxgmac_netdev_ops;
}

static void fxgmac_rx_refresh(struct fxgmac_channel *channel)
{
	struct fxgmac_ring *ring = channel->rx_ring;
	struct fxgmac_pdata *priv = channel->priv;
	struct fxgmac_desc_data *desc_data;

	while (ring->dirty != ring->cur) {
		desc_data = FXGMAC_GET_DESC_DATA(ring, ring->dirty);

		/* Reset desc_data values */
		fxgmac_desc_data_unmap(priv, desc_data);

		if (fxgmac_rx_buffe_map(priv, ring, desc_data))
			break;

		fxgmac_desc_rx_reset(desc_data);
		ring->dirty =
			FXGMAC_GET_ENTRY(ring->dirty, ring->dma_desc_count);
	}

	/* Make sure everything is written before the register write */
	wmb();

	/* Update the Rx Tail Pointer Register with address of
	 * the last cleaned entry
	 */
	desc_data = FXGMAC_GET_DESC_DATA(ring, (ring->dirty - 1) &
					 (ring->dma_desc_count - 1));
	FXGMAC_DMA_IO_WR(channel, DMA_CH_RDTR_LO,
			 lower_32_bits(desc_data->dma_desc_addr));
}

static struct sk_buff *fxgmac_create_skb(struct fxgmac_pdata *priv,
					 struct napi_struct *napi,
					 struct fxgmac_desc_data *desc_data,
					 unsigned int len)
{
	unsigned int copy_len;
	struct sk_buff *skb;
	u8 *packet;

	skb = napi_alloc_skb(napi, desc_data->rx.hdr.dma_len);
	if (!skb)
		return NULL;

	/* Start with the header buffer which may contain just the header
	 * or the header plus data
	 */
	dma_sync_single_range_for_cpu(priv->dev, desc_data->rx.hdr.dma_base,
				      desc_data->rx.hdr.dma_off,
				      desc_data->rx.hdr.dma_len,
				      DMA_FROM_DEVICE);

	packet = page_address(desc_data->rx.hdr.pa.pages) +
		 desc_data->rx.hdr.pa.pages_offset;
	copy_len = min(desc_data->rx.hdr.dma_len, len);
	skb_copy_to_linear_data(skb, packet, copy_len);
	skb_put(skb, copy_len);

	return skb;
}

static int fxgmac_tx_poll(struct fxgmac_channel *channel)
{
	struct fxgmac_pdata *priv = channel->priv;
	unsigned int cur, tx_packets = 0, tx_bytes = 0;
	struct fxgmac_ring *ring = channel->tx_ring;
	struct net_device *netdev = priv->netdev;
	struct fxgmac_desc_data *desc_data;
	struct fxgmac_dma_desc *dma_desc;
	struct netdev_queue *txq;
	int processed = 0;

	/* Nothing to do if there isn't a Tx ring for this channel */
	if (!ring)
		return 0;

	if (ring->cur != ring->dirty && (netif_msg_tx_done(priv)))
		yt_dbg(priv, "%s, ring_cur=%d,ring_dirty=%d,qIdx=%d\n",
		       __func__, ring->cur, ring->dirty, channel->queue_index);

	cur = ring->cur;

	/* Be sure we get ring->cur before accessing descriptor data */
	smp_rmb();

	txq = netdev_get_tx_queue(netdev, channel->queue_index);
	while (ring->dirty != cur) {
		desc_data = FXGMAC_GET_DESC_DATA(ring, ring->dirty);
		dma_desc = desc_data->dma_desc;

		if (!fxgmac_is_tx_complete(dma_desc))
			break;

		/* Make sure descriptor fields are read after reading
		 * the OWN bit
		 */
		dma_rmb();

		if (netif_msg_tx_done(priv))
			fxgmac_dump_tx_desc(priv, ring, ring->dirty, 1, 0);

		if (fxgmac_is_last_desc(dma_desc)) {
			tx_packets += desc_data->tx.packets;
			tx_bytes += desc_data->tx.bytes;
		}

		/* Free the SKB and reset the descriptor for re-use */
		fxgmac_desc_data_unmap(priv, desc_data);
		fxgmac_desc_tx_reset(desc_data);

		processed++;
		ring->dirty =
			FXGMAC_GET_ENTRY(ring->dirty, ring->dma_desc_count);
	}

	if (!processed)
		return 0;

	netdev_tx_completed_queue(txq, tx_packets, tx_bytes);

	/* Make sure ownership is written to the descriptor */
	smp_wmb();
	if (ring->tx.queue_stopped == 1 &&
	    (fxgmac_desc_tx_avail(ring) > FXGMAC_TX_DESC_MIN_FREE)) {
		ring->tx.queue_stopped = 0;
		netif_tx_wake_queue(txq);
	}

	return processed;
}

static int fxgmac_one_poll_tx(struct napi_struct *napi, int budget)
{
	struct fxgmac_channel *channel =
		container_of(napi, struct fxgmac_channel, napi_tx);
	struct fxgmac_pdata *priv = channel->priv;
	int ret;

	ret = fxgmac_tx_poll(channel);
	if (napi_complete_done(napi, 0))
		fxgmac_enable_msix_one_irq(priv, MSI_ID_TXQ0);

	return ret;
}

static int fxgmac_dev_read(struct fxgmac_channel *channel)
{
	struct fxgmac_pdata *priv = channel->priv;
	struct fxgmac_ring *ring = channel->rx_ring;
	struct net_device *netdev = priv->netdev;
	static unsigned int cnt_incomplete;
	struct fxgmac_desc_data *desc_data;
	struct fxgmac_dma_desc *dma_desc;
	struct fxgmac_pkt_info *pkt_info;
	u32 ipce, iphe, rxparser;
	unsigned int err, etlt;

	desc_data = FXGMAC_GET_DESC_DATA(ring, ring->cur);
	dma_desc = desc_data->dma_desc;
	pkt_info = &ring->pkt_info;

	/* Check for data availability */
	if (FXGMAC_GET_BITS_LE(dma_desc->desc3, RX_NORMAL_DESC3, OWN))
		return 1;

	/* Make sure descriptor fields are read after reading the OWN bit */
	dma_rmb();

	if (netif_msg_rx_status(priv))
		fxgmac_dump_rx_desc(priv, ring, ring->cur);

	/* Normal Descriptor, be sure Context Descriptor bit is off */
	FXGMAC_SET_BITS(pkt_info->attr, ATTR_RX, CONTEXT, 0);

	/* Indicate if a Context Descriptor is next */
	/* Get the header length */
	if (FXGMAC_GET_BITS_LE(dma_desc->desc3, RX_NORMAL_DESC3, FD)) {
		desc_data->rx.hdr_len = FXGMAC_GET_BITS_LE(dma_desc->desc2,
							   RX_NORMAL_DESC2, HL);
	}

	/* Get the pkt_info length */
	desc_data->rx.len =
		FXGMAC_GET_BITS_LE(dma_desc->desc3, RX_NORMAL_DESC3, PL);

	if (!FXGMAC_GET_BITS_LE(dma_desc->desc3, RX_NORMAL_DESC3, LD)) {
		/* Not all the data has been transferred for this pkt_info */
		FXGMAC_SET_BITS(pkt_info->attr, ATTR_RX, INCOMPLETE, 1);
		cnt_incomplete++;
		return 0;
	}

	if ((cnt_incomplete) && netif_msg_rx_status(priv))
		yt_dbg(priv, "%s, rx back to normal and incomplete cnt=%u\n",
		       __func__, cnt_incomplete);
	cnt_incomplete = 0;

	/* This is the last of the data for this pkt_info */
	FXGMAC_SET_BITS(pkt_info->attr, ATTR_RX, INCOMPLETE, 0);

	/* Set checksum done indicator as appropriate */
	if (netdev->features & NETIF_F_RXCSUM) {
		ipce = FXGMAC_GET_BITS_LE(dma_desc->desc1, RX_NORMAL_DESC1_WB,
					  IPCE);
		iphe = FXGMAC_GET_BITS_LE(dma_desc->desc1, RX_NORMAL_DESC1_WB,
					  IPHE);
		if (!ipce && !iphe)
			FXGMAC_SET_BITS(pkt_info->attr, ATTR_RX, CSUM_DONE, 1);
		else
			return 0;
	}

	/* Check for errors (only valid in last descriptor) */
	err = FXGMAC_GET_BITS_LE(dma_desc->desc3, RX_NORMAL_DESC3, ES);
	rxparser = FXGMAC_GET_BITS_LE(dma_desc->desc2, RX_NORMAL_DESC2_WB,
				      RAPARSER);
	/* Error or incomplete parsing due to ECC error */
	if (err || rxparser == 0x7) {
		FXGMAC_SET_BITS(pkt_info->errors, ERRORS_RX, FRAME, 1);
		return 0;
	}

	etlt = FXGMAC_GET_BITS_LE(dma_desc->desc3, RX_NORMAL_DESC3, ETLT);
	if (etlt == 0x4 && (netdev->features & NETIF_F_HW_VLAN_CTAG_RX)) {
		FXGMAC_SET_BITS(pkt_info->attr, ATTR_RX, VLAN_CTAG, 1);
		pkt_info->vlan_ctag = FXGMAC_GET_BITS_LE(dma_desc->desc0,
							 RX_NORMAL_DESC0, OVT);
	}

	return 0;
}

static unsigned int fxgmac_desc_rx_dirty(struct fxgmac_ring *ring)
{
	unsigned int dirty;

	if (ring->dirty <= ring->cur)
		dirty = ring->cur - ring->dirty;
	else
		dirty = ring->dma_desc_count - ring->dirty + ring->cur;

	return dirty;
}

static int fxgmac_rx_poll(struct fxgmac_channel *channel, int budget)
{
	struct fxgmac_pdata *priv = channel->priv;
	struct fxgmac_ring *ring = channel->rx_ring;
	struct net_device *netdev = priv->netdev;
	u32 context_next, context, incomplete;
	struct fxgmac_desc_data *desc_data;
	struct fxgmac_pkt_info *pkt_info;
	struct napi_struct *napi;
	u32 len, max_len;
	int packet_count = 0;

	struct sk_buff *skb;

	/* Nothing to do if there isn't a Rx ring for this channel */
	if (!ring)
		return 0;

	napi = (priv->per_channel_irq) ? &channel->napi_rx : &priv->napi;
	pkt_info = &ring->pkt_info;

	while (packet_count < budget) {
		memset(pkt_info, 0, sizeof(*pkt_info));
		skb = NULL;
		len = 0;

read_again:
		desc_data = FXGMAC_GET_DESC_DATA(ring, ring->cur);

		if (fxgmac_desc_rx_dirty(ring) > FXGMAC_RX_DESC_MAX_DIRTY)
			fxgmac_rx_refresh(channel);

		if (fxgmac_dev_read(channel))
			break;

		ring->cur = FXGMAC_GET_ENTRY(ring->cur, ring->dma_desc_count);
		incomplete = FXGMAC_GET_BITS(pkt_info->attr, ATTR_RX, INCOMPLETE);
		context_next = FXGMAC_GET_BITS(pkt_info->attr, ATTR_RX, CONTEXT_NEXT);
		context = FXGMAC_GET_BITS(pkt_info->attr, ATTR_RX, CONTEXT);

		if (incomplete || context_next)
			goto read_again;

		if (pkt_info->errors) {
			dev_kfree_skb(skb);
			priv->netdev->stats.rx_dropped++;
			yt_err(priv, "error in received packet\n");
			goto next_packet;
		}

		if (!context) {
			len = desc_data->rx.len;
			if (len == 0) {
				if (net_ratelimit())
					yt_err(priv,
					       "A packet of length 0 was received\n");
				priv->netdev->stats.rx_length_errors++;
				priv->netdev->stats.rx_dropped++;
				goto next_packet;
			}

			if (len && !skb) {
				skb = fxgmac_create_skb(priv, napi, desc_data,
							len);
				if (unlikely(!skb)) {
					if (net_ratelimit())
						yt_err(priv,
						       "create skb err\n");
					priv->netdev->stats.rx_dropped++;
					goto next_packet;
				}
			}
			max_len = netdev->mtu + ETH_HLEN;
			if (!(netdev->features & NETIF_F_HW_VLAN_CTAG_RX) &&
			    skb->protocol == htons(ETH_P_8021Q))
				max_len += VLAN_HLEN;

			if (len > max_len) {
				if (net_ratelimit())
					yt_err(priv,
					       "len %d larger than max size %d\n",
					       len, max_len);
				priv->netdev->stats.rx_length_errors++;
				priv->netdev->stats.rx_dropped++;
				dev_kfree_skb(skb);
				goto next_packet;
			}
		}

		if (!skb) {
			priv->netdev->stats.rx_dropped++;
			goto next_packet;
		}

		if (netif_msg_pktdata(priv))
			fxgmac_dbg_pkt(priv, skb, false);

		skb_checksum_none_assert(skb);
		if (netdev->features & NETIF_F_RXCSUM)
			skb->ip_summed = CHECKSUM_UNNECESSARY;

		if (FXGMAC_GET_BITS(pkt_info->attr, ATTR_RX, VLAN_CTAG))
			__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q),
					       pkt_info->vlan_ctag);

		if (FXGMAC_GET_BITS(pkt_info->attr, ATTR_RX, RSS_HASH))
			skb_set_hash(skb, pkt_info->rss_hash,
				     pkt_info->rss_hash_type);

		skb->dev = netdev;
		skb->protocol = eth_type_trans(skb, netdev);
		skb_record_rx_queue(skb, channel->queue_index);
		napi_gro_receive(napi, skb);

next_packet:
		packet_count++;
		priv->netdev->stats.rx_packets++;
		priv->netdev->stats.rx_bytes += len;
	}

	return packet_count;
}

static int fxgmac_one_poll_rx(struct napi_struct *napi, int budget)
{
	struct fxgmac_channel *channel =
		container_of(napi, struct fxgmac_channel, napi_rx);
	int processed = fxgmac_rx_poll(channel, budget);

	if (processed < budget && (napi_complete_done(napi, processed)))
		fxgmac_enable_msix_one_irq(channel->priv, channel->queue_index);

	return processed;
}

static int fxgmac_all_poll(struct napi_struct *napi, int budget)
{
	struct fxgmac_channel *channel;
	struct fxgmac_pdata *priv;
	int processed = 0;

	priv = container_of(napi, struct fxgmac_pdata, napi);
	do {
		channel = priv->channel_head;
		/* Only support 1 tx channel, poll ch 0. */
		fxgmac_tx_poll(priv->channel_head + 0);
		for (u32 i = 0; i < priv->channel_count; i++, channel++)
			processed += fxgmac_rx_poll(channel, budget);
	} while (false);

	/* If we processed everything, we are done */
	if (processed < budget) {
		/* Turn off polling */
		if (napi_complete_done(napi, processed))
			fxgmac_enable_mgm_irq(priv);
	}

	if ((processed) && (netif_msg_rx_status(priv)))
		yt_dbg(priv, "%s, received : %d\n", __func__, processed);

	return processed;
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
