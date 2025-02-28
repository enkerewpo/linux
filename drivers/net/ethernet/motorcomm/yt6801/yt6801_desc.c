// SPDX-License-Identifier: GPL-2.0+
/* Copyright (c) 2022 - 2024 Motorcomm Electronic Technology Co.,Ltd. */

#include "yt6801.h"
#include "yt6801_desc.h"

void fxgmac_desc_data_unmap(struct fxgmac_pdata *priv,
			    struct fxgmac_desc_data *desc_data)
{
	if (desc_data->skb_dma) {
		if (desc_data->mapped_as_page) {
			dma_unmap_page(priv->dev, desc_data->skb_dma,
				       desc_data->skb_dma_len, DMA_TO_DEVICE);
		} else {
			dma_unmap_single(priv->dev, desc_data->skb_dma,
					 desc_data->skb_dma_len, DMA_TO_DEVICE);
		}
		desc_data->skb_dma = 0;
		desc_data->skb_dma_len = 0;
	}

	if (desc_data->skb) {
		dev_kfree_skb_any(desc_data->skb);
		desc_data->skb = NULL;
	}

	if (desc_data->rx.hdr.pa.pages)
		put_page(desc_data->rx.hdr.pa.pages);

	if (desc_data->rx.hdr.pa_unmap.pages) {
		dma_unmap_page(priv->dev, desc_data->rx.hdr.pa_unmap.pages_dma,
			       desc_data->rx.hdr.pa_unmap.pages_len,
			       DMA_FROM_DEVICE);
		put_page(desc_data->rx.hdr.pa_unmap.pages);
	}

	if (desc_data->rx.buf.pa.pages)
		put_page(desc_data->rx.buf.pa.pages);

	if (desc_data->rx.buf.pa_unmap.pages) {
		dma_unmap_page(priv->dev, desc_data->rx.buf.pa_unmap.pages_dma,
			       desc_data->rx.buf.pa_unmap.pages_len,
			       DMA_FROM_DEVICE);
		put_page(desc_data->rx.buf.pa_unmap.pages);
	}
	memset(&desc_data->tx, 0, sizeof(desc_data->tx));
	memset(&desc_data->rx, 0, sizeof(desc_data->rx));

	desc_data->mapped_as_page = 0;
}

static int fxgmac_ring_init(struct fxgmac_pdata *priv, struct fxgmac_ring *ring,
			    unsigned int dma_desc_count)
{
	/* Descriptors */
	ring->dma_desc_count = dma_desc_count;
	ring->dma_desc_head =
		dma_alloc_coherent(priv->dev, (sizeof(struct fxgmac_dma_desc) *
				   dma_desc_count),
				   &ring->dma_desc_head_addr, GFP_KERNEL);
	if (!ring->dma_desc_head)
		return -ENOMEM;

	/* Array of descriptor data */
	ring->desc_data_head = kcalloc(dma_desc_count,
				       sizeof(struct fxgmac_desc_data),
				       GFP_KERNEL);
	if (!ring->desc_data_head)
		return -ENOMEM;

	return 0;
}

static void fxgmac_ring_free(struct fxgmac_pdata *priv,
			     struct fxgmac_ring *ring)
{
	if (!ring)
		return;

	if (ring->desc_data_head) {
		for (u32 i = 0; i < ring->dma_desc_count; i++)
			fxgmac_desc_data_unmap(priv,
					       FXGMAC_GET_DESC_DATA(ring, i));

		kfree(ring->desc_data_head);
		ring->desc_data_head = NULL;
	}

	if (ring->rx_hdr_pa.pages) {
		dma_unmap_page(priv->dev, ring->rx_hdr_pa.pages_dma,
			       ring->rx_hdr_pa.pages_len, DMA_FROM_DEVICE);
		put_page(ring->rx_hdr_pa.pages);

		ring->rx_hdr_pa.pages = NULL;
		ring->rx_hdr_pa.pages_len = 0;
		ring->rx_hdr_pa.pages_offset = 0;
		ring->rx_hdr_pa.pages_dma = 0;
	}

	if (ring->rx_buf_pa.pages) {
		dma_unmap_page(priv->dev, ring->rx_buf_pa.pages_dma,
			       ring->rx_buf_pa.pages_len, DMA_FROM_DEVICE);
		put_page(ring->rx_buf_pa.pages);

		ring->rx_buf_pa.pages = NULL;
		ring->rx_buf_pa.pages_len = 0;
		ring->rx_buf_pa.pages_offset = 0;
		ring->rx_buf_pa.pages_dma = 0;
	}
	if (ring->dma_desc_head) {
		dma_free_coherent(priv->dev, (sizeof(struct fxgmac_dma_desc) *
				  ring->dma_desc_count), ring->dma_desc_head,
				  ring->dma_desc_head_addr);
		ring->dma_desc_head = NULL;
	}
}

static void fxgmac_rings_free(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;

	fxgmac_ring_free(priv, channel->tx_ring);

	for (u32 i = 0; i < priv->channel_count; i++, channel++)
		fxgmac_ring_free(priv, channel->rx_ring);
}

static int fxgmac_rings_alloc(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;
	int ret;

	ret = fxgmac_ring_init(priv, channel->tx_ring, priv->tx_desc_count);
	if (ret < 0) {
		yt_err(priv, "error initializing Tx ring");
		goto err_init_ring;
	}

	for (u32 i = 0; i < priv->channel_count; i++, channel++) {
		ret = fxgmac_ring_init(priv, channel->rx_ring,
				       priv->rx_desc_count);
		if (ret < 0) {
			yt_err(priv, "error initializing Rx ring\n");
			goto err_init_ring;
		}
	}
	return 0;

err_init_ring:
	fxgmac_rings_free(priv);
	return ret;
}

static void fxgmac_channels_free(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel = priv->channel_head;

	kfree(channel->tx_ring);
	channel->tx_ring = NULL;

	kfree(channel->rx_ring);
	channel->rx_ring = NULL;

	kfree(channel);
	priv->channel_head = NULL;
}

void fxgmac_channels_rings_free(struct fxgmac_pdata *priv)
{
	fxgmac_rings_free(priv);
	fxgmac_channels_free(priv);
}

#ifdef CONFIG_PCI_MSI
static void fxgmac_set_msix_tx_irq(struct fxgmac_pdata *priv,
				   struct fxgmac_channel *channel, u32 i)
{
	if (i != 0) /*only one tx*/
		return;

	priv->channel_irq[FXGMAC_MAX_DMA_RX_CHANNELS] =
		priv->msix_entries[FXGMAC_MAX_DMA_RX_CHANNELS].vector;
	channel->dma_irq_tx = priv->channel_irq[FXGMAC_MAX_DMA_RX_CHANNELS];
}
#endif

static int fxgmac_channels_alloc(struct fxgmac_pdata *priv)
{
	struct fxgmac_channel *channel_head, *channel;
	struct fxgmac_ring *tx_ring, *rx_ring;
	int ret = -ENOMEM;

	channel_head = kcalloc(priv->channel_count,
			       sizeof(struct fxgmac_channel), GFP_KERNEL);

	if (!channel_head)
		return ret;

	tx_ring = kcalloc(FXGMAC_TX_1_RING, sizeof(struct fxgmac_ring),
			  GFP_KERNEL);
	if (!tx_ring)
		goto err_tx_ring;

	rx_ring = kcalloc(priv->rx_ring_count, sizeof(struct fxgmac_ring),
			  GFP_KERNEL);
	if (!rx_ring)
		goto err_rx_ring;

	channel = channel_head;
	for (u32 i = 0; i < priv->channel_count; i++, channel++) {
		snprintf(channel->name, sizeof(channel->name), "channel-%u", i);
		channel->priv = priv;
		channel->queue_index = i;
		channel->dma_regs = (priv)->hw_addr + MAC_OFFSET + DMA_CH_BASE +
				    (DMA_CH_INC * i);

		if (priv->per_channel_irq) {
			priv->channel_irq[i] = priv->msix_entries[i].vector;

			if (IS_ENABLED(CONFIG_PCI_MSI))
				fxgmac_set_msix_tx_irq(priv, channel, i);

			/* Get the per DMA rx interrupt */
			ret = priv->channel_irq[i];
			if (ret < 0) {
				yt_err(priv, "get_irq %u err\n", i + 1);
				goto err_irq;
			}

			channel->dma_irq_rx = ret;
		}

		if (i < FXGMAC_TX_1_RING)
			channel->tx_ring = tx_ring++;

		if (i < priv->rx_ring_count)
			channel->rx_ring = rx_ring++;
	}

	priv->channel_head = channel_head;
	return 0;

err_irq:
	kfree(rx_ring);

err_rx_ring:
	kfree(tx_ring);

err_tx_ring:
	kfree(channel_head);

	yt_err(priv, "%s err:%d\n", __func__, ret);
	return ret;
}

int fxgmac_channels_rings_alloc(struct fxgmac_pdata *priv)
{
	int ret;

	ret = fxgmac_channels_alloc(priv);
	if (ret < 0)
		goto err_alloc;

	ret = fxgmac_rings_alloc(priv);
	if (ret < 0)
		goto err_alloc;

	return 0;

err_alloc:
	fxgmac_channels_rings_free(priv);
	return ret;
}
