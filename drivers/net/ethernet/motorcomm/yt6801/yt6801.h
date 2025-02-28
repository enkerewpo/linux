/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (c) 2022 - 2024 Motorcomm Electronic Technology Co.,Ltd. */

#ifndef YT6801_H
#define YT6801_H

#include <linux/dma-mapping.h>
#include <linux/timecounter.h>
#include <linux/pm_wakeup.h>
#include <linux/workqueue.h>
#include <linux/crc32poly.h>
#include <linux/if_vlan.h>
#include <linux/bitrev.h>
#include <linux/bitops.h>
#include <linux/mdio.h>
#include <linux/phy.h>

#ifdef CONFIG_PCI_MSI
#include <linux/pci.h>
#endif

#include "yt6801_type.h"

#define FXGMAC_DRV_NAME		"yt6801"
#define FXGMAC_DRV_DESC		"Motorcomm Gigabit Ethernet Driver"

#define FXGMAC_RX_BUF_ALIGN	64
#define FXGMAC_TX_MAX_BUF_SIZE	(0x3fff & ~(FXGMAC_RX_BUF_ALIGN - 1))
#define FXGMAC_RX_MIN_BUF_SIZE	(ETH_FRAME_LEN + ETH_FCS_LEN + VLAN_HLEN)

/* Descriptors required for maximum contiguous TSO/GSO packet */
#define FXGMAC_TX_MAX_SPLIT	((GSO_MAX_SIZE / FXGMAC_TX_MAX_BUF_SIZE) + 1)

/* Maximum possible descriptors needed for a SKB */
#define FXGMAC_TX_MAX_DESC_NR	(MAX_SKB_FRAGS + FXGMAC_TX_MAX_SPLIT + 2)

#define FXGMAC_DMA_STOP_TIMEOUT		5
#define FXGMAC_JUMBO_PACKET_MTU		9014
#define FXGMAC_MAX_DMA_RX_CHANNELS	4
#define FXGMAC_MAX_DMA_TX_CHANNELS	1
#define FXGMAC_MAX_DMA_CHANNELS                                           \
	(FXGMAC_MAX_DMA_RX_CHANNELS + FXGMAC_MAX_DMA_TX_CHANNELS)

struct fxgmac_ring_buf {
	struct sk_buff *skb;
	dma_addr_t skb_dma;
	unsigned int skb_len;
};

/* Common Tx and Rx DMA hardware descriptor */
struct fxgmac_dma_desc {
	__le32 desc0;
	__le32 desc1;
	__le32 desc2;
	__le32 desc3;
};

/* Page allocation related values */
struct fxgmac_page_alloc {
	struct page *pages;
	unsigned int pages_len;
	unsigned int pages_offset;
	dma_addr_t pages_dma;
};

/* Ring entry buffer data */
struct fxgmac_buffer_data {
	struct fxgmac_page_alloc pa;
	struct fxgmac_page_alloc pa_unmap;

	dma_addr_t dma_base;
	unsigned long dma_off;
	unsigned int dma_len;
};

struct fxgmac_tx_desc_data {
	unsigned int packets;		/* BQL packet count */
	unsigned int bytes;		/* BQL byte count */
};

struct fxgmac_rx_desc_data {
	struct fxgmac_buffer_data hdr;	/* Header locations */
	struct fxgmac_buffer_data buf;	/* Payload locations */
	unsigned short hdr_len;		/* Length of received header */
	unsigned short len;		/* Length of received packet */
};

struct fxgmac_pkt_info {
	struct sk_buff *skb;
#define ATTR_TX_CSUM_ENABLE_POS		0
#define ATTR_TX_CSUM_ENABLE_LEN		1
#define ATTR_TX_TSO_ENABLE_POS		1
#define ATTR_TX_TSO_ENABLE_LEN		1
#define ATTR_TX_VLAN_CTAG_POS		2
#define ATTR_TX_VLAN_CTAG_LEN		1
#define ATTR_TX_PTP_POS			3
#define ATTR_TX_PTP_LEN			1
#define ATTR_RX_CSUM_DONE_POS		0
#define ATTR_RX_CSUM_DONE_LEN		1
#define ATTR_RX_VLAN_CTAG_POS		1
#define ATTR_RX_VLAN_CTAG_LEN		1
#define ATTR_RX_INCOMPLETE_POS		2
#define ATTR_RX_INCOMPLETE_LEN		1
#define ATTR_RX_CONTEXT_NEXT_POS	3
#define ATTR_RX_CONTEXT_NEXT_LEN	1
#define ATTR_RX_CONTEXT_POS		4
#define ATTR_RX_CONTEXT_LEN		1
#define ATTR_RX_RX_TSTAMP_POS		5
#define ATTR_RX_RX_TSTAMP_LEN		1
#define ATTR_RX_RSS_HASH_POS		6
#define ATTR_RX_RSS_HASH_LEN		1
	unsigned int attr;
#define ERRORS_RX_CRC_POS		2
#define ERRORS_RX_CRC_LEN		1
#define ERRORS_RX_FRAME_POS		3
#define ERRORS_RX_FRAME_LEN		1
#define ERRORS_RX_LENGTH_POS		0
#define ERRORS_RX_LENGTH_LEN		1
#define ERRORS_RX_OVERRUN_POS		1
#define ERRORS_RX_OVERRUN_LEN		1
	unsigned int errors;
	unsigned int desc_count; /* descriptors needed for this packet */
	unsigned int length;
	unsigned int tx_packets;
	unsigned int tx_bytes;

	unsigned int header_len;
	unsigned int tcp_header_len;
	unsigned int tcp_payload_len;
	unsigned short mss;
	unsigned short vlan_ctag;

	u64 rx_tstamp;
	u32 rss_hash;
	enum pkt_hash_types rss_hash_type;
};

struct fxgmac_desc_data {
	struct fxgmac_dma_desc *dma_desc;  /* Virtual address of descriptor */
	dma_addr_t dma_desc_addr;          /* DMA address of descriptor */
	struct sk_buff *skb;               /* Virtual address of SKB */
	dma_addr_t skb_dma;                /* DMA address of SKB data */
	unsigned int skb_dma_len;          /* Length of SKB DMA area */

	/* Tx/Rx -related data */
	struct fxgmac_tx_desc_data tx;
	struct fxgmac_rx_desc_data rx;

	unsigned int mapped_as_page;
};

struct fxgmac_ring {
	struct fxgmac_pkt_info pkt_info;  /* packet related information */

	/* Virtual/DMA addresses of DMA descriptor list */
	struct fxgmac_dma_desc *dma_desc_head;
	dma_addr_t dma_desc_head_addr;
	unsigned int dma_desc_count;

	/* Array of descriptor data corresponding the DMA descriptor
	 * (always use the FXGMAC_GET_DESC_DATA macro to access this data)
	 */
	struct fxgmac_desc_data *desc_data_head;

	/* Page allocation for RX buffers */
	struct fxgmac_page_alloc rx_hdr_pa;
	struct fxgmac_page_alloc rx_buf_pa;

	/* Ring index values
	 * cur  - Tx: index of descriptor to be used for current transfer
	 *        Rx: index of descriptor to check for packet availability
	 * dirty - Tx: index of descriptor to check for transfer complete
	 *         Rx: index of descriptor to check for buffer reallocation
	 */
	unsigned int cur;
	unsigned int dirty;

	struct {
		unsigned int xmit_more;
		unsigned int queue_stopped;
		unsigned short cur_mss;
		unsigned short cur_vlan_ctag;
	} tx;
} ____cacheline_aligned;

struct fxgmac_channel {
	char name[16];

	/* Address of private data area for device */
	struct fxgmac_pdata *priv;

	/* Queue index and base address of queue's DMA registers */
	unsigned int queue_index;

	/* Per channel interrupt irq number */
	u32 dma_irq_rx;
	char dma_irq_rx_name[IFNAMSIZ + 32];
	u32 dma_irq_tx;
	char dma_irq_tx_name[IFNAMSIZ + 32];

	/* Netdev related settings */
	struct napi_struct napi_tx;
	struct napi_struct napi_rx;

	void __iomem *dma_regs;
	struct fxgmac_ring *tx_ring;
	struct fxgmac_ring *rx_ring;
} ____cacheline_aligned;

/* This structure contains flags that indicate what hardware features
 * or configurations are present in the device.
 */
struct fxgmac_hw_features {
	unsigned int version;		/* HW Version */

	/* HW Feature Register0 */
	unsigned int phyifsel;		/* PHY interface support */
	unsigned int vlhash;		/* VLAN Hash Filter */
	unsigned int sma;		/* SMA(MDIO) Interface */
	unsigned int rwk;		/* PMT remote wake-up packet */
	unsigned int mgk;		/* PMT magic packet */
	unsigned int mmc;		/* RMON module */
	unsigned int aoe;		/* ARP Offload */
	unsigned int ts;		/* IEEE 1588-2008 Advanced Timestamp */
	unsigned int eee;		/* Energy Efficient Ethernet */
	unsigned int tx_coe;		/* Tx Checksum Offload */
	unsigned int rx_coe;		/* Rx Checksum Offload */
	unsigned int addn_mac;		/* Additional MAC Addresses */
	unsigned int ts_src;		/* Timestamp Source */
	unsigned int sa_vlan_ins;	/* Source Address or VLAN Insertion */

	/* HW Feature Register1 */
	unsigned int rx_fifo_size;	/* MTL Receive FIFO Size */
	unsigned int tx_fifo_size;	/* MTL Transmit FIFO Size */
	unsigned int adv_ts_hi;		/* Advance Timestamping High Word */
	unsigned int dma_width;		/* DMA width */
	unsigned int dcb;		/* DCB Feature */
	unsigned int sph;		/* Split Header Feature */
	unsigned int tso;		/* TCP Segmentation Offload */
	unsigned int dma_debug;		/* DMA Debug Registers */
	unsigned int rss;		/* Receive Side Scaling */
	unsigned int tc_cnt;		/* Number of Traffic Classes */
	unsigned int avsel;		/* AV Feature Enable */
	unsigned int ravsel;		/* Rx Side Only AV Feature Enable */
	unsigned int hash_table_size;	/* Hash Table Size */
	unsigned int l3l4_filter_num;	/* Number of L3-L4 Filters */

	/* HW Feature Register2 */
	unsigned int rx_q_cnt;		/* Number of MTL Receive Queues */
	unsigned int tx_q_cnt;		/* Number of MTL Transmit Queues */
	unsigned int rx_ch_cnt;		/* Number of DMA Receive Channels */
	unsigned int tx_ch_cnt;		/* Number of DMA Transmit Channels */
	unsigned int pps_out_num;	/* Number of PPS outputs */
	unsigned int aux_snap_num;	/* Number of Aux snapshot inputs */

	u32 hwfr3;			/* HW Feature Register3 */
};

struct fxgmac_resources {
	void __iomem *addr;
	int irq;
};

enum fxgmac_dev_state {
	FXGMAC_DEV_OPEN		= 0x0,
	FXGMAC_DEV_CLOSE	= 0x1,
	FXGMAC_DEV_STOP		= 0x2,
	FXGMAC_DEV_START	= 0x3,
	FXGMAC_DEV_SUSPEND	= 0x4,
	FXGMAC_DEV_RESUME	= 0x5,
	FXGMAC_DEV_PROBE	= 0xFF,
};

struct fxgmac_pdata {
	struct net_device *netdev;
	struct device *dev;
	struct phy_device *phydev;

	struct fxgmac_hw_features hw_feat;	/* Hardware features */
	void __iomem *hw_addr;			/* Registers base */

	/* Rings for Tx/Rx on a DMA channel */
	struct fxgmac_channel *channel_head;
	unsigned int channel_count;
	unsigned int rx_ring_count;
	unsigned int rx_desc_count;
	unsigned int rx_q_count;
#define FXGMAC_TX_1_RING	1
#define FXGMAC_TX_1_Q		1
	unsigned int tx_desc_count;

	unsigned long sysclk_rate;		/* Device clocks */
	unsigned int pblx8;			/* Tx/Rx common settings */

	/* Tx settings */
	unsigned int tx_sf_mode;
	unsigned int tx_threshold;
	unsigned int tx_pbl;
	unsigned int tx_osp_mode;
	unsigned int tx_hang_restart_queuing;

	/* Rx settings */
	unsigned int rx_sf_mode;
	unsigned int rx_threshold;
	unsigned int rx_pbl;

	/* Tx coalescing settings */
	unsigned int tx_usecs;
	unsigned int tx_frames;

	/* Rx coalescing settings */
	unsigned int rx_riwt;
	unsigned int rx_usecs;
	unsigned int rx_frames;

	/* Flow control settings */
	unsigned int tx_pause;
	unsigned int rx_pause;

	unsigned int mtu;
	unsigned int rx_buf_size;	/* Current Rx buffer size */

	/* Device interrupt */
	int dev_irq;
	unsigned int per_channel_irq;
	u32 channel_irq[FXGMAC_MAX_DMA_CHANNELS];
	struct msix_entry *msix_entries;
#define INT_FLAG_INTERRUPT_POS		0
#define INT_FLAG_INTERRUPT_LEN		5
#define INT_FLAG_MSI_POS		1
#define INT_FLAG_MSI_LEN		1
#define INT_FLAG_MSIX_POS		3
#define INT_FLAG_MSIX_LEN		1
#define INT_FLAG_LEGACY_POS		4
#define INT_FLAG_LEGACY_LEN		1
#define INT_FLAG_RX_NAPI_POS		18
#define INT_FLAG_RX_NAPI_LEN		4
#define INT_FLAG_PER_RX_NAPI_LEN	1
#define INT_FLAG_RX_IRQ_POS		22
#define INT_FLAG_RX_IRQ_LEN		4
#define INT_FLAG_PER_RX_IRQ_LEN		1
#define INT_FLAG_TX_NAPI_POS		26
#define INT_FLAG_TX_NAPI_LEN		1
#define INT_FLAG_TX_IRQ_POS		27
#define INT_FLAG_TX_IRQ_LEN		1
#define INT_FLAG_LEGACY_NAPI_POS	30
#define INT_FLAG_LEGACY_NAPI_LEN	1
#define INT_FLAG_LEGACY_IRQ_POS		31
#define INT_FLAG_LEGACY_IRQ_LEN		1
	u32 int_flag;		/* interrupt flag */

	/* Netdev related settings */
	unsigned char mac_addr[ETH_ALEN];
	netdev_features_t netdev_features;
	struct napi_struct napi;

	int mac_speed;
	int mac_duplex;

	u32 msg_enable;
	u32 reg_nonstick[(MSI_PBA - GLOBAL_CTRL0) >> 2];

	struct work_struct restart_work;
	enum fxgmac_dev_state dev_state;
#define FXGMAC_POWER_STATE_DOWN			0
#define FXGMAC_POWER_STATE_UP			1
	unsigned long powerstate;
	struct mutex mutex; /* Driver lock */

	char drv_name[32];
	char drv_ver[32];
};

extern int fxgmac_net_powerup(struct fxgmac_pdata *priv);
extern int fxgmac_net_powerdown(struct fxgmac_pdata *priv);
extern int fxgmac_drv_probe(struct device *dev, struct fxgmac_resources *res);
extern void fxgmac_phy_reset(struct fxgmac_pdata *priv);

#endif /* YT6801_H */
