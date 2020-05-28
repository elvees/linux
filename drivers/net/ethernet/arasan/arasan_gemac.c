/*
 * Copyright 2007, 2008 SMSC
 * Copyright 2015 ELVEES NeoTek CJSC
 * Copyright 2017 RnD Center "ELVEES", JSC
 *
 * Based on the driver for smsc9420
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/if_vlan.h>
#include <linux/of_mdio.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/crc32.h>
#include <generated/utsrelease.h>

#include "arasan_gemac.h"

#define ARASAN_GEMAC_FEATURES (PHY_GBIT_FEATURES | SUPPORTED_FIBRE | \
			       SUPPORTED_BNC)

#define print_reg(reg) netdev_info(pd->dev, \
				   "offset 0x%x : value 0x%x\n", \
				   reg, \
				   arasan_gemac_readl(pd, reg))

void arasan_gemac_dump_regs(struct arasan_gemac_pdata *pd)
{
	netdev_info(pd->dev, "Arasan GEMAC register dump:\n");

	print_reg(DMA_CONFIGURATION);
	print_reg(DMA_CONTROL);
	print_reg(DMA_STATUS_AND_IRQ);
	print_reg(DMA_INTERRUPT_ENABLE);
	print_reg(DMA_TRANSMIT_AUTO_POLL_COUNTER);
	print_reg(DMA_TRANSMIT_POLL_DEMAND);
	print_reg(DMA_RECEIVE_POLL_DEMAND);
	print_reg(DMA_TRANSMIT_BASE_ADDRESS);
	print_reg(DMA_RECEIVE_BASE_ADDRESS);
	print_reg(DMA_MISSED_FRAME_COUNTER);
	print_reg(DMA_STOP_FLUSH_COUNTER);
	print_reg(DMA_RECEIVE_INTERRUPT_MITIGATION);
	print_reg(DMA_CURRENT_TRANSMIT_DESCRIPTOR_POINTER);
	print_reg(DMA_CURRENT_TRANSMIT_BUFFER_POINTER);
	print_reg(DMA_CURRENT_RECEIVE_DESCRIPTOR_POINTER);
	print_reg(DMA_CURRENT_RECEIVE_BUFFER_POINTER);

	print_reg(MAC_GLOBAL_CONTROL);
	print_reg(MAC_TRANSMIT_CONTROL);
	print_reg(MAC_RECEIVE_CONTROL);
	print_reg(MAC_ADDRESS_CONTROL);
	print_reg(MAC_ADDRESS1_HIGH);
	print_reg(MAC_ADDRESS1_MED);
	print_reg(MAC_ADDRESS1_LOW);
	print_reg(MAC_INTERRUPT_STATUS);
	print_reg(MAC_INTERRUPT_ENABLE);
}

static void arasan_gemac_get_drvinfo(struct net_device *dev,
				     struct ethtool_drvinfo *info)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);

	strlcpy(info->driver, pd->pdev->dev.driver->name, sizeof(info->driver));
	strlcpy(info->version, UTS_RELEASE, sizeof(info->version));
}

static int arasan_gemac_get_settings(struct net_device *dev,
				     struct ethtool_cmd *ecmd)
{
	struct arasan_gemac_pdata *priv = netdev_priv(dev);

	if (!priv->phy_dev)
		return -ENODEV;
	return phy_ethtool_gset(priv->phy_dev, ecmd);
}

static int arasan_gemac_set_settings(struct net_device *dev,
				     struct ethtool_cmd *ecmd)
{
	struct arasan_gemac_pdata *priv = netdev_priv(dev);

	if (!priv->phy_dev)
		return -ENODEV;
	return phy_ethtool_sset(priv->phy_dev, ecmd);
}

static u32 arasan_gemac_get_msglevel(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);

	return pd->msg_enable;
}

static void arasan_gemac_set_msglevel(struct net_device *dev, u32 val)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);

	pd->msg_enable = val;
}

static int arasan_gemac_nway_reset(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);

	if (!pd->phy_dev)
		return -ENODEV;

	return genphy_restart_aneg(pd->phy_dev);
}

static const struct ethtool_ops arasan_gemac_ethtool_ops = {
	.get_drvinfo = arasan_gemac_get_drvinfo,
	.get_settings = arasan_gemac_get_settings,
	.set_settings = arasan_gemac_set_settings,
	.get_msglevel = arasan_gemac_get_msglevel,
	.set_msglevel = arasan_gemac_set_msglevel,
	.get_link = ethtool_op_get_link,
	.nway_reset = arasan_gemac_nway_reset,
};

static void arasan_gemac_set_hwaddr(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);

	u8 *dev_addr = dev->dev_addr;

	arasan_gemac_writel(pd, MAC_ADDRESS1_LOW,
			    MAC_ADDRESS1_LOW_SIXTH_BYTE(dev_addr[5]) |
			    MAC_ADDRESS1_LOW_FIFTH_BYTE(dev_addr[4]));

	arasan_gemac_writel(pd, MAC_ADDRESS1_MED,
			    MAC_ADDRESS1_MED_FOURTH_BYTE(dev_addr[3]) |
			    MAC_ADDRESS1_MED_THIRD_BYTE(dev_addr[2]));

	arasan_gemac_writel(pd, MAC_ADDRESS1_HIGH,
			    MAC_ADDRESS1_HIGH_SECOND_BYTE(dev_addr[1]) |
			    MAC_ADDRESS1_HIGH_FIRST_BYTE(dev_addr[0]));
}

static void arasan_gemac_get_hwaddr(struct arasan_gemac_pdata *pd)
{
	netdev_info(pd->dev, "Using random hw address\n");
	eth_hw_addr_random(pd->dev);
}

static void arasan_gemac_dma_soft_reset(struct arasan_gemac_pdata *pd)
{
	/* Reset the DMA controller to the default state */
	arasan_gemac_writel(pd, DMA_CONFIGURATION,
			    DMA_CONFIGURATION_SOFT_RESET);

	/* FIXME
	 * mdelay or msleep ?
	 */
	mdelay(10);

	/* Write the default value to deassert the reset signal */
	arasan_gemac_writel(pd, DMA_CONFIGURATION,
			    DMA_CONFIGURATION_BURST_LENGTH(4));
}

static void arasan_gemac_setup_frame_limits(struct arasan_gemac_pdata *pd,
					    int mtu)
{
	/* FIXME: extra_sz = 12 for 3 VLAN TAG ??? */
	const int extra_sz = 0;
	int sz = mtu_to_frame_sz(mtu);
	/* Frame length violation is set if received frame exceed max */
	arasan_gemac_writel(pd, MAC_MAXIMUM_FRAME_SIZE, sz + extra_sz);

	/* Jabber error is set if received frame exceeds jabber size */
	arasan_gemac_writel(pd, MAC_RECEIVE_JABBER_SIZE, sz + extra_sz);

	/* EOP will be sent if transmitted frame exceeds jabber size */
	arasan_gemac_writel(pd, MAC_TRANSMIT_JABBER_SIZE, sz + extra_sz);
}

static void arasan_gemac_setup_fifo_thresholds(struct arasan_gemac_pdata *pd)
{
	/* limitation required by vendor */
	const int max = ARASAN_FIFO_SZ - 8;

	/* FIXME: It can damp difference between DMA and GEMAC speed.
	 * DMA has been stopped if it crosses full threshold.
	 * At this time GEMAC still transmit data to a link and
	 * DMA can be resumed when GEMAC crosses empty threshold.
	 * Because GEMAC still transmits it can flush FIFO before DMA
	 * brings new data, thus packet will be dropped.
	 * This scenario hasn't been confirmed. */
	const int min = 8;

	/* each location is 32 bits */
	arasan_gemac_writel(pd, MAC_TRANSMIT_FIFO_ALMOST_FULL, max);
	arasan_gemac_writel(pd, MAC_TRANSMIT_FIFO_ALMOST_EMPTY_THRESHOLD, min);
}

static void arasan_gemac_init(struct arasan_gemac_pdata *pd)
{
	u32 reg;

	arasan_gemac_writel(pd, MAC_ADDRESS_CONTROL,
			    MAC_ADDRESS_CONTROL_ADDRESS_ENABLE(1));

	reg = arasan_gemac_readl(pd, MAC_RECEIVE_CONTROL);
	reg |= MAC_RECEIVE_CONTROL_STORE_AND_FORWARD;
	arasan_gemac_writel(pd, MAC_RECEIVE_CONTROL, reg);

	arasan_gemac_setup_fifo_thresholds(pd);

	arasan_gemac_setup_frame_limits(pd, pd->dev->mtu);

	arasan_gemac_set_hwaddr(pd->dev);
}

static int arasan_gemac_alloc_rx_desc(struct arasan_gemac_pdata *pd, int index)
{
	struct sk_buff *skb;
	dma_addr_t mapping;
	int len;
	bool last = index == (RX_RING_SIZE - 1);

	skb = netdev_alloc_skb(pd->dev, mtu_to_buf_sz(pd->dev->mtu));
	if (unlikely(!skb))
		return -ENOMEM;

	len = skb_tailroom(skb);

	mapping = dma_map_single(&pd->pdev->dev, skb_tail_pointer(skb), len,
				 DMA_FROM_DEVICE);

	if (dma_mapping_error(&pd->pdev->dev, mapping)) {
		dev_kfree_skb_any(skb);
		netdev_warn(pd->dev, "dma_map_single failed!\n");
		return -ENOMEM;
	}

	pd->rx_buffers[index].skb = skb;
	pd->rx_buffers[index].mapping = mapping;

	/* check if we are at the last descriptor and need to set EOR */
	pd->rx_ring[index].misc = last ? DMA_RDES1_EOR | len : len;
	pd->rx_ring[index].buffer1 = mapping + NET_IP_ALIGN;

	/* ensures that descriptor has been initialized */
	dma_wmb();

	/* assign ownership to DMAC */
	pd->rx_ring[index].status = DMA_RDES0_OWN_BIT;

	/* Strictly speaking, a barrier is required here.
	 * Caller should provide it.
	 */

	return 0;
}

static void arasan_gemac_free_rx_desc(struct arasan_gemac_pdata *pd, int index)
{
	struct arasan_gemac_ring_info *desc = &pd->rx_buffers[index];
	int len;

	if (desc->skb) {
		len = skb_tailroom(desc->skb);
		WARN_ON(len == 0);
		dma_unmap_single(&pd->pdev->dev, desc->mapping, len,
				 DMA_FROM_DEVICE);
		dev_kfree_skb_any(desc->skb);

		desc->skb = NULL;
		desc->mapping = 0;
	}
}

static void arasan_gemac_free_tx_desc(struct arasan_gemac_pdata *pd, int index)
{
	struct arasan_gemac_ring_info *desc = &pd->tx_buffers[index];

	if (desc->skb) {
		WARN_ON(!desc->mapping);
		dma_unmap_single(&pd->pdev->dev, desc->mapping, desc->skb->len,
				 DMA_TO_DEVICE);
		dev_kfree_skb_any(desc->skb);

		desc->skb = NULL;
		desc->mapping = 0;
	}
}

static void arasan_gemac_free_tx_ring(struct arasan_gemac_pdata *pd)
{
	int i;
	int dma_sz = TX_RING_SIZE * sizeof(struct arasan_gemac_dma_desc);

	if (pd->tx_buffers) {
		for (i = 0; i < TX_RING_SIZE; i++)
			arasan_gemac_free_tx_desc(pd, i);

		kfree(pd->tx_buffers);
		pd->tx_buffers = NULL;
	}

	if (pd->tx_ring) {
		dma_free_coherent(&pd->pdev->dev, dma_sz, pd->tx_ring,
				  pd->tx_dma_addr);
		pd->tx_ring = NULL;
	}

	pd->tx_ring_head = 0;
	pd->tx_ring_tail = 0;
}

static void arasan_gemac_free_rx_ring(struct arasan_gemac_pdata *pd)
{
	int i;
	int dma_sz = RX_RING_SIZE * sizeof(struct arasan_gemac_dma_desc);

	if (pd->rx_buffers) {
		for (i = 0; i < RX_RING_SIZE; i++)
			arasan_gemac_free_rx_desc(pd, i);

		kfree(pd->rx_buffers);
		pd->rx_buffers = NULL;
	}

	if (pd->rx_ring) {
		dma_free_coherent(&pd->pdev->dev, dma_sz, pd->rx_ring,
				  pd->rx_dma_addr);
		pd->rx_ring = NULL;
	}

	pd->rx_ring_head = 0;
	pd->rx_ring_tail = 0;
}

static int arasan_gemac_alloc_tx_ring(struct arasan_gemac_pdata *pd)
{
	int dma_sz = TX_RING_SIZE * sizeof(struct arasan_gemac_dma_desc);
	int cpu_sz = TX_RING_SIZE * sizeof(struct arasan_gemac_ring_info);

	pd->tx_ring = dma_zalloc_coherent(&pd->pdev->dev, dma_sz,
					  &pd->tx_dma_addr, GFP_KERNEL);
	if (!pd->tx_ring)
		return -ENOMEM;

	pd->tx_buffers = kzalloc(cpu_sz, GFP_KERNEL);

	if (!pd->tx_buffers)
		return -ENOMEM;

	pd->tx_ring_head = 0;
	pd->tx_ring_tail = 0;

	/* Memory barrier is required here and is provided by writel(). */
	arasan_gemac_writel(pd, DMA_TRANSMIT_BASE_ADDRESS, pd->tx_dma_addr);

	return 0;
}

static int arasan_gemac_alloc_rx_ring(struct arasan_gemac_pdata *pd)
{
	int i;
	int dma_sz = RX_RING_SIZE * sizeof(struct arasan_gemac_dma_desc);
	int cpu_sz = RX_RING_SIZE * sizeof(struct arasan_gemac_ring_info);

	pd->rx_ring = dma_zalloc_coherent(&pd->pdev->dev, dma_sz,
					  &pd->rx_dma_addr, GFP_KERNEL);
	if (!pd->rx_ring)
		return -ENOMEM;

	pd->rx_buffers = kzalloc(cpu_sz, GFP_KERNEL);

	if (!pd->rx_buffers)
		return -ENOMEM;

	/* now allocate the entire ring of skbs */
	for (i = 0; i < RX_RING_SIZE; i++) {
		if (arasan_gemac_alloc_rx_desc(pd, i)) {
			netdev_err(pd->dev,
				   "failed to allocate rx skb %d\n", i);
			return -ENOMEM;
		}
	}

	pd->rx_ring_head = 0;
	pd->rx_ring_tail = 0;

	/* Memory barrier is required here and is provided by writel(). */
	arasan_gemac_writel(pd, DMA_RECEIVE_BASE_ADDRESS, pd->rx_dma_addr);

	return 0;
}

static inline void arasan_gemac_tx_update_stats(struct net_device *dev,
						u32 status, u32 length)
{
	if (unlikely(status & 0x7fffffff)) {
		dev->stats.tx_errors++;
	} else {
		dev->stats.tx_packets++;
		dev->stats.tx_bytes += (length & 0xFFF);
	}
}

/* Check for completed dma transfers, update stats and free skbs */
static bool arasan_gemac_try_complete_tx(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	int freed = 0;
	unsigned long flags;

	/* try_complete_tx() can be called in IRQ handler or start_xmit() */
	if (!spin_trylock_irqsave(&pd->tx_freelock, flags))
		return false;

	do {
		u32 status, misc;

		int tail = pd->tx_ring_tail;

		/* synchronize with start_xmit() */
		int head = smp_load_acquire(&pd->tx_ring_head);

		if (tail == head)
			break;

		/* ensures that CPU sees actual state */
		dma_rmb();

		status = pd->tx_ring[tail].status;
		misc = pd->tx_ring[tail].misc;

		/* Check if DMA still owns this descriptor */
		if (unlikely(DMA_TDES0_OWN_BIT & status))
			break;

		arasan_gemac_tx_update_stats(dev, status, misc);

		arasan_gemac_free_tx_desc(pd, tail);
		freed++;

		/* synchronize for start_xmit() */
		smp_store_release(&pd->tx_ring_tail, (tail + 1) % TX_RING_SIZE);
	} while (true);

	spin_unlock_irqrestore(&pd->tx_freelock, flags);
	return freed > 0;
}

/* Transmit packet */
static int arasan_gemac_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	dma_addr_t mapping;
	int head, tail;
	u32 tmp_desc1;

	arasan_gemac_try_complete_tx(dev);

	head = pd->tx_ring_head;

	/* synchronize with complete_tx() */
	tail = smp_load_acquire(&pd->tx_ring_tail);

	WARN_ON(pd->tx_ring[head].status & DMA_TDES0_OWN_BIT);

	mapping = dma_map_single(&pd->pdev->dev, skb->data,
				 skb->len, DMA_TO_DEVICE);

	if (dma_mapping_error(&pd->pdev->dev, mapping)) {
		netdev_warn(dev, "dma_map_single failed, dropping packet\n");
		return NETDEV_TX_BUSY;
	}

	/* skb_tx_timestamp() should be called before
	 * preparing the descriptor, because at this time the DMA can work
	 * without kicking.
	 */

	skb_tx_timestamp(skb);

	pd->tx_buffers[head].skb = skb;
	pd->tx_buffers[head].mapping = mapping;

	if (unlikely(((head + 2) % TX_RING_SIZE) == tail))
		netif_stop_queue(pd->dev);

	tmp_desc1 = (DMA_TDES1_LS | DMA_TDES1_FS | ((u32)skb->len & 0xFFF));

	/* check if we are at the last descriptor and need to set EOR */
	if (unlikely(head == (TX_RING_SIZE - 1)))
		tmp_desc1 |= DMA_TDES1_EOR;

	pd->tx_ring[head].buffer1 = mapping;
	pd->tx_ring[head].misc = tmp_desc1;

	/* ensures that descriptor has been initialized */
	dma_wmb();

	/* assign ownership to DMAC */
	pd->tx_ring[head].status = DMA_TDES0_OWN_BIT;

	/* synchronize head for complete_tx() */
	smp_store_release(&pd->tx_ring_head, (head + 1)  % TX_RING_SIZE);

	/* Memory barrier is required here and is provided by writel().
	 * kick the DMA
	 */
	arasan_gemac_writel(pd, DMA_TRANSMIT_POLL_DEMAND, 1);

	return NETDEV_TX_OK;
}

static void arasan_gemac_alloc_new_rx_buffers(struct arasan_gemac_pdata *pd)
{
	while (pd->rx_ring_tail != pd->rx_ring_head) {
		WARN_ON(pd->rx_buffers[pd->rx_ring_tail].skb);

		if (arasan_gemac_alloc_rx_desc(pd, pd->rx_ring_tail))
			break;

		pd->rx_ring_tail = (pd->rx_ring_tail + 1) % RX_RING_SIZE;
	}
}

static void arasan_gemac_rx_handoff(struct arasan_gemac_pdata *pd,
				    const int index, const u32 status)
{
	struct net_device *dev = pd->dev;
	struct sk_buff *skb;
	u16 packet_length = (status & 0x3fff);

	/* remove crc from packet lendth */
	packet_length -= 4;

	dev->stats.rx_packets++;
	dev->stats.rx_bytes += packet_length;

	dma_unmap_single(&pd->pdev->dev, pd->rx_buffers[index].mapping,
			 packet_length, DMA_FROM_DEVICE);

	skb = pd->rx_buffers[index].skb;

	pd->rx_buffers[index].skb = NULL;
	pd->rx_buffers[index].mapping = 0;

	skb_reserve(skb, NET_IP_ALIGN);
	skb_put(skb, packet_length);

	skb->protocol = eth_type_trans(skb, dev);

	netif_receive_skb(skb);
}

static void arasan_gemac_rx_count_stats(struct net_device *dev, u32 desc_status)
{
	if (unlikely(!((desc_status & DMA_RDES0_FD) &&
		       (desc_status & DMA_RDES0_LD))))
		dev->stats.rx_length_errors++;
}

static int arasan_gemac_rx_poll(struct napi_struct *napi, int budget)
{
	struct arasan_gemac_pdata *pd =
		container_of(napi, struct arasan_gemac_pdata, napi);

	struct net_device *dev = pd->dev;
	u32 drop_frame_cnt, dma_intr_ena, status;
	int work_done;

	for (work_done = 0; work_done < budget; work_done++) {
		/* ensures that CPU sees actual state */
		dma_rmb();

		status = pd->rx_ring[pd->rx_ring_head].status;

		/* stop if DMAC owns this dma descriptor */
		if (status & DMA_RDES0_OWN_BIT)
			break;

		arasan_gemac_rx_count_stats(dev, status);
		arasan_gemac_rx_handoff(pd, pd->rx_ring_head, status);
		pd->rx_ring_head = (pd->rx_ring_head + 1) % RX_RING_SIZE;
	}

	arasan_gemac_alloc_new_rx_buffers(pd);

	drop_frame_cnt = arasan_gemac_readl(pd, DMA_MISSED_FRAME_COUNTER);
	dev->stats.rx_dropped += drop_frame_cnt;

	/* Memory barrier is required here and is provided by writel().
	 * Kick RXDMA.
	 */
	arasan_gemac_writel(pd, DMA_RECEIVE_POLL_DEMAND, 1);

	if (work_done < budget) {
		napi_complete(&pd->napi);
		/* re-enable RX DMA interrupts */
		dma_intr_ena = arasan_gemac_readl(pd, DMA_INTERRUPT_ENABLE);
		dma_intr_ena |= DMA_INTERRUPT_ENABLE_RECEIVE_DONE;
		arasan_gemac_writel(pd, DMA_INTERRUPT_ENABLE, dma_intr_ena);
	}
	return work_done;
}

static int arasan_gemac_try_up_tx_threshold(struct arasan_gemac_pdata *pd)
{
	int threshold;
	/* if threshold is equal max threshold that GEMAC will work
	 * in Store and Forward mode. */
	const int maxthreshold = 1518;

	/* get current threshold */
	threshold = arasan_gemac_readl(pd, MAC_TRANSMIT_PACKET_START_THRESHOLD);

	if (threshold >= maxthreshold)
		return false;

	threshold = min(maxthreshold, threshold + 32);

	arasan_gemac_writel(pd, MAC_TRANSMIT_PACKET_START_THRESHOLD, threshold);
	return true;
}

static void arasan_gemac_set_threshold(struct arasan_gemac_pdata *pd)
{
	int tx_tr, rx_tr;

	/* set initial TX threshold recommended by vendor */
	switch (pd->phy_dev->speed) {
	case SPEED_10:
		tx_tr = 64;
		break;
	case SPEED_100:
		tx_tr = 128;
		break;
	case SPEED_1000:
	default:
		tx_tr = 1024;
	}

	/* no obvious rules for RX threshold */
	rx_tr = 64;

	arasan_gemac_writel(pd, MAC_TRANSMIT_PACKET_START_THRESHOLD, tx_tr);
	arasan_gemac_writel(pd, MAC_RECEIVE_PACKET_START_THRESHOLD, rx_tr);

	/* Underrun interrupt is enabled to adjust TX threshold
	 * if underrun condition occurs */
	arasan_gemac_writel(pd, MAC_INTERRUPT_ENABLE,
			    MAC_INTERRUPT_ENABLE_UNDERRUN);
}

void arasan_gemac_mac_interrupt(struct arasan_gemac_pdata *pd)
{
	u32 sts, irq, clr = 0;

	sts = arasan_gemac_readl(pd, MAC_INTERRUPT_STATUS);

	if (sts & MAC_IRQ_STATUS_UNDERRUN) {
		clr |= MAC_IRQ_STATUS_UNDERRUN;
		/* Underrun condition occurs when DMA doesn't have time
		 * for deliver rest part of packet to FIFO. We can increase
		 * GEMAC start transmitting threshold.
		 * TODO: Inform upper layer that packet has been dropped. */
		if (!arasan_gemac_try_up_tx_threshold(pd)) {
			irq = arasan_gemac_readl(pd, MAC_INTERRUPT_ENABLE);
			irq &= ~MAC_INTERRUPT_ENABLE_UNDERRUN;
			arasan_gemac_writel(pd, MAC_INTERRUPT_ENABLE, irq);
		}
	}

	arasan_gemac_writel(pd, MAC_INTERRUPT_STATUS, clr);
}

static irqreturn_t arasan_gemac_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	u32 int_sts, ints_to_clear;

	int_sts = arasan_gemac_readl(pd, DMA_STATUS_AND_IRQ);

	ints_to_clear = 0;

	if (int_sts & DMA_STATUS_AND_IRQ_TRANS_DESC_UNAVAIL) {
		ints_to_clear |= DMA_STATUS_AND_IRQ_TRANS_DESC_UNAVAIL;

		if (arasan_gemac_try_complete_tx(dev))
			netif_wake_queue(pd->dev);
	}

	if (int_sts & DMA_STATUS_AND_IRQ_RECEIVE_DONE) {
		/* mask RX DMAC interrupts */
		u32 dma_intr_ena = arasan_gemac_readl(pd, DMA_INTERRUPT_ENABLE);

		dma_intr_ena &= (~DMA_INTERRUPT_ENABLE_RECEIVE_DONE);
		arasan_gemac_writel(pd, DMA_INTERRUPT_ENABLE, dma_intr_ena);

		ints_to_clear |= DMA_STATUS_AND_IRQ_RECEIVE_DONE;
		napi_schedule(&pd->napi);
	}

	if (int_sts & DMA_STATUS_AND_IRQ_MAC) {
		ints_to_clear |= DMA_STATUS_AND_IRQ_MAC;
		arasan_gemac_mac_interrupt(pd);
	}

	if (ints_to_clear)
		arasan_gemac_writel(pd, DMA_STATUS_AND_IRQ, ints_to_clear);

	return IRQ_HANDLED;
}

static void arasan_gemac_stop_tx_dma(struct arasan_gemac_pdata *pd)
{
	u32 reg;
	int timeout = 1000;

	/* disable TX DMAC */
	reg = arasan_gemac_readl(pd, DMA_CONTROL);
	reg &= ~DMA_CONTROL_START_TRANSMIT_DMA;
	arasan_gemac_writel(pd, DMA_CONTROL, reg);

	/* Wait max 20 ms for transmit process to stop */
	while (--timeout) {
		reg = arasan_gemac_readl(pd, DMA_STATUS_AND_IRQ);
		if (!DMA_STATUS_AND_IRQ_TRANSMIT_DMA_STATE(reg))
			break;
		usleep_range(10, 20);
	}

	if (!timeout)
		netdev_warn(pd->dev, "TX DMAC failed to stop\n");

	/* ACK Tx DMAC stop bit */
	arasan_gemac_writel(pd, DMA_STATUS_AND_IRQ,
			    DMA_STATUS_AND_IRQ_TX_DMA_STOPPED);
}

static void arasan_gemac_start_tx_dma(struct arasan_gemac_pdata *pd)
{
	u32 reg;

	reg = arasan_gemac_readl(pd, DMA_CONTROL);
	reg |= DMA_CONTROL_START_TRANSMIT_DMA;
	arasan_gemac_writel(pd, DMA_CONTROL, reg);
}

static void arasan_gemac_stop_tx_mac(struct arasan_gemac_pdata *pd)
{
	u32 reg;

	/* mask TX DMAC interrupts */
	reg = arasan_gemac_readl(pd, DMA_INTERRUPT_ENABLE);
	reg &= ~DMA_INTERRUPT_ENABLE_TRANSMIT_DONE;
	arasan_gemac_writel(pd, DMA_INTERRUPT_ENABLE, reg);

	/* writel() guarantees that interrupts will be masked
	 * before stopping MAC TX
	 */

	reg = arasan_gemac_readl(pd, MAC_TRANSMIT_CONTROL);
	reg &= ~MAC_TRANSMIT_CONTROL_TRANSMIT_ENABLE;
	arasan_gemac_writel(pd, MAC_TRANSMIT_CONTROL, reg);
}

static void arasan_gemac_start_tx_mac(struct arasan_gemac_pdata *pd)
{
	u32 reg;

	reg = arasan_gemac_readl(pd, MAC_TRANSMIT_CONTROL);
	reg |= MAC_TRANSMIT_CONTROL_TRANSMIT_ENABLE;
	arasan_gemac_writel(pd, MAC_TRANSMIT_CONTROL, reg);
}

static void arasan_gemac_stop_tx(struct arasan_gemac_pdata *pd)
{
	arasan_gemac_stop_tx_dma(pd);
	arasan_gemac_stop_tx_mac(pd);
}

static void arasan_gemac_stop_rx_dma(struct arasan_gemac_pdata *pd)
{
	u32 reg;
	int timeout = 1000;

	/* stop RX DMAC */
	reg = arasan_gemac_readl(pd, DMA_CONTROL);
	reg &= ~DMA_CONTROL_START_RECEIVE_DMA;
	arasan_gemac_writel(pd, DMA_CONTROL, reg);

	/* Wait max 20 ms for receive process to stop */
	while (--timeout) {
		reg = arasan_gemac_readl(pd, DMA_STATUS_AND_IRQ);
		if (!DMA_STATUS_AND_IRQ_RECEIVE_DMA_STATE(reg))
			break;
		usleep_range(10, 20);
	}

	if (!timeout)
		netdev_warn(pd->dev, "RX DMAC failed to stop\n");

	/* ACK the Rx DMAC stop bit */
	arasan_gemac_writel(pd, DMA_STATUS_AND_IRQ,
			    DMA_STATUS_AND_IRQ_RX_DMA_STOPPED);
}

static void arasan_gemac_start_rx_dma(struct arasan_gemac_pdata *pd)
{
	u32 reg;

	reg = arasan_gemac_readl(pd, DMA_CONTROL);
	reg |= DMA_CONTROL_START_RECEIVE_DMA;
	arasan_gemac_writel(pd, DMA_CONTROL, reg);
}

static void arasan_gemac_stop_rx_mac(struct arasan_gemac_pdata *pd)
{
	u32 reg;

	/* mask RX DMAC interrupts */
	reg = arasan_gemac_readl(pd, DMA_INTERRUPT_ENABLE);
	reg &= ~DMA_INTERRUPT_ENABLE_RECEIVE_DONE;
	arasan_gemac_writel(pd, DMA_INTERRUPT_ENABLE, reg);

	/* writel() guarantees that interrupts will be masked
	 * before stopping RX MAC.
	 */

	reg = arasan_gemac_readl(pd, MAC_RECEIVE_CONTROL);
	reg &= ~MAC_RECEIVE_CONTROL_RECEIVE_ENABLE;
	arasan_gemac_writel(pd, MAC_RECEIVE_CONTROL, reg);
}

static void arasan_gemac_start_rx_mac(struct arasan_gemac_pdata *pd)
{
	u32 reg;

	reg = arasan_gemac_readl(pd, MAC_RECEIVE_CONTROL);
	reg |= MAC_RECEIVE_CONTROL_RECEIVE_ENABLE;
	arasan_gemac_writel(pd, MAC_RECEIVE_CONTROL, reg);
}

static void arasan_gemac_stop_rx(struct arasan_gemac_pdata *pd)
{
	arasan_gemac_stop_rx_mac(pd);
	arasan_gemac_stop_rx_dma(pd);
}

static void arasan_gemac_start_rx(struct arasan_gemac_pdata *pd)
{
	arasan_gemac_start_rx_mac(pd);
	arasan_gemac_start_rx_dma(pd);
}

static void arasan_gemac_start_tx(struct arasan_gemac_pdata *pd)
{
	arasan_gemac_start_tx_mac(pd);
	arasan_gemac_start_tx_dma(pd);
}

static void arasan_gemac_stop_mac(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);

	netif_tx_disable(dev);
	napi_disable(&pd->napi);

	arasan_gemac_stop_tx(pd);
	arasan_gemac_free_tx_ring(pd);

	arasan_gemac_stop_rx(pd);
	arasan_gemac_free_rx_ring(pd);

	arasan_gemac_dma_soft_reset(pd);
}

static int arasan_gemac_stop(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);

	arasan_gemac_stop_mac(dev);

	phy_stop(pd->phy_dev);
	/* TODO: We should somehow power down PHY */

	phy_disconnect(pd->phy_dev);
	mdiobus_unregister(pd->mii_bus);
	mdiobus_free(pd->mii_bus);

	return 0;
}

static int arasan_gemac_get_gpio(struct platform_device *pdev, char *name)
{
	int rc, gpio;
	struct device_node *np = pdev->dev.of_node;

	if (!np)
		return -ENODEV;

	gpio = of_get_named_gpio(np, name, 0);
	if (!gpio_is_valid(gpio))
		return gpio;

	rc = devm_gpio_request_one(&pdev->dev, gpio,
				   GPIOF_OUT_INIT_LOW, name);

	if (rc)
		return rc;

	return gpio;
}

static void arasan_gemac_reset_phy(struct platform_device *pdev)
{
	int gpio;

	gpio = arasan_gemac_get_gpio(pdev, "phy-reset-gpios");
	if (!gpio_is_valid(gpio)) {
		dev_warn(&pdev->dev, "Failed to get phy-reset-gpios\n");
		return;
	}

	/* FIXME
	 * 20 msec is actually too much for phy resetting. But if we set
	 * the reset time less than 20 msec check patch script is failed.
	 */

	msleep(20);
	gpio_set_value(gpio, 1);
}

static int arasan_gemac_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct arasan_gemac_pdata *pd = bus->priv;
	int value;

	arasan_gemac_writel(pd, MAC_MDIO_CONTROL,
			    MAC_MDIO_CONTROL_READ_WRITE(1) |
			    MAC_MDIO_CONTROL_REG_ADDR(regnum) |
			    MAC_MDIO_CONTROL_PHY_ADDR(mii_id) |
			    MAC_MDIO_CONTROL_START_FRAME(1));

	/* wait for end of transfer */
	while ((arasan_gemac_readl(pd, MAC_MDIO_CONTROL) >> 15))
		cpu_relax();

	value = arasan_gemac_readl(pd, MAC_MDIO_DATA);

	return value;
}

static int arasan_gemac_mdio_write(struct mii_bus *bus, int mii_id,
				   int regnum, u16 value)
{
	struct arasan_gemac_pdata *pd = bus->priv;

	arasan_gemac_writel(pd, MAC_MDIO_DATA, value);

	arasan_gemac_writel(pd, MAC_MDIO_CONTROL,
			    MAC_MDIO_CONTROL_START_FRAME(1) |
			    MAC_MDIO_CONTROL_PHY_ADDR(mii_id) |
			    MAC_MDIO_CONTROL_REG_ADDR(regnum) |
			    MAC_MDIO_CONTROL_READ_WRITE(0));

	/* wait for end of transfer */
	while ((arasan_gemac_readl(pd, MAC_MDIO_CONTROL) >> 15))
		cpu_relax();

	return 0;
}

/* Reconfigure Arasan GEMAC according to speed and duplex value */
static void arasan_gemac_reconfigure(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	struct phy_device *phydev = pd->phy_dev;
	u32 reg;

	reg = arasan_gemac_readl(pd, MAC_GLOBAL_CONTROL);
	reg &= ~(MAC_GLOBAL_CONTROL_SPEED(3) |
		 MAC_GLOBAL_CONTROL_DUPLEX_MODE(1));

	switch (phydev->duplex) {
	case DUPLEX_HALF:
		break;
	case DUPLEX_FULL:
		reg |= MAC_GLOBAL_CONTROL_DUPLEX_MODE(DUPLEX_FULL);
		break;
	default:
		netdev_err(dev, "Unknown duplex (%d)\n", phydev->duplex);
		return;
	}

	switch (phydev->speed) {
	case SPEED_100:
		reg |= MAC_GLOBAL_CONTROL_SPEED(1);

		if (gpio_is_valid(pd->txclk_125en))
			gpio_set_value(pd->txclk_125en, 0);
		break;
	case SPEED_1000:
		reg |= MAC_GLOBAL_CONTROL_SPEED(2);

		if (gpio_is_valid(pd->txclk_125en))
			gpio_set_value(pd->txclk_125en, 1);
		break;
	default:
		netdev_err(dev, "Unknown speed (%d)\n", phydev->speed);
		return;
	}
	arasan_gemac_set_threshold(pd);

	arasan_gemac_writel(pd, MAC_GLOBAL_CONTROL, reg);
}

static void arasan_gemac_handle_link_change(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	struct phy_device *phydev = pd->phy_dev;
	unsigned long flags;

	int status_change = 0;

	spin_lock_irqsave(&pd->lock, flags);

	if ((phydev->link) &&
	    ((pd->speed != phydev->speed) || (pd->duplex != phydev->duplex))) {
		arasan_gemac_reconfigure(dev);
		pd->speed = phydev->speed;
		pd->duplex = phydev->duplex;
		status_change = 1;
	}

	if (phydev->link != pd->link) {
		if (!phydev->link) {
			pd->speed = 0;
			pd->duplex = -1;
		}
		pd->link = phydev->link;
		status_change = 1;
	}

	spin_unlock_irqrestore(&pd->lock, flags);

	if (status_change) {
		if (phydev->link) {
			netif_carrier_on(dev);
			netdev_info(dev, "link up (%d/%s)\n",
				    phydev->speed,
				    phydev->duplex == DUPLEX_FULL ?
				    "Full" : "Half");
		} else {
			netif_carrier_off(dev);
			netdev_info(dev, "link down\n");
		}
	}
}

static int arasan_gemac_mii_probe(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	struct phy_device *phydev;

	phydev = phy_find_first(pd->mii_bus);
	if (!phydev) {
		netdev_err(dev, "no PHY found\n");
		return -ENXIO;
	}

	phydev = phy_connect(dev, dev_name(&phydev->dev),
			     arasan_gemac_handle_link_change,
			     pd->phy_interface);

	if (IS_ERR(phydev)) {
		netdev_err(dev, "Could not attach to PHY\n");
		return PTR_ERR(phydev);
	}

	netdev_info(dev,
		    "attached PHY driver [%s] (mii_bus:phy_addr=%s, irq=%d)\n",
		    phydev->drv->name, dev_name(&phydev->dev), phydev->irq);

	phydev->supported &= ARASAN_GEMAC_FEATURES;
	if (pd->phy_interface == PHY_INTERFACE_MODE_MII)
		phydev->supported &= ~PHY_1000BT_FEATURES;

	phydev->advertising = phydev->supported;

	pd->link = 0;
	pd->speed = 0;
	pd->duplex = -1;
	pd->phy_dev = phydev;

	return 0;
}

static int arasan_gemac_mii_init(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	struct device_node *np;
	int err = -ENXIO;
	u32 divisor;

	pd->mii_bus = mdiobus_alloc();
	if (!pd->mii_bus) {
		err = -ENOMEM;
		goto err_out;
	}

	pd->mii_bus->name = "arasan-gemac-mii-bus";
	pd->mii_bus->read = &arasan_gemac_mdio_read;
	pd->mii_bus->write = &arasan_gemac_mdio_write;
	/* TODO: pd->mii_bus->reset also should be implemented to allow
	 * reset of Ethernet PHY from user space (see MII-TOOL utility)
	 */

	/* Maximum allowing MDC clock is 2.5 MHz */
	divisor = DIV_ROUND_UP(clk_get_rate(pd->hclk), 2500000);
	arasan_gemac_writel(pd, MAC_MDIO_CLOCK_DIVISION_CONTROL, divisor);

	snprintf(pd->mii_bus->id, MII_BUS_ID_SIZE, "%s-0x%x",
		 pd->pdev->name, pd->pdev->id);

	pd->mii_bus->priv = pd;
	pd->mii_bus->parent = &pd->dev->dev;

	pd->mii_bus->irq = pd->phy_irq;

	np = pd->pdev->dev.of_node;
	if (np) {
		/* try dt phy registration */
		err = of_mdiobus_register(pd->mii_bus, np);

		if (err) {
			netdev_err(dev,
				   "Failed to register mdio bus, error: %d\n",
				   err);
			goto err_out_free_mdiobus;
		}
	} else {
		netdev_err(dev, "Missing device tree node\n");
		goto err_out_free_mdiobus;
	}

	err = arasan_gemac_mii_probe(dev);
	if (err)
		goto err_out_unregister_bus;

	return 0;

err_out_unregister_bus:
	mdiobus_unregister(pd->mii_bus);
err_out_free_mdiobus:
	mdiobus_free(pd->mii_bus);
err_out:
	return err;
}

int arasan_gemac_start_mac(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	int result;

	result = arasan_gemac_alloc_tx_ring(pd);
	if (result) {
		netdev_err(pd->dev, "Failed to Initialize tx dma ring\n");
		goto err;
	}

	result = arasan_gemac_alloc_rx_ring(pd);
	if (result) {
		netdev_err(pd->dev, "Failed to Initialize rx dma ring\n");
		goto err;
	}

	arasan_gemac_init(pd);

	napi_enable(&pd->napi);

	/* Enable interrupts */
	arasan_gemac_writel(pd, DMA_INTERRUPT_ENABLE,
			    DMA_INTERRUPT_ENABLE_RECEIVE_DONE |
			    DMA_INTERRUPT_ENABLE_TRANS_DESC_UNAVAIL |
			    DMA_INTERRUPT_ENABLE_MAC);

	/* Enable packet transmission */
	arasan_gemac_start_tx(pd);

	/* Enable packet reception */
	arasan_gemac_start_rx(pd);

	netif_start_queue(dev);

	return 0;
err:
	/* explicit cleanup even if something is partially initialized */
	arasan_gemac_free_tx_ring(pd);
	arasan_gemac_free_rx_ring(pd);
	return result;
}

/* Open the Ethernet interface */
static int arasan_gemac_open(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	int res;

	res = arasan_gemac_start_mac(dev);
	if (res)
		return res;

	res = arasan_gemac_mii_init(dev);
	if (res) {
		netdev_err(dev, "Failed to initialize Phy\n");
		arasan_gemac_stop_mac(dev);
		return res;
	}
	/* schedule a link state check */
	phy_start(pd->phy_dev);

	return 0;
}

static void arasan_gemac_set_rx_mode(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	struct netdev_hw_addr *ha;
	u32 hash[4] = { 0 };
	u32 reg;

	reg = arasan_gemac_readl(pd, MAC_ADDRESS_CONTROL);

	/* enable promiscuous mode if requested */
	if (dev->flags & IFF_PROMISC)
		reg |= MAC_ADDRESS_CONTROL_PROMISCUOUS_MODE;
	else
		reg &= ~MAC_ADDRESS_CONTROL_PROMISCUOUS_MODE;

	arasan_gemac_writel(pd, MAC_ADDRESS_CONTROL, reg);

	/* receive multicast frames */
	if (dev->flags & IFF_ALLMULTI) {
		hash[0] = 0x0000ffff;
		hash[1] = 0x0000ffff;
		hash[2] = 0x0000ffff;
		hash[3] = 0x0000ffff;
	} else {
		netdev_for_each_mc_addr(ha, dev) {
			u32 crc = ether_crc(ETH_ALEN, ha->addr);
			int bit = (crc >> 26) & 0x3f;

			hash[bit / 16] |= BIT(bit % 16);
		}
	}

	arasan_gemac_writel(pd, MAC_HASH_TABLE1, hash[0]);
	arasan_gemac_writel(pd, MAC_HASH_TABLE2, hash[1]);
	arasan_gemac_writel(pd, MAC_HASH_TABLE3, hash[2]);
	arasan_gemac_writel(pd, MAC_HASH_TABLE4, hash[3]);
}

static int arasan_gemac_set_mac_address(struct net_device *dev, void *addr)
{
	if (netif_running(dev))
		return -EBUSY;

	/* sa_family is validated by calling code */
	ether_addr_copy(dev->dev_addr, ((struct sockaddr *)addr)->sa_data);
	arasan_gemac_set_hwaddr(dev);

	return 0;
}

static int arasan_gemac_change_mtu(struct net_device *dev, int new_mtu)
{
	if (new_mtu > ARASAN_JUMBO_MTU || new_mtu < 68)
		return -EINVAL;

	if (netif_running(dev))
		arasan_gemac_stop_mac(dev);

	dev->mtu = new_mtu;

	if (netif_running(dev))
		arasan_gemac_start_mac(dev);

	return 0;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void arasan_gemac_poll_controller(struct net_device *dev)
{
	unsigned long flags;

	local_irq_save(flags);
	arasan_gemac_interrupt(dev->irq, dev);
	local_irq_restore(flags);
}
#endif

static const struct net_device_ops arasan_gemac_netdev_ops = {
	.ndo_open       = arasan_gemac_open,
	.ndo_stop       = arasan_gemac_stop,
	.ndo_start_xmit = arasan_gemac_start_xmit,
	.ndo_set_rx_mode = arasan_gemac_set_rx_mode,
	.ndo_set_mac_address = arasan_gemac_set_mac_address,
	.ndo_change_mtu = arasan_gemac_change_mtu,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = arasan_gemac_poll_controller,
#endif
};

#if defined(CONFIG_OF)
static const struct of_device_id arasan_gemac_dt_ids[] = {
	{ .compatible = "elvees,arasan-gemac" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, arasan_gemac_dt_ids);
#endif

static int arasan_gemac_probe(struct platform_device *pdev)
{
	struct resource *regs;
	struct net_device *dev;
	struct arasan_gemac_pdata *pd;
	int res;
	const char *mac;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENOENT;

	/* Allocate and set up ethernet device */
	dev = alloc_etherdev(sizeof(struct arasan_gemac_pdata));
	if (!dev)
		return -ENOMEM;

	pd = netdev_priv(dev);
	pd->pdev = pdev;
	pd->dev = dev;
	spin_lock_init(&pd->lock);
	spin_lock_init(&pd->tx_freelock);

	/* Try to get and enable Arasan GEMAC hclk */
	pd->hclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pd->hclk)) {
		res = PTR_ERR(pd->hclk);
		dev_err(&pdev->dev,
			"failed to get Arasan GEMAC hclk (%u)\n", res);
		goto err_free_dev;
	}

	res = clk_prepare_enable(pd->hclk);
	if (res) {
		dev_err(&pdev->dev,
			"failed to enable Arasan GEMAC hclk (%u)\n", res);
		goto err_free_dev;
	}

	/* physical base address */
	dev->base_addr = regs->start;
	pd->regs = devm_ioremap(&pdev->dev, regs->start, resource_size(regs));
	if (!pd->regs) {
		res = -ENOMEM;
		goto err_disable_clocks;
	}

	/* Install the interrupt handler */
	dev->irq = platform_get_irq(pdev, 0);
	res = devm_request_irq(&pdev->dev, dev->irq, arasan_gemac_interrupt,
			       0, dev->name, dev);
	if (res)
		goto err_disable_clocks;

	arasan_gemac_reset_phy(pdev);

	dev->netdev_ops = &arasan_gemac_netdev_ops;
	dev->ethtool_ops = &arasan_gemac_ethtool_ops;
	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	netif_napi_add(dev, &pd->napi, arasan_gemac_rx_poll, NAPI_WEIGHT);

	res = of_get_phy_mode(pdev->dev.of_node);
	if (res < 0)
		goto err_disable_clocks;

	if (res != PHY_INTERFACE_MODE_MII && res != PHY_INTERFACE_MODE_GMII) {
		dev_err(&pdev->dev, "\"%s\" PHY interface is not supported\n",
			phy_modes(res));
		res = -ENODEV;
		goto err_disable_clocks;
	}

	if (res == PHY_INTERFACE_MODE_GMII) {
		pd->txclk_125en = arasan_gemac_get_gpio(pdev,
							"txclk-125en-gpios");
		if (!gpio_is_valid(pd->txclk_125en)) {
			dev_warn(&pdev->dev,
				 "Failed to get txclk-125en-gpios\n");
			dev_warn(&pdev->dev,
				 "Reverting PHY interface to MII\n");
			res = PHY_INTERFACE_MODE_MII;
		}
	}

	pd->phy_interface = res;

	mac = of_get_mac_address(pdev->dev.of_node);
	if (mac)
		ether_addr_copy(pd->dev->dev_addr, mac);
	else
		arasan_gemac_get_hwaddr(pd);

	/* Register the network interface */
	res = register_netdev(dev);
	if (res)
		goto err_disable_clocks;

	netif_carrier_off(dev);

	arasan_gemac_dma_soft_reset(pd);

	/* Display ethernet banner */
	netdev_info(dev, "Arasan GEMAC ethernet at 0x%08lx int=%d (%pM)\n",
		    dev->base_addr, dev->irq, dev->dev_addr);

	return 0;

err_disable_clocks:
	clk_disable_unprepare(pd->hclk);
err_free_dev:
	free_netdev(dev);

	return res;
}

static int arasan_gemac_remove(struct platform_device *pdev)
{
	struct net_device *dev;
	struct arasan_gemac_pdata *pd;

	dev = platform_get_drvdata(pdev);
	if (!dev)
		return 0;

	pd = netdev_priv(dev);

	unregister_netdev(dev);
	clk_disable_unprepare(pd->hclk);
	free_netdev(dev);

	return 0;
}

static struct platform_driver arasan_gemac_driver = {
	.driver = {
		.name = "arasan-gemac",
		.of_match_table = of_match_ptr(arasan_gemac_dt_ids),
	},
	.probe = arasan_gemac_probe,
	.remove = arasan_gemac_remove,
};

module_platform_driver(arasan_gemac_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Arasan GEMAC ethernet driver");
MODULE_AUTHOR("Dmitriy Zagrebin <dzagrebin@elvees.com>");
