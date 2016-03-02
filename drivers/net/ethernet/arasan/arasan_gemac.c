/*
 * Copyright 2015 ELVEES NeoTek CJSC
 * Copyright 2007, 2008 SMSC
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

#include "arasan_gemac.h"

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
	print_reg(MAC_INTERRUPT);
	print_reg(MAC_INTERRUPT_ENABLE);
}

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
	u32 reg;

	reg = arasan_gemac_readl(pd, DMA_CONFIGURATION);
	arasan_gemac_writel(pd, DMA_CONFIGURATION,
			    (reg | DMA_CONFIGURATION_SOFT_RESET));
	/* FIXME
	 * mdelay or msleep ?
	 */

	mdelay(10);
	arasan_gemac_writel(pd, DMA_CONFIGURATION, reg);
}

static void arasan_gemac_init(struct arasan_gemac_pdata *pd)
{
	u32 reg;

	arasan_gemac_writel(pd, MAC_ADDRESS_CONTROL, 1);
	arasan_gemac_writel(pd, MAC_TRANSMIT_FIFO_ALMOST_FULL, (512 - 8));
	arasan_gemac_writel(pd, MAC_TRANSMIT_PACKET_START_THRESHOLD, 128);
	arasan_gemac_writel(pd, MAC_RECEIVE_PACKET_START_THRESHOLD, 64);

	reg = arasan_gemac_readl(pd, MAC_RECEIVE_CONTROL);
	reg |= MAC_RECEIVE_CONTROL_STORE_AND_FORWARD;
	arasan_gemac_writel(pd, MAC_RECEIVE_CONTROL, reg);

	reg = arasan_gemac_readl(pd, DMA_CONFIGURATION);
	reg |= DMA_CONFIGURATION_WAIT_FOR_DONE;
	arasan_gemac_writel(pd, DMA_CONFIGURATION, reg);

	arasan_gemac_set_hwaddr(pd->dev);
}

static int arasan_gemac_alloc_rx_buffer(struct arasan_gemac_pdata *pd, int index)
{
	struct sk_buff *skb = netdev_alloc_skb(pd->dev, PKT_BUF_SZ);
	dma_addr_t mapping;

	WARN_ON(pd->rx_buffers[index].skb);
	WARN_ON(pd->rx_buffers[index].mapping);

	if (unlikely(!skb))
		return -ENOMEM;

	mapping = dma_map_single(&pd->pdev->dev, skb_tail_pointer(skb),
				 PKT_BUF_SZ, DMA_FROM_DEVICE);

	if (dma_mapping_error(&pd->pdev->dev, mapping)) {
		dev_kfree_skb_any(skb);
		netdev_warn(pd->dev, "dma_map_single failed!\n");
		return -ENOMEM;
	}

	pd->rx_buffers[index].skb = skb;
	pd->rx_buffers[index].mapping = mapping;
	pd->rx_ring[index].buffer1 = mapping + NET_IP_ALIGN;
	pd->rx_ring[index].status = DMA_RDES0_OWN_BIT;

	/* FIXME
	 * Do we really need wmb here ?
	 */
	wmb();

	return 0;
}

static void arasan_gemac_free_tx_ring(struct arasan_gemac_pdata *pd)
{
	int i;

	if (!pd->tx_buffers)
		return;

	for (i = 0; i < TX_RING_SIZE; i++) {
		struct sk_buff *skb = pd->tx_buffers[i].skb;

		if (skb) {
			WARN_ON(!pd->tx_buffers[i].mapping);
			dma_unmap_single(&pd->pdev->dev,
					 pd->tx_buffers[i].mapping,
					 skb->len, DMA_TO_DEVICE);
			dev_kfree_skb_any(skb);
		}

		pd->tx_ring[i].status = 0;
		pd->tx_ring[i].misc = 0;
		pd->tx_ring[i].buffer1 = 0;
		pd->tx_ring[i].buffer2 = 0;
	}

	/* FIXME
	 * Do we really need wmb here ?
	 */
	wmb();

	kfree(pd->tx_buffers);
	pd->tx_buffers = NULL;

	pd->tx_ring_head = 0;
	pd->tx_ring_tail = 0;
}

static void arasan_gemac_free_rx_ring(struct arasan_gemac_pdata *pd)
{
	int i;

	if (!pd->rx_buffers)
		return;

	for (i = 0; i < RX_RING_SIZE; i++) {
		if (pd->rx_buffers[i].skb)
			dev_kfree_skb_any(pd->rx_buffers[i].skb);

		if (pd->rx_buffers[i].mapping)
			dma_unmap_single(&pd->pdev->dev,
					 pd->rx_buffers[i].mapping,
					 PKT_BUF_SZ, DMA_FROM_DEVICE);

		pd->rx_ring[i].status = 0;
		pd->rx_ring[i].misc = 0;
		pd->rx_ring[i].buffer1 = 0;
		pd->rx_ring[i].buffer2 = 0;
	}

	/* FIXME
	 * Do we really need wmb here ?
	 */
	wmb();

	kfree(pd->rx_buffers);
	pd->rx_buffers = NULL;

	pd->rx_ring_head = 0;
	pd->rx_ring_tail = 0;
}

static int arasan_gemac_alloc_tx_ring(struct arasan_gemac_pdata *pd)
{
	int i;

	pd->tx_buffers = kmalloc_array(TX_RING_SIZE,
		sizeof(struct arasan_gemac_ring_info), GFP_KERNEL);

	if (!pd->tx_buffers)
		return -ENOMEM;

	/* Initialize the TX Ring */
	for (i = 0; i < TX_RING_SIZE; i++) {
		pd->tx_buffers[i].skb = NULL;
		pd->tx_buffers[i].mapping = 0;
		pd->tx_ring[i].status = 0;
		pd->tx_ring[i].misc = 0;
		pd->tx_ring[i].buffer1 = 0;
		pd->tx_ring[i].buffer2 = 0;
	}
	pd->tx_ring[TX_RING_SIZE - 1].misc = DMA_TDES1_EOR; /* End of ring */
	wmb();

	pd->tx_ring_head = 0;
	pd->tx_ring_tail = 0;

	arasan_gemac_writel(pd, DMA_TRANSMIT_BASE_ADDRESS, pd->tx_dma_addr);

	return 0;
}

static int arasan_gemac_alloc_rx_ring(struct arasan_gemac_pdata *pd)
{
	int i;

	pd->rx_buffers = kmalloc_array(RX_RING_SIZE,
		sizeof(struct arasan_gemac_ring_info), GFP_KERNEL);

	if (pd->rx_buffers == NULL)
		goto out;

	/* initialize the rx ring */
	for (i = 0; i < RX_RING_SIZE; i++) {
		pd->rx_ring[i].status = 0;
		pd->rx_ring[i].misc = PKT_BUF_SZ;
		pd->rx_ring[i].buffer2 = 0;
		pd->rx_buffers[i].skb = NULL;
		pd->rx_buffers[i].mapping = 0;
	}
	pd->rx_ring[RX_RING_SIZE - 1].misc = (PKT_BUF_SZ | DMA_RDES1_EOR);

	/* now allocate the entire ring of skbs */
	for (i = 0; i < RX_RING_SIZE; i++) {
		if (arasan_gemac_alloc_rx_buffer(pd, i)) {
			netdev_err(pd->dev,
				   "failed to allocate rx skb %d\n", i);
			goto out_free_rx_skbs;
		}
	}

	pd->rx_ring_head = 0;
	pd->rx_ring_tail = 0;

	arasan_gemac_writel(pd, DMA_RECEIVE_BASE_ADDRESS, pd->rx_dma_addr);

	return 0;

out_free_rx_skbs:
	arasan_gemac_free_rx_ring(pd);
out:
	return -ENOMEM;
}

/* Open the ethernet interface */
static int arasan_gemac_open(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	u32 result, reg;

	pd->rx_ring = dma_alloc_coherent(&pd->pdev->dev,
		(RX_RING_SIZE * sizeof(struct arasan_gemac_dma_desc)),
		&pd->rx_dma_addr, GFP_KERNEL);

	if (!pd->rx_ring)
		return -ENOMEM;

	pd->tx_ring = dma_alloc_coherent(&pd->pdev->dev,
		(TX_RING_SIZE * sizeof(struct arasan_gemac_dma_desc)),
		&pd->tx_dma_addr, GFP_KERNEL);

	if (!pd->tx_ring)
		return -ENOMEM;

	result = arasan_gemac_alloc_tx_ring(pd);
	if (result) {
		netdev_err(pd->dev, "Failed to Initialize tx dma ring\n");
		return result;
	}

	result = arasan_gemac_alloc_rx_ring(pd);
	if (result) {
		netdev_err(pd->dev, "Failed to Initialize rx dma ring\n");
		return result;
	}

	arasan_gemac_init(pd);

	/* schedule a link state check */
	phy_start(pd->phy_dev);

	napi_enable(&pd->napi);

	/* Enable interrupts */
	arasan_gemac_writel(pd, DMA_INTERRUPT_ENABLE,
			    DMA_INTERRUPT_ENABLE_RECEIVE_DONE |
			    DMA_INTERRUPT_ENABLE_TRANSMIT_DONE);

	/* Enable packet transmission */
	reg = arasan_gemac_readl(pd, MAC_TRANSMIT_CONTROL);
	reg |= MAC_TRANSMIT_CONTROL_TRANSMIT_ENABLE;
	arasan_gemac_writel(pd, MAC_TRANSMIT_CONTROL, reg);

	/* Enable packet reception */
	reg = arasan_gemac_readl(pd, MAC_RECEIVE_CONTROL);
	reg |= MAC_RECEIVE_CONTROL_RECEIVE_ENABLE;
	arasan_gemac_writel(pd, MAC_RECEIVE_CONTROL, reg);

	/* Start transmit and receive DMA */
	arasan_gemac_writel(pd, DMA_CONTROL,
			    DMA_CONTROL_START_RECEIVE_DMA |
			    DMA_CONTROL_START_TRANSMIT_DMA);

	netif_start_queue(dev);

	return 0;
}

static void arasan_gemac_tx_update_stats(struct net_device *dev,
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
static void arasan_gemac_complete_tx(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);

	while (pd->tx_ring_tail != pd->tx_ring_head) {
		int index = pd->tx_ring_tail;
		u32 status, misc;

		/* FIXME
		 * Do we really need rmb here ?
		 */
		rmb();

		status = pd->tx_ring[index].status;
		misc = pd->tx_ring[index].misc;

		/* Check if DMA still owns this descriptor */
		if (unlikely(DMA_TDES0_OWN_BIT & status))
			break;

		arasan_gemac_tx_update_stats(dev, status, misc);

		WARN_ON(!pd->tx_buffers[index].skb);
		WARN_ON(!pd->tx_buffers[index].mapping);

		dma_unmap_single(&pd->pdev->dev, pd->tx_buffers[index].mapping,
				 pd->tx_buffers[index].skb->len, DMA_TO_DEVICE);

		pd->tx_buffers[index].mapping = 0;

		dev_kfree_skb_any(pd->tx_buffers[index].skb);
		pd->tx_buffers[index].skb = NULL;

		pd->tx_ring[index].buffer1 = 0;

		/* FIXME
		 * Do we really need wmb here ?
		 */
		wmb();

		pd->tx_ring_tail = (pd->tx_ring_tail + 1) % TX_RING_SIZE;
	}
}

/* Transmit packet */
static int arasan_gemac_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	dma_addr_t mapping;
	int index = pd->tx_ring_head;
	u32 tmp_desc1;
	bool about_to_take_last_desc =
		(((pd->tx_ring_head + 2) % TX_RING_SIZE) == pd->tx_ring_tail);

	arasan_gemac_complete_tx(dev);

	/* FIXME
	 * Do we really need rmb here ?
	 */
	rmb();

	WARN_ON(pd->tx_ring[index].status & DMA_TDES0_OWN_BIT);
	WARN_ON(pd->tx_buffers[index].skb);
	WARN_ON(pd->tx_buffers[index].mapping);

	mapping = dma_map_single(&pd->pdev->dev, skb->data,
				 skb->len, DMA_TO_DEVICE);

	if (dma_mapping_error(&pd->pdev->dev, mapping)) {
		netdev_warn(dev, "dma_map_single failed, dropping packet\n");
		return NETDEV_TX_BUSY;
	}

	pd->tx_buffers[index].skb = skb;
	pd->tx_buffers[index].mapping = mapping;

	tmp_desc1 = (DMA_TDES1_LS | DMA_TDES1_FS | ((u32)skb->len & 0xFFF));
	if (unlikely(about_to_take_last_desc)) {
		tmp_desc1 |= DMA_TDES1_IOC;
		netif_stop_queue(pd->dev);
	}

	/* check if we are at the last descriptor and need to set EOR */
	if (unlikely(index == (TX_RING_SIZE - 1)))
		tmp_desc1 |= DMA_TDES1_EOR;

	pd->tx_ring[index].buffer1 = mapping;
	pd->tx_ring[index].misc = tmp_desc1;

	/* FIXME
	 * Do we really need wmb here ?
	 */
	wmb();

	/* increment head */
	pd->tx_ring_head = (pd->tx_ring_head + 1) % TX_RING_SIZE;

	/* assign ownership to DMAC */
	pd->tx_ring[index].status = DMA_TDES0_OWN_BIT;

	/* FIXME
	 * Do we really need rmb here ?
	 */
	wmb();

	skb_tx_timestamp(skb);

	/* kick the DMA */
	arasan_gemac_writel(pd, DMA_TRANSMIT_POLL_DEMAND, 1);

	return NETDEV_TX_OK;
}


static void arasan_gemac_alloc_new_rx_buffers(struct arasan_gemac_pdata *pd)
{
	while (pd->rx_ring_tail != pd->rx_ring_head) {
		if (arasan_gemac_alloc_rx_buffer(pd, pd->rx_ring_tail))
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
			 PKT_BUF_SZ, DMA_FROM_DEVICE);

	pd->rx_buffers[index].mapping = 0;

	skb = pd->rx_buffers[index].skb;
	pd->rx_buffers[index].skb = NULL;

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
		/* FIXME
		 * Do we really need rmb here ?
		 */
		rmb();

		status = pd->rx_ring[pd->rx_ring_head].status;

		/* stop if DMAC owns this dma descriptor */
		if (status & DMA_RDES0_OWN_BIT)
			break;

		arasan_gemac_rx_count_stats(dev, status);
		arasan_gemac_rx_handoff(pd, pd->rx_ring_head, status);
		pd->rx_ring_head = (pd->rx_ring_head + 1) % RX_RING_SIZE;
		arasan_gemac_alloc_new_rx_buffers(pd);
	}

	drop_frame_cnt = arasan_gemac_readl(pd, DMA_MISSED_FRAME_COUNTER);
	dev->stats.rx_dropped += drop_frame_cnt;

	/* Kick RXDMA */
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

static irqreturn_t arasan_gemac_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	u32 int_sts, ints_to_clear;

	int_sts = arasan_gemac_readl(pd, DMA_STATUS_AND_IRQ);

	ints_to_clear = 0;

	if (int_sts & DMA_STATUS_AND_IRQ_TRANSFER_DONE) {
		ints_to_clear |= DMA_STATUS_AND_IRQ_TRANSFER_DONE;
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

	if (ints_to_clear)
		arasan_gemac_writel(pd, DMA_STATUS_AND_IRQ, ints_to_clear);

	return IRQ_HANDLED;
}

static void arasan_gemac_stop_tx(struct arasan_gemac_pdata *pd)
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

	/* mask TX DMAC interrupts */
	reg = arasan_gemac_readl(pd, DMA_INTERRUPT_ENABLE);
	reg &= ~DMA_INTERRUPT_ENABLE_TRANSMIT_DONE;
	arasan_gemac_writel(pd, DMA_INTERRUPT_ENABLE, reg);

	/* We must guarantee that interrupts will be masked
	 * before stoping MAC TX
	 */
	wmb();

	/* stop MAC TX */
	reg = arasan_gemac_readl(pd, MAC_TRANSMIT_CONTROL);
	reg &= ~MAC_TRANSMIT_CONTROL_TRANSMIT_ENABLE;
	arasan_gemac_writel(pd, MAC_TRANSMIT_CONTROL, reg);
}

static void arasan_gemac_stop_rx(struct arasan_gemac_pdata *pd)
{
	int timeout = 1000;
	u32 reg;

	/* mask RX DMAC interrupts */
	reg = arasan_gemac_readl(pd, DMA_INTERRUPT_ENABLE);
	reg &= ~DMA_INTERRUPT_ENABLE_RECEIVE_DONE;
	arasan_gemac_writel(pd, DMA_INTERRUPT_ENABLE, reg);

	/* We must guarantee that interrupts will be masked
	 * before stoping RX MAC
	 */
	wmb();

	/* stop RX MAC */
	reg = arasan_gemac_readl(pd, MAC_RECEIVE_CONTROL);
	reg &= ~MAC_RECEIVE_CONTROL_RECEIVE_ENABLE;
	arasan_gemac_writel(pd, MAC_RECEIVE_CONTROL, reg);

	/* We must guarantee that RX MAC will be stopped
	 * before stoping DMA
	 */
	wmb();

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

static int arasan_gemac_stop(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);

	netif_tx_disable(dev);
	napi_disable(&pd->napi);

	arasan_gemac_stop_tx(pd);
	arasan_gemac_free_tx_ring(pd);

	arasan_gemac_stop_rx(pd);
	arasan_gemac_free_rx_ring(pd);

	arasan_gemac_dma_soft_reset(pd);

	phy_stop(pd->phy_dev);
	/* TODO: We should somehow power down PHY */

	dma_free_coherent(&pd->pdev->dev,
			  RX_RING_SIZE * sizeof(struct arasan_gemac_dma_desc),
			  pd->rx_ring, pd->rx_dma_addr);
	pd->rx_ring = NULL;

	dma_free_coherent(&pd->pdev->dev,
			  TX_RING_SIZE * sizeof(struct arasan_gemac_dma_desc),
			  pd->tx_ring, pd->tx_dma_addr);
	pd->tx_ring = NULL;

	return 0;
}

static void arasan_gemac_reset_phy(struct platform_device *pdev)
{
	int err, phy_reset;
	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "failed to get device node\n");
		return;
	}

	phy_reset = of_get_named_gpio(np, "phy-reset-gpios", 0);

	if (!gpio_is_valid(phy_reset)) {
		dev_err(&pdev->dev, "phy-reset-gpios is not valid\n");
		return;
	}

	err = devm_gpio_request_one(&pdev->dev, phy_reset,
				    GPIOF_OUT_INIT_LOW, "phy-reset");

	if (err) {
		dev_err(&pdev->dev,
			"failed to request gpio %d (phy-reset) : %d\n",
			phy_reset, err);
		return;
	}

	/* FIXME
	 * 20 msec is actually too much for phy resetting. But if we set
	 * the reset time less than 20 msec check patch script is failed.
	 */

	msleep(20);
	gpio_set_value(phy_reset, 1);
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

static void arasan_gemac_handle_link_change(struct net_device *dev)
{
	struct arasan_gemac_pdata *pd = netdev_priv(dev);
	struct phy_device *phydev = pd->phy_dev;
	unsigned long flags;

	int status_change = 0;

	spin_lock_irqsave(&pd->lock, flags);

	if ((phydev->link) &&
	    ((pd->speed != phydev->speed) || (pd->duplex != phydev->duplex))) {
		u32 reg;

		reg = arasan_gemac_readl(pd, MAC_GLOBAL_CONTROL);
		reg &= ~(MAC_GLOBAL_CONTROL_SPEED(3) |
			 MAC_GLOBAL_CONTROL_DUPLEX_MODE(1));

		if (phydev->duplex)
			reg |= MAC_GLOBAL_CONTROL_DUPLEX_MODE(phydev->duplex);

		if (phydev->speed == SPEED_100)
			reg |= MAC_GLOBAL_CONTROL_SPEED(1);

		arasan_gemac_writel(pd, MAC_GLOBAL_CONTROL, reg);

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

	phydev->supported &= PHY_BASIC_FEATURES;

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

	pd->mii_bus = mdiobus_alloc();
	if (!pd->mii_bus) {
		err = -ENOMEM;
		goto err_out;
	}

	pd->mii_bus->name = "arasan-gemac-mii-bus";
	pd->mii_bus->read = &arasan_gemac_mdio_read;
	pd->mii_bus->write = &arasan_gemac_mdio_write;

	snprintf(pd->mii_bus->id, MII_BUS_ID_SIZE, "%s-0x%x",
		 pd->pdev->name, pd->pdev->id);

	pd->mii_bus->priv = pd;
	pd->mii_bus->parent = &pd->dev->dev;

	pd->mii_bus->irq = pd->phy_irq;

	arasan_gemac_mdio_write(pd->mii_bus, 7, 18, 0x67);

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

static int arasan_gemac_set_mac_address(struct net_device *dev, void *addr)
{
	if (netif_running(dev))
		return -EBUSY;

	/* sa_family is validated by calling code */
	ether_addr_copy(dev->dev_addr, ((struct sockaddr *)addr)->sa_data);
	arasan_gemac_set_hwaddr(dev);

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
	.ndo_set_mac_address = arasan_gemac_set_mac_address,
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
	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	netif_napi_add(dev, &pd->napi, arasan_gemac_rx_poll, NAPI_WEIGHT);

	res = of_get_phy_mode(pdev->dev.of_node);
	if (res < 0)
		pd->phy_interface = PHY_INTERFACE_MODE_MII;
	else
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

	res = arasan_gemac_mii_init(dev);
	if (res)
		goto err_out_unregister_netdev;

	/* Display ethernet banner */
	netdev_info(dev, "Arasan GEMAC ethernet at 0x%08lx int=%d (%pM)\n",
		    dev->base_addr, dev->irq, dev->dev_addr);

	return 0;

err_out_unregister_netdev:
	unregister_netdev(dev);
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

	if (pd->phy_dev)
		phy_disconnect(pd->phy_dev);

	mdiobus_unregister(pd->mii_bus);
	mdiobus_free(pd->mii_bus);

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
