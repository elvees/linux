/*
 * Copyright 2018-2019 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <uapi/linux/elvees-swic.h>

#include "regs.h"

#define ELVEES_SWIC_MAX_DEVICES		2

#define ELVEES_SWIC_MTU_DEFAULT		SZ_16K
#define ELVEES_SWIC_TX_BUF_SIZE		SZ_16K

#define ELVEES_SWIC_DMA_CSR_WN		3
#define ELVEES_SWIC_EOP			0b01
#define ELVEES_SWIC_EEP			0b10

#define ELVEES_SWIC_RX_BUF_SIZE		SZ_16K
#define RX_RING_SIZE			(ELVEES_SWIC_MAX_PACKET_SIZE / \
					 ELVEES_SWIC_RX_BUF_SIZE + 1)

static u32 swic_major;
static DECLARE_BITMAP(swic_dev, ELVEES_SWIC_MAX_DEVICES);
static struct class *swic_class;

struct elvees_swic_buf_desc {
	struct elvees_swic_dma_desc {
		u32 run;
		u32 ir;
		u32 cp;
		u32 csr;
	} dma;

	void *vaddr;
	u8 ready4dma;
} __aligned(8);

struct elvees_swic_packet_desc {
	unsigned size     : 25;
	unsigned padding0 : 4;
	unsigned type     : 2;
	unsigned valid    : 1;
	u32 padding1;
} __aligned(8);

struct elvees_swic_ring_head {
	u32 desc_num;
	u32 offset;
};

struct elvees_swic_private_data {
	void __iomem *regs;

	unsigned long mtu;

	struct elvees_swic_buf_desc *rx_data_ring;

	struct elvees_swic_ring_head dma_head;
	struct elvees_swic_ring_head cpu_head;

	struct dma_pool *desc_pool;
	struct elvees_swic_packet_desc *tx_desc;
	struct elvees_swic_packet_desc *rx_desc;

	u8 *tx_data;
	bool tx_data_done;
	bool rx_desc_done;
	bool link;

	wait_queue_head_t write_wq;
	wait_queue_head_t read_wq;

	struct mutex swic_write_lock;
	struct mutex swic_read_lock;

	dma_addr_t tx_desc_dma_addr;
	dma_addr_t tx_data_dma_addr;
	dma_addr_t rx_desc_dma_addr;
	dma_addr_t rx_data_dma_addr;

	struct clk *aclk;
	struct clk *txclk;

	struct device *dev;
	struct cdev cdev;
};

static int elvees_swic_rx_ring_create(struct elvees_swic_private_data *pdata)
{
	struct elvees_swic_buf_desc *ring;
	int i, ret = 0;

	ring = dma_alloc_coherent(pdata->dev, (RX_RING_SIZE * sizeof(*ring)),
				  &pdata->rx_data_dma_addr, GFP_KERNEL);
	if (!ring)
		return -ENOMEM;

	for (i = 0; i < RX_RING_SIZE; i++) {
		ring[i].ready4dma = 1;
		ring[i].vaddr = dma_alloc_coherent(pdata->dev,
						   ELVEES_SWIC_RX_BUF_SIZE,
						   &ring[i].dma.ir,
						   GFP_KERNEL);
		if (!ring[i].vaddr) {
			ret = -ENOMEM;
			goto alloc_fail;
		}

		if (i < RX_RING_SIZE - 1)
			ring[i].dma.cp = pdata->rx_data_dma_addr +
					 (i + 1) * sizeof(*ring);

		ring[i].dma.csr = SET_FIELD(SWIC_DMA_CSR_WN, 0) |
				  SET_FIELD(SWIC_DMA_CSR_CHEN, 1) |
				  SET_FIELD(SWIC_DMA_CSR_IM, 1) |
				  SET_FIELD(SWIC_DMA_CSR_WCX,
				  (ELVEES_SWIC_RX_BUF_SIZE / 8) - 1);
	}

	ring[RX_RING_SIZE - 1].dma.cp = pdata->rx_data_dma_addr;

	pdata->dma_head.offset = 0;
	pdata->cpu_head.offset = 0;
	pdata->dma_head.desc_num = 0;
	pdata->cpu_head.desc_num = 0;
	pdata->rx_data_ring = ring;

	return 0;

alloc_fail:
	for (; i >= 0; i--)
		dma_free_coherent(pdata->dev,
				  ELVEES_SWIC_RX_BUF_SIZE,
				  ring[i].vaddr,
				  ring[i].dma.ir);

	dma_free_coherent(pdata->dev, (RX_RING_SIZE * sizeof(*ring)),
			  ring, pdata->rx_data_dma_addr);

	return ret;
}

static int elvees_swic_rx_ring_destroy(struct elvees_swic_private_data *pdata)
{
	struct elvees_swic_buf_desc *ring = pdata->rx_data_ring;
	int i;

	for (i = 0; i < RX_RING_SIZE; i++)
		dma_free_coherent(pdata->dev, ELVEES_SWIC_RX_BUF_SIZE,
				  ring[i].vaddr, ring[i].dma.ir);

	dma_free_coherent(pdata->dev, (RX_RING_SIZE * sizeof(*ring)),
			  ring, pdata->rx_data_dma_addr);

	return 0;
}

static u32 swic_readl(struct elvees_swic_private_data *pdata, u32 reg)
{
	return readl(pdata->regs + reg);
}

static void swic_writel(struct elvees_swic_private_data *pdata, u32 reg,
			u32 value)
{
	 writel(value, pdata->regs + reg);
}

static void elvees_swic_stop_dma(struct elvees_swic_private_data *pdata,
				 u32 base_addr)
{
	u32 reg;

	swic_writel(pdata, base_addr + SWIC_DMA_RUN, 0);

	reg = swic_readl(pdata, SWIC_MODE_CR);
	swic_writel(pdata, SWIC_MODE_CR, reg | SWIC_MODE_CR_LINK_RESET |
					 SWIC_MODE_CR_LINK_DISABLE);

	while (swic_readl(pdata, base_addr + SWIC_DMA_RUN) &
	       SWIC_DMA_CSR_RUN) {
	}

	reg = swic_readl(pdata, SWIC_MODE_CR);
	swic_writel(pdata, SWIC_MODE_CR, (reg & ~SWIC_MODE_CR_LINK_RESET));
}

static void elvees_swic_reset(struct elvees_swic_private_data *pdata)
{
	swic_writel(pdata, SWIC_MODE_CR, SWIC_MODE_CR_LINK_DISABLE |
					 SWIC_MODE_CR_LINK_RESET);

	swic_writel(pdata, SWIC_TX_SPEED, 0);

	swic_writel(pdata, SWIC_CNT_RX_PACK, 0);

	/* Stop RX DATA DMA since it's the only running DMA.
	 * Stopping DMA resets all FIFOs.
	 */
	elvees_swic_stop_dma(pdata, SWIC_DMA_RX_DATA);

	pdata->dma_head.offset = 0;
	pdata->cpu_head.offset = 0;
	pdata->dma_head.desc_num = 0;
	pdata->cpu_head.desc_num = 0;
}

static int elvees_swic_open(struct inode *inode, struct file *file)
{
	struct elvees_swic_private_data *pdata;

	pdata = container_of(inode->i_cdev, struct elvees_swic_private_data,
			     cdev);

	file->private_data = pdata;

	return 0;
}

static int elvees_swic_release(struct inode *inode, struct file *file)
{
	return 0;
}

static void elvees_swic_start_dma(struct elvees_swic_private_data *pdata,
				  u32 base_addr, u32 ir, u32 csr_wn,
				  u32 csr_wcx)
{
	u32 reg = SET_FIELD(SWIC_DMA_CSR_WN, csr_wn) |
		  SET_FIELD(SWIC_DMA_CSR_WCX, csr_wcx);

	swic_writel(pdata, base_addr + SWIC_DMA_CSR, reg);
	swic_writel(pdata, base_addr + SWIC_DMA_IR, ir);

	swic_writel(pdata, base_addr + SWIC_DMA_RUN, SWIC_DMA_CSR_RUN);
}

static int elvees_swic_set_link(struct elvees_swic_private_data *pdata,
				unsigned int arg)
{
	u32 reg;
	unsigned long rate;

	elvees_swic_reset(pdata);

	if (arg == 0) {
		/* No interrupt if link is disabled in a regular way */
		pdata->link = false;
		wake_up_interruptible(&pdata->write_wq);
		wake_up_interruptible(&pdata->read_wq);
		return 0;
	}

	reg = SWIC_MODE_CR_LINK_START | SWIC_MODE_CR_LINK_MASK |
	      SWIC_MODE_CR_ERR_MASK | SWIC_MODE_CR_COEFF_10_WR;

	swic_writel(pdata, SWIC_MODE_CR, reg);

	rate = clk_get_rate(pdata->aclk);
	rate = DIV_ROUND_UP(rate, 10000000);
	reg = SET_FIELD(SWIC_TX_SPEED_COEFF_10, rate);

	/*
	 * Field TX_SPEED is set to 0x0(4.8 Mbit/s).
	 * This is required to establish link.
	 */
	swic_writel(pdata, SWIC_TX_SPEED, SWIC_TX_SPEED_PLL_TX_EN |
					  SWIC_TX_SPEED_LVDS_EN | reg);

	swic_writel(pdata, SWIC_DMA_RX_DATA + SWIC_DMA_CP,
		    pdata->rx_data_dma_addr | 1);

	/* Wait for RX DMA to fetch first registers block from memory */
	udelay(1);

	swic_writel(pdata, SWIC_DMA_RX_DATA + SWIC_DMA_RUN,
		    SWIC_DMA_CSR_RUN);

	return 0;
}

static int elvees_swic_get_link_state(struct elvees_swic_private_data *pdata,
				      void __user *arg)
{
	enum swic_link_state state;

	switch (swic_readl(pdata, SWIC_STATUS) & SWIC_STATUS_LINK_STATE) {
	case SWIC_STATUS_LINK_STATE_RESET:
		state = LINK_ERROR_RESET;
		break;
	case SWIC_STATUS_LINK_STATE_WAIT:
		state = LINK_ERROR_WAIT;
		break;
	case SWIC_STATUS_LINK_STATE_STARTED:
		state = LINK_STARTED;
		break;
	case SWIC_STATUS_LINK_STATE_READY:
		state = LINK_READY;
		break;
	case SWIC_STATUS_LINK_STATE_CONNECTING:
		state = LINK_CONNECTING;
		break;
	case SWIC_STATUS_LINK_STATE_RUN:
		state = LINK_RUN;
		break;
	}

	return copy_to_user(arg, &state,
			    sizeof(enum swic_link_state));
}

static u32 get_rx_speed_kbps(struct elvees_swic_private_data *pdata)
{
	u32 reg = swic_readl(pdata, SWIC_RX_SPEED);
	int aclk_khz = clk_get_rate(pdata->aclk) / 1000;

	return reg * aclk_khz / 100;
}

static u32 get_tx_speed_kbps(struct elvees_swic_private_data *pdata)
{
	u32 reg = swic_readl(pdata, SWIC_TX_SPEED);

	reg = GET_FIELD(reg, SWIC_TX_SPEED_TX_SPEED);

	if (reg == TX_SPEED_2P4)
		return 2400;
	else if (reg == TX_SPEED_4P8)
		return 4800;
	else
		return 48000 * (reg - 1) + 72000;
}

static u32 elvees_swic_get_speed(struct elvees_swic_private_data *pdata,
				 void __user *arg
)
{
	struct elvees_swic_speed speed;

	speed.rx = get_rx_speed_kbps(pdata);
	speed.tx = get_tx_speed_kbps(pdata);

	return copy_to_user(arg, &speed, sizeof(struct elvees_swic_speed));
}

static int elvees_swic_set_speed(struct elvees_swic_private_data *pdata,
				 unsigned long arg)
{
	u32 reg;

	if (arg != TX_SPEED_2P4 && arg > TX_SPEED_408)
		return -EINVAL;

	reg = swic_readl(pdata, SWIC_TX_SPEED);
	reg &= ~SWIC_TX_SPEED_TX_SPEED;
	reg |= SET_FIELD(SWIC_TX_SPEED_TX_SPEED, arg);
	swic_writel(pdata, SWIC_TX_SPEED, reg);

	return 0;
}

static int elvees_swic_set_mtu(struct elvees_swic_private_data *pdata,
			       unsigned long arg)
{
	if (arg == 0 || arg > ELVEES_SWIC_MAX_PACKET_SIZE)
		return -EINVAL;

	pdata->mtu = arg;

	return 0;
}

static long elvees_swic_ioctl(struct file *file,
			      unsigned int cmd,
			      unsigned long arg)
{
	struct elvees_swic_private_data *pdata =
		(struct elvees_swic_private_data *)file->private_data;

	void __user *const uptr = (void __user *)arg;

	switch (cmd) {
	case SWICIOC_SET_LINK:
		return elvees_swic_set_link(pdata, arg);
	case SWICIOC_GET_LINK_STATE:
		return elvees_swic_get_link_state(pdata, uptr);
	case SWICIOC_SET_TX_SPEED:
		return elvees_swic_set_speed(pdata, arg);
	case SWICIOC_GET_SPEED:
		return elvees_swic_get_speed(pdata, uptr);
	case SWICIOC_SET_MTU:
		return elvees_swic_set_mtu(pdata, arg);
	case SWICIOC_GET_MTU:
		return copy_to_user(uptr, &pdata->mtu, sizeof(unsigned long));
	}

	return -ENOTTY;
}

static bool elvees_swic_check_link(struct elvees_swic_private_data *pdata)
{
	u32 reg = swic_readl(pdata, SWIC_STATUS) & SWIC_STATUS_LINK_STATE;

	if (reg == SWIC_STATUS_LINK_STATE_RUN)
		return true;

	return false;
}

static int swic_transmit_packet(struct elvees_swic_private_data *pdata,
				char __user *buf, size_t size,
				size_t *transmitted)
{
	size_t chunk, chunk_aligned;
	int ret;
	u32 dma_copied;

	pdata->tx_desc->valid = 1;
	pdata->tx_desc->size = size;
	pdata->tx_desc->type = ELVEES_SWIC_EOP;

	elvees_swic_start_dma(pdata, SWIC_DMA_TX_DESC,
			      pdata->tx_desc_dma_addr, 0, 0);

	while (size > 0) {
		pdata->tx_data_done = false;

		chunk = min_t(size_t, size, ELVEES_SWIC_TX_BUF_SIZE);

		/*
		 * This is required because DMA can only transmit
		 * a data which is a multiple of 8 bytes
		 */
		chunk_aligned = ALIGN(chunk, 8);

		ret = copy_from_user(pdata->tx_data, buf, chunk);
		if (ret)
			goto stop_desc_dma;

		elvees_swic_start_dma(pdata, SWIC_DMA_TX_DATA,
				      pdata->tx_data_dma_addr,
				      ELVEES_SWIC_DMA_CSR_WN,
				      (chunk_aligned / 8) - 1);

		ret = wait_event_interruptible(pdata->write_wq,
					       pdata->tx_data_done ||
					       !pdata->link);
		if (ret == -ERESTARTSYS || !pdata->link)
			goto stop_data_dma;

		buf += chunk;
		*transmitted += chunk;
		size -= chunk;
	}

	return 0;

stop_data_dma:
	elvees_swic_stop_dma(pdata, SWIC_DMA_TX_DATA);

	dma_copied = swic_readl(pdata, SWIC_DMA_TX_DATA + SWIC_DMA_CSR);
	dma_copied = GET_FIELD(dma_copied, SWIC_DMA_CSR_WCX);
	dma_copied = chunk_aligned - ((dma_copied + 1) * 8);
	*transmitted += dma_copied;

stop_desc_dma:
	elvees_swic_stop_dma(pdata, SWIC_DMA_TX_DESC);
	return ret;
}

static ssize_t elvees_swic_write(struct file *file, const char __user *buf,
				 size_t size, loff_t *ppos)
{
	struct elvees_swic_private_data *pdata = file->private_data;
	size_t packet_size, transmitted = 0;
	int ret;

	mutex_lock(&pdata->swic_write_lock);

	if (!elvees_swic_check_link(pdata)) {
		mutex_unlock(&pdata->swic_write_lock);
		return -ENOLINK;
	}

	while (size > 0) {
		packet_size = min_t(size_t, size, pdata->mtu);

		ret = swic_transmit_packet(pdata, (char __user *)buf +
					   transmitted, packet_size,
					   &transmitted);
		if (ret)
			goto exit;

		size -= packet_size;
	}

exit:
	mutex_unlock(&pdata->swic_write_lock);

	return transmitted;
}

static inline void elvees_swic_move_head(struct elvees_swic_ring_head *p)
{
	p->desc_num = (p->desc_num == RX_RING_SIZE - 1) ? 0 : p->desc_num + 1;
	p->offset = 0;
}

static ssize_t elvees_swic_read(struct file *file, char __user *buf,
				size_t size, loff_t *ppos)
{
	struct elvees_swic_private_data *pdata = file->private_data;
	struct elvees_swic_buf_desc *ring = pdata->rx_data_ring;
	struct elvees_swic_ring_head *head = &pdata->cpu_head;
	size_t desc_size, completed = 0, chunk;
	int ret;

	mutex_lock(&pdata->swic_read_lock);

	if (!elvees_swic_check_link(pdata)) {
		mutex_unlock(&pdata->swic_read_lock);
		return -ENOLINK;
	}

	if (size < ELVEES_SWIC_MAX_PACKET_SIZE) {
		mutex_unlock(&pdata->swic_read_lock);
		return -ENOBUFS;
	}

	pdata->rx_desc_done = false;

	elvees_swic_start_dma(pdata, SWIC_DMA_RX_DESC,
			      pdata->rx_desc_dma_addr, 0, 0);

	ret = wait_event_interruptible(pdata->read_wq, pdata->rx_desc_done ||
						       !pdata->link);
	if (ret == -ERESTARTSYS || !pdata->link)
		goto stop_desc_dma;

	if (pdata->rx_desc->type == ELVEES_SWIC_EEP)
		goto stop_data_dma;

	desc_size = pdata->rx_desc->size;

	/*TODO: Add waiting for all packet data is actually copied by RX DMA */
	if (head->offset != 0) {
		chunk = min_t(size_t, ELVEES_SWIC_RX_BUF_SIZE - head->offset,
			      desc_size);

		ret = copy_to_user(buf, ring[head->desc_num].vaddr +
					head->offset, chunk);
		if (ret)
			goto stop_data_dma;

		if (chunk == ELVEES_SWIC_RX_BUF_SIZE - head->offset) {
			ring[head->desc_num].ready4dma = 1;
			elvees_swic_move_head(head);
			swic_writel(pdata, SWIC_DMA_RX_DATA + SWIC_DMA_RUN,
				    SWIC_DMA_CSR_RUN);
		} else
			head->offset += ALIGN(chunk, 8);

		completed += chunk;
		desc_size -= chunk;
	}

	while (desc_size > 0) {
		chunk = min_t(size_t, ELVEES_SWIC_RX_BUF_SIZE, desc_size);

		ret = copy_to_user(buf + completed, ring[head->desc_num].vaddr,
				   chunk);
		if (ret)
			goto stop_data_dma;

		completed += chunk;
		desc_size -= chunk;

		if (chunk == ELVEES_SWIC_RX_BUF_SIZE) {
			ring[head->desc_num].ready4dma = 1;
			elvees_swic_move_head(head);
			swic_writel(pdata, SWIC_DMA_RX_DATA + SWIC_DMA_RUN,
				    SWIC_DMA_CSR_RUN);
		} else
			head->offset = ALIGN(chunk, 8);
	}

	goto exit;

stop_desc_dma:
	elvees_swic_stop_dma(pdata, SWIC_DMA_RX_DESC);

stop_data_dma:
	elvees_swic_stop_dma(pdata, SWIC_DMA_RX_DATA);

exit:
	mutex_unlock(&pdata->swic_read_lock);

	return completed;
}

static const struct file_operations elvees_swic_fops = {
	.owner = THIS_MODULE,
	.open = elvees_swic_open,
	.release = elvees_swic_release,
	.unlocked_ioctl = elvees_swic_ioctl,
	.write = elvees_swic_write,
	.read = elvees_swic_read
};

static int elvees_swic_dev_register(struct elvees_swic_private_data *pdata)
{
	struct device *dev;
	int minor, ret;

	static DEFINE_SPINLOCK(lock);

	/* Find a free minor number */
	spin_lock(&lock);

	minor = find_first_zero_bit(swic_dev, ELVEES_SWIC_MAX_DEVICES);
	if (minor == ELVEES_SWIC_MAX_DEVICES) {
		spin_unlock(&lock);
		dev_err(pdata->dev, "Failed to get free minor\n");
		return -ENFILE;
	}

	set_bit(minor, swic_dev);

	spin_unlock(&lock);

	cdev_init(&pdata->cdev, &elvees_swic_fops);
	pdata->cdev.owner = THIS_MODULE;
	ret = cdev_add(&pdata->cdev, MKDEV(swic_major, minor), 1);
	if (ret) {
		dev_err(pdata->dev, "Failed to add cdev\n");
		goto free_minor;
	}

	dev = device_create(swic_class, pdata->dev, MKDEV(swic_major, minor),
			    NULL, "spacewire%d", minor);
	if (IS_ERR(dev)) {
		dev_err(pdata->dev, "Failed to create files\n");
		ret = PTR_ERR(dev);
		goto cdev_free;
	}
	return 0;

cdev_free:
	cdev_del(&pdata->cdev);

free_minor:
	clear_bit(minor, swic_dev);
	return ret;
}

static void elvees_swic_dev_unregister(struct elvees_swic_private_data *pdata)
{
	int minor = MINOR(pdata->cdev.dev);

	device_destroy(swic_class, MKDEV(swic_major, minor));

	cdev_del(&pdata->cdev);

	clear_bit(minor, swic_dev);
}

static irqreturn_t elvees_swic_dma_rx_desc_ih(int irq, void *data)
{
	struct elvees_swic_private_data *pdata = data;

	swic_readl(pdata, SWIC_DMA_RX_DESC + SWIC_DMA_CSR);
	pdata->rx_desc_done = true;
	wake_up_interruptible(&pdata->read_wq);

	return IRQ_HANDLED;
}

static irqreturn_t elvees_swic_dma_rx_data_ih(int irq, void *data)
{
	struct elvees_swic_private_data *pdata = data;
	struct elvees_swic_buf_desc *ring = pdata->rx_data_ring;
	struct elvees_swic_ring_head *head = &pdata->dma_head;

	swic_readl(pdata, SWIC_DMA_RX_DATA + SWIC_DMA_CSR);

	/* Mark current buffer as completed */
	ring[head->desc_num].ready4dma = 0;

	elvees_swic_move_head(head);

	/* Start RX Data DMA if next buffer is available */
	if (ring[head->desc_num].ready4dma == 1)
		swic_writel(pdata, SWIC_DMA_RX_DATA + SWIC_DMA_RUN,
			    SWIC_DMA_CSR_RUN);

	return IRQ_HANDLED;
}

static irqreturn_t elvees_swic_dma_tx_desc_ih(int irq, void *data)
{
	struct elvees_swic_private_data *pdata = data;

	swic_readl(pdata, SWIC_DMA_TX_DESC + SWIC_DMA_CSR);

	return IRQ_HANDLED;
}

static irqreturn_t elvees_swic_dma_tx_data_ih(int irq, void *data)
{
	struct elvees_swic_private_data *pdata = data;

	swic_readl(pdata, SWIC_DMA_TX_DATA + SWIC_DMA_CSR);
	pdata->tx_data_done = true;
	wake_up_interruptible(&pdata->write_wq);

	return IRQ_HANDLED;
}

static irqreturn_t elvees_swic_ih(int irq, void *data)
{
	u32 reg;
	struct elvees_swic_private_data *pdata = data;

	reg = swic_readl(pdata, SWIC_STATUS);

	if (reg & SWIC_STATUS_CONNECTED) {
		reg |= SWIC_STATUS_GOT_FIRST_BIT;
		pdata->link = true;
		dev_dbg(pdata->dev, "Connection is set\n");
	}

	if (reg & SWIC_STATUS_DC_ERR) {
		reg |= SWIC_STATUS_DC_ERR;
		dev_dbg(pdata->dev, "Disconnection error\n");
	}

	if (reg & SWIC_STATUS_P_ERR) {
		reg |= SWIC_STATUS_P_ERR;
		dev_dbg(pdata->dev, "Parity error\n");
	}

	if (reg & SWIC_STATUS_ESC_ERR) {
		reg |= SWIC_STATUS_ESC_ERR;
		dev_dbg(pdata->dev, "ESC sequence error\n");
	}

	if (reg & SWIC_STATUS_CREDIT_ERR) {
		reg |= SWIC_STATUS_CREDIT_ERR;
		dev_dbg(pdata->dev, "Credit error\n");
	}

	if (reg & SWIC_STATUS_ERR) {
		pdata->link = false;
		wake_up_interruptible(&pdata->write_wq);
		wake_up_interruptible(&pdata->read_wq);
	}

	swic_writel(pdata, SWIC_STATUS, reg);

	return IRQ_HANDLED;
}

static int elvees_swic_irq_request(struct platform_device *pdev,
				   struct elvees_swic_private_data *pdata)
{
	int irq, ret, i;

	struct {
		irqreturn_t (*handler)(int, void*);
		const char *name;
	} descs[] = {
		{ elvees_swic_dma_rx_desc_ih, "dma_rx_desc_irq" },
		{ elvees_swic_dma_rx_data_ih, "dma_rx_data_irq" },
		{ elvees_swic_dma_tx_desc_ih, "dma_tx_desc_irq" },
		{ elvees_swic_dma_tx_data_ih, "dma_tx_data_irq" },
		{ elvees_swic_ih, "connected_irq" }
	};

	for (i = 0; i < ARRAY_SIZE(descs); i++) {
		irq = platform_get_irq(pdev, i);
		ret = devm_request_irq(&pdev->dev, irq, descs[i].handler,
				       0, descs[i].name, pdata);
		if (ret) {
			dev_err(&pdev->dev, "Failed to get interrupt resource\n");
			return ret;
		}
	}

	return 0;
}

static int elvees_swic_probe(struct platform_device *pdev)
{
	struct elvees_swic_private_data *pdata;
	struct resource *regs;
	int ret;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "Failed to get memory resource\n");
		return -ENOENT;
	}

	pdata = devm_kzalloc(&pdev->dev,
			     sizeof(struct elvees_swic_private_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->desc_pool = dma_pool_create("swic-desc-pool", &pdev->dev,
					sizeof(struct elvees_swic_packet_desc),
					64, 0);
	if (!pdata->desc_pool)
		return -ENOMEM;

	pdata->tx_data = dma_alloc_coherent(pdata->dev,
					    ELVEES_SWIC_TX_BUF_SIZE,
					    &pdata->tx_data_dma_addr,
					    GFP_KERNEL);
	if (!pdata->tx_data) {
		ret = -ENOMEM;
		goto dma_pool_destroy;
	}

	pdata->tx_desc = dma_pool_alloc(pdata->desc_pool, GFP_KERNEL,
					&pdata->tx_desc_dma_addr);
	if (!pdata->tx_desc) {
		ret = -ENOMEM;
		goto tx_data_free;
	}

	pdata->rx_desc = dma_pool_alloc(pdata->desc_pool, GFP_KERNEL,
					&pdata->rx_desc_dma_addr);
	if (!pdata->rx_desc) {
		ret = -ENOMEM;
		goto tx_desc_free;
	}

	pdata->dev = &pdev->dev;

	pdata->txclk = devm_clk_get(&pdev->dev, "txclk");
	if (IS_ERR(pdata->txclk)) {
		dev_err(&pdev->dev, "Failed to found txclk\n");
		ret = PTR_ERR(pdata->txclk);
		goto rx_desc_free;
	}

	pdata->aclk = devm_clk_get(&pdev->dev, "aclk");
	if (IS_ERR(pdata->aclk)) {
		dev_err(&pdev->dev, "Failed to found aclk\n");
		ret = PTR_ERR(pdata->aclk);
		goto rx_desc_free;
	}

	ret = clk_prepare_enable(pdata->txclk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable txclk\n");
		goto rx_desc_free;
	}

	ret = clk_prepare_enable(pdata->aclk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable aclk\n");
		goto disable_txclk;
	}

	pdata->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(pdata->regs)) {
		dev_err(&pdev->dev, "Failed to remap mem resource\n");
		goto disable_aclk;
	}

	ret = elvees_swic_irq_request(pdev, pdata);
	if (ret)
		goto disable_aclk;

	ret = elvees_swic_dev_register(pdata);
	if (ret)
		goto disable_aclk;

	mutex_init(&pdata->swic_write_lock);
	mutex_init(&pdata->swic_read_lock);

	platform_set_drvdata(pdev, pdata);

	init_waitqueue_head(&pdata->write_wq);
	init_waitqueue_head(&pdata->read_wq);

	pdata->mtu = ELVEES_SWIC_MTU_DEFAULT;

	ret = elvees_swic_rx_ring_create(pdata);
	if (ret) {
		elvees_swic_dev_unregister(pdata);
		goto disable_aclk;
	}

	dev_info(&pdev->dev,
		 "ELVEES SWIC @ 0x%p; SWIC version %x\n",
		 pdata->regs, swic_readl(pdata, SWIC_HW_VER));

	return 0;

disable_aclk:
	clk_disable_unprepare(pdata->aclk);

disable_txclk:
	clk_disable_unprepare(pdata->txclk);

rx_desc_free:
	dma_pool_free(pdata->desc_pool, pdata->rx_desc,
		      pdata->rx_desc_dma_addr);

tx_desc_free:
	dma_pool_free(pdata->desc_pool, pdata->tx_desc,
		      pdata->tx_desc_dma_addr);

tx_data_free:
	dma_free_coherent(pdata->dev, ELVEES_SWIC_TX_BUF_SIZE,
			  pdata->tx_data, pdata->tx_data_dma_addr);

dma_pool_destroy:
	dma_pool_destroy(pdata->desc_pool);
	return ret;
}

static int elvees_swic_remove(struct platform_device *pdev)
{
	struct elvees_swic_private_data *pdata = platform_get_drvdata(pdev);

	dma_pool_free(pdata->desc_pool, pdata->rx_desc,
		      pdata->rx_desc_dma_addr);

	dma_pool_free(pdata->desc_pool, pdata->tx_desc,
		      pdata->tx_desc_dma_addr);

	dma_free_coherent(pdata->dev, ELVEES_SWIC_TX_BUF_SIZE,
			  pdata->tx_data, pdata->tx_data_dma_addr);

	dma_pool_destroy(pdata->desc_pool);

	elvees_swic_rx_ring_destroy(pdata);

	clk_disable_unprepare(pdata->txclk);
	clk_disable_unprepare(pdata->aclk);

	elvees_swic_dev_unregister(pdata);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id elvees_swic_ids[] = {
	{ .compatible = "elvees,mcom02-swic" },
	{}
};
MODULE_DEVICE_TABLE(of, elvees_swic_ids);
#endif

static struct platform_driver elvees_swic_driver = {
	.probe = elvees_swic_probe,
	.remove = elvees_swic_remove,
	.driver = {
		.name = "elvees-swic",
		.of_match_table = of_match_ptr(elvees_swic_ids)
	}
};

static int __init elvees_swic_module_init(void)
{
	int ret;
	dev_t dev;

	swic_class = class_create(THIS_MODULE, "spacewire");
	if (IS_ERR(swic_class)) {
		ret = PTR_ERR(swic_class);
		goto out;
	}

	ret = alloc_chrdev_region(&dev, 0, ELVEES_SWIC_MAX_DEVICES,
				  "elvees-swic");
	if (ret)
		goto class_destroy;

	swic_major = MAJOR(dev);
	ret = platform_driver_register(&elvees_swic_driver);
	if (ret)
		goto chr_remove;

	return 0;

chr_remove:
	unregister_chrdev_region(dev, ELVEES_SWIC_MAX_DEVICES);

class_destroy:
	class_destroy(swic_class);

out:
	return ret;
}

static void __exit elvees_swic_module_exit(void)
{
	platform_driver_unregister(&elvees_swic_driver);
	unregister_chrdev_region(MKDEV(swic_major, 0), ELVEES_SWIC_MAX_DEVICES);
	class_destroy(swic_class);
}

module_init(elvees_swic_module_init);
module_exit(elvees_swic_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ELVEES SWIC driver");
