// SPDX-License-Identifier: GPL-2.0
/* Copyright 2021 RnD Center "ELVEES", JSC
 *
 * This driver is based on Bosch M_CAN driver.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/iopoll.h>
#include <linux/can/dev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/reset.h>
#include <linux/property.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>

/* napi related */
#define MFBSP_CAN_NAPI_WEIGHT	64

/* message ram configuration data length */
#define MRAM_CFG_LEN	7

/* registers definition */
enum mfbsp_can_reg {
	// 0x000 - 0x014
	MFBSP_CAN_CCCR	= 0x18,
	MFBSP_CAN_BTP	= 0x1c,
	MFBSP_CAN_TSCC	= 0x20,
	MFBSP_CAN_TSCV	= 0x24,
	// 0x028 - 0x02c
	MFBSP_CAN_TS64CTRT	= 0x30,
	MFBSP_CAN_TS64H	= 0x34,
	MFBSP_CAN_TS64L	= 0x38,
	// 0x03c
	MFBSP_CAN_ECR	= 0x40,
	MFBSP_CAN_PSR	= 0x44,
	// 0x048 - 0x04c
	MFBSP_CAN_IR	= 0x50,
	MFBSP_CAN_IE	= 0x54,
	// 0x058 - 0x07c
	MFBSP_CAN_GFC	= 0x80,
	MFBSP_CAN_SIDFC	= 0x84,
	MFBSP_CAN_XIDFC	= 0x88,
	// 0x08c
	MFBSP_CAN_XIDAM	= 0x90,
	MFBSP_CAN_HPMS	= 0x94,
	// 0x098 - 0x09c
	MFBSP_CAN_RXF0C	= 0xa0,
	MFBSP_CAN_RXF0S	= 0xa4,
	MFBSP_CAN_RXF0A	= 0xa8,
	// 0x0ac - 0x0bc
	MFBSP_CAN_TXBC	= 0xc0,
	// 0x0c4 - 0x0c8
	MFBSP_CAN_TXBRP	= 0xcc,
	MFBSP_CAN_TXBAR	= 0xd0,
	MFBSP_CAN_TXBCR	= 0xd4,
	MFBSP_CAN_TXBTO	= 0xd8,
	MFBSP_CAN_TXBCF	= 0xdc,
	MFBSP_CAN_TXBTIE	= 0xe0,
	MFBSP_CAN_TXBCIE	= 0xe4,
	// 0x0e8 - 0x0ec
	MFBSP_CAN_TXEFC	= 0xf0,
	MFBSP_CAN_TXEFS	= 0xf4,
	// 0x0f8 - 0x100
	MFBSP_CAN_TTRMC = 0x104,
	MFBSP_CAN_TTOCF = 0x108,
	MFBSP_CAN_TTMLM = 0x10c,
	MFBSP_CAN_TURCF = 0x110,
	MFBSP_CAN_TTOCN = 0x114,
	// 0118 - 0x11c
	MFBSP_CAN_TTIR	= 0x120,
	MFBSP_CAN_TTIE	= 0x124,
	// 0x128
	MFBSP_CAN_TTOST = 0x12c,
	MFBSP_CAN_TURNA = 0x130,
	MFBSP_CAN_TTLGT = 0x134,
	MFBSP_CAN_TTCTC = 0x138,
	// 013c - 0x14c
	MFBSP_CAN_STATTX	= 0x150,
	MFBSP_CAN_STATRX	= 0x154,
	MFBSP_CAN_STATWRN	= 0x158,
	MFBSP_CAN_STATEP	= 0x15c,
	MFBSP_CAN_STATBO	= 0x160,
	MFBSP_CAN_STATLOST	= 0x164
};

/* mfbsp_can lec values */
enum mfbsp_can_lec_type {
	LEC_NO_ERROR = 0,
	LEC_STUFF_ERROR,
	LEC_FORM_ERROR,
	LEC_ACK_ERROR,
	LEC_BIT1_ERROR,
	LEC_BIT0_ERROR,
	LEC_CRC_ERROR,
	LEC_UNUSED,
};

enum mfbsp_can_mram_cfg {
	MRAM_SIDF = 0,
	MRAM_XIDF,
	MRAM_RXF0,
	MRAM_TXB,
	MRAM_TXE,
	MRAM_EVE,
	MRAM_CFG_NUM,
};

/* CC Control Register(CCCR) */
#define CCCR_DAR		BIT(6)
#define CCCR_MON		BIT(5)
#define CCCR_CSR		BIT(4)
#define CCCR_CSA		BIT(3)
#define CCCR_LBM		BIT(2)
#define CCCR_CCE		BIT(1)
#define CCCR_INIT		BIT(0)
#define CCCR_CANFD		0x10
#define CCCR_CFG_TIMEOUT	10

/* Nominal Bit Timing & Prescaler Register (NBTP) */
#define BTP_BRP_SHIFT		16
#define BTP_BRP_MASK		(0x3ff << BTP_BRP_SHIFT)
#define BTP_TSEG1_SHIFT		8
#define BTP_TSEG1_MASK		(0x3f << BTP_TSEG1_SHIFT)
#define BTP_TSEG2_SHIFT		4
#define BTP_TSEG2_MASK		(0xf << BTP_TSEG2_SHIFT)
#define BTP_SJW_SHIFT		0
#define BTP_SJW_MASK		(0xf << BTP_SJW_SHIFT)

/* Timestamp counter configuration register */
#define TSCC_TCP_SHIFT	16
#define TSCC_TCP_MASK	(0xf << TSCC_TCP_SHIFT)
#define TSCC_TSS_SHIFT	0
#define TSCC_TSS_MASK	(0x3 << TSCC_TSS_SHIFT)

/* Timestamp counter value register */
#define TSCV_TSC_SHIFT	0
#define TSCV_TSC_MASK	(0xffff << TSCC_TCP_SHIFT)

/* 64bit timestamp counter configuration register */
#define TS64CTR_EN_CANCEL_BY_TS64	BIT(31)
#define TS64CTR_DIV_MAX_SHIFT		4
#define TS64CTR_DIV_MAX_MASK		(0xfffff << TS64CTR_DIV_MAX_SHIFT)
#define TS64CTR_USE_EXT_INCR		BIT(3)
#define TS64CTR_USE_EXT_RESET		BIT(2)
#define TS64CTR_RESET				BIT(1)
#define TS64CTR_ENABLE				BIT(0)

/* Error Counter Register(ECR) */
#define ECR_RP				BIT(15)
#define ECR_REC_SHIFT		8
#define ECR_REC_MASK		(0x7f << ECR_REC_SHIFT)
#define ECR_TEC_SHIFT		0
#define ECR_TEC_MASK		0xff

/* Protocol Status Register(PSR) */
#define PSR_BO			BIT(7)
#define PSR_EW			BIT(6)
#define PSR_EP			BIT(5)
#define PSR_LEC_MASK	0x7

/* Interrupt Register(IR) */
#define IR_ALL_INT	0xffffffff

/* Interrupt Register Bits */
#define IR_STE		BIT(31)
#define IR_FOE		BIT(30)
#define IR_ACKE		BIT(29)
#define IR_BE		BIT(28)
#define IR_CRCE		BIT(27)
// 26 not used
#define IR_BO		BIT(25)
#define IR_EW		BIT(24)
#define IR_EP		BIT(23)
// 22:17 not used
#define IR_TSW		BIT(16)
#define IR_TEFL		BIT(15)
#define IR_TEFF		BIT(14)
#define IR_TEFW		BIT(13)
#define IR_TEFN		BIT(12)
// 11 not used
#define IR_TCF		BIT(10)
#define IR_TC		BIT(9)
#define IR_HPM		BIT(8)
// 7:4 not used
#define IR_RF0L		BIT(3)
#define IR_RF0F		BIT(2)
#define IR_RF0W		BIT(1)
#define IR_RF0N		BIT(0)
#define IR_ERR_STATE	(IR_BO | IR_EW | IR_EP)

/* Interrupts */
#define IR_ERR_LEC	(IR_STE	| IR_FOE | IR_ACKE | IR_BE | IR_CRCE)
#define IR_ERR_BUS	(IR_ERR_LEC | IR_TSW | IR_TEFL | IR_RF0L)
#define IR_ERR_ALL	(IR_ERR_STATE | IR_ERR_BUS)

/* Interrupt Line Select (ILS) */
#define ILS_ALL_INT0	0x0
#define ILS_ALL_INT1	0xFFFFFFFF

/* Interrupt Line Enable (ILE) */
#define ILE_EINT1	BIT(1)
#define ILE_EINT0	BIT(0)

/* Rx FIFO 0/1 Configuration (RXF0C/RXF1C) */
#define RXFC_FOM		BIT(31)
#define RXFC_FWM_SHIFT	24
#define RXFC_FWM_MASK	(0x7f << RXFC_FWM_SHIFT)
#define RXFC_FS_SHIFT	16
#define RXFC_FS_MASK	(0x7f << RXFC_FS_SHIFT)

/* Rx FIFO 0/1 Status (RXF0S/RXF1S) */
#define RXFS_RFL	BIT(25)
#define RXFS_FF		BIT(24)
#define RXFS_FPI_SHIFT	16
#define RXFS_FPI_MASK	(0x3f << RXFS_FPI_SHIFT)
#define RXFS_FGI_SHIFT	8
#define RXFS_FGI_MASK	(0x3f << RXFS_FGI_SHIFT)
#define RXFS_FFL_SHIFT	0
#define RXFS_FFL_MASK	(0x7f << RXFS_FFL_SHIFT)

/* Tx Buffer Configuration(TXBC) */
#define TXBC_PRI_MODE	BIT(0)

/* Tx Event FIFO Configuration (TXEFC) */
#define TXEFC_EFWM_SHIFT	24
#define TXEFC_EFWM_MASK		(0x3f << TXEFC_EFWM_SHIFT)

/* Tx Event FIFO Status (TXEFS) */
#define TXEFS_TEFL		BIT(25)
#define TXEFS_EFF		BIT(24)
#define TXEFS_EFFL_SHIFT	0
#define TXEFS_EFFL_MASK		(0x3f << TXEFS_EFFL_SHIFT)

/* Message RAM Configuration (in bytes) */
#define SIDF_ELEMENT_SIZE	4
#define XIDF_ELEMENT_SIZE	8
#define RXF0_ELEMENT_SIZE	24
#define TXE_ELEMENT_SIZE	16
#define TXB_ELEMENT_SIZE	24
#define EVE_ELEMENT_SIZE	8

/* Message RAM Elements */
#define MFBSP_CAN_FIFO_ID		0x0
#define MFBSP_CAN_FIFO_DLC		0x4
#define MFBSP_CAN_FIFO_DATA(n)	(0x8 + ((n) << 2))
#define MFBSP_CAN_FIFO_TS64H	0x10
#define MFBSP_CAN_FIFO_TS64L	0x14

/* Rx Buffer Element */
/* R0 */
#define RX_BUF_XTD		BIT(30)
#define RX_BUF_RTR		BIT(29)
/* R1 */
#define RX_BUF_ANMF		BIT(31)
#define RX_BUF_FDF		BIT(21)
#define RX_BUF_BRS		BIT(20)

/* Tx Buffer Element */
/* T0 */
#define TX_BUF_XTD		BIT(30)
#define TX_BUF_RTR		BIT(29)
/* T1 */
#define TX_BUF_EFC		BIT(23)
#define TX_BUF_MM_SHIFT		24
#define TX_BUF_MM_MASK		(0xff << TX_BUF_MM_SHIFT)

/* Tx event FIFO Element */
/* E1 */
#define TX_EVENT_MM_SHIFT	TX_BUF_MM_SHIFT
#define TX_EVENT_MM_MASK	(0xff << TX_EVENT_MM_SHIFT)

/* address offset and element number for each FIFO/Buffer in the Message RAM */
struct mram_cfg {
	u16	off;
	u8	num;
};

/* mfbsp_can private data structure */
struct mfbsp_can_priv {
	struct can_priv can;	/* must be the first member */
	struct napi_struct napi;
	struct net_device *dev;
	struct device *device;
	struct clk *clk;
	void __iomem *base;
	u32 irqstatus;

	void __iomem *can_cfg_regs;
	struct reset_control	*rst;

	/* message ram configuration */
	void __iomem *mram_base;
	struct mram_cfg mcfg[MRAM_CFG_NUM];
};

static inline u32 mfbsp_can_read(const struct mfbsp_can_priv *priv,
				 enum mfbsp_can_reg reg)
{
	return readl(priv->base + reg);
}

static inline void mfbsp_can_write(const struct mfbsp_can_priv *priv,
				   enum mfbsp_can_reg reg, u32 val)
{
	writel(val, priv->base + reg);
}

static inline u32 mfbsp_can_fifo_read(const struct mfbsp_can_priv *priv,
				      u32 fgi, unsigned int offset)
{
	return readl(priv->mram_base + priv->mcfg[MRAM_RXF0].off +
		     fgi * RXF0_ELEMENT_SIZE + offset);
}

static inline void mfbsp_can_fifo_write(const struct mfbsp_can_priv *priv,
					u32 fpi, unsigned int offset, u32 val)
{
	writel(val, priv->mram_base + priv->mcfg[MRAM_TXB].off +
	       fpi * TXB_ELEMENT_SIZE + offset);
}

static inline u32 mfbsp_can_txe_fifo_read(const struct mfbsp_can_priv *priv,
					  u32 fgi, u32 offset) {
	return readl(priv->mram_base + priv->mcfg[MRAM_TXE].off +
		     fgi * TXE_ELEMENT_SIZE + offset);
}

static inline bool mfbsp_can_tx_fifo_full(const struct mfbsp_can_priv *priv)
{
	return !!(mfbsp_can_read(priv, MFBSP_CAN_TXEFS) & TXEFS_EFF);
}

static inline void mfbsp_can_config_endisable(const struct mfbsp_can_priv *priv,
					      bool enable)
{
	u32 cccr = mfbsp_can_read(priv, MFBSP_CAN_CCCR);
	u32 val = 0, status = 0, ret = 0;

	if (enable) {
		/* enable mfbsp_can configuration */
		mfbsp_can_write(priv, MFBSP_CAN_CCCR, cccr | CCCR_INIT);
		udelay(5);
		/* CCCR.CCE can only be set/reset while CCCR.INIT = '1' */
		mfbsp_can_write(priv, MFBSP_CAN_CCCR,
				cccr | CCCR_INIT | CCCR_CCE);
	} else {
		mfbsp_can_write(priv, MFBSP_CAN_CCCR,
				cccr & ~(CCCR_INIT | CCCR_CCE));
	}

	/* there's a delay for module initialization */
	if (enable)
		val = CCCR_INIT | CCCR_CCE;

	ret = readl_poll_timeout(priv->base + MFBSP_CAN_CCCR, status,
				 ((status & (CCCR_INIT | CCCR_CCE)) == val),
				 1, CCCR_CFG_TIMEOUT);
	if (ret) {
		netdev_warn(priv->dev, "Failed to %s device\n",
			    enable ? "enable" : "disable");
		return;
	}
}

static void mfbsp_can_read_fifo(struct net_device *dev, u32 rxfs)
{
	struct net_device_stats *stats = &dev->stats;
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	struct canfd_frame *cf;
	struct sk_buff *skb;
	u32 id, fgi, dlc;
	int i;

	/* calculate the fifo rx index */
	fgi = (rxfs & RXFS_FGI_MASK) >> RXFS_FGI_SHIFT;
	dlc = mfbsp_can_fifo_read(priv, fgi, MFBSP_CAN_FIFO_DLC);
	if (dlc & RX_BUF_FDF)
		skb = alloc_canfd_skb(dev, &cf);
	else
		skb = alloc_can_skb(dev, (struct can_frame **)&cf);
	if (!skb) {
		stats->rx_dropped++;
		return;
	}

	if (dlc & RX_BUF_FDF)
		cf->len = can_dlc2len((dlc >> 16) & 0x0F);
	else
		cf->len = get_can_dlc((dlc >> 16) & 0x0F);

	id = mfbsp_can_fifo_read(priv, fgi, MFBSP_CAN_FIFO_ID);
	if (id & RX_BUF_XTD)
		cf->can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		cf->can_id = id & CAN_SFF_MASK;

	if (!(dlc & RX_BUF_FDF) && (id & RX_BUF_RTR)) {
		cf->can_id |= CAN_RTR_FLAG;
	} else {
		if (dlc & RX_BUF_BRS)
			cf->flags |= CANFD_BRS;

		for (i = 0; i < cf->len; i += 4)
			*(u32 *)(cf->data + i) =
				mfbsp_can_fifo_read(priv, fgi,
						    MFBSP_CAN_FIFO_DATA(i / 4));
	}

	/* acknowledge rx fifo 0 */
	mfbsp_can_write(priv, MFBSP_CAN_RXF0A, fgi);

	stats->rx_packets++;
	stats->rx_bytes += cf->len;

	netif_receive_skb(skb);
}

static int mfbsp_can_do_rx_poll(struct net_device *dev, int quota)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	u32 pkts = 0;
	u32 rxfs;

	rxfs = mfbsp_can_read(priv, MFBSP_CAN_RXF0S);
	if (!(rxfs & RXFS_FFL_MASK)) {
		netdev_dbg(dev, "no messages in fifo0\n");
		return 0;
	}

	while ((rxfs & RXFS_FFL_MASK) && (quota > 0)) {
		mfbsp_can_read_fifo(dev, rxfs);
		quota--;
		pkts++;
		rxfs = mfbsp_can_read(priv, MFBSP_CAN_RXF0S);
	}

	return pkts;
}

static int mfbsp_can_handle_lost_msg(struct net_device *dev)
{
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb;
	struct can_frame *frame;

	netdev_err(dev, "msg lost in rxf0\n");

	stats->rx_errors++;
	stats->rx_over_errors++;

	skb = alloc_can_err_skb(dev, &frame);
	if (unlikely(!skb))
		return 0;

	frame->can_id |= CAN_ERR_CRTL;
	frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;

	netif_receive_skb(skb);

	return 1;
}

static int mfbsp_can_handle_lec_err(struct net_device *dev,
				    enum mfbsp_can_lec_type lec_type)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;

	priv->can.can_stats.bus_error++;
	stats->rx_errors++;

	/* propagate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	/* check for 'last error code' which tells us the
	 * type of the last error to occur on the CAN bus
	 */
	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

	switch (lec_type) {
	case LEC_STUFF_ERROR:
		netdev_dbg(dev, "stuff error\n");
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		break;
	case LEC_FORM_ERROR:
		netdev_dbg(dev, "form error\n");
		cf->data[2] |= CAN_ERR_PROT_FORM;
		break;
	case LEC_ACK_ERROR:
		netdev_dbg(dev, "ack error\n");
		cf->data[3] = CAN_ERR_PROT_LOC_ACK;
		break;
	case LEC_BIT1_ERROR:
		netdev_dbg(dev, "bit1 error\n");
		cf->data[2] |= CAN_ERR_PROT_BIT1;
		break;
	case LEC_BIT0_ERROR:
		netdev_dbg(dev, "bit0 error\n");
		cf->data[2] |= CAN_ERR_PROT_BIT0;
		break;
	case LEC_CRC_ERROR:
		netdev_dbg(dev, "CRC error\n");
		cf->data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
		break;
	default:
		break;
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_receive_skb(skb);

	return 1;
}

static int mfbsp_can_get_berr_counter(const struct net_device *dev,
				      struct can_berr_counter *bec)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	unsigned int ecr;

	ecr = mfbsp_can_read(priv, MFBSP_CAN_ECR);
	bec->rxerr = (ecr & ECR_REC_MASK) >> ECR_REC_SHIFT;
	bec->txerr = (ecr & ECR_TEC_MASK) >> ECR_TEC_SHIFT;

	return 0;
}

static int mfbsp_can_handle_state_change(struct net_device *dev,
					 enum can_state new_state)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	struct can_berr_counter bec;
	unsigned int ecr;

	switch (new_state) {
	case CAN_STATE_ERROR_WARNING:
		/* error warning state */
		priv->can.can_stats.error_warning++;
		priv->can.state = CAN_STATE_ERROR_WARNING;
		break;
	case CAN_STATE_ERROR_PASSIVE:
		/* error passive state */
		priv->can.can_stats.error_passive++;
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
		break;
	case CAN_STATE_BUS_OFF:
		/* bus-off state */
		priv->can.state = CAN_STATE_BUS_OFF;
		priv->can.can_stats.bus_off++;
		can_bus_off(dev);
		break;
	default:
		break;
	}

	/* propagate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	mfbsp_can_get_berr_counter(dev, &bec);

	switch (new_state) {
	case CAN_STATE_ERROR_WARNING:
		/* error warning state */
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = (bec.txerr > bec.rxerr) ?
			      CAN_ERR_CRTL_TX_WARNING :
			      CAN_ERR_CRTL_RX_WARNING;
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
		break;
	case CAN_STATE_ERROR_PASSIVE:
		/* error passive state */
		cf->can_id |= CAN_ERR_CRTL;
		ecr = mfbsp_can_read(priv, MFBSP_CAN_ECR);
		if (ecr & ECR_RP)
			cf->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
		if (bec.txerr > 127)
			cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
		break;
	case CAN_STATE_BUS_OFF:
		/* bus-off state */
		cf->can_id |= CAN_ERR_BUSOFF;
		break;
	default:
		break;
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_receive_skb(skb);

	return 1;
}

static int mfbsp_can_handle_state_errors(struct net_device *dev, u32 psr)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	int work_done = 0;

	if ((psr & PSR_EW) &&
	    priv->can.state != CAN_STATE_ERROR_WARNING) {
		netdev_dbg(dev, "entered error warning state\n");
		work_done += mfbsp_can_handle_state_change(dev,
					CAN_STATE_ERROR_WARNING);
	}

	if ((psr & PSR_EP) &&
	    priv->can.state != CAN_STATE_ERROR_PASSIVE) {
		netdev_dbg(dev, "entered error passive state\n");
		work_done += mfbsp_can_handle_state_change(dev,
					CAN_STATE_ERROR_PASSIVE);
	}

	if ((psr & PSR_BO) &&
	    priv->can.state != CAN_STATE_BUS_OFF) {
		netdev_dbg(dev, "entered error bus off state\n");
		work_done += mfbsp_can_handle_state_change(dev,
					CAN_STATE_BUS_OFF);
	}

	return work_done;
}

static inline bool is_lec_err(u32 psr)
{
	psr &= LEC_UNUSED;

	return psr && (psr != LEC_UNUSED);
}

static int mfbsp_can_handle_bus_errors(struct net_device *dev,
				       u32 irqstatus,
				       u32 psr)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	int work_done = 0;

	if (irqstatus & IR_RF0L)
		work_done += mfbsp_can_handle_lost_msg(dev);

	/* handle lec errors on the bus */
	if ((priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) &&
	    is_lec_err(psr))
		work_done += mfbsp_can_handle_lec_err(dev, psr & LEC_UNUSED);

	return work_done;
}

static int mfbsp_can_poll(struct napi_struct *napi, int quota)
{
	struct net_device *dev = napi->dev;
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	int work_done = 0;
	u32 irqstatus, psr;

	irqstatus = priv->irqstatus | mfbsp_can_read(priv, MFBSP_CAN_IR);
	if (!irqstatus)
		return 0;

	psr = mfbsp_can_read(priv, MFBSP_CAN_PSR);
	if (irqstatus & IR_ERR_STATE)
		work_done += mfbsp_can_handle_state_errors(dev, psr);

	if (irqstatus & IR_ERR_BUS)
		work_done += mfbsp_can_handle_bus_errors(dev, irqstatus, psr);

	if (irqstatus & IR_RF0N)
		work_done += mfbsp_can_do_rx_poll(dev, (quota - work_done));

	if (work_done < quota)
		napi_complete_done(napi, work_done);

	return work_done;
}

static void mfbsp_can_echo_tx_event(struct net_device *dev)
{
	u32 txe_count = 0;
	u32 mfbsp_can_txefs;
	u32 fgi = 0;
	int i = 0;
	unsigned int msg_mark;

	struct mfbsp_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;

	/* read tx event fifo status */
	mfbsp_can_txefs = mfbsp_can_read(priv, MFBSP_CAN_TXEFS);

	/* Get Tx Event fifo element count */
	txe_count = (mfbsp_can_txefs & TXEFS_EFFL_MASK)
			>> TXEFS_EFFL_SHIFT;

	/* Get and process all sent elements */
	for (i = 0; i < txe_count; i++) {
		/* retrieve get index */
		fgi = 0;

		/* get message marker */
		msg_mark = (mfbsp_can_txe_fifo_read(priv, fgi, 4) &
				TX_EVENT_MM_MASK) >> TX_EVENT_MM_SHIFT;

		/* update stats */
		stats->tx_bytes += can_get_echo_skb(dev, msg_mark);
		stats->tx_packets++;
	}
}

static irqreturn_t mfbsp_can_isr(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	u32 ir;

	ir = mfbsp_can_read(priv, MFBSP_CAN_IR);
	if (!ir)
		return IRQ_NONE;

	/* ACK all irqs */
	if (ir & IR_ALL_INT)
		mfbsp_can_write(priv, MFBSP_CAN_IR, ir);

	/* schedule NAPI in case of
	 * - rx IRQ
	 * - state change IRQ
	 * - bus error IRQ and bus error reporting
	 */
	if ((ir & IR_RF0N) || (ir & IR_ERR_ALL)) {
		priv->irqstatus = ir;
		napi_schedule(&priv->napi);
	}

	if (ir & IR_TC) {
		/* Transmission Complete Interrupt*/
		stats->tx_bytes += can_get_echo_skb(dev, 0);
		stats->tx_packets++;
		netif_wake_queue(dev);
	} else if (ir & IR_TEFN) {
		/* New TX FIFO Element arrived */
		mfbsp_can_echo_tx_event(dev);
		if (netif_queue_stopped(dev) && !mfbsp_can_tx_fifo_full(priv))
			netif_wake_queue(dev);
	}

	return IRQ_HANDLED;
}

static const struct can_bittiming_const mfbsp_can_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 4,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 40,
	.tseg2_min = 3,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 37,
	.sjw_max = 3,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

static int mfbsp_can_set_bittiming(struct net_device *dev)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	u16 brp, sjw, tseg1, tseg2;
	u32 reg_btp;

	brp = bt->brp - 1;
	sjw = bt->sjw - 1;
	tseg1 = bt->prop_seg + bt->phase_seg1 - 1;
	tseg2 = bt->phase_seg2 - 1;

	dev_dbg(priv->device, "tseg1 = %d, tseg2 = %d, sjw = %d, brp = %d\n",
		tseg1, tseg2, sjw, brp);

	reg_btp = (brp << BTP_BRP_SHIFT) | (sjw << BTP_SJW_SHIFT) |
		(tseg1 << BTP_TSEG1_SHIFT) | (tseg2 << BTP_TSEG2_SHIFT);
	mfbsp_can_write(priv, MFBSP_CAN_BTP, reg_btp);

	return 0;
}

static void mfbsp_can_chip_config(struct net_device *dev)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	u32 cccr;

	mfbsp_can_config_endisable(priv, true);

	/* Accept Non-matching Frames Into FIFO */
	mfbsp_can_write(priv, MFBSP_CAN_GFC, 0x0);

	/* rx fifo configuration, blocking mode, fifo size 1 */
	mfbsp_can_write(priv, MFBSP_CAN_RXF0C,
			(priv->mcfg[MRAM_RXF0].num << RXFC_FS_SHIFT) |
			priv->mcfg[MRAM_RXF0].off);

	cccr = mfbsp_can_read(priv, MFBSP_CAN_CCCR);

	cccr &= ~(CCCR_MON | CCCR_LBM | CCCR_DAR);

	/* Loopback Mode */
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		cccr |= CCCR_LBM;

	/* Enable Monitoring */
	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		cccr |= CCCR_MON;

	/* Enable One-shot mode */
	if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		cccr |= CCCR_DAR;

	/* Write config */
	mfbsp_can_write(priv, MFBSP_CAN_CCCR, cccr);

	/* Enable interrupts */
	mfbsp_can_write(priv, MFBSP_CAN_IR, IR_ALL_INT);
	if (!(priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING))
		mfbsp_can_write(priv, MFBSP_CAN_IE, IR_ALL_INT &
				~(IR_ERR_LEC));
	else
		mfbsp_can_write(priv, MFBSP_CAN_IE, IR_ALL_INT);

	/* set bittiming params */
	mfbsp_can_set_bittiming(dev);

	mfbsp_can_config_endisable(priv, false);
}

static void mfbsp_can_start(struct net_device *dev)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);

	/* basic mfbsp_can configuration */
	mfbsp_can_chip_config(dev);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
}

static int mfbsp_can_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		mfbsp_can_start(dev);
		netif_wake_queue(dev);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static void free_mfbsp_can_dev(struct net_device *dev)
{
	free_candev(dev);
}

static struct net_device *mfbsp_can_alloc_dev(struct platform_device *pdev,
					      void __iomem *addr,
					      u32 tx_fifo_size)
{
	struct net_device *dev;
	struct mfbsp_can_priv *priv;
	unsigned int echo_buffer_count;

	echo_buffer_count = tx_fifo_size;

	dev = alloc_candev(sizeof(*priv), echo_buffer_count);
	if (!dev) {
		dev = NULL;
		return dev;
	}
	priv = netdev_priv(dev);
	netif_napi_add(dev, &priv->napi, mfbsp_can_poll, MFBSP_CAN_NAPI_WEIGHT);

	/* Shared properties of all CAN */
	priv->dev = dev;
	priv->base = addr;
	priv->can.do_set_mode = mfbsp_can_set_mode;
	priv->can.do_get_berr_counter = mfbsp_can_get_berr_counter;

	/* Set CAN supported operations */
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
					CAN_CTRLMODE_LISTENONLY |
					CAN_CTRLMODE_ONE_SHOT |
					CAN_CTRLMODE_BERR_REPORTING;

	/* Set properties depending on CAN */
	can_set_static_ctrlmode(dev, 0);
	priv->can.bittiming_const = &mfbsp_can_bittiming_const;
	priv->can.data_bittiming_const = &mfbsp_can_bittiming_const;

	return dev;
}

static int mfbsp_can_open(struct net_device *dev)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	int err;

	/* open the can device */
	err = open_candev(dev);
	if (err) {
		netdev_err(dev, "Failed to open can device\n");
		return err;
	}

	/* register interrupt handler */
	err = request_irq(dev->irq, mfbsp_can_isr, IRQF_SHARED,
			  dev->name, dev);
	if (err < 0) {
		netdev_err(dev, "Failed to request interrupt\n");
		goto exit_irq_fail;
	}

	/* start the mfbsp_can controller */
	mfbsp_can_start(dev);

	napi_enable(&priv->napi);
	netif_start_queue(dev);

	return 0;

exit_irq_fail:
	close_candev(dev);
	return err;
}

static void mfbsp_can_stop(struct net_device *dev)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);

	/* set the state as STOPPED */
	priv->can.state = CAN_STATE_STOPPED;
}

static int mfbsp_can_close(struct net_device *dev)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);
	mfbsp_can_stop(dev);
	free_irq(dev->irq, dev);
	close_candev(dev);

	return 0;
}

static netdev_tx_t mfbsp_can_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	struct mfbsp_can_priv *priv = netdev_priv(dev);
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	u32 id;
	int i;

	int putidx = 0;
	// txbrp - register shows which tx buffers are busy
	u32 value = mfbsp_can_read(priv, MFBSP_CAN_TXBRP);

	while (value & 0x1) {
		value /= 2;
		putidx++;
	}

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	/* Generate ID field for TX buffer Element */
	if (cf->can_id & CAN_EFF_FLAG) {
		id = cf->can_id & CAN_EFF_MASK;
		id |= TX_BUF_XTD;
	} else {
		id = (cf->can_id & CAN_SFF_MASK);
	}

	if (cf->can_id & CAN_RTR_FLAG)
		id |= TX_BUF_RTR;

	netif_stop_queue(dev);

	/* message ram configuration */
	mfbsp_can_fifo_write(priv, putidx, MFBSP_CAN_FIFO_ID, id);
	mfbsp_can_fifo_write(priv, putidx, MFBSP_CAN_FIFO_DLC,
			     can_len2dlc(cf->len) << 16);
	for (i = 0; i < cf->len; i += 4)
		mfbsp_can_fifo_write(priv, putidx,
				     MFBSP_CAN_FIFO_DATA(i / 4),
				     *(u32 *)(cf->data + i));

	/* TODO: set timestamp somehow else */
	mfbsp_can_fifo_write(priv, putidx, MFBSP_CAN_FIFO_TS64H, 0xffffffff);
	mfbsp_can_fifo_write(priv, putidx, MFBSP_CAN_FIFO_TS64L, 0xffffffff);

	can_put_echo_skb(skb, dev, 0);

	mfbsp_can_write(priv, MFBSP_CAN_TXBTIE, 0x1 << putidx);
	mfbsp_can_write(priv, MFBSP_CAN_TXBAR, 0x1 << putidx);

	return NETDEV_TX_OK;
}

static const struct net_device_ops mfbsp_can_netdev_ops = {
	.ndo_open = mfbsp_can_open,
	.ndo_stop = mfbsp_can_close,
	.ndo_start_xmit = mfbsp_can_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static int register_mfbsp_can_dev(struct net_device *dev)
{
	dev->flags |= IFF_ECHO;	/* we support local echo */
	dev->netdev_ops = &mfbsp_can_netdev_ops;

	return register_candev(dev);
}

static void mfbsp_can_init_ram(struct mfbsp_can_priv *priv)
{
	int end, i, start;

	/* initialize the entire Message RAM in use to avoid possible
	 * ECC/parity checksum errors when reading an uninitialized buffer
	 */
	start = priv->mcfg[MRAM_SIDF].off;
	end = priv->mcfg[MRAM_EVE].off +
		priv->mcfg[MRAM_EVE].num * EVE_ELEMENT_SIZE;
	for (i = start; i < end; i += 4)
		__raw_writel(0x0, priv->mram_base + i);
}

static void mfbsp_can_of_parse_mram(struct mfbsp_can_priv *priv,
				    const u32 *mram_config_vals)
{
	priv->mcfg[MRAM_SIDF].off = mram_config_vals[0];
	priv->mcfg[MRAM_SIDF].num = mram_config_vals[1];

	priv->mcfg[MRAM_XIDF].off = priv->mcfg[MRAM_SIDF].off +
			priv->mcfg[MRAM_SIDF].num * SIDF_ELEMENT_SIZE;
	priv->mcfg[MRAM_XIDF].num = mram_config_vals[2];

	priv->mcfg[MRAM_RXF0].off = priv->mcfg[MRAM_XIDF].off +
			priv->mcfg[MRAM_XIDF].num * XIDF_ELEMENT_SIZE;
	priv->mcfg[MRAM_RXF0].num = mram_config_vals[3];

	priv->mcfg[MRAM_TXB].off = priv->mcfg[MRAM_RXF0].off +
			priv->mcfg[MRAM_RXF0].num * RXF0_ELEMENT_SIZE;
	priv->mcfg[MRAM_TXB].num = mram_config_vals[4];

	priv->mcfg[MRAM_TXE].off = priv->mcfg[MRAM_TXB].off +
			priv->mcfg[MRAM_TXB].num * TXB_ELEMENT_SIZE;
	priv->mcfg[MRAM_TXE].num = mram_config_vals[5];

	priv->mcfg[MRAM_EVE].off = priv->mcfg[MRAM_TXE].off +
			priv->mcfg[MRAM_TXE].num * TXE_ELEMENT_SIZE;
	priv->mcfg[MRAM_EVE].num = mram_config_vals[6];

	dev_dbg(priv->device,
		"mram_base %p\nsidf 0x%x %d\nxidf 0x%x %d\nrxf0 0x%x %d\n"
		"txb 0x%x %d\ntxe 0x%x %d\neve 0x%x %d\n",
		priv->mram_base,
		priv->mcfg[MRAM_SIDF].off, priv->mcfg[MRAM_SIDF].num,
		priv->mcfg[MRAM_XIDF].off, priv->mcfg[MRAM_XIDF].num,
		priv->mcfg[MRAM_RXF0].off, priv->mcfg[MRAM_RXF0].num,
		priv->mcfg[MRAM_TXB].off, priv->mcfg[MRAM_TXB].num,
		priv->mcfg[MRAM_TXE].off, priv->mcfg[MRAM_TXE].num,
		priv->mcfg[MRAM_EVE].off, priv->mcfg[MRAM_EVE].num);

	mfbsp_can_init_ram(priv);
}

static int mfbsp_can_plat_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct mfbsp_can_priv *priv;
	struct resource *res;
	void __iomem *addr;
	int ret;
	u32 mram_config_vals[MRAM_CFG_LEN];
	u32 tx_fifo_size;

	/* get message ram configuration */
	ret = of_property_read_u32_array(pdev->dev.of_node, "can,mram-cfg",
					 mram_config_vals,
					 sizeof(mram_config_vals) / 4);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get Message RAM configuration.");
		return ret;
	}

	/* Get TX FIFO size
	 * Defines the total amount of echo buffers for loopback
	 */
	tx_fifo_size = mram_config_vals[4];

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "can");
	addr = devm_ioremap_resource(&pdev->dev, res);

	/* allocate the mfbsp_can device */
	dev = mfbsp_can_alloc_dev(pdev, addr, tx_fifo_size);
	if (!dev) {
		ret = -ENOMEM;
		return ret;
	}

	priv = netdev_priv(dev);

	dev->irq = platform_get_irq_byname(pdev, "rx");

	if (IS_ERR(addr) || dev->irq < 0) {
		ret = -EINVAL;
		goto failed_free_dev;
	}

	priv->device = &pdev->dev;

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		ret = -ENODEV;
		goto failed_free_dev;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		goto disable_clk_ret;

	priv->can.clock.freq = clk_get_rate(priv->clk);

	/* message ram could be shared */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "filters_and_elements");
	if (!res) {
		ret = -ENODEV;
		goto disable_clk_ret;
	}

	priv->mram_base = devm_ioremap(&pdev->dev,
				       res->start,
				       resource_size(res));
	if (!priv->mram_base) {
		ret = -ENOMEM;
		goto disable_clk_ret;
	}

	/* can config register */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg");
	if (!res) {
		ret = -ENODEV;
		goto disable_clk_ret;
	}

	priv->can_cfg_regs = devm_ioremap(&pdev->dev,
					  res->start,
					  resource_size(res));
	if (!priv->can_cfg_regs) {
		ret = -ENOMEM;
		goto disable_clk_ret;
	}

	/* this enables can mode in mfbsp
	 * TODO: change it somehow prettier.
	 */
	writel(0x1, priv->can_cfg_regs);

	priv->rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(priv->rst)) {
		if (PTR_ERR(priv->rst) == -EPROBE_DEFER) {
			ret = -EPROBE_DEFER;
			goto failed_free_dev;
		}
	} else {
		reset_control_deassert(priv->rst);
	}

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	ret = register_mfbsp_can_dev(dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register %s (err=%d)\n",
			KBUILD_MODNAME, ret);
		goto assert_reset;
	}

	mfbsp_can_of_parse_mram(priv, mram_config_vals);

	dev_info(&pdev->dev, "%s device registered (irq=%d)\n",
		 KBUILD_MODNAME, dev->irq);

	return 0;

assert_reset:
	reset_control_assert(priv->rst);
disable_clk_ret:
	clk_disable_unprepare(priv->clk);
failed_free_dev:
	free_mfbsp_can_dev(dev);
	return ret;
}

/* TODO: runtime PM with power down or sleep mode  */

static __maybe_unused int mfbsp_can_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct mfbsp_can_priv *priv = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_device_detach(ndev);
		mfbsp_can_stop(ndev);
	}

	pinctrl_pm_select_sleep_state(dev);

	priv->can.state = CAN_STATE_SLEEPING;

	return 0;
}

static __maybe_unused int mfbsp_can_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct mfbsp_can_priv *priv = netdev_priv(ndev);

	pinctrl_pm_select_default_state(dev);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	if (netif_running(ndev)) {
		mfbsp_can_init_ram(priv);
		mfbsp_can_start(ndev);
		netif_device_attach(ndev);
		netif_start_queue(ndev);
	}

	return 0;
}

static int mfbsp_can_plat_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct mfbsp_can_priv *priv = netdev_priv(dev);

	clk_disable_unprepare(priv->clk);

	reset_control_assert(priv->rst);
	unregister_candev(dev);
	platform_set_drvdata(pdev, NULL);

	free_mfbsp_can_dev(dev);

	return 0;
}

static const struct dev_pm_ops mfbsp_can_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(mfbsp_can_suspend, mfbsp_can_resume)
};

static const struct of_device_id mfbsp_can_of_table[] = {
	{ .compatible = "elvees,mfbsp-can" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mfbsp_can_of_table);

static struct platform_driver mfbsp_can_plat_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = mfbsp_can_of_table,
		.pm	= &mfbsp_can_pmops,
	},
	.probe = mfbsp_can_plat_probe,
	.remove = mfbsp_can_plat_remove,
};

module_platform_driver(mfbsp_can_plat_driver);

MODULE_AUTHOR("Viktor Podusenko <vpodusenko@elvees.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN bus driver for ELVEES MFBSP CAN Controller");
