/*
 * Copyright 2015 ELVEES NeoTek CJSC
 * Copyright 2017 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _ARASAN_GEMAC_H
#define _ARASAN_GEMAC_H

#include <linux/clk.h>
#include <linux/genalloc.h>
#include <linux/if_vlan.h>
#include <linux/netdevice.h>
#include <linux/net_tstamp.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/reset.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/timecounter.h>
#include <linux/ethtool.h>

/* GEMAC TX descriptor can describe 4K buffer.
 * But currently some unexplored bugs are observed if we set Jumbo frame
 * more than 3500 bytes. This bugs lead to lack of transmission. */
#define ARASAN_JUMBO_MTU 3500U

#define mtu_to_frame_sz(x) ((x) + VLAN_ETH_HLEN)
#define mtu_to_buf_sz(x) (mtu_to_frame_sz(x) + NET_IP_ALIGN + 4)

#define TX_RING_SIZE (128)
#define RX_RING_SIZE (128)
#define NAPI_WEIGHT (64)

#define TX_FIFO_ALMOST_FULL_THRESHOLD_WORD_SIZE_BYTES 4
#define FIFO_SIZE_BYTES 8192

/* Arasan GEMAC register offsets */

#define DMA_CONFIGURATION                         0x0000
#define DMA_CONTROL                               0x0004
#define DMA_STATUS_AND_IRQ                        0x0008
#define DMA_INTERRUPT_ENABLE                      0x000C
#define DMA_TRANSMIT_AUTO_POLL_COUNTER            0x0010
#define DMA_TRANSMIT_POLL_DEMAND                  0x0014
#define DMA_RECEIVE_POLL_DEMAND                   0x0018
#define DMA_TRANSMIT_BASE_ADDRESS                 0x001C
#define DMA_RECEIVE_BASE_ADDRESS                  0x0020
#define DMA_MISSED_FRAME_COUNTER                  0x0024
#define DMA_STOP_FLUSH_COUNTER                    0x0028
#define DMA_RECEIVE_INTERRUPT_MITIGATION          0x002C
#define DMA_CURRENT_TRANSMIT_DESCRIPTOR_POINTER   0x0030
#define DMA_CURRENT_TRANSMIT_BUFFER_POINTER       0x0034
#define DMA_CURRENT_RECEIVE_DESCRIPTOR_POINTER    0x0038
#define DMA_CURRENT_RECEIVE_BUFFER_POINTER        0x003C

#define MAC_GLOBAL_CONTROL                        0x0100
#define MAC_TRANSMIT_CONTROL                      0x0104
#define MAC_RECEIVE_CONTROL                       0x0108
#define MAC_MAXIMUM_FRAME_SIZE                    0x010C
#define MAC_TRANSMIT_JABBER_SIZE                  0x0110
#define MAC_RECEIVE_JABBER_SIZE                   0x0114
#define MAC_ADDRESS_CONTROL                       0x0118
#define MAC_MDIO_CLOCK_DIVISION_CONTROL           0x011C
#define MAC_ADDRESS1_HIGH                         0x0120
#define MAC_ADDRESS1_MED                          0x0124
#define MAC_ADDRESS1_LOW                          0x0128
#define MAC_ADDRESS2_HIGH                         0x012C
#define MAC_ADDRESS2_MED                          0x0130
#define MAC_ADDRESS2_LOW                          0x0134
#define MAC_ADDRESS3_HIGH                         0x0138
#define MAC_ADDRESS3_MED                          0x013C
#define MAC_ADDRESS3_LOW                          0x0140
#define MAC_ADDRESS4_HIGH                         0x0144
#define MAC_ADDRESS4_MED                          0x0148
#define MAC_ADDRESS4_LOW                          0x014C
#define MAC_HASH_TABLE1                           0x0150
#define MAC_HASH_TABLE2                           0x0154
#define MAC_HASH_TABLE3                           0x0158
#define MAC_HASH_TABLE4                           0x015C

#define MAC_MDIO_CONTROL                          0x01A0
#define MAC_MDIO_DATA                             0x01A4
#define MAC_RX_STATCTR_CONTROL                    0x01A8
#define MAC_RX_STATCTR_DATA_HIGH                  0x01AC
#define MAC_RX_STATCTR_DATA_LOW                   0x01B0
#define MAC_TX_STATCTR_CONTROL                    0x01B4
#define MAC_TX_STATCTR_DATA_HIGH                  0x01B8
#define MAC_TX_STATCTR_DATA_LOW                   0x01BC
#define MAC_TRANSMIT_FIFO_ALMOST_FULL             0x01C0
#define MAC_TRANSMIT_PACKET_START_THRESHOLD       0x01C4
#define MAC_RECEIVE_PACKET_START_THRESHOLD        0x01C8
#define MAC_TRANSMIT_FIFO_ALMOST_EMPTY_THRESHOLD  0x01CC
#define MAC_INTERRUPT_STATUS                      0x01E0
#define MAC_INTERRUPT_ENABLE                      0x01E4
#define MAC_VLAN_TPID1                            0x01E8
#define MAC_VLAN_TPID2                            0x01EC
#define MAC_VLAN_TPID3                            0x01F0

#define MODULE_1588_CR                            0x0300
#define INC_ATTRIBUTES                            0x0304
#define PTP_ETHER_TYPE                            0x0308
#define PTP_MESSAGE_ID                            0x030C
#define PTP_UDP_PORT_REG                          0x0310
#define SYS_TIME_VAL_LO                           0x0320
#define SYS_TIME_VAL_UP                           0x0324
#define SYS_TIME_ADJ_CTRL_LO                      0x0328
#define SYS_TIME_ADJ_CTRL_UP                      0x032C
#define TX_TSTAMP_VAL_LO                          0x0330
#define TX_TSTAMP_VAL_UP                          0x0334
#define RX_TSTAMP_VAL_LO                          0x0340
#define RX_TSTAMP_VAL_UP                          0x0344
#define RX_PTP_PACKET_ATTR_LO                     0x0348
#define RX_PTP_PACKET_ATTR_MI                     0x034C
#define RX_PTP_PACKET_ATTR_HI                     0x0350
#define MODULE_1588_INT_REG                       0x0360
#define MODULE_1588_INT_EN_REG                    0x0364

#define ARASAN_REGS_END                           0x0368

/* Arasan GEMAC register fields */

#define DMA_CONFIGURATION_SOFT_RESET                  BIT(0)
#define DMA_CONFIGURATION_BURST_LENGTH(VAL)           ((VAL) << 1)
#define DMA_CONFIGURATION_WAIT_FOR_DONE               BIT(16)
#define DMA_CONFIGURATION_64BIT_MODE                  BIT(18)

#define DMA_CONTROL_START_TRANSMIT_DMA                BIT(0)
#define DMA_CONTROL_START_RECEIVE_DMA                 BIT(1)

#define DMA_STATUS_AND_IRQ_TRANSMIT_DONE              BIT(0)
#define DMA_STATUS_AND_IRQ_TRANS_DESC_UNAVAIL         BIT(1)
#define DMA_STATUS_AND_IRQ_TX_DMA_STOPPED             BIT(2)
#define DMA_STATUS_AND_IRQ_RECEIVE_DONE               BIT(4)
#define DMA_STATUS_AND_IRQ_RX_DMA_STOPPED             BIT(6)
#define DMA_STATUS_AND_IRQ_MAC_INTERRUPT              BIT(8)
#define DMA_STATUS_AND_IRQ_TRANSMIT_DMA_STATE(VAL)    (((VAL) & 0x7000) >> 16)
#define DMA_STATUS_AND_IRQ_RECEIVE_DMA_STATE(VAL)     (((VAL) & 0xf0000) >> 20)

#define DMA_INTERRUPT_ENABLE_TRANSMIT_DONE            BIT(0)
#define DMA_INTERRUPT_ENABLE_TRANS_DESC_UNAVAIL       BIT(1)
#define DMA_INTERRUPT_ENABLE_RECEIVE_DONE             BIT(4)
#define DMA_INTERRUPT_ENABLE_MAC                      BIT(8)

#define MAC_GLOBAL_CONTROL_SPEED(VAL)                 ((VAL) << 0)
#define MAC_GLOBAL_CONTROL_DUPLEX_MODE(VAL)           ((VAL) << 2)

#define MAC_TRANSMIT_CONTROL_TRANSMIT_ENABLE          BIT(0)
#define MAC_TRANSMIT_CONTROL_TRANSMIT_AUTO_RETRY      BIT(3)

#define MAC_RECEIVE_CONTROL_RECEIVE_ENABLE            BIT(0)
#define MAC_RECEIVE_CONTROL_STORE_AND_FORWARD         BIT(3)

#define MAC_ADDRESS_CONTROL_ADDRESS_ENABLE(NUM)       BIT((NUM) - 1)
#define MAC_ADDRESS_CONTROL_PROMISCUOUS_MODE          BIT(8)

#define MAC_ADDRESS1_LOW_SIXTH_BYTE(VAL)              ((VAL) << 8)
#define MAC_ADDRESS1_LOW_FIFTH_BYTE(VAL)              ((VAL) << 0)
#define MAC_ADDRESS1_MED_FOURTH_BYTE(VAL)             ((VAL) << 8)
#define MAC_ADDRESS1_MED_THIRD_BYTE(VAL)              ((VAL) << 0)
#define MAC_ADDRESS1_HIGH_SECOND_BYTE(VAL)            ((VAL) << 8)
#define MAC_ADDRESS1_HIGH_FIRST_BYTE(VAL)             ((VAL) << 0)

#define MAC_MDIO_CONTROL_READ_WRITE(VAL)              ((VAL) << 10)
#define MAC_MDIO_CONTROL_REG_ADDR(VAL)                ((VAL) << 5)
#define MAC_MDIO_CONTROL_PHY_ADDR(VAL)                ((VAL) << 0)
#define MAC_MDIO_CONTROL_START_FRAME(VAL)             ((VAL) << 15)

#define MAC_STATCTR_CONTROL_START_READ                BIT(15)
#define MAC_STATCTR_DATA_HIGH(VAL)                    (((VAL) & 0xffff) << 16)
#define MAC_STATCTR_DATA_LOW(VAL)                     ((VAL) & 0xffff)

#define MAC_INTERRUPT_ENABLE_UNDERRUN                 BIT(0)
#define MAC_IRQ_STATUS_UNDERRUN                       BIT(0)

#define RX_PTP_PACKET_V2_L2                           0x0
#define RX_PTP_PACKET_V1_L4                           0x1
#define RX_PTP_PACKET_V2_L2_L4                        0x2

#define MODULE_1588_CR_TX_TSTAMP_EN                   BIT(1)
#define MODULE_1588_CR_RX_TSTAMP_EN                   BIT(2)
#define MODULE_1588_CR_RX_PACKET_TYPE(VAL, TYPE)   \
	(((VAL) & 0xffffffc7) | ((TYPE) << 3))

#define INC_ATTRIBUTES_VAL(VAL)                       ((VAL) & 0xffffff)
#define INC_ATTRIBUTES_PERIOD(VAL)                    ((VAL) << 24)

#define PTP_ETHERTYPE                                 0x88f7

#define PTP_SYNC_MESSAGE_ID                           0x0
#define PTP_DELAY_REQ_MESSAGE_ID                      0x1

#define PTP_UDP_PORT_319                              0x013f

#define MODULE_1588_RX_TSTAMP_IRQ                     BIT(1)
#define MODULE_1588_TX_TSTAMP_IRQ                     BIT(0)

#define MODULE_1588_RX_TSTAMP_IRQ_EN                  BIT(1)
#define MODULE_1588_TX_TSTAMP_IRQ_EN                  BIT(0)

/* DMA descriptor fields */

#define DMA_RDES0_OWN_BIT      BIT(31)
#define DMA_RDES0_FD           BIT(30)
#define DMA_RDES0_LD           BIT(29)
#define DMA_RDES1_PTP_PACKET   BIT(31)
#define DMA_RDES1_PTP_TSTMP    BIT(30)
#define DMA_RDES1_EOR          BIT(26)

#define DMA_TDES0_OWN_BIT      BIT(31)
#define DMA_TDES0_TSTAMP       BIT(30)
#define DMA_TDES1_IOC          BIT(31)
#define DMA_TDES1_LS           BIT(30)
#define DMA_TDES1_FS           BIT(29)
#define DMA_TDES1_EOR          BIT(26)

/* Tx stats counter numbers */

#define ARASAN_GEMAC_TX_OK                            0
#define ARASAN_GEMAC_TX_TOTAL                         1
#define ARASAN_GEMAC_TX_OCTETS_OK                     2
#define ARASAN_GEMAC_TX_EOP_ERR                       3
#define ARASAN_GEMAC_TX_SINGLE_COLLISION              4
#define ARASAN_GEMAC_TX_MULTIPLE_COLLISION            5
#define ARASAN_GEMAC_TX_LATE_COLLISION                6
#define ARASAN_GEMAC_TX_EXCESSIVE_COLLISION           7
#define ARASAN_GEMAC_TX_UNICAST                       8
#define ARASAN_GEMAC_TX_MULTICAST                     9
#define ARASAN_GEMAC_TX_BROADCAST                     10
#define ARASAN_GEMAC_TX_PAUSE                         11

/* Rx stats counter numbers */

#define ARASAN_GEMAC_RX_OK                            0
#define ARASAN_GEMAC_RX_TOTAL                         1
#define ARASAN_GEMAC_RX_CRC_ERR                       2
#define ARASAN_GEMAC_RX_ALIGN_ERR                     3
#define ARASAN_GEMAC_RX_ERROR                         4
#define ARASAN_GEMAC_RX_OCTET_OK                      5
#define ARASAN_GEMAC_RX_OCTET_TOTAL                   6
#define ARASAN_GEMAC_RX_UNICAST                       7
#define ARASAN_GEMAC_RX_MULTICAST                     8
#define ARASAN_GEMAC_RX_BROADCAST                     9
#define ARASAN_GEMAC_RX_PAUSE                         10
#define ARASAN_GEMAC_RX_LEN_ERR                       11
#define ARASAN_GEMAC_RX_UNDER_SIZED                   12
#define ARASAN_GEMAC_RX_OVER_SIZED                    13
#define ARASAN_GEMAC_RX_FRAGMENTS                     14
#define ARASAN_GEMAC_RX_JABBER                        15
#define ARASAN_GEMAC_RX_LEN_64                        16
#define ARASAN_GEMAC_RX_LEN_65_127                    17
#define ARASAN_GEMAC_RX_LEN_128_255                   18
#define ARASAN_GEMAC_RX_LEN_256_511                   19
#define ARASAN_GEMAC_RX_LEN_512_1023                  20
#define ARASAN_GEMAC_RX_LEN_1024_1518                 21
#define ARASAN_GEMAC_RX_LEN_1519_PLUS                 22
#define ARASAN_GEMAC_RX_DROPPED_BUFF_FULL             23
#define ARASAN_GEMAC_RX_TRUNC_BUFF_FULL               24

#define arasan_gemac_readl(port, reg) readl((port)->regs + (reg))
#define arasan_gemac_writel(port, reg, value) \
	writel((value), (port)->regs + (reg))

#define CLOCKS_NUM 3

#define CLOCK_BUS 0
#define CLOCK_TXC 1
#define CLOCK_1588 2

struct arasan_gemac_statistic {
	char stat_string[ETH_GSTRING_LEN];
	int offset;
	u32 ctrl_reg;
	u32 data_h_reg;
	u32 data_l_reg;
};

#define ARASAN_GEMAC_TX_STAT(name, title) {	\
	.stat_string = title,			\
	.offset = ARASAN_GEMAC_TX_##name,	\
	.ctrl_reg = MAC_TX_STATCTR_CONTROL,	\
	.data_h_reg = MAC_TX_STATCTR_DATA_HIGH,	\
	.data_l_reg = MAC_TX_STATCTR_DATA_LOW	\
}

#define ARASAN_GEMAC_RX_STAT(name, title) {	\
	.stat_string = title,			\
	.offset = ARASAN_GEMAC_RX_##name,	\
	.ctrl_reg = MAC_RX_STATCTR_CONTROL,	\
	.data_h_reg = MAC_RX_STATCTR_DATA_HIGH,	\
	.data_l_reg = MAC_RX_STATCTR_DATA_LOW	\
}

static const struct arasan_gemac_statistic arasan_gemac_stats[] = {
	ARASAN_GEMAC_TX_STAT(OK, "frames_tx_ok"),
	ARASAN_GEMAC_TX_STAT(TOTAL, "frames_tx_total"),
	ARASAN_GEMAC_TX_STAT(OCTETS_OK, "octets_tx_ok"),
	ARASAN_GEMAC_TX_STAT(EOP_ERR, "frames_tx_eop_err"),
	ARASAN_GEMAC_TX_STAT(SINGLE_COLLISION, "frames_tx_single_collision"),
	ARASAN_GEMAC_TX_STAT(MULTIPLE_COLLISION,
			     "frames_tx_multiple_collision"),
	ARASAN_GEMAC_TX_STAT(LATE_COLLISION, "frames_tx_late_collision"),
	ARASAN_GEMAC_TX_STAT(EXCESSIVE_COLLISION,
			     "frames_tx_excessive_collision"),
	ARASAN_GEMAC_TX_STAT(UNICAST, "frames_tx_unicast"),
	ARASAN_GEMAC_TX_STAT(MULTICAST, "frames_tx_multicast"),
	ARASAN_GEMAC_TX_STAT(BROADCAST, "frames_tx_broadcast"),
	ARASAN_GEMAC_TX_STAT(PAUSE, "frames_tx_pause"),

	ARASAN_GEMAC_RX_STAT(OK, "frames_rx_ok"),
	ARASAN_GEMAC_RX_STAT(TOTAL, "frames_rx_total"),
	ARASAN_GEMAC_RX_STAT(CRC_ERR, "frames_rx_crc_err"),
	ARASAN_GEMAC_RX_STAT(ALIGN_ERR, "frames_rx_align_err"),
	ARASAN_GEMAC_RX_STAT(ERROR, "frames_rx_error"),
	ARASAN_GEMAC_RX_STAT(OCTET_OK, "octet_rx_ok"),
	ARASAN_GEMAC_RX_STAT(OCTET_TOTAL, "octet_rx_total"),
	ARASAN_GEMAC_RX_STAT(UNICAST, "frames_rx_unicast"),
	ARASAN_GEMAC_RX_STAT(MULTICAST, "frames_rx_multicast"),
	ARASAN_GEMAC_RX_STAT(BROADCAST, "frames_rx_broadcast"),
	ARASAN_GEMAC_RX_STAT(PAUSE, "frames_rx_pause"),
	ARASAN_GEMAC_RX_STAT(LEN_ERR, "frames_rx_length_err"),
	ARASAN_GEMAC_RX_STAT(UNDER_SIZED, "frames_rx_under_sized"),
	ARASAN_GEMAC_RX_STAT(OVER_SIZED, "frames_rx_over_sized"),
	ARASAN_GEMAC_RX_STAT(FRAGMENTS, "frames_rx_fragments"),
	ARASAN_GEMAC_RX_STAT(JABBER, "frames_rx_jabber"),
	ARASAN_GEMAC_RX_STAT(LEN_64, "frames_rx_len_64"),
	ARASAN_GEMAC_RX_STAT(LEN_65_127, "frames_rx_len_65_127"),
	ARASAN_GEMAC_RX_STAT(LEN_128_255, "frames_rx_len_128_255"),
	ARASAN_GEMAC_RX_STAT(LEN_256_511, "frames_rx_len_256_511"),
	ARASAN_GEMAC_RX_STAT(LEN_512_1023, "frames_rx_len_512_1023"),
	ARASAN_GEMAC_RX_STAT(LEN_1024_1518, "frames_rx_len_1024_1518"),
	ARASAN_GEMAC_RX_STAT(LEN_1519_PLUS, "frames_rx_len_1519_plus"),
	ARASAN_GEMAC_RX_STAT(DROPPED_BUFF_FULL, "frames_rx_dropped_buff_full"),
	ARASAN_GEMAC_RX_STAT(TRUNC_BUFF_FULL, "frames_rx_truncated_buff_full"),
};

#define ARASAN_GEMAC_STATS_LEN ARRAY_SIZE(arasan_gemac_stats)

struct arasan_gemac_dma_desc {
	u32 status;
	u32 misc;
	u32 buffer1;
	u32 buffer2;
};

struct arasan_gemac_ring_info {
	struct sk_buff *skb;
	dma_addr_t mapping;
};

struct arasan_gemac_ptp {
	struct ptp_clock *clock;
	struct ptp_clock_info clock_info;
	struct hwtstamp_config tstamp_config;

	u32 clk_1588_freq;
	/* ts_lock used to prevent concurrent access to clk_1588 registers */
	spinlock_t ts_lock;

	u32 sw_mul;
	struct cyclecounter cycle_counter;
	struct timecounter time_counter;
};

struct arasan_gemac_pdata {
	void __iomem *regs;

	struct platform_device *pdev;
	struct net_device      *dev;

	/* driver lock */
	spinlock_t lock;

	struct clk_bulk_data clks[CLOCKS_NUM];
	struct reset_control *rst;

	struct arasan_gemac_dma_desc  *rx_ring;
	struct arasan_gemac_dma_desc  *tx_ring;
	struct arasan_gemac_ring_info *tx_buffers;
	struct arasan_gemac_ring_info *rx_buffers;

	dma_addr_t rx_dma_addr;
	dma_addr_t tx_dma_addr;

	/* lock for descriptor completion */
	spinlock_t tx_freelock;
	int tx_ring_head, tx_ring_tail;
	int rx_ring_head, rx_ring_tail;

	struct napi_struct napi;
	struct gen_pool *desc_pool;

	struct mii_bus      *mii_bus;
	struct phy_device   *phy_dev;
	unsigned int        link;
	unsigned int        speed;
	unsigned int        duplex;
	u32                 msg_enable;
	u32                 hwfifo_size;
	u32                 mdc_freq;
	u32                 tx_threshold;
	u8                  axi_width64;

	u64 ethtool_stats[ARASAN_GEMAC_STATS_LEN];

	phy_interface_t     phy_interface;
	int phy_irq[PHY_MAX_ADDR];

	struct arasan_gemac_ptp ptp;
};

int arasan_gemac_ptp_hwstamp_set(struct arasan_gemac_pdata *pd);
void arasan_gemac_ptp_do_txstamp(struct arasan_gemac_pdata *pd, struct sk_buff *skb);
void arasan_gemac_ptp_do_rxstamp(struct arasan_gemac_pdata *pd, struct sk_buff *skb);
int arasan_gemac_ptp_init(struct arasan_gemac_pdata *pd);

#endif /* _ARASAN_GEMAC_H */
