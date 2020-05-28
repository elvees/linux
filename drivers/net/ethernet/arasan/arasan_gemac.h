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

/* GEMAC TX descriptor can describe 4K buffer.
 * But currently some unexplored bugs are observed if we set Jumbo frame
 * more than 3500 bytes. This bugs lead to lack of transmission. */
#define ARASAN_JUMBO_MTU 3500U

/* GEMAC FIFO depth in 32 bit words */
#define ARASAN_FIFO_SZ 1024

#define mtu_to_frame_sz(x) ((x) + VLAN_ETH_HLEN)
#define mtu_to_buf_sz(x) (mtu_to_frame_sz(x) + NET_IP_ALIGN + 4)

#define TX_RING_SIZE (128)
#define RX_RING_SIZE (128)
#define NAPI_WEIGHT (64)

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

/* Arasan GEMAC register fields */

#define DMA_CONFIGURATION_SOFT_RESET                  BIT(0)
#define DMA_CONFIGURATION_BURST_LENGTH(VAL)           ((VAL) << 1)
#define DMA_CONFIGURATION_WAIT_FOR_DONE               BIT(16)

#define DMA_CONTROL_START_TRANSMIT_DMA                BIT(0)
#define DMA_CONTROL_START_RECEIVE_DMA                 BIT(1)

#define DMA_STATUS_AND_IRQ_TRANSMIT_DONE              BIT(0)
#define DMA_STATUS_AND_IRQ_TRANS_DESC_UNAVAIL         BIT(1)
#define DMA_STATUS_AND_IRQ_TX_DMA_STOPPED             BIT(2)
#define DMA_STATUS_AND_IRQ_RECEIVE_DONE               BIT(4)
#define DMA_STATUS_AND_IRQ_RX_DMA_STOPPED             BIT(6)
#define DMA_STATUS_AND_IRQ_MAC_INTERRUPT              BIT(11)
#define DMA_STATUS_AND_IRQ_TRANSMIT_DMA_STATE(VAL)    (((VAL) & 0x7000) >> 16)
#define DMA_STATUS_AND_IRQ_RECEIVE_DMA_STATE(VAL)     (((VAL) & 0xf0000) >> 20)

#define DMA_INTERRUPT_ENABLE_TRANSMIT_DONE            BIT(0)
#define DMA_INTERRUPT_ENABLE_TRANS_DESC_UNAVAIL       BIT(1)
#define DMA_INTERRUPT_ENABLE_RECEIVE_DONE             BIT(4)

#define DMA_INTERRUPT_ENABLE_MAC                      BIT(8)
#define DMA_STATUS_AND_IRQ_MAC                        BIT(8)

#define MAC_GLOBAL_CONTROL_SPEED(VAL)                 ((VAL) << 0)
#define MAC_GLOBAL_CONTROL_DUPLEX_MODE(VAL)           ((VAL) << 2)

#define MAC_TRANSMIT_CONTROL_TRANSMIT_ENABLE          BIT(0)

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

#define MAC_INTERRUPT_ENABLE_UNDERRUN                 BIT(0)
#define MAC_IRQ_STATUS_UNDERRUN                       BIT(0)

/* DMA descriptor fields */

#define DMA_RDES0_OWN_BIT      BIT(31)
#define DMA_RDES0_FD           BIT(30)
#define DMA_RDES0_LD           BIT(29)
#define DMA_RDES1_EOR          BIT(26)

#define DMA_TDES0_OWN_BIT      BIT(31)
#define DMA_TDES1_IOC          BIT(31)
#define DMA_TDES1_LS           BIT(30)
#define DMA_TDES1_FS           BIT(29)
#define DMA_TDES1_EOR          BIT(26)

#define arasan_gemac_readl(port, reg) readl((port)->regs + (reg))
#define arasan_gemac_writel(port, reg, value) \
	writel((value), (port)->regs + (reg))

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

struct arasan_gemac_pdata {
	void __iomem *regs;

	struct platform_device *pdev;
	struct net_device      *dev;

	/* driver lock */
	spinlock_t lock;

	struct clk *hclk;

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

	struct mii_bus      *mii_bus;
	struct phy_device   *phy_dev;
	unsigned int        link;
	unsigned int        speed;
	unsigned int        duplex;
	int                 txclk_125en;
	u32                 msg_enable;

	phy_interface_t     phy_interface;
	int phy_irq[PHY_MAX_ADDR];
};

#endif /* _ARASAN_GEMAC_H */
