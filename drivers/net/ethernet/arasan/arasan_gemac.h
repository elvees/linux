/*
 *  Copyright 2015 ELVEES NeoTek CJSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _ARASAN_GEMAC_H
#define _ARASAN_GEMAC_H

#define PKT_BUF_SZ (VLAN_ETH_FRAME_LEN + NET_IP_ALIGN + 4)
#define TX_RING_SIZE (32)
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
#define MAC_INTERRUPT                             0x01E0
#define MAC_INTERRUPT_ENABLE                      0x01E4
#define MAC_VLAN_TPID1                            0x01E8
#define MAC_VLAN_TPID2                            0x01EC
#define MAC_VLAN_TPID3                            0x01F0

/* Arasan GEMAC register fields */

#define DMA_CONFIGURATION_SOFT_RESET                  (1 << 0)
#define DMA_CONFIGURATION_WAIT_FOR_DONE               (1 << 16)

#define DMA_CONTROL_START_TRANSMIT_DMA                (1 << 0)
#define DMA_CONTROL_START_RECEIVE_DMA                 (1 << 1)

#define DMA_STATUS_AND_IRQ_TRANSFER_DONE              (1 << 0)
#define DMA_STATUS_AND_IRQ_TRANS_DESC_UNVAIL          (1 << 1)
#define DMA_STATUS_AND_IRQ_TX_DMA_STOPPED             (1 << 2)
#define DMA_STATUS_AND_IRQ_RECEIVE_DONE               (1 << 4)
#define DMA_STATUS_AND_IRQ_MAC_INTERRUPT              (1 << 11)
#define DMA_STATUS_AND_IRQ_TRANSMIT_DMA_STATE(VAL)    (((VAL) & 0x7000) >> 16)

#define DMA_INTERRUPT_ENABLE_TRANSMIT_DONE            (1 << 0)
#define DMA_INTERRUPT_ENABLE_RECEIVE_DONE             (1 << 4)

#define MAC_GLOBAL_CONTROL_SPEED(VAL)                 ((VAL) << 0)
#define MAC_GLOBAL_CONTROL_DUPLEX_MODE(VAL)           ((VAL) << 2)

#define MAC_RECEIVE_CONTROL_STORE_AND_FORWARD         (1 << 3)

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

/* DMA descriptor fields */

#define DMA_RDES0_OWN_BIT      (1 << 31)
#define DMA_RDES0_FD           (1 << 30)
#define DMA_RDES0_LD           (1 << 29)

#define DMA_RDES1_EOR          (1 << 26)

#define DMA_TDES0_OWN_BIT      (1 << 31)
#define DMA_TDES1_IOC          (1 << 31)
#define DMA_TDES1_LS           (1 << 30)
#define DMA_TDES1_FS           (1 << 29)
#define DMA_TDES1_EOR          (1 << 26)

#define arasan_gemac_readl(port, reg) __raw_readl((port)->regs + reg)
#define arasan_gemac_writel(port, reg, value) __raw_writel((value), (port)->regs + reg)

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

	spinlock_t lock;

	struct arasan_gemac_dma_desc  *rx_ring;
	struct arasan_gemac_dma_desc  *tx_ring;
	struct arasan_gemac_ring_info *tx_buffers;
	struct arasan_gemac_ring_info *rx_buffers;

	dma_addr_t rx_dma_addr;
	dma_addr_t tx_dma_addr;

	int tx_ring_head, tx_ring_tail;
	int rx_ring_head, rx_ring_tail;

	struct napi_struct napi;

	struct mii_bus      *mii_bus;
	struct phy_device   *phy_dev;
	unsigned int        link;
	unsigned int        speed;
	unsigned int        duplex;

	phy_interface_t     phy_interface;
	int phy_irq[PHY_MAX_ADDR];
};

#endif /* _ARASAN_GEMAC_H */
