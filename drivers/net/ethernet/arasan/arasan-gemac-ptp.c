// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 RnD Center "ELVEES", JSC
 *
 * 1588 PTP support for ARASAN GEMAC device.
 */

#include "arasan-gemac.h"

// Set PTP_TIMER_RATE to 1 GHz:
// PTP_TIMER_RATE = CLK_1588_CONST_FREQ * CLOCK_INC_VALUE / CLOCK_INC_PERIOD
#define CLK_1588_CONST_FREQ 125000000
#define CLOCK_INC_VALUE 8
#define CLOCK_INC_PERIOD 1

#define SW_DIV 1000000000ULL

static u64 arasan_gemac_ptp_timer_read(struct arasan_gemac_pdata *pd,
				       int timer_lo_reg)
{
	int timer_up_reg = 0;
	u64 cycle = 0;
	unsigned long flags;

	if (timer_lo_reg == SYS_TIME_VAL_LO) {
		timer_up_reg = SYS_TIME_VAL_UP;
	} else if (timer_lo_reg == TX_TSTAMP_VAL_LO) {
		timer_up_reg = TX_TSTAMP_VAL_UP;
	} else if (timer_lo_reg == RX_TSTAMP_VAL_LO) {
		timer_up_reg = RX_TSTAMP_VAL_UP;
	} else {
		netdev_err(pd->dev, "Wrong PTP timer LO reg %d\n",
			   timer_lo_reg);
		return 0;
	}

	spin_lock_irqsave(&pd->ptp.ts_lock, flags);
	cycle = arasan_gemac_readl(pd, timer_lo_reg);
	cycle |= ((u64)arasan_gemac_readl(pd, timer_up_reg)) << 32;
	spin_unlock_irqrestore(&pd->ptp.ts_lock, flags);

	return cycle;
}

static u64 arasan_gemac_ptp_cc_read(const struct cyclecounter *cc)
{
	struct arasan_gemac_ptp *ptp =
		container_of(cc, struct arasan_gemac_ptp, cycle_counter);
	struct arasan_gemac_pdata *pd =
		container_of(ptp, struct arasan_gemac_pdata, ptp);

	return arasan_gemac_ptp_timer_read(pd, SYS_TIME_VAL_LO);
}

static u64 arasan_gemac_ptp_precise_div_mul(u64 val, u32 mul, u32 div)
{
	/* Math operation sequence is important to prevent overflow
	 * and loss of accuracy of result value
	 */
	return val / div * mul + (val % div) * mul / div;
}

static u64 arasan_gemac_ptp_get_real_time_ns(struct arasan_gemac_pdata *pd,
					     int timer_lo_reg)
{
	u64 cycle_delta, cycle_now;
	u64 ns_offset, real_time_ns;

	cycle_now = arasan_gemac_ptp_timer_read(pd, timer_lo_reg);

	cycle_delta = cycle_now - pd->ptp.time_counter.cycle_last;
	ns_offset = cyclecounter_cyc2ns(pd->ptp.time_counter.cc, cycle_delta,
					pd->ptp.time_counter.mask,
					&pd->ptp.time_counter.frac);
	real_time_ns = arasan_gemac_ptp_precise_div_mul(ns_offset,
							pd->ptp.sw_mul, SW_DIV) +
		       pd->ptp.time_counter.nsec;

	pd->ptp.time_counter.cycle_last = cycle_now;
	pd->ptp.time_counter.nsec = real_time_ns;

	return pd->ptp.time_counter.nsec;
}

static int arasan_gemac_ptp_adjfine(struct ptp_clock_info *ptpci,
				    long scaled_ppm)
{
	struct arasan_gemac_ptp *ptp =
		container_of(ptpci, struct arasan_gemac_ptp, clock_info);
	struct arasan_gemac_pdata *pd =
		container_of(ptp, struct arasan_gemac_pdata, ptp);
	int ppb;
	bool decrease_freq = false;
	u64 real_time_ns;

	if (scaled_ppm < 0) {
		decrease_freq = true;
		scaled_ppm = -scaled_ppm;
	}

	/* convert ppm offset with frac field to ppb to deal with integers */
	ppb = scaled_ppm_to_ppb(scaled_ppm);

	/* Call to arasan_gemac_ptp_get_real_time_ns() to update tc with previouts sw_mul*/
	real_time_ns = arasan_gemac_ptp_get_real_time_ns(pd, SYS_TIME_VAL_LO);

	ptp->sw_mul = (decrease_freq) ? (SW_DIV - ppb) : (SW_DIV + ppb);

	return 0;
}

static int arasan_gemac_ptp_adjtime(struct ptp_clock_info *ptpci, s64 delta)
{
	struct arasan_gemac_ptp *ptp =
		container_of(ptpci, struct arasan_gemac_ptp, clock_info);
	unsigned long flags;

	spin_lock_irqsave(&ptp->ts_lock, flags);
	timecounter_adjtime(&ptp->time_counter, delta);
	spin_unlock_irqrestore(&ptp->ts_lock, flags);

	/* TBD: Took from cavium_ptp_adjtime(). Is it really required here? */
	smp_mb();

	return 0;
}

static int arasan_gemac_ptp_gettimex64(struct ptp_clock_info *ptpci,
				       struct timespec64 *ts,
				       struct ptp_system_timestamp *sts)
{
	struct arasan_gemac_ptp *ptp =
		container_of(ptpci, struct arasan_gemac_ptp, clock_info);
	struct arasan_gemac_pdata *pd =
		container_of(ptp, struct arasan_gemac_pdata, ptp);
	u64 real_time_ns;

	ptp_read_system_prets(sts);

	real_time_ns = arasan_gemac_ptp_get_real_time_ns(pd, SYS_TIME_VAL_LO);
	*ts = ns_to_timespec64(real_time_ns);

	ptp_read_system_postts(sts);

	return 0;
}

static int arasan_gemac_ptp_settime64(struct ptp_clock_info *ptpci,
				      const struct timespec64 *ts)
{
	struct arasan_gemac_ptp *ptp =
		container_of(ptpci, struct arasan_gemac_ptp, clock_info);
	unsigned long flags;
	u64 nsec;

	nsec = timespec64_to_ns(ts);

	spin_lock_irqsave(&ptp->ts_lock, flags);
	timecounter_init(&ptp->time_counter, &ptp->cycle_counter, nsec);
	spin_unlock_irqrestore(&ptp->ts_lock, flags);

	return 0;
}

static const struct ptp_clock_info arasan_gemac_ptp_info = {
	.owner = THIS_MODULE,
	.name = "arasan-gemac-ptp",
	/* max_adj = 300000000 allows to adjust PTP freq in a range: 1 GHz +/- 300 MHz */
	.max_adj = 300000000,
	.n_alarm = 0,
	.n_ext_ts = 0,
	.n_per_out = 0,
	.n_pins = 0,
	.pps = 0,
	.adjfine = arasan_gemac_ptp_adjfine,
	.adjtime = arasan_gemac_ptp_adjtime,
	.gettimex64 = arasan_gemac_ptp_gettimex64,
	.settime64 = arasan_gemac_ptp_settime64,
};

int arasan_gemac_ptp_hwstamp_set(struct arasan_gemac_pdata *pd)
{
	u32 regval;

	switch (pd->ptp.tstamp_config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		regval = arasan_gemac_readl(pd, MODULE_1588_CR);
		regval &= ~MODULE_1588_CR_RX_TSTAMP_EN;
		arasan_gemac_writel(pd, MODULE_1588_CR, regval);
		break;

	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
		regval = arasan_gemac_readl(pd, MODULE_1588_CR);
		regval = MODULE_1588_CR_RX_PACKET_TYPE(regval,
						       RX_PTP_PACKET_V1_L4) |
			 MODULE_1588_CR_RX_TSTAMP_EN;
		arasan_gemac_writel(pd, MODULE_1588_CR, regval);

		arasan_gemac_writel(pd, PTP_MESSAGE_ID, PTP_SYNC_MESSAGE_ID);
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
		regval = arasan_gemac_readl(pd, MODULE_1588_CR);
		regval = MODULE_1588_CR_RX_PACKET_TYPE(regval,
						       RX_PTP_PACKET_V2_L2) |
			 MODULE_1588_CR_RX_TSTAMP_EN;
		arasan_gemac_writel(pd, MODULE_1588_CR, regval);

		arasan_gemac_writel(pd, PTP_MESSAGE_ID, PTP_SYNC_MESSAGE_ID);
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
		regval = arasan_gemac_readl(pd, MODULE_1588_CR);
		regval = MODULE_1588_CR_RX_PACKET_TYPE(regval,
						       RX_PTP_PACKET_V2_L2_L4) |
			 MODULE_1588_CR_RX_TSTAMP_EN;
		arasan_gemac_writel(pd, MODULE_1588_CR, regval);

		arasan_gemac_writel(pd, PTP_MESSAGE_ID, PTP_SYNC_MESSAGE_ID);
		break;

	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		regval = arasan_gemac_readl(pd, MODULE_1588_CR);
		regval = MODULE_1588_CR_RX_PACKET_TYPE(regval,
						       RX_PTP_PACKET_V1_L4) |
			 MODULE_1588_CR_RX_TSTAMP_EN;
		arasan_gemac_writel(pd, MODULE_1588_CR, regval);
		arasan_gemac_writel(pd, PTP_MESSAGE_ID,
				    PTP_DELAY_REQ_MESSAGE_ID);
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
		regval = arasan_gemac_readl(pd, MODULE_1588_CR);
		regval = MODULE_1588_CR_RX_PACKET_TYPE(regval,
						       RX_PTP_PACKET_V2_L2) |
			 MODULE_1588_CR_RX_TSTAMP_EN;
		arasan_gemac_writel(pd, MODULE_1588_CR, regval);
		arasan_gemac_writel(pd, PTP_MESSAGE_ID,
				    PTP_DELAY_REQ_MESSAGE_ID);
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		regval = arasan_gemac_readl(pd, MODULE_1588_CR);
		regval = MODULE_1588_CR_RX_PACKET_TYPE(regval,
						       RX_PTP_PACKET_V2_L2_L4) |
			 MODULE_1588_CR_RX_TSTAMP_EN;
		arasan_gemac_writel(pd, MODULE_1588_CR, regval);
		arasan_gemac_writel(pd, PTP_MESSAGE_ID,
				    PTP_DELAY_REQ_MESSAGE_ID);
		break;
	default:
		regval = arasan_gemac_readl(pd, MODULE_1588_CR);
		regval &= ~MODULE_1588_CR_RX_TSTAMP_EN;
		arasan_gemac_writel(pd, MODULE_1588_CR, regval);
		pd->ptp.tstamp_config.rx_filter = HWTSTAMP_FILTER_NONE;
		return -ERANGE;
	}

	switch (pd->ptp.tstamp_config.tx_type) {
	case HWTSTAMP_TX_OFF:
		regval = arasan_gemac_readl(pd, MODULE_1588_CR);
		regval &= ~MODULE_1588_CR_TX_TSTAMP_EN;
		arasan_gemac_writel(pd, MODULE_1588_CR, regval);
		break;
	case HWTSTAMP_TX_ON:
	case HWTSTAMP_TX_ONESTEP_SYNC:
	case HWTSTAMP_TX_ONESTEP_P2P:
		regval = arasan_gemac_readl(pd, MODULE_1588_CR);
		regval |= MODULE_1588_CR_TX_TSTAMP_EN;
		arasan_gemac_writel(pd, MODULE_1588_CR, regval);
		break;
	default:
		return -ERANGE;
	}
	return 0;
}
EXPORT_SYMBOL(arasan_gemac_ptp_hwstamp_set);

static int arasan_gemac_ptp_tx_hwtstamp(struct arasan_gemac_pdata *pd,
					u64 *tx_real_time_ns)
{
	if (!(arasan_gemac_readl(pd, MODULE_1588_CR) &
	      MODULE_1588_CR_TX_TSTAMP_EN)) {
		netdev_err(pd->dev, "TX tstamp not enabled\n");
		return -EIO;
	}

	*tx_real_time_ns =
		arasan_gemac_ptp_get_real_time_ns(pd, TX_TSTAMP_VAL_LO);

	return 0;
}

void arasan_gemac_ptp_do_txstamp(struct arasan_gemac_pdata *pd,
				 struct sk_buff *skb)
{
	struct skb_shared_hwtstamps shhwtstamps;
	u64 tx_real_time_ns;
	int res;

	if (pd->ptp.tstamp_config.tx_type == HWTSTAMP_TX_OFF)
		return;

	res = arasan_gemac_ptp_tx_hwtstamp(pd, &tx_real_time_ns);

	if (res < 0)
		return;

	memset(&shhwtstamps, 0, sizeof(shhwtstamps));
	shhwtstamps.hwtstamp = ns_to_ktime(tx_real_time_ns);
	skb_tstamp_tx(skb, &shhwtstamps);
}
EXPORT_SYMBOL(arasan_gemac_ptp_do_txstamp);

static int arasan_gemac_ptp_rx_hwtstamp(struct arasan_gemac_pdata *pd,
					u64 *rx_real_time_ns)
{
	if (!(arasan_gemac_readl(pd, MODULE_1588_CR) &
	      MODULE_1588_CR_RX_TSTAMP_EN)) {
		netdev_err(pd->dev, "RX tstamp not enabled\n");
		return -EIO;
	}

	*rx_real_time_ns =
		arasan_gemac_ptp_get_real_time_ns(pd, RX_TSTAMP_VAL_LO);

	return 0;
}

void arasan_gemac_ptp_do_rxstamp(struct arasan_gemac_pdata *pd, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps *shhwtstamps = skb_hwtstamps(skb);
	u64 rx_real_time_ns;
	int res;

	if (pd->ptp.tstamp_config.rx_filter == HWTSTAMP_FILTER_NONE)
		return;

	res = arasan_gemac_ptp_rx_hwtstamp(pd, &rx_real_time_ns);
	if (res < 0)
		return;

	memset(shhwtstamps, 0, sizeof(struct skb_shared_hwtstamps));
	shhwtstamps->hwtstamp = ns_to_ktime(rx_real_time_ns);
}
EXPORT_SYMBOL(arasan_gemac_ptp_do_rxstamp);

int arasan_gemac_ptp_init(struct arasan_gemac_pdata *pd)
{
	int clk_1588_rate;
	struct cyclecounter *cc;

	clk_1588_rate = clk_get_rate(pd->clks[CLOCK_1588].clk);
	if (clk_1588_rate != CLK_1588_CONST_FREQ) {
		clk_set_rate(pd->clks[CLOCK_1588].clk, CLK_1588_CONST_FREQ);
		clk_1588_rate = clk_get_rate(pd->clks[CLOCK_1588].clk);
		if (clk_1588_rate != CLK_1588_CONST_FREQ) {
			netdev_err(pd->dev,
				   "Failed to set clk_1588 to 125 MHz, clk_1588 freq = %d Hz\n",
				   clk_1588_rate);
			return -EIO;
		}
		netdev_info(pd->dev, "Set clk_1588 freq to 125 MHz\n");
	}

	pd->ptp.clock_info = arasan_gemac_ptp_info;

	pd->ptp.clock = ptp_clock_register(&pd->ptp.clock_info, &pd->pdev->dev);
	if (IS_ERR(pd->ptp.clock)) {
		netdev_err(pd->dev, "PTP clock register failed\n");
		pd->ptp.clock = NULL;
		return PTR_ERR(pd->ptp.clock);
	}

	/* Timestamping is required for PTP System timer.
	 * FIXME: maybe should be moved to arasan_gemac_ptp_hwstamp_set()
	 * where it's enabling different timestamps
	 */
	arasan_gemac_writel(pd, MODULE_1588_CR,
			    MODULE_1588_CR_RX_TSTAMP_EN |
			    MODULE_1588_CR_TX_TSTAMP_EN);

	arasan_gemac_writel(pd, PTP_ETHER_TYPE, PTP_ETHERTYPE);
	arasan_gemac_writel(pd, PTP_UDP_PORT_REG, PTP_UDP_PORT_319);

	spin_lock_init(&pd->ptp.ts_lock);

	cc = &pd->ptp.cycle_counter;
	cc->read = arasan_gemac_ptp_cc_read;
	cc->mask = 0;
	cc->mult = 1;
	cc->shift = 0;

	timecounter_init(&pd->ptp.time_counter, &pd->ptp.cycle_counter,
			 ktime_to_ns(ktime_get_real()));
	arasan_gemac_writel(pd, INC_ATTRIBUTES,
			    INC_ATTRIBUTES_VAL(CLOCK_INC_VALUE) |
			    INC_ATTRIBUTES_PERIOD(CLOCK_INC_PERIOD));

	pd->ptp.sw_mul = SW_DIV;

	return 0;
}
EXPORT_SYMBOL(arasan_gemac_ptp_init);

int arasan_gemac_ptp_deinit(struct arasan_gemac_pdata *pd)
{
	if (pd->ptp.clock)
		ptp_clock_unregister(pd->ptp.clock);

	arasan_gemac_writel(pd, MODULE_1588_CR,
			    ~MODULE_1588_CR_RX_TSTAMP_EN &
			    ~MODULE_1588_CR_TX_TSTAMP_EN);
	return 0;
}
