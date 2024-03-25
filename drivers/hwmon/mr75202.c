// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021-2024 RnD Center "ELVEES", JSC
 *
 * This is a hardware monitoring driver for Moortec MR75202 PVT controller
 * which is used to configure & control Moortec embedded analog IPs like
 * temperature sensors (TS), voltage monitors (VM) and process detectors (PD).
 */
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/of.h>
#include <linux/bitfield.h>

#include <linux/io.h>
#include <linux/iopoll.h>

#define MR75202_ID 0x0

#define MR75202_TS_BASE 0x10
#define MR75202_VM_BASE 0x184
#define MR75202_PD_BASE 0xab0

#define MR75202_TS_REG(a) (MR75202_TS_BASE + (a))
#define MR75202_VM_REG(a) (MR75202_VM_BASE + (a))

#define MR75202_TS_CLK_SYNTH         MR75202_TS_REG(0x0)
#define MR75202_TS_SDIF_DISABLE      MR75202_TS_REG(0x4)
#define MR75202_TS_SDIF_STATUS       MR75202_TS_REG(0x8)
#define MR75202_TS_SDIF_CTRL         MR75202_TS_REG(0xc)
#define MR75202_TS_SDIF              MR75202_TS_REG(0x10)
#define MR75202_TS_SDIF_RDATA(n)     MR75202_TS_REG(0x14 + (n) * 0x4)
#define MR75202_TS_DATA(n)           MR75202_TS_REG(0x34 + (n) * 0x4)
#define MR75202_TS_SMPL_CTRL         MR75202_TS_REG(0x54)
#define MR75202_TS_SMPL_HI_CLR       MR75202_TS_REG(0x58)
#define MR75202_TS_SMPL_LO_SET       MR75202_TS_REG(0x5c)
#define MR75202_TS_SMPL_STATUS       MR75202_TS_REG(0x60)
#define MR75202_TS_SMPL_CTR          MR75202_TS_REG(0x64)
#define MR75202_TS_SMPL_HILO(n)      MR75202_TS_REG(0x68 + (n) * 0x4)
#define MR75202_TS_IRQ_STATUS        MR75202_TS_REG(0x88)
#define MR75202_TS_ALARMA_IRQ_STATUS MR75202_TS_REG(0x8c)
#define MR75202_TS_ALARMB_IRQ_STATUS MR75202_TS_REG(0x90)
#define MR75202_TS_DONE_IRQ          MR75202_TS_REG(0x94)
#define MR75202_TS_FAULT_IRQ         MR75202_TS_REG(0x98)
#define MR75202_TS_ALARM_IRQ(n)      MR75202_TS_REG(0x9c + (n) * 0x4)
#define MR75202_TS_DONE_IRQ_ENA      MR75202_TS_REG(0xbc)
#define MR75202_TS_FAULT_IRQ_ENA     MR75202_TS_REG(0xc0)
#define MR75202_TS_ALARM_IRQ_ENA(n)  MR75202_TS_REG(0xc4 + (n) * 0x4)
#define MR75202_TS_DONE_IRQ_SRC      MR75202_TS_REG(0xe4)
#define MR75202_TS_FAULT_IRQ_SRC     MR75202_TS_REG(0xe8)
#define MR75202_TS_ALARM_IRQ_SRC(n)  MR75202_TS_REG(0xec + (n) * 0x4)
#define MR75202_TS_DONE_IRQ_TEST     MR75202_TS_REG(0x10c)
#define MR75202_TS_FAULT_IRQ_TEST    MR75202_TS_REG(0x110)
#define MR75202_TS_ALARM_IRQ_TEST(n) MR75202_TS_REG(0x114 + (n) * 0x4)
#define MR75202_TS_ALARMA_CFG(n)     MR75202_TS_REG(0x134 + (n) * 0x4)
#define MR75202_TS_ALARMB_CFG(n)     MR75202_TS_REG(0x154 + (n) * 0x4)

#define MR75202_TS_CLK_SYNTH_LO     GENMASK(7, 0)
#define MR75202_TS_CLK_SYNTH_HI     GENMASK(15, 8)
#define MR75202_TS_CLK_SYNTH_STROBE GENMASK(19, 16)
#define MR75202_TS_CLK_SYNTH_ENA    BIT(24)

#define MR75202_TS_SDIF_STATUS_BUSY BIT(0)
#define MR75202_TS_SDIF_STATUS_LOCK BIT(1)

#define MR75202_TS_SDIF_WDATA GENMASK(23, 0)
#define MR75202_TS_SDIF_ADDR  GENMASK(26, 24)
#define MR75202_TS_SDIF_WRN   BIT(27)
#define MR75202_TS_SDIF_PROG  BIT(31)

#define MR75202_VM_CLK_SYNTH          MR75202_VM_REG(0x0)
#define MR75202_VM_SDIF_DISABLE       MR75202_VM_REG(0x4)
#define MR75202_VM_SDIF_STATUS        MR75202_VM_REG(0x8)
#define MR75202_VM_SDIF_CTRL          MR75202_VM_REG(0xc)
#define MR75202_VM_SDIF               MR75202_VM_REG(0x10)
#define MR75202_VM_SDIF_RDATA(n)      MR75202_VM_REG(0x14 + (n) * 0x4)
#define MR75202_VM_DATA(n)            MR75202_VM_REG(0x34 + (n) * 0x40)
#define MR75202_VM_SMPL_CTRL          MR75202_VM_REG(0x234)
#define MR75202_VM_SMPL_HI_CLR(n)     MR75202_VM_REG(0x238 + (n) * 0x4)
#define MR75202_VM_SMPL_LO_SET(n)     MR75202_VM_REG(0x258 + (n) * 0x4)
#define MR75202_VM_SMPL_STATUS        MR75202_VM_REG(0x278)
#define MR75202_VM_SMPL_CTR           MR75202_VM_REG(0x27c)
#define MR75202_VM_SMPL_HILO(n)       MR75202_VM_REG(0x280 + (n) * 0x40)
#define MR75202_VM_IRQ_STATUS         MR75202_VM_REG(0x480)
#define MR75202_VM_ALARMA_IRQ_STATUS  MR75202_VM_REG(0x484)
#define MR75202_VM_ALARMB_IRQ_STATUS  MR75202_VM_REG(0x488)
#define MR75202_VM_DONE_IRQ           MR75202_VM_REG(0x48c)
#define MR75202_VM_FAULT_IRQ          MR75202_VM_REG(0x490)
#define MR75202_VM_ALARM_IRQ(n)       MR75202_VM_REG(0x494 + (n) * 0x4)
#define MR75202_VM_DONE_IRQ_ENA       MR75202_VM_REG(0x4b4)
#define MR75202_VM_FAULT_IRQ_ENA      MR75202_VM_REG(0x4b8)
#define MR75202_VM_ALARM_IRQ_ENA(n)   MR75202_VM_REG(0x4bc + (n) * 0x4)
#define MR75202_VM_DONE_IRQ_SRC       MR75202_VM_REG(0x4dc)
#define MR75202_VM_FAULT_IRQ_SRC      MR75202_VM_REG(0x4e0)
#define MR75202_VM_ALARM_IRQ_SRC(n)   MR75202_VM_REG(0x4e4 + (n) * 0x4)
#define MR75202_VM_DONE_IRQ_TEST      MR75202_VM_REG(0x504)
#define MR75202_VM_FAULT_IRQ_TEST     MR75202_VM_REG(0x508)
#define MR75202_VM_ALARM_IRQ_TEST(n)  MR75202_VM_REG(0x50c + (n) * 0x4)
#define MR75202_VM_ALARMA_CFG(n)      MR75202_VM_REG(0x52c + (n) * 0x40)
#define MR75202_VM_ALARMB_CFG(n)      MR75202_VM_REG(0x72c + (n) * 0x40)

#define MR75202_VM_CLK_SYNTH_LO     GENMASK(7, 0)
#define MR75202_VM_CLK_SYNTH_HI     GENMASK(15, 8)
#define MR75202_VM_CLK_SYNTH_STROBE GENMASK(19, 16)
#define MR75202_VM_CLK_SYNTH_ENA    BIT(24)

#define MR75202_VM_SDIF_STATUS_BUSY BIT(0)
#define MR75202_VM_SDIF_STATUS_LOCK BIT(1)

#define MR75202_VM_SDIF_WDATA GENMASK(23, 0)
#define MR75202_VM_SDIF_ADDR  GENMASK(26, 24)
#define MR75202_VM_SDIF_WRN   BIT(27)
#define MR75202_VM_SDIF_PROG  BIT(31)

#define MR75202_SDA_IP_CTRL 0x0
#define MR75202_SDA_IP_CFG 0x1
#define MR75202_SDA_IP_CFGA 0x2
#define MR75202_SDA_IP_DATA 0x3
#define MR75202_SDA_IP_POLLING 0x4
#define MR75202_SDA_IP_TMR 0x5

#define MR75202_SDA_IP_CTRL_PD       BIT(0)
#define MR75202_SDA_IP_CTRL_RESETN   BIT(1)
#define MR75202_SDA_IP_CTRL_RUN_ONCE BIT(2)
#define MR75202_SDA_IP_CTRL_RUN_CONT BIT(3)
#define MR75202_SDA_IP_CTRL_CLOAD    BIT(4)
#define MR75202_SDA_IP_CTRL_AUTO     BIT(8)
#define MR75202_SDA_IP_CTRL_STOP     BIT(9)
#define MR75202_SDA_IP_CTRL_VM_MODE  BIT(10)

#define MR75202_SDA_IP_DATA_DAT   GENMASK(15, 0)
#define MR75202_SDA_IP_DATA_TYPE  BIT(16)
#define MR75202_SDA_IP_DATA_FAULT BIT(17)
#define MR75202_SDA_IP_DATA_DONE  BIT(18)
#define MR75202_SDA_IP_DATA_CH    GENMASK(23, 20)

#define MR75202_DELAY_US 500
#define MR75202_TIMEOUT_US 50000

struct mr75202_priv {
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rst;
	const char *temp_labels[8];
	const char *in_labels[8];
	struct mutex mutex;
	size_t nts;
	size_t nvm;
};

static umode_t mr75202_is_visible(const void *data,
				  enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	const struct mr75202_priv *priv = data;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			return 0444;
		case hwmon_temp_label:
			if (priv->temp_labels[channel])
				return 0444;
		}
		break;

	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			return 0444;
		case hwmon_in_label:
			if (priv->in_labels[channel])
				return 0444;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int mr75202_write_ts(struct mr75202_priv *priv, u32 address, u32 data)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(priv->base + MR75202_TS_SDIF_STATUS, val,
				 !(val & MR75202_TS_SDIF_STATUS_BUSY),
				 MR75202_DELAY_US, MR75202_TIMEOUT_US);
	if (ret)
		return ret;

	data = FIELD_PREP(MR75202_TS_SDIF_WDATA, data) |
	       FIELD_PREP(MR75202_TS_SDIF_ADDR, address) |
	       MR75202_TS_SDIF_WRN | MR75202_TS_SDIF_PROG;
	writel(data, priv->base + MR75202_TS_SDIF);

	return 0;
}

static int mr75202_write_vm(struct mr75202_priv *priv, u32 address, u32 data)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(priv->base + MR75202_VM_SDIF_STATUS, val,
				 !(val & MR75202_VM_SDIF_STATUS_BUSY),
				 MR75202_DELAY_US, MR75202_TIMEOUT_US);
	if (ret)
		return ret;

	data = FIELD_PREP(MR75202_VM_SDIF_WDATA, data) |
	       FIELD_PREP(MR75202_VM_SDIF_ADDR, address) |
	       MR75202_VM_SDIF_WRN | MR75202_VM_SDIF_PROG;
	writel(data, priv->base + MR75202_VM_SDIF);

	return 0;
}

static int mr75202_read_vm(struct mr75202_priv *priv, u32 address, u32 channel)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(priv->base + MR75202_VM_SDIF_STATUS, val,
				 !(val & MR75202_VM_SDIF_STATUS_BUSY),
				 MR75202_DELAY_US, MR75202_TIMEOUT_US);
	if (ret)
		return ret;

	val = FIELD_PREP(MR75202_VM_SDIF_ADDR, address) | MR75202_VM_SDIF_PROG;
	writel(val, priv->base + MR75202_VM_SDIF);
	ret = readl_poll_timeout(priv->base + MR75202_VM_SDIF_STATUS, val,
				 !(val & MR75202_VM_SDIF_STATUS_BUSY),
				 MR75202_DELAY_US, MR75202_TIMEOUT_US);
	if (ret)
		return ret;

	return readl(priv->base + MR75202_VM_SDIF_RDATA(channel));
}

static int mr75202_read_temp(struct device *dev, u32 attr, int channel,
			     long *val)
{
	struct mr75202_priv *priv = dev_get_drvdata(dev);
	u32 nbs;
	int ret;

	mutex_lock(&priv->mutex);

	switch (attr) {
	case hwmon_temp_input:
		/* Deassert sample done bit in TS_SMPL_STATUS by reading old
		 * data. */
		readl(priv->base + MR75202_TS_DATA(channel));
		mr75202_write_ts(priv, MR75202_SDA_IP_CTRL,
				 MR75202_SDA_IP_CTRL_RUN_ONCE |
				 MR75202_SDA_IP_CTRL_AUTO);

		/* TODO: This takes ~8 ms, so it may be worth using IRQs */
		ret = readl_poll_timeout(priv->base + MR75202_TS_SMPL_STATUS,
					 nbs, nbs & BIT(channel),
					 MR75202_DELAY_US, MR75202_TIMEOUT_US);
		if (ret)
			break;

		nbs = readl(priv->base + MR75202_TS_DATA(channel));
		nbs &= MR75202_SDA_IP_DATA_DAT;

		/* From TS4 datasheet and MR74127 (electrical specification for
		 * TSMC 28HPC+). There is no overflow up to 400 degree Celsius.
		 * hwmon requires temperature to be in millidegrees. */
		*val = (long)nbs * 237500 / 4094 - 81100;

		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	mutex_unlock(&priv->mutex);

	return ret;
}

static int mr75202_read_in(struct device *dev, u32 attr, int channel, long *val)
{
	struct mr75202_priv *priv = dev_get_drvdata(dev);
	u32 regval;
	int ret;

	mutex_lock(&priv->mutex);

	switch (attr) {
	case hwmon_in_input:
		/* By unknown reason single run and auto mode are not working.
		 * Here we start continuous run without auto mode and stop it
		 * after first data received. */
		mr75202_write_vm(priv, MR75202_SDA_IP_CTRL,
				 MR75202_SDA_IP_CTRL_RESETN |
				 MR75202_SDA_IP_CTRL_RUN_CONT);

		ret = read_poll_timeout(mr75202_read_vm, regval,
					regval & MR75202_SDA_IP_DATA_DONE,
					MR75202_DELAY_US, MR75202_TIMEOUT_US, 0,
					priv, MR75202_SDA_IP_DATA, channel);

		mr75202_write_vm(priv, MR75202_SDA_IP_CTRL,
				 MR75202_SDA_IP_CTRL_RESETN |
				 MR75202_SDA_IP_CTRL_STOP);

		if (ret)
			break;

		if (regval & (MR75202_SDA_IP_DATA_TYPE |
			      MR75202_SDA_IP_DATA_FAULT)) {
			ret = -EAGAIN;
			break;
		}

		regval &= MR75202_SDA_IP_DATA_DAT;

		/* From VM datasheet and MR74140 (electrical specification for
		 * TSMC 28HPC+). */
		*val = (regval + 1) * 1224 / 256;

		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	mutex_unlock(&priv->mutex);

	return ret;
}

static int mr75202_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_temp:
		return mr75202_read_temp(dev, attr, channel, val);
	case hwmon_in:
		return mr75202_read_in(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int mr75202_read_string(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, const char **str)
{
	struct mr75202_priv *priv = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_temp_label:
		*str = priv->temp_labels[channel];
		return 0;
	case hwmon_in_label:
		*str = priv->in_labels[channel];
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int mr75202_init_temp(struct device *dev, struct mr75202_priv *priv)
{
	/* TODO: skip if no temperature sensors */
	/* TODO: split-out TSx-specific code into separate function */

	/*
	 * Specified by TS4 datasheet. Use the following configuration:
	 *
	 * - duty cycle = 50%;
	 * - minimum possible frequency.
	 *
	 * Don't check for maximum frequency, because it must always fit.
	 */
	const u32 minfreq = 1000000;

	unsigned long rate = clk_get_rate(priv->clk);
	unsigned int div = rate / (2 * minfreq);

	if (div < 1 || div > 256) {
		dev_err(dev,
			"Failed to setup clock rate for temperature sensor\n");
		return -EINVAL;
	}

	/* MR75202 doesn't have a register with device configuration, but we
	 * can probe it with a trick. */
	do {
		writel(BIT(priv->nts), priv->base + MR75202_TS_SDIF_DISABLE);
	} while (readl(priv->base + MR75202_TS_SDIF_DISABLE) == BIT(priv->nts++));
	priv->nts -= 1;

	/* MR75202_TS_SDIF_DISABLE is already zeroed, no need to do it again */
	writel(0, priv->base + MR75202_TS_SDIF_CTRL);

	dev_info(dev, "Configure TS SDA clock: %lu Hz, div = %u\n",
		 rate / (2 * div), div);
	writel(FIELD_PREP(MR75202_TS_CLK_SYNTH_LO, div - 1) |
	       FIELD_PREP(MR75202_TS_CLK_SYNTH_HI, div - 1) |
	       FIELD_PREP(MR75202_TS_CLK_SYNTH_STROBE, 1) |
	       MR75202_TS_CLK_SYNTH_ENA, priv->base + MR75202_TS_CLK_SYNTH);

	/* No need to deassert sensor PD and RESET, this will be done
	 * automatically by Serial Data Adapter/PVT slave (MR75005) as we use
	 * data readout in auto-mode. */

	/* TS4 sensor power-up time should be >= 50 us */
	return mr75202_write_ts(priv, MR75202_SDA_IP_TMR,
				DIV_ROUND_UP(50 * rate, 1000000));

	/* TODO: Reset samples counter */
	/* TODO: Disable interrupts */
}

static int mr75202_init_in(struct device *dev, struct mr75202_priv *priv)
{
	/* TODO: skip if no voltage sensors */
	/* TODO: split-out VMx-specific code into separate function */

	const u32 freq = 1000000;

	unsigned long rate = clk_get_rate(priv->clk);
	unsigned int div = rate / (2 * freq);

	if (div < 1 || div > 256) {
		dev_err(dev,
			"Failed to setup clock rate for voltage sensor\n");
		return -EINVAL;
	}

	/* MR75202 doesn't have a register with device configuration, but we
	 * can probe it with a trick. */
	do {
		writel(BIT(priv->nvm), priv->base + MR75202_VM_SDIF_DISABLE);
	} while (readl(priv->base + MR75202_VM_SDIF_DISABLE) == BIT(priv->nvm++));
	priv->nvm -= 1;

	writel(0, priv->base + MR75202_VM_SDIF_CTRL);

	dev_info(dev, "Configure VM SDA clock: %lu Hz, div = %u\n",
		 rate / (2 * div), div);
	writel(FIELD_PREP(MR75202_VM_CLK_SYNTH_LO, div - 1) |
	       FIELD_PREP(MR75202_VM_CLK_SYNTH_HI, div - 1) |
	       FIELD_PREP(MR75202_VM_CLK_SYNTH_STROBE, 1) |
	       MR75202_VM_CLK_SYNTH_ENA, priv->base + MR75202_VM_CLK_SYNTH);

	/* No need to deassert sensor PD and RESET, this will be done
	 * automatically by Serial Data Adapter/PVT slave (MR75005) as we use
	 * data readout in auto-mode. */

	/* VM sensor power-up time should be <= 10 us*/
	return mr75202_write_vm(priv, MR75202_SDA_IP_TMR,
				DIV_ROUND_UP(5 * rate, 1000000));

	/* TODO: Reset samples counter */
	/* TODO: Disable interrupts */
}


static int mr75202_init(struct device *dev, struct mr75202_priv *priv)
{
	mr75202_init_temp(dev, priv);
	mr75202_init_in(dev, priv);

	return 0;
}

static const struct hwmon_ops mr75202_hwmon_ops = {
	.is_visible = mr75202_is_visible,
	.read = mr75202_read,
	.read_string = mr75202_read_string
};

static const u32 mr75202_chip_config[] = {
	HWMON_C_REGISTER_TZ,
	0
};

static const struct hwmon_channel_info mr75202_channel_chip_info = {
	.type = hwmon_chip,
	.config = mr75202_chip_config
};

static struct hwmon_channel_info mr75202_channel_temp_info = {
	.type = hwmon_temp
	/* .config is initialized in mr75202_init_chip_info() */
};

static struct hwmon_channel_info mr75202_channel_in_info = {
	.type = hwmon_in
};

static const struct hwmon_channel_info *mr75202_channel_info[] = {
	&mr75202_channel_chip_info,
	&mr75202_channel_temp_info,
	&mr75202_channel_in_info,
	NULL
};

static const struct hwmon_chip_info mr75202_chip_info = {
	.ops = &mr75202_hwmon_ops,
	.info = mr75202_channel_info
};

/* TODO: Make mr75202_init_chip_info() allocate all structures dynamically.
 *   With static declaration driver supports only one instance of a device. */
static const struct hwmon_chip_info *mr75202_init_chip_info(struct device *dev,
						const struct mr75202_priv *priv)
{
	size_t i;
	u32 *config_temp = devm_kcalloc(dev, priv->nts + 1, sizeof(u32), GFP_KERNEL);
	u32 *config_in = devm_kcalloc(dev, priv->nvm + 1, sizeof(u32), GFP_KERNEL);

	if (!config_temp || !config_in)
		return NULL;

	memset32(config_temp, HWMON_T_INPUT, priv->nts);
	for (i = 0; i < min(priv->nts, ARRAY_SIZE(priv->temp_labels)); i++)
		if (priv->temp_labels[i])
			config_temp[i] |= HWMON_T_LABEL;

	mr75202_channel_temp_info.config = config_temp;

	memset32(config_in, HWMON_I_INPUT, priv->nvm);
	for (i = 0; i < min(priv->nvm, ARRAY_SIZE(priv->in_labels)); i++)
		if (priv->in_labels[i])
			config_in[i] |= HWMON_I_LABEL;

	mr75202_channel_in_info.config = config_in;

	return &mr75202_chip_info;
}

static int mr75202_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *hdev;
	struct mr75202_priv *priv;
	const struct hwmon_chip_info *info;
	const char *name = "mr75202";
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base)) {
		dev_err(&pdev->dev, "Failed to ioremap regs\n");
		return PTR_ERR(priv->base);
	}

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		return PTR_ERR(priv->clk);
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clock\n");
		return ret;
	}

	priv->rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(priv->rst)) {
		dev_err(&pdev->dev, "Failed to get reset control\n");
		ret = PTR_ERR(priv->rst);
		goto err_clk_disable;
	}

	ret = reset_control_deassert(priv->rst);
	if (ret) {
		dev_err(&pdev->dev, "Failed to deassert reset\n");
		goto err_clk_disable;
	}

	/* Optional properies with chip name and channel labels */
	of_property_read_string(pdev->dev.of_node, "moortec,name", &name);
	of_property_read_string_array(pdev->dev.of_node, "moortec,temp-labels",
				      priv->temp_labels,
				      ARRAY_SIZE(priv->temp_labels));
	of_property_read_string_array(pdev->dev.of_node, "moortec,volt-labels",
				      priv->in_labels,
				      ARRAY_SIZE(priv->in_labels));

	mutex_init(&priv->mutex);
	ret = mr75202_init(&pdev->dev, priv);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to initialize Moortec PVT controller\n");
		goto err_reset_assert;
	}

	info = mr75202_init_chip_info(&pdev->dev, priv);
	if (!info) {
		ret = -ENOMEM;
		goto err_reset_assert;
	}

	hdev = devm_hwmon_device_register_with_info(&pdev->dev, name,
						    priv, info, NULL);
	if (IS_ERR(hdev)) {
		dev_err(&pdev->dev, "Failed to registed hwmon device\n");
		ret = PTR_ERR(hdev);
		goto err_reset_assert;
	}

	dev_set_drvdata(&pdev->dev, hdev);
	dev_info(&pdev->dev, "%zu temperature sensors\n", priv->nts);
	dev_info(&pdev->dev, "%zu voltage sensors\n", priv->nvm);

	return 0;

err_reset_assert:
	mutex_destroy(&priv->mutex);
	reset_control_assert(priv->rst);
err_clk_disable:
	clk_disable_unprepare(priv->clk);

	return ret;
}

static int mr75202_remove(struct platform_device *pdev)
{
	struct device *hdev = dev_get_drvdata(&pdev->dev);
	struct mr75202_priv *priv = dev_get_drvdata(hdev);

	mutex_destroy(&priv->mutex);
	reset_control_assert(priv->rst);
	clk_disable_unprepare(priv->clk);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mr75202_of_match[] = {
	{ .compatible = "moortec,mr75202" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mr75202_of_match);
#endif

static struct platform_driver mr75202_driver = {
	.driver = {
		.name = "mr75202",
		.of_match_table = of_match_ptr(mr75202_of_match),
	},
	.probe = mr75202_probe,
	.remove = mr75202_remove
};
module_platform_driver(mr75202_driver);
MODULE_LICENSE("GPL v2");
