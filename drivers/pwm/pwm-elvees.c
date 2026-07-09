// SPDX-License-Identifier: GPL-2.0+
/*
 * ELVEES PWM controller driver
 *
 * Copyright 2017-2023 RnD Center "ELVEES", JSC
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/of.h>

#define NUM_PWM_CHANNEL			4

#define PWM_DEVICE_OFFSET		0x100

/* PWM registers */
#define PWM_CLKCTL			0x08
#define PWM_CTRPRD			0x10
#define PWM_CTRCNT			0x14
#define PWM_CMPA			0x24
#define PWM_EMCTLA			0x2C
#define PWM_EMCTLB			0x30
#define PWM_EMSWFR			0x34
#define PWM_CTRRUN			0x80

#define PWM_CLKCTL_CNTMODE(v)		((v) << 0)
#define PWM_CLKCTL_LOADPRD(v)		((v) << 3)
#define PWM_CLKCTL_SYNCOSEL(v)		((v) << 4)

#define PWM_EMCTLA_EZRO(v)		((v) << 0)
#define PWM_EMCTLA_EPRD(v)		((v) << 2)
#define PWM_EMCTLA_ECMPAI(v)		((v) << 4)
#define PWM_EMCTLA_ECMPAD(v)		((v) << 6)
#define PWM_EMCTLA_ECMPBI(v)		((v) << 8)
#define PWM_EMCTLA_ECMPBD(v)		((v) << 10)

#define PWM_EMSWFR_ACTSFA(v)		((v) << 0)
#define PWM_EMSWFR_ONESFA(v)		((v) << 2)
#define PWM_EMSWFR_ACTSFB(v)		((v) << 3)
#define PWM_EMSWFR_ONESFB(v)		((v) << 5)

#define PWM_CTRRUN_RUN(port, v)		((v) << (port) * 8)

enum {
	PWM_CNTMODE_UP,
	PWM_CNTMODE_DOWN,
	PWM_CNTMODE_UP_DOWN,
	PWM_CNTMODE_NONE
};

enum {
	PWM_LOADPRD_ENABLE,
	PWM_LOADPRD_DISABLE
};

enum {
	PWM_SYNCOSEL_SYNCI,
	PWM_SYNCOSEL_0,
	PWM_SYNCOSEL_CMPB,
	PWM_SYNCOSEL_DISABLE
};

enum {
	PWM_EMCTL_ACTION_NONE,
	PWM_EMCTL_ACTION_CLEAR,
	PWM_EMCTL_ACTION_SET,
	PWM_EMCTL_ACTION_TOGGLE
};

enum {
	PWM_RUN_STOP_CNT,
	PWM_RUN_STOP_EVENT,
	PWM_RUN_START,
	PWM_RUN_MASK
};

struct elvees_pwm_chip {
	struct clk *clk;
	void __iomem *mmio_base;
	u32 out_channel[NUM_PWM_CHANNEL]; /* 0 == OUTA, >0 == OUTB */
	u32 state_on_disable[NUM_PWM_CHANNEL];
};

static inline void elvees_pwm_writel(struct elvees_pwm_chip *chip,
				   u32 reg, u32 val, u32 port)
{
	writel(val, chip->mmio_base + reg + port * PWM_DEVICE_OFFSET);
}

static inline u32 elvees_pwm_readl(struct elvees_pwm_chip *chip, u32 reg, u32 port)
{
	return readl(chip->mmio_base + reg + port * PWM_DEVICE_OFFSET);
}

static int elvees_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct elvees_pwm_chip *pwm_chip = pwmchip_get_drvdata(chip);
	u32 val;

	elvees_pwm_writel(pwm_chip, pwm_chip->out_channel[pwm->hwpwm] ?
			PWM_EMCTLB : PWM_EMCTLA,
			PWM_EMCTLA_ECMPAD(PWM_EMCTL_ACTION_SET) |
			PWM_EMCTLA_EPRD(PWM_EMCTL_ACTION_CLEAR),
			pwm->hwpwm);

	elvees_pwm_writel(pwm_chip, PWM_CLKCTL,
			PWM_CLKCTL_CNTMODE(PWM_CNTMODE_DOWN) |
			PWM_CLKCTL_LOADPRD(PWM_LOADPRD_DISABLE) |
			PWM_CLKCTL_SYNCOSEL(PWM_SYNCOSEL_DISABLE),
			pwm->hwpwm);

	val = elvees_pwm_readl(pwm_chip, PWM_CTRRUN, pwm->hwpwm);

	val &= ~PWM_CTRRUN_RUN(pwm->hwpwm, PWM_RUN_MASK);

	elvees_pwm_writel(pwm_chip, PWM_CTRRUN, val, pwm->hwpwm);

	return 0;
}

static int elvees_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			    int duty_ns, int period_ns)
{
	struct elvees_pwm_chip *pwm_chip = pwmchip_get_drvdata(chip);
	u64 div, clk_rate = clk_get_rate(pwm_chip->clk);
	u32 period, duty;

	div = clk_rate * period_ns;
	period = DIV_ROUND_CLOSEST_ULL(div, NSEC_PER_SEC);

	div = clk_rate * duty_ns;
	duty = DIV_ROUND_CLOSEST_ULL(div, NSEC_PER_SEC);

	elvees_pwm_writel(pwm_chip, PWM_CTRPRD, period, pwm->hwpwm);
	elvees_pwm_writel(pwm_chip, PWM_CMPA, duty, pwm->hwpwm);

	return 0;
}

static int elvees_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
				  enum pwm_polarity polarity)
{
	struct elvees_pwm_chip *pwm_chip = pwmchip_get_drvdata(chip);
	u32 val;

	if (polarity == PWM_POLARITY_NORMAL) {
		val = PWM_EMCTLA_ECMPAD(PWM_EMCTL_ACTION_SET) |
		      PWM_EMCTLA_EPRD(PWM_EMCTL_ACTION_CLEAR);
	} else {
		val = PWM_EMCTLA_ECMPAD(PWM_EMCTL_ACTION_CLEAR) |
		      PWM_EMCTLA_EPRD(PWM_EMCTL_ACTION_SET);
	}

	elvees_pwm_writel(pwm_chip, pwm_chip->out_channel[pwm->hwpwm] ?
			PWM_EMCTLB : PWM_EMCTLA, val, pwm->hwpwm);

	return 0;
}

static int elvees_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct elvees_pwm_chip *pwm_chip = pwmchip_get_drvdata(chip);
	u32 val;

	val = elvees_pwm_readl(pwm_chip, PWM_CTRRUN, pwm->hwpwm);

	val |= PWM_CTRRUN_RUN(pwm->hwpwm, PWM_RUN_START);

	elvees_pwm_writel(pwm_chip, PWM_CTRRUN, val, pwm->hwpwm);

	return 0;
}

static void elvees_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct elvees_pwm_chip *pwm_chip = pwmchip_get_drvdata(chip);
	u32 val, disable_state, outport;

	disable_state = pwm_chip->state_on_disable[pwm->hwpwm];
	outport = pwm_chip->out_channel[pwm->hwpwm];

	val = elvees_pwm_readl(pwm_chip, PWM_CTRRUN, pwm->hwpwm);

	val &= ~PWM_CTRRUN_RUN(pwm->hwpwm, PWM_RUN_MASK);

	elvees_pwm_writel(pwm_chip, PWM_CTRRUN, val, pwm->hwpwm);

	if (disable_state != PWM_EMCTL_ACTION_NONE) {
		val = outport ?
			PWM_EMSWFR_ACTSFB(disable_state) | PWM_EMSWFR_ONESFB(1) :
			PWM_EMSWFR_ACTSFA(disable_state) | PWM_EMSWFR_ONESFA(1);
		elvees_pwm_writel(pwm_chip, PWM_EMSWFR, val, pwm->hwpwm);
		elvees_pwm_writel(pwm_chip, PWM_CTRCNT, 0, pwm->hwpwm);
	}
}

static int elvees_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	int err;
	bool enabled = pwm->state.enabled;

	if (state->polarity != pwm->state.polarity) {
		if (enabled) {
			elvees_pwm_disable(chip, pwm);
			enabled = false;
		}

		err = elvees_pwm_set_polarity(chip, pwm, state->polarity);
		if (err)
			return err;
	}

	if (!state->enabled) {
		if (enabled)
			elvees_pwm_disable(chip, pwm);

		return 0;
	}

	err = elvees_pwm_config(pwm->chip, pwm, state->duty_cycle, state->period);
	if (err)
		return err;

	if (!enabled)
		err = elvees_pwm_enable(chip, pwm);

	return err;
}

static const struct pwm_ops elvees_pwm_ops = {
	.request	= elvees_pwm_request,
	.apply		= elvees_pwm_apply,
};

static int elvees_pwm_parse_dt(struct pwm_chip *pwm_chip)
{
	struct device_node *node = dev_of_node(&pwm_chip->dev);
	int ret;
	struct elvees_pwm_chip *elvees_pwm_chip = pwmchip_get_drvdata(pwm_chip);

	ret = of_property_read_u32_array(node, "elvees,output-channel",
				   elvees_pwm_chip->out_channel, NUM_PWM_CHANNEL);
	if (ret < 0)
		dev_warn(&pwm_chip->dev, "elvees,output-channel DT property missing\n");

	ret = of_property_read_u32_array(node, "elvees,state-on-disable",
				   elvees_pwm_chip->state_on_disable, NUM_PWM_CHANNEL);
	if (ret < 0)
		dev_warn(&pwm_chip->dev, "elvees,state-on-disable DT property missing\n");

	return 0;
}

static int elvees_pwm_probe(struct platform_device *pdev)
{
	struct elvees_pwm_chip *elvees_pwm_chip;
	struct pwm_chip *chip;
	int ret;

	chip = devm_pwmchip_alloc(&pdev->dev, NUM_PWM_CHANNEL, sizeof(*elvees_pwm_chip));

	if (IS_ERR(chip))
		return PTR_ERR(chip);
	elvees_pwm_chip = pwmchip_get_drvdata(chip);


	elvees_pwm_chip->mmio_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(elvees_pwm_chip->mmio_base))
		return PTR_ERR(elvees_pwm_chip->mmio_base);


	elvees_pwm_chip->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(elvees_pwm_chip->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(elvees_pwm_chip->clk),
				     "failed to init clock\n");

	chip->ops = &elvees_pwm_ops;

	ret = elvees_pwm_parse_dt(chip);

	platform_set_drvdata(pdev, elvees_pwm_chip);

	ret = devm_pwmchip_add(&pdev->dev, chip);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "failed to add pwmchip\n");

	return 0;
}

static const struct of_device_id elvees_pwm_of_match[] = {
	{ .compatible	= "elvees,elvees-pwm" },
	{},
};
MODULE_DEVICE_TABLE(of, elvees_pwm_of_match);

static struct platform_driver elvees_pwm_driver = {
	.driver	= {
		.name	= "elvees-pwm",
		.of_match_table	= elvees_pwm_of_match,
	},
	.probe	= elvees_pwm_probe,
};
module_platform_driver(elvees_pwm_driver);

MODULE_DESCRIPTION("ELVEES PWM controller driver");
MODULE_LICENSE("GPL");
