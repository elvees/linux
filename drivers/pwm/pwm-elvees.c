// SPDX-License-Identifier: GPL-2.0+
/*
 * ELVEES PWM controller driver
 *
 * Copyright 2017-2023 RnD Center "ELVEES", JSC
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define NUM_PWM_CHANNEL 4

#define PWM_DEVICE_OFFSET 0x100

/* PWM registers */
#define PWM_CLKCTL 0x08
#define PWM_CTRPRD 0x10
#define PWM_CTRCNT 0x14
#define PWM_CMPA 0x24
#define PWM_EMCTLA 0x2C
#define PWM_EMCTLB 0x30
#define PWM_EMSWFR 0x34

/*
 * CTRRUN is a single shared register at offset 0x80 from the
 * PWM controller base. Each port occupies a 2-bit field:
 *   port 0: bits [1:0]
 *   port 1: bits [9:8]
 *   port 2: bits [17:16]
 *   port 3: bits [25:24]
 */
#define PWM_CTRRUN 0x80

#define PWM_CLKCTL_CNTMODE(v) ((v) << 0)
#define PWM_CLKCTL_LOADPRD(v) ((v) << 3)
#define PWM_CLKCTL_SYNCOSEL(v) ((v) << 4)

#define PWM_EMCTLA_EZRO(v) ((v) << 0)
#define PWM_EMCTLA_EPRD(v) ((v) << 2)
#define PWM_EMCTLA_ECMPAI(v) ((v) << 4)
#define PWM_EMCTLA_ECMPAD(v) ((v) << 6)
#define PWM_EMCTLA_ECMPBI(v) ((v) << 8)
#define PWM_EMCTLA_ECMPBD(v) ((v) << 10)

#define PWM_EMSWFR_ACTSFA(v) ((v) << 0)
#define PWM_EMSWFR_ONESFA(v) ((v) << 2)
#define PWM_EMSWFR_ACTSFB(v) ((v) << 3)
#define PWM_EMSWFR_ONESFB(v) ((v) << 5)

/* CTRRUN is shared: port field is at bits [port*8+1 : port*8+0] */
#define PWM_CTRRUN_RUN(port, v) ((v) << ((port) * 8))

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
	spinlock_t ctrrun_lock;
};

/* Per-port register access: base + port * 0x100 + reg_offset */
static inline void elvees_pwm_writel(struct elvees_pwm_chip *chip, u32 reg,
				     u32 val, u32 port)
{
	writel(val, chip->mmio_base + reg + port * PWM_DEVICE_OFFSET);
}

static inline u32 elvees_pwm_readl(struct elvees_pwm_chip *chip, u32 reg, u32 port)
{
	return readl(chip->mmio_base + reg + port * PWM_DEVICE_OFFSET);
}

/*
 * CTRRUN is a single shared register (always at base + 0x80, i.e. port 0
 * in the per-port addressing scheme). The port is selected by the bit
 * field position (port * 8 shift).
 */
static void elvees_pwm_set_run(struct elvees_pwm_chip *pwm_chip, u32 port, u32 run)
{
	u32 val;

	guard(spinlock_irqsave)(&pwm_chip->ctrrun_lock);
	val = readl(pwm_chip->mmio_base + PWM_CTRRUN);
	val &= ~PWM_CTRRUN_RUN(port, PWM_RUN_MASK);
	if (run == PWM_RUN_START)
		val |= PWM_CTRRUN_RUN(port, PWM_RUN_START);

	writel(val, pwm_chip->mmio_base + PWM_CTRRUN);
}

static void
elvees_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm, enum pwm_polarity polarity)
{
	struct elvees_pwm_chip *pwm_chip = pwmchip_get_drvdata(chip);
	u32 val;

	if (polarity == PWM_POLARITY_NORMAL)
		val = PWM_EMCTLA_ECMPAD(PWM_EMCTL_ACTION_SET) |
		      PWM_EMCTLA_EPRD(PWM_EMCTL_ACTION_CLEAR);
	else
		val = PWM_EMCTLA_ECMPAD(PWM_EMCTL_ACTION_CLEAR) |
		      PWM_EMCTLA_EPRD(PWM_EMCTL_ACTION_SET);

	elvees_pwm_writel(pwm_chip,
			  pwm_chip->out_channel[pwm->hwpwm] ? PWM_EMCTLB : PWM_EMCTLA,
			  val, pwm->hwpwm);
}

static int elvees_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct elvees_pwm_chip *pwm_chip = pwmchip_get_drvdata(chip);

	elvees_pwm_set_polarity(chip, pwm, pwm->state.polarity);

	elvees_pwm_writel(pwm_chip, PWM_CLKCTL,
			PWM_CLKCTL_CNTMODE(PWM_CNTMODE_DOWN) |
			PWM_CLKCTL_LOADPRD(PWM_LOADPRD_ENABLE) |
			PWM_CLKCTL_SYNCOSEL(PWM_SYNCOSEL_DISABLE),
			pwm->hwpwm);

	elvees_pwm_set_run(pwm_chip, pwm->hwpwm, PWM_RUN_STOP_CNT);

	return 0;
}

static int elvees_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			     u64 duty_ns, u64 period_ns)
{
	struct elvees_pwm_chip *pwm_chip = pwmchip_get_drvdata(chip);
	u64 clk_rate = clk_get_rate(pwm_chip->clk);
	u64 period, duty;

	if (!clk_rate)
		return -EINVAL;

	period = mul_u64_u64_div_u64(clk_rate, period_ns, NSEC_PER_SEC);
	duty = mul_u64_u64_div_u64(clk_rate, duty_ns, NSEC_PER_SEC);

	if (!period)
		return -EINVAL;

	if (period > U32_MAX)
		period = U32_MAX;
	if (duty > period)
		duty = period;

	/*
	 * Both CTRPRD (CLKCTL[LOADPRD] = 0) and CMPA (CMPCTL reset value)
	 * are written to their shadow registers and take effect at the next
	 * period boundary, so a running counter can be updated in place.
	 */
	elvees_pwm_writel(pwm_chip, PWM_CTRPRD, (u32)period, pwm->hwpwm);
	elvees_pwm_writel(pwm_chip, PWM_CMPA, (u32)duty, pwm->hwpwm);

	return 0;
}

static void elvees_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct elvees_pwm_chip *pwm_chip = pwmchip_get_drvdata(chip);

	/*
	 * Start from CTRCNT = 0, so the first count reloads the active
	 * CTRPRD/CMPA from the shadow registers instead of finishing a
	 * period left over from the last disable.
	 */
	elvees_pwm_writel(pwm_chip, PWM_CTRCNT, 0, pwm->hwpwm);
	elvees_pwm_set_run(pwm_chip, pwm->hwpwm, PWM_RUN_START);
}

static void elvees_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct elvees_pwm_chip *pwm_chip = pwmchip_get_drvdata(chip);
	u32 val, disable_state, outport;

	disable_state = pwm_chip->state_on_disable[pwm->hwpwm];
	outport = pwm_chip->out_channel[pwm->hwpwm];

	elvees_pwm_set_run(pwm_chip, pwm->hwpwm, PWM_RUN_STOP_CNT);

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
	bool enabled = pwm->state.enabled;
	int err;

	if (state->polarity != pwm->state.polarity) {
		if (enabled) {
			elvees_pwm_disable(chip, pwm);
			enabled = false;
		}

		elvees_pwm_set_polarity(chip, pwm, state->polarity);
	}

	if (!state->enabled) {
		if (enabled)
			elvees_pwm_disable(chip, pwm);

		return 0;
	}

	err = elvees_pwm_config(chip, pwm, state->duty_cycle, state->period);
	if (err)
		return err;

	if (!enabled)
		elvees_pwm_enable(chip, pwm);

	return 0;
}

static const struct pwm_ops elvees_pwm_ops = {
	.request = elvees_pwm_request,
	.apply = elvees_pwm_apply,
};

static int elvees_pwm_parse_dt(struct pwm_chip *chip)
{
	/*
	 * Since v6.9 chip->dev is the pwmchipN class device, which has no
	 * of_node. DT properties live on the parent platform device.
	 */
	struct device *dev = pwmchip_parent(chip);
	struct device_node *node = dev_of_node(dev);
	struct elvees_pwm_chip *elvees_pwm_chip = pwmchip_get_drvdata(chip);
	int ret;

	/*
	 * Both properties are optional. of_property_read_u32_array() returns
	 * -EINVAL if a property is absent and leaves the array untouched, so
	 * the zero-initialised defaults are kept: OUTA for output-channel and
	 * ACTION_NONE (no forced state on disable) for state-on-disable.
	 */
	ret = of_property_read_u32_array(node, "elvees,output-channel",
				   elvees_pwm_chip->out_channel, NUM_PWM_CHANNEL);
	if (ret < 0 && ret != -EINVAL)
		dev_warn(dev, "invalid 'elvees,output-channel' property (%d)\n",
			 ret);

	ret = of_property_read_u32_array(node, "elvees,state-on-disable",
				   elvees_pwm_chip->state_on_disable, NUM_PWM_CHANNEL);
	if (ret < 0 && ret != -EINVAL)
		dev_warn(dev, "invalid 'elvees,state-on-disable' property (%d)\n",
			 ret);

	return 0;
}

static int elvees_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct elvees_pwm_chip *elvees_pwm_chip;
	struct pwm_chip *chip;
	int ret;

	chip = devm_pwmchip_alloc(dev, NUM_PWM_CHANNEL, sizeof(*elvees_pwm_chip));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	elvees_pwm_chip = pwmchip_get_drvdata(chip);
	spin_lock_init(&elvees_pwm_chip->ctrrun_lock);

	elvees_pwm_chip->mmio_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(elvees_pwm_chip->mmio_base))
		return PTR_ERR(elvees_pwm_chip->mmio_base);

	elvees_pwm_chip->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(elvees_pwm_chip->clk))
		return dev_err_probe(dev, PTR_ERR(elvees_pwm_chip->clk),
				     "failed to init clock\n");

	chip->ops = &elvees_pwm_ops;

	ret = elvees_pwm_parse_dt(chip);
	if (ret)
		return ret;

	ret = devm_pwmchip_add(dev, chip);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to add pwmchip\n");

	return 0;
}

static const struct of_device_id elvees_pwm_of_match[] = {
	{ .compatible = "elvees,elvees-pwm" },
	{}
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
