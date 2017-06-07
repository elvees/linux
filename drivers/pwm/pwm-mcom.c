// SPDX-License-Identifier: GPL-2.0+
/*
 * ELVEES MCom-03 PWM controller driver
 *
 * Copyright 2017 RnD Center "ELVEES", JSC
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define NUM_PWM_CHANNEL			4

#define PWM_DEVICE_OFFSET		0x100

/* PWM registers */
#define PWM_CLKCTL			0x08
#define PWM_CTRPRD			0x10
#define PWM_CMPA			0x24
#define PWM_EMCTLA			0x2C
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

struct mcom_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk;
	void __iomem *mmio_base;
};

static inline struct mcom_pwm_chip *to_mcom_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct mcom_pwm_chip, chip);
}

static inline void mcom_pwm_writel(struct mcom_pwm_chip *chip,
				   u32 reg, u32 val, u32 port)
{
	writel(val, chip->mmio_base + reg + port * PWM_DEVICE_OFFSET);
}

static inline u32 mcom_pwm_readl(struct mcom_pwm_chip *chip, u32 reg, u32 port)
{
	return readl(chip->mmio_base + reg + port * PWM_DEVICE_OFFSET);
}

static int mcom_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct mcom_pwm_chip *pwm_chip = to_mcom_pwm_chip(chip);
	u32 val;

	mcom_pwm_writel(pwm_chip, PWM_EMCTLA,
			PWM_EMCTLA_ECMPAI(PWM_EMCTL_ACTION_SET) |
			PWM_EMCTLA_EPRD(PWM_EMCTL_ACTION_CLEAR),
			pwm->hwpwm);

	mcom_pwm_writel(pwm_chip, PWM_CLKCTL,
			PWM_CLKCTL_CNTMODE(PWM_CNTMODE_UP) |
			PWM_CLKCTL_LOADPRD(PWM_LOADPRD_DISABLE) |
			PWM_CLKCTL_SYNCOSEL(PWM_SYNCOSEL_DISABLE),
			pwm->hwpwm);

	val = mcom_pwm_readl(pwm_chip, PWM_CTRRUN, pwm->hwpwm);

	val &= ~PWM_CTRRUN_RUN(pwm->hwpwm, PWM_RUN_MASK);

	mcom_pwm_writel(pwm_chip, PWM_CTRRUN, val, pwm->hwpwm);

	return 0;
}

static int mcom_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			    int duty_ns, int period_ns)
{
	struct mcom_pwm_chip *pwm_chip = to_mcom_pwm_chip(chip);
	u64 div, clk_rate = clk_get_rate(pwm_chip->clk);
	u32 period, duty;

	div = clk_rate * period_ns;
	period = DIV_ROUND_CLOSEST_ULL(div, NSEC_PER_SEC);

	div = clk_rate * duty_ns;
	duty = DIV_ROUND_CLOSEST_ULL(div, NSEC_PER_SEC);

	mcom_pwm_writel(pwm_chip, PWM_CTRPRD, period, pwm->hwpwm);
	mcom_pwm_writel(pwm_chip, PWM_CMPA, duty, pwm->hwpwm);

	return 0;
}

static int mcom_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
				  enum pwm_polarity polarity)
{
	struct mcom_pwm_chip *pwm_chip = to_mcom_pwm_chip(chip);
	u32 val;

	if (polarity == PWM_POLARITY_NORMAL) {
		val = PWM_EMCTLA_ECMPAI(PWM_EMCTL_ACTION_SET) |
		      PWM_EMCTLA_EPRD(PWM_EMCTL_ACTION_CLEAR);
	} else {
		val = PWM_EMCTLA_ECMPAI(PWM_EMCTL_ACTION_CLEAR) |
		      PWM_EMCTLA_EPRD(PWM_EMCTL_ACTION_SET);
	}

	mcom_pwm_writel(pwm_chip, PWM_EMCTLA, val, pwm->hwpwm);

	return 0;
}

static int mcom_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct mcom_pwm_chip *pwm_chip = to_mcom_pwm_chip(chip);
	u32 val;

	val = mcom_pwm_readl(pwm_chip, PWM_CTRRUN, pwm->hwpwm);

	val |= PWM_CTRRUN_RUN(pwm->hwpwm, PWM_RUN_START);

	mcom_pwm_writel(pwm_chip, PWM_CTRRUN, val, pwm->hwpwm);

	return 0;
}

static void mcom_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct mcom_pwm_chip *pwm_chip = to_mcom_pwm_chip(chip);
	u32 val;

	val = mcom_pwm_readl(pwm_chip, PWM_CTRRUN, pwm->hwpwm);

	val &= ~PWM_CTRRUN_RUN(pwm->hwpwm, PWM_RUN_MASK);

	mcom_pwm_writel(pwm_chip, PWM_CTRRUN, val, pwm->hwpwm);
}

static const struct pwm_ops mcom_pwm_ops = {
	.request	= mcom_pwm_request,
	.config		= mcom_pwm_config,
	.set_polarity	= mcom_pwm_set_polarity,
	.enable		= mcom_pwm_enable,
	.disable	= mcom_pwm_disable,
	.owner		= THIS_MODULE,
};

static int mcom_pwm_probe(struct platform_device *pdev)
{
	struct mcom_pwm_chip *pwm_chip;
	struct resource *r;
	int ret;

	pwm_chip = devm_kzalloc(&pdev->dev, sizeof(*pwm_chip), GFP_KERNEL);
	if (!pwm_chip)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	pwm_chip->mmio_base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(pwm_chip->mmio_base))
		return PTR_ERR(pwm_chip->mmio_base);

	platform_set_drvdata(pdev, pwm_chip);

	pwm_chip->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pwm_chip->clk)) {
		dev_err(&pdev->dev, "failed to get clock: %ld\n",
			PTR_ERR(pwm_chip->clk));
		return PTR_ERR(pwm_chip->clk);
	}

	ret = clk_prepare_enable(pwm_chip->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable PWM clock\n");
		return ret;
	}

	pwm_chip->chip.dev = &pdev->dev;
	pwm_chip->chip.ops = &mcom_pwm_ops;
	pwm_chip->chip.base = -1;
	pwm_chip->chip.npwm = NUM_PWM_CHANNEL;

	ret = pwmchip_add(&pwm_chip->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
		clk_disable_unprepare(pwm_chip->clk);
		return ret;
	}

	return 0;
}

static int mcom_pwm_remove(struct platform_device *pdev)
{
	struct mcom_pwm_chip *pwm_chip = platform_get_drvdata(pdev);
	int ret;

	ret = pwmchip_remove(&pwm_chip->chip);
	if (ret < 0)
		return ret;

	clk_disable_unprepare(pwm_chip->clk);

	return 0;
}

static const struct of_device_id mcom_pwm_of_match[] = {
	{ .compatible	= "elvees,mcom-pwm" },
	{},
};
MODULE_DEVICE_TABLE(of, mcom_pwm_of_match);

static struct platform_driver mcom_pwm_driver = {
	.driver	= {
		.name	= "mcom-pwm",
		.of_match_table	= mcom_pwm_of_match,
	},
	.probe	= mcom_pwm_probe,
	.remove	= mcom_pwm_remove,
};
module_platform_driver(mcom_pwm_driver);

MODULE_DESCRIPTION("ELVEES MCom-03 PWM controller driver");
MODULE_AUTHOR("Alexander Barunin <abarunin@elvees.com>");
MODULE_LICENSE("GPL");
