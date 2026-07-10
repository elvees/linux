// SPDX-License-Identifier: GPL-2.0
// Copyright 2025 RnD Center "ELVEES", JSC

// OTP module for MCom-03R

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>

#include <soc/elvees/mcom03/mcom03-helper.h>
#include <soc/elvees/mcom03/mcom03-sip.h>

#define mcom03r_otp_sip(id, param0, param1) \
	mcom03_sip_smccc_smc(MCOM03R_SIP_OTP, (id), (param0), (param1), 0, 0, 0, 0)

#define to_mcom03r_otp_priv(x) container_of(x, struct mcom03r_otp_priv, kobj)

#define mcom03r_otp_single_attr(nm) mcom03r_otp_single_attr_##nm
#define mcom03r_otp_line_attr(nm) mcom03r_otp_line_attr_##nm
#define mcom03r_otp_multi_line_attr(nm) mcom03r_otp_multi_line_attr_##nm

#define mcom03r_otp_declare_single_attr(nm, fd, ft)					\
	static ssize_t mcom03r_otp_show_single_##nm(struct kobject *kobj,		\
						    struct kobj_attribute *attr,	\
						    char *buf)				\
	{										\
		int ln = 0, ret;							\
		size_t sz = PAGE_SIZE;							\
		const struct mcom03r_otp_priv *priv = to_mcom03r_otp_priv(kobj);	\
		mutex_lock(&mcom03r_otp_mutex);						\
		ret = mcom03_sprintf(buf, &sz, &ln, ft "\n", priv->regs->fd);		\
		if (ret)								\
			ln = ret;							\
		mutex_unlock(&mcom03r_otp_mutex);					\
		return ln;								\
	}										\
	static struct kobj_attribute mcom03r_otp_single_attr(nm) =			\
		__ATTR(nm, 0444, mcom03r_otp_show_single_##nm, NULL)

#define mcom03r_otp_declare_line_attr(nm, fd, ft, dl)					\
	static ssize_t mcom03r_otp_show_line_##nm(struct kobject *kobj,			\
						  struct kobj_attribute *attr,		\
						  char *buf)				\
	{										\
		int ln = 0, ret, i;							\
		size_t sz = PAGE_SIZE;							\
		const struct mcom03r_otp_priv *priv = to_mcom03r_otp_priv(kobj);	\
		mutex_lock(&mcom03r_otp_mutex);						\
		for (i = 0; i < ARRAY_SIZE(priv->regs->fd) - 1; ++i) {			\
			ret = mcom03_sprintf(buf, &sz, &ln, ft dl, priv->regs->fd[i]);	\
			if (ret) {							\
				mutex_unlock(&mcom03r_otp_mutex);			\
				return ret;						\
			}								\
		}									\
		ret = mcom03_sprintf(buf, &sz, &ln, ft "\n", priv->regs->fd[i]);	\
		if (ret)								\
			ln = ret;							\
		mutex_unlock(&mcom03r_otp_mutex);					\
		return ln;								\
	}										\
	static struct kobj_attribute mcom03r_otp_line_attr(nm) =			\
		__ATTR(nm, 0444, mcom03r_otp_show_line_##nm, NULL)

#define mcom03r_otp_declare_multi_line_attr(nm, fd, ft, dl, br)					\
	static ssize_t mcom03r_otp_show_multi_line_##nm(struct kobject *kobj,			\
							struct kobj_attribute *attr,		\
							char *buf)				\
	{											\
		int ln = 0, ret, i;								\
		size_t sz = PAGE_SIZE;								\
		const struct mcom03r_otp_priv *priv = to_mcom03r_otp_priv(kobj);		\
		mutex_lock(&mcom03r_otp_mutex);							\
		for (i = 0; i < ARRAY_SIZE(priv->regs->fd) - 1; ++i) {				\
			if ((i % br) == (br - 1)) {						\
				ret = mcom03_sprintf(buf, &sz, &ln, ft "\n", priv->regs->fd[i]);\
				if (ret) {							\
					mutex_unlock(&mcom03r_otp_mutex);			\
					return ret;						\
				}								\
			} else {								\
				ret = mcom03_sprintf(buf, &sz, &ln, ft dl, priv->regs->fd[i]);	\
				if (ret) {							\
					mutex_unlock(&mcom03r_otp_mutex);			\
					return ret;						\
				}								\
			}									\
		}										\
		ret = mcom03_sprintf(buf, &sz, &ln, ft "\n", priv->regs->fd[i]);		\
		if (ret)									\
			ln = ret;								\
		mutex_unlock(&mcom03r_otp_mutex);						\
		return ln;									\
	}											\
	static struct kobj_attribute mcom03r_otp_multi_line_attr(nm) =				\
		__ATTR(nm, 0444, mcom03r_otp_show_multi_line_##nm, NULL)

struct otp {
	u32 fuse1;
	u32 fuse0;
	union {
		u32 flags;
		struct {
			u32 reserved0 : 16;
			u32 force_sign : 1;
			u32 force_encrypt : 1;
			u32 reserved1 : 1;
			u32 disable_log : 1;
			u32 enable_watchdog : 1;
			u32 reserved2 : 11;
		} flags_bits;
	};
	u32 serial;
	u8 duk[16];
	u8 rotpk[32];
	u32 reserved3;
	u32 crls[7];
	u32 reserved4;
	u32 crls_protection[7];
	u32 fuse1_redundant;
	u32 fuse0_redundant;
	u8 reserved5[52];
	u8 fw_lockable[68];
	u8 fw_nolock[256];
};

struct mcom03r_otp_priv {
	struct kobject kobj;
	dma_addr_t buf;
	size_t buf_size;
	struct otp *regs;
};

static DEFINE_MUTEX(mcom03r_otp_mutex);

mcom03r_otp_declare_single_attr(fuse1, fuse1, "%#010x");
mcom03r_otp_declare_single_attr(fuse0, fuse0, "%#010x");
mcom03r_otp_declare_single_attr(flags_reserved0, flags_bits.reserved0, "%#06x");
mcom03r_otp_declare_single_attr(flags_force_sign, flags_bits.force_sign, "%u");
mcom03r_otp_declare_single_attr(flags_force_encrypt, flags_bits.force_encrypt, "%u");
mcom03r_otp_declare_single_attr(flags_reserved1, flags_bits.reserved1, "%u");
mcom03r_otp_declare_single_attr(flags_disable_log, flags_bits.disable_log, "%u");
mcom03r_otp_declare_single_attr(flags_enable_watchdog, flags_bits.enable_watchdog, "%u");
mcom03r_otp_declare_single_attr(flags_reserved2, flags_bits.reserved2, "%#06x");
mcom03r_otp_declare_single_attr(serial, serial, "%#010x");
mcom03r_otp_declare_line_attr(duk, duk, "%02x", "");
mcom03r_otp_declare_line_attr(rotpk, rotpk, "%02x", "");
mcom03r_otp_declare_single_attr(reserved3, reserved3, "%#010x");
mcom03r_otp_declare_line_attr(revocation_list, crls, "%#010x", " ");
mcom03r_otp_declare_single_attr(reserved4, reserved4, "%#010x");
mcom03r_otp_declare_line_attr(revocation_list_protection, crls_protection, "%#010x", " ");
mcom03r_otp_declare_single_attr(fuse1_redundant, fuse1_redundant, "%#010x");
mcom03r_otp_declare_single_attr(fuse0_redundant, fuse0_redundant, "%#010x");
mcom03r_otp_declare_multi_line_attr(reserved5, reserved5, "%02x", " ", 8);
mcom03r_otp_declare_multi_line_attr(fw_defined_lockable, fw_lockable, "%02x", " ", 8);
mcom03r_otp_declare_multi_line_attr(fw_defined_nolock, fw_nolock, "%02x", " ", 8);

static struct attribute *mcom03r_otp_attrs[] = {
	&mcom03r_otp_single_attr(fuse1).attr,
	&mcom03r_otp_single_attr(fuse0).attr,
	&mcom03r_otp_single_attr(flags_reserved0).attr,
	&mcom03r_otp_single_attr(flags_force_sign).attr,
	&mcom03r_otp_single_attr(flags_force_encrypt).attr,
	&mcom03r_otp_single_attr(flags_reserved1).attr,
	&mcom03r_otp_single_attr(flags_disable_log).attr,
	&mcom03r_otp_single_attr(flags_enable_watchdog).attr,
	&mcom03r_otp_single_attr(flags_reserved2).attr,
	&mcom03r_otp_single_attr(serial).attr,
	&mcom03r_otp_line_attr(duk).attr,
	&mcom03r_otp_line_attr(rotpk).attr,
	&mcom03r_otp_single_attr(reserved3).attr,
	&mcom03r_otp_line_attr(revocation_list).attr,
	&mcom03r_otp_single_attr(reserved4).attr,
	&mcom03r_otp_line_attr(revocation_list_protection).attr,
	&mcom03r_otp_single_attr(fuse1_redundant).attr,
	&mcom03r_otp_single_attr(fuse0_redundant).attr,
	&mcom03r_otp_multi_line_attr(reserved5).attr,
	&mcom03r_otp_multi_line_attr(fw_defined_lockable).attr,
	&mcom03r_otp_multi_line_attr(fw_defined_nolock).attr,
	NULL,
};

static struct attribute_group mcom03r_otp_group = {
	.attrs = mcom03r_otp_attrs,
};

static const struct kobj_type mcom03r_otp_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
};

static int mcom03r_otp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mcom03r_otp_priv *priv;
	int ret;

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return dev_err_probe(dev, PTR_ERR(priv), "Failed to alloc priv struct\n");

	platform_set_drvdata(pdev, priv);

	priv->buf_size = sizeof(struct otp);

	priv->regs = dma_alloc_coherent(dev, priv->buf_size, &priv->buf, GFP_KERNEL);
	if (!priv->regs)
		return dev_err_probe(dev, PTR_ERR(priv->regs),
				     "Failed to alloc dma coherent buffer\n");

	ret = mcom03r_otp_sip(MCOM03R_SIP_OTP_GET_DUMP, priv->buf, sizeof(struct otp));
	if (ret) {
		dma_free_coherent(dev, priv->buf_size, priv->regs, priv->buf);
		return dev_err_probe(dev, ret, "Failed to get OTP\n");
	}

	ret = kobject_init_and_add(&priv->kobj, &mcom03r_otp_ktype, firmware_kobj, "otp");
	if (ret) {
		dma_free_coherent(dev, priv->buf_size, priv->regs, priv->buf);
		kobject_put(&priv->kobj);
		return dev_err_probe(dev, ret,
				     "Failed to init and add kobject to firmware one\n");
	}

	ret = sysfs_create_group(&priv->kobj, &mcom03r_otp_group);
	if (ret) {
		dma_free_coherent(dev, priv->buf_size, priv->regs, priv->buf);
		kobject_put(&priv->kobj);
		return dev_err_probe(dev, ret, "Failed to create sysfs group\n");
	}

	dev_info(dev, "Probe successfully\n");

	return 0;
}

static int mcom03r_otp_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mcom03r_otp_priv *priv = platform_get_drvdata(pdev);

	if (priv) {
		sysfs_remove_group(&priv->kobj, &mcom03r_otp_group);
		dma_free_coherent(dev, priv->buf_size, priv->regs, priv->buf);
		kobject_put(&priv->kobj);
	}

	dev_info(dev, "Remove successfully\n");

	return 0;
}

static const struct of_device_id mcom03r_otp_of_match[] = {
	{ .compatible = "elvees,mcom03r-otp" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mcom03r_otp_of_match);

static struct platform_driver mcom03r_otp_driver = {
	.probe		= mcom03r_otp_probe,
	.remove		= mcom03r_otp_remove,
	.driver		= {
		.name   = "mcom03r-otp",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mcom03r_otp_of_match),
	},
};

module_platform_driver(mcom03r_otp_driver);

MODULE_DESCRIPTION("sysfs interface to ELVEES MCom-03R SoC OTP module");
MODULE_LICENSE("GPL");
