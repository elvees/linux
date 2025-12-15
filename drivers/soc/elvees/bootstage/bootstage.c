// SPDX-License-Identifier: GPL-2.0
// Copyright 2025 RnD Center "ELVEES", JSC

// Bootstage module for MCom-03

#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/sysfs.h>

#include <soc/elvees/mcom03/bootstage.h>
#include <soc/elvees/mcom03/mcom03-helper.h>
#include <soc/elvees/mcom03/mcom03-sip.h>

#define NAME "ELVEES MCom-03 SoC bootstage module" /* Module name */

#define mcom03_bootstage_sip(id, param) \
	mcom03_sip_smccc_smc(MCOM03_SIP_BOOTSTAGE, (id), (param), 0, 0, 0, 0, 0)

struct bootstage {
	uint32_t id;
	const char *desc;
	int64_t timestamp;
};

static struct bootstage bootstages[] = {
	// Boot firmware bootstages
	{ BOOTSTAGE_ID_SBL_S1_START, "sbl-s1 start", -ENOENT },
	{ BOOTSTAGE_ID_DDRINIT_START, "ddrinit start", -ENOENT },
	{ BOOTSTAGE_ID_SBL_S2_START, "sbl-s2 start", -ENOENT },
	{ BOOTSTAGE_ID_SBL_S2_LOAD_START, "sbl-s2 load start", -ENOENT },
	{ BOOTSTAGE_ID_SBL_S2_LOAD_COMPLETE, "sbl-s2 load complete", -ENOENT },
	{ BOOTSTAGE_ID_SBL_S3_START, "sbl-s3 start", -ENOENT },
	{ BOOTSTAGE_ID_TF_A_START, "tf-a(BL31) start", -ENOENT },
	{ BOOTSTAGE_ID_START_UBOOT_F, "U-Boot(BL33) start", -ENOENT },
	{ BOOTSTAGE_ID_RUN_OS, "U-Boot(BL33) Linux Kernel start", -ENOENT },
};

static struct kobject *bootstage_kobj;

static void mcom03_bootstage_get_timestamps(void)
{
	long timestamp;
	const char *desc;
	uint32_t id;
	int i;

	for (i = 0; i < ARRAY_SIZE(bootstages); ++i) {
		desc = bootstages[i].desc;
		id = bootstages[i].id;

		timestamp = mcom03_bootstage_sip(MCOM03_SIP_BOOTSTAGE_GET_TIMESTAMP, id);
		bootstages[i].timestamp = timestamp;
		pr_debug("Bootstage #%d (%s) timestamp=%ld\n", i, desc, timestamp);
	}
}

static ssize_t mcom03_bootstage_show(struct kobject *kobj, struct kobj_attribute *attr,
				     char *buf)
{
	int len = 0, ret, i;
	size_t size = PAGE_SIZE;

	for (i = 0; i < ARRAY_SIZE(bootstages) - 1; ++i) {
		if (!i) {
			ret = mcom03_sprintf(buf, &size, &len, "{");
			if (ret)
				return ret;
		} else {
			ret = mcom03_sprintf(buf, &size, &len, ",");
			if (ret)
				return ret;
		}

		ret = mcom03_sprintf(buf, &size, &len,
				     "\"%d\":{\"desc\":\"%s\",\"timestamp\":\"%ld\"}",
				     bootstages[i].id,
				     bootstages[i].desc,
				     bootstages[i].timestamp);
		if (ret)
			return ret;
	}

	ret = mcom03_sprintf(buf, &size, &len,
			     ",\"%d\":{\"desc\":\"%s\",\"timestamp\":\"%ld\"}}\n",
			     bootstages[i].id,
			     bootstages[i].desc,
			     bootstages[i].timestamp);
	if (ret)
		return ret;

	return len;
}

static struct kobj_attribute mcom03_bootstage_show_attribute =
	__ATTR(timestamps, 0444, mcom03_bootstage_show, NULL);

static struct attribute *bootstage_attrs[] = {
	&mcom03_bootstage_show_attribute.attr,
	NULL,
};

static struct attribute_group bootstage_attr_group = {
	.attrs = bootstage_attrs,
};

static int __init bootstage_init(void)
{
	int ret;

	mcom03_bootstage_get_timestamps();

	bootstage_kobj = kobject_create_and_add("bootstage", firmware_kobj);
	if (!bootstage_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(bootstage_kobj, &bootstage_attr_group);
	if (ret) {
		kobject_put(bootstage_kobj);
		return ret;
	}

	pr_info("%s: loaded successfully\n", NAME);

	return 0;
}
module_init(bootstage_init);

static void __exit bootstage_exit(void)
{
	sysfs_remove_group(bootstage_kobj, &bootstage_attr_group);
	kobject_put(bootstage_kobj);
	pr_info("%s: unloaded successfully\n", NAME);
}
module_exit(bootstage_exit);

MODULE_DESCRIPTION("sysfs interface to " NAME);
MODULE_LICENSE("GPL");
