// SPDX-License-Identifier: GPL-2.0
// Copyright 2025 RnD Center "ELVEES", JSC

// Bootstage module for MCom-03

#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/sysfs.h>

#include <soc/elvees/mcom03/bootstage.h>
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

static int mcom03_bootstage_sprintf(char *buf, size_t *size, int *pos, const char *fmt, ...)
{
	va_list args;
	int n;

	if (*pos >= *size)
		return -ENOMEM;

	va_start(args, fmt);
	n = vsnprintf(NULL, 0, fmt, args);
	va_end(args);

	if (n < 0)
		return -EINVAL;
	if (n >= *size)
		return -ENOMEM;

	va_start(args, fmt);
	n = vsnprintf(&buf[*pos], *size, fmt, args);
	va_end(args);

	*pos += n;
	*size -= n;

	return 0;
}

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
		if (timestamp >= 0)
			bootstages[i].timestamp = timestamp;
		else
			pr_debug("Failed to get bootstage #%d (%s), ret=%ld\n", i, desc, timestamp);
	}
}

static int mcom03_bootstage_show_one(struct bootstage *bs, char *buf, size_t *size, int *pos)
{
	int ret;

	ret = mcom03_bootstage_sprintf(buf, size, pos, "\"%d\":{", bs->id);
	if (ret)
		return ret;

	ret = mcom03_bootstage_sprintf(buf, size, pos, "\"desc\":\"%s\",", bs->desc);
	if (ret)
		return ret;

	ret = mcom03_bootstage_sprintf(buf, size, pos, "\"timestamp\":\"%ld\"}", bs->timestamp);
	if (ret)
		return ret;

	return 0;
}

static ssize_t mcom03_bootstage_show(struct kobject *kobj, struct kobj_attribute *attr,
				     char *buf)
{
	int len = 0, ret, i;
	size_t size = PAGE_SIZE;

	ret = mcom03_bootstage_sprintf(buf, &size, &len, "{");
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(bootstages) - 1; ++i) {
		if (bootstages[i].timestamp < 0)
			continue;

		ret = mcom03_bootstage_show_one(&bootstages[i], buf, &size, &len);
		if (ret)
			return ret;

		ret = mcom03_bootstage_sprintf(buf, &size, &len, ",");
		if (ret)
			return ret;
	}

	if (bootstages[i].timestamp >= 0) {
		ret = mcom03_bootstage_show_one(&bootstages[i], buf, &size, &len);
		if (ret)
			return ret;
	}

	ret = mcom03_bootstage_sprintf(buf, &size, &len, "}\n");
	if (ret)
		return ret;

	return len;
}

static struct kobj_attribute mcom03_bootstage_show_attribute =
	__ATTR(timestamps, 0444, mcom03_bootstage_show, NULL);

static struct attribute *attrs[] = {
	&mcom03_bootstage_show_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static int __init bootstage_init(void)
{
	int retval;

	mcom03_bootstage_get_timestamps();

	bootstage_kobj = kobject_create_and_add("bootstage", firmware_kobj);
	if (!bootstage_kobj)
		return -ENOMEM;

	retval = sysfs_create_group(bootstage_kobj, &attr_group);
	if (retval) {
		kobject_put(bootstage_kobj);
		return retval;
	}

	pr_info("%s: loaded successfully\n", NAME);

	return 0;
}
module_init(bootstage_init);

static void __exit bootstage_exit(void)
{
	kobject_put(bootstage_kobj);
	pr_info("%s: unloaded successfully\n", NAME);
}
module_exit(bootstage_exit);

MODULE_DESCRIPTION("sysfs interface to " NAME);
MODULE_LICENSE("GPL");
