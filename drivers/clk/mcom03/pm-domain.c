// SPDX-License-Identifier: GPL-2.0

// Copyright 2025 RnD Center "ELVEES", JSC

#include <dt-bindings/soc/elvees,mcom03.h>
#include <linux/mfd/syscon.h>
#include <linux/slab.h>
#include <soc/elvees/mcom03/mcom03-sip.h>

#include "mcom03-clk.h"

#define SERVICE_SUBS_PSTATUS_OFFSET 0x4
#define SERVICE_SUBS_PSTATUS_MASK 0x1f

#define SERVICE_SUBS_TOP_CLKGATE 0x1008

#define mcom03_pm_sip(id, arg0) \
	mcom03_sip_smccc_smc(MCOM03_SIP_POWER_DOMAIN, (id), (arg0), 0, 0, 0, 0, 0)

#define to_mcom03_pd(gpd) container_of(gpd, struct mcom03_pm_domain, genpd)

static int _mcom03_power_domain_on(struct mcom03_pm_domain *pd,
				   const char *name,
				   bool restore_clocks)
{
	int ret;

	if (!pd->is_supported) {
		/* In current kernel version this code is unreachable because
		 * genpd does not call .power_on() callback if .power_off()
		 * returned error. Leave this condition because behaviour can be
		 * changed in future versions.
		 */
		pd->is_enabled = true;
		return 0;
	}

	pr_info("Try to power on %s\n", name);
	mutex_lock(&pd->lock);
	ret = mcom03_pm_sip(MCOM03_SIP_POWER_DOMAIN_ENABLE, pd->id);
	if (ret) {
		mutex_unlock(&pd->lock);
		pr_err("Failed to enable %s, SIP call returns %d\n", name, ret);
		return ret;
	}

	pd->is_enabled = true;
	if (restore_clocks)
		mcom03_clk_restore(pd->clk_provider);

	mutex_unlock(&pd->lock);

	return 0;
}

static int mcom03_power_domain_on_direct(struct mcom03_pm_domain *pd,
					 const char *name)
{
	u32 subsys_clkgates[] = {
		[MCOM03_SUBSYSTEM_MEDIA] = BIT(1),
		[MCOM03_SUBSYSTEM_CPU] = BIT(2),
		[MCOM03_SUBSYSTEM_SDR] = BIT(3),
		/* Other subsystems are not supported in
		 * mcom03_power_domain_init() function.
		 */
	};
	u32 mask = subsys_clkgates[pd->id];
	u32 val;
	int ret;

	pr_info("Try to power on %s directly\n", name);
	regmap_write(pd->service_subs_urb, pd->offset, PP_ON);
	ret = regmap_read_poll_timeout(pd->service_subs_urb,
				       pd->offset + SERVICE_SUBS_PSTATUS_OFFSET,
				       val,
				       (val & 0x1f) == PP_ON,
				       0,
				       2000000);
	if (ret)
		return ret;

	ret = regmap_write_bits(pd->service_subs_urb, SERVICE_SUBS_TOP_CLKGATE,
				mask, mask);
	pd->is_enabled = true;

	return ret;
}

static int mcom03_power_domain_on(struct generic_pm_domain *domain)
{
	struct mcom03_pm_domain *pd = to_mcom03_pd(domain);

	return _mcom03_power_domain_on(pd, domain->name, true);
}

static int mcom03_power_domain_off(struct generic_pm_domain *domain)
{
	int ret;
	struct mcom03_pm_domain *pd = to_mcom03_pd(domain);

	if (!pd->is_supported)
		return -ENODEV;

	pr_info("Try to power off %s\n", domain->name);
	mutex_lock(&pd->lock);
	pd->is_enabled = false;
	ret = mcom03_pm_sip(MCOM03_SIP_POWER_DOMAIN_DISABLE, pd->id);
	mutex_unlock(&pd->lock);
	if (ret)
		pr_err("Failed to disable %s, SIP call returns %d\n",
		       domain->name, ret);

	return ret;
}

static int mcom03_power_domain_is_enabled(struct mcom03_pm_domain *pd)
{
	u32 val;

	regmap_read(pd->service_subs_urb,
		    pd->offset + SERVICE_SUBS_PSTATUS_OFFSET,
		    &val);

	return (val & SERVICE_SUBS_PSTATUS_MASK) == PP_ON;
}

struct mcom03_pm_domain *mcom03_power_domain_init(struct device_node *node,
						  u32 id, bool pd_enable)
{
	struct mcom03_pm_domain *pd;
	const char *name;
	u32 offset;
	int ret;

	if (id == MCOM03_SUBSYSTEM_SDR) {
		offset = 0x8;
		name = "SDR power domain";
	} else if (id == MCOM03_SUBSYSTEM_MEDIA) {
		offset = 0x10;
		name = "Media power domain";
	} else {
		if (pd_enable)
			pr_err("%pOFf: Invalid subsystem id (%#x)\n", node, id);

		return NULL;
	}

	pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return NULL;

	pd->offset = offset;
	pd->genpd.name = name;

	ret = mcom03_pm_sip(MCOM03_SIP_POWER_DOMAIN_CHECK_SUPPORT, id);
	if (ret)
		pr_info("%s: Board firmware does not support power domain control\n",
			name);

	pd->is_supported = !ret;
	pd->id = id;
	pd->genpd.power_on = mcom03_power_domain_on;
	pd->genpd.power_off = mcom03_power_domain_off;
	mutex_init(&pd->lock);
	pd->service_subs_urb =
		syscon_regmap_lookup_by_phandle(node, "elvees,service-urb");
	if (!pd->service_subs_urb) {
		pr_err("%pOFf: Failed to get regmap from 'elvees,service-urb' property!\n",
		       node);
		goto err_cleanup;
	}

	pd->is_enabled = mcom03_power_domain_is_enabled(pd);
	if (!pd->is_enabled) {
		if (likely(pd->is_supported)) {
			_mcom03_power_domain_on(pd, name, false);
		} else {
			/* Domain is powered off, but control via SIP is not
			 * supported. Try to enable domain directly without SIP.
			 * This may be unstable.
			 */
			ret = mcom03_power_domain_on_direct(pd, name);
			WARN(ret, "%s: Failed to enable domain (%d)",
			     name, ret);
		}
	}

	/* Power domain can be disabled on some boards. In this case genpd
	 * must not be created.
	 */
	if (!pd_enable) {
		kfree(pd);
		return NULL;
	}

	ret = pm_genpd_init(&pd->genpd, NULL, !pd->is_enabled);
	if (ret) {
		pr_err("Failed to init %s (%d)\n", pd->genpd.name, ret);
		goto err_cleanup;
	}

	ret = of_genpd_add_provider_simple(node, &pd->genpd);
	if (ret) {
		pr_err("Failed to add provider for %s (%d)\n",
		       pd->genpd.name, ret);
		goto err_cleanup;
	}

	pr_info("%s is initialized\n", pd->genpd.name);

	return pd;

err_cleanup:
	kfree(pd);
	return NULL;
}
