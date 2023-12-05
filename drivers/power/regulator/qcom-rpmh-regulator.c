// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.

#include <linux/err.h>
#include <dm/device_compat.h>
#include <dm/device.h>
#include <dm/devres.h>
#include <dm/lists.h>
#include <power/regulator.h>
#include <log.h>

#include <soc/qcom/cmd-db.h>
#include <soc/qcom/rpmh.h>

#include <dt-bindings/regulator/qcom,rpmh-regulator.h>

/**
 * enum rpmh_regulator_type - supported RPMh accelerator types
 * @VRM:	RPMh VRM accelerator which supports voting on enable, voltage,
 *		and mode of LDO, SMPS, and BOB type PMIC regulators.
 * @XOB:	RPMh XOB accelerator which supports voting on the enable state
 *		of PMIC regulators.
 */
enum rpmh_regulator_type {
	VRM,
	XOB,
};

enum rpmh_regulator_mode {
	REGULATOR_MODE_RETENTION,
	REGULATOR_MODE_LPM,
	REGULATOR_MODE_AUTO,
	REGULATOR_MODE_HPM,
};

#define RPMH_REGULATOR_REG_VRM_VOLTAGE		0x0
#define RPMH_REGULATOR_REG_ENABLE		0x4
#define RPMH_REGULATOR_REG_VRM_MODE		0x8

#define PMIC4_LDO_MODE_RETENTION		4
#define PMIC4_LDO_MODE_LPM			5
#define PMIC4_LDO_MODE_HPM			7

#define PMIC4_SMPS_MODE_RETENTION		4
#define PMIC4_SMPS_MODE_PFM			5
#define PMIC4_SMPS_MODE_AUTO			6
#define PMIC4_SMPS_MODE_PWM			7

#define PMIC4_BOB_MODE_PASS			0
#define PMIC4_BOB_MODE_PFM			1
#define PMIC4_BOB_MODE_AUTO			2
#define PMIC4_BOB_MODE_PWM			3

#define PMIC5_LDO_MODE_RETENTION		3
#define PMIC5_LDO_MODE_LPM			4
#define PMIC5_LDO_MODE_HPM			7

#define PMIC5_SMPS_MODE_RETENTION		3
#define PMIC5_SMPS_MODE_PFM			4
#define PMIC5_SMPS_MODE_AUTO			6
#define PMIC5_SMPS_MODE_PWM			7

#define PMIC5_BOB_MODE_PASS			2
#define PMIC5_BOB_MODE_PFM			4
#define PMIC5_BOB_MODE_AUTO			6
#define PMIC5_BOB_MODE_PWM			7

/**
 * struct linear_range - table of selector - value pairs
 *
 * Define a lookup-table for range of values. Intended to help when looking
 * for a register value matching certaing physical measure (like voltage).
 * Usable when increment of one in register always results a constant increment
 * of the physical measure (like voltage).
 *
 * @min:  Lowest value in range
 * @min_sel: Lowest selector for range
 * @max_sel: Highest selector for range
 * @step: Value step size
 */
struct linear_range {
	unsigned int min;
	unsigned int min_sel;
	unsigned int max_sel;
	unsigned int step;
};

/* Initialize struct linear_range for regulators */
#define REGULATOR_LINEAR_RANGE(_min_uV, _min_sel, _max_sel, _step_uV)	\
{									\
	.min		= _min_uV,					\
	.min_sel	= _min_sel,					\
	.max_sel	= _max_sel,					\
	.step		= _step_uV,					\
}

/**
 * struct rpmh_vreg_hw_data - RPMh regulator hardware configurations
 * @regulator_type:		RPMh accelerator type used to manage this
 *				regulator
 * @ops:			Pointer to regulator ops callback structure
 * @voltage_range:		The single range of voltages supported by this
 *				PMIC regulator type
 * @n_voltages:			The number of unique voltage set points defined
 *				by voltage_range
 * @hpm_min_load_uA:		Minimum load current in microamps that requires
 *				high power mode (HPM) operation.  This is used
 *				for LDO hardware type regulators only.
 * @pmic_mode_map:		Array indexed by regulator framework mode
 *				containing PMIC hardware modes.  Must be large
 *				enough to index all framework modes supported
 *				by this regulator hardware type.
 * @of_map_mode:		Maps an RPMH_REGULATOR_MODE_* mode value defined
 *				in device tree to a regulator framework mode
 */
struct rpmh_vreg_hw_data {
	enum rpmh_regulator_type		regulator_type;
	const struct dm_regulator_ops		*ops;
	struct linear_range			voltage_range;
	int					n_voltages;
	int					hpm_min_load_uA;
	struct dm_regulator_mode		*pmic_mode_map;
	int					n_modes;
	unsigned int				(*of_map_mode)(unsigned int mode);
};

/**
 * struct rpmh_vreg - individual RPMh regulator data structure encapsulating a
 *		single regulator device
 * @dev:			Device pointer for the top-level PMIC RPMh
 *				regulator parent device.  This is used as a
 *				handle in RPMh write requests.
 * @addr:			Base address of the regulator resource within
 *				an RPMh accelerator
 * @rdesc:			Regulator descriptor
 * @hw_data:			PMIC regulator configuration data for this RPMh
 *				regulator
 * @always_wait_for_ack:	Boolean flag indicating if a request must always
 *				wait for an ACK from RPMh before continuing even
 *				if it corresponds to a strictly lower power
 *				state (e.g. enabled --> disabled).
 * @enabled:			Flag indicating if the regulator is enabled or
 *				not
 * @bypassed:			Boolean indicating if the regulator is in
 *				bypass (pass-through) mode or not.  This is
 *				only used by BOB rpmh-regulator resources.
 * @uv:				Selector used for get_voltage_sel() and
 *				set_value() callbacks
 * @mode:			RPMh VRM regulator current framework mode
 */
struct rpmh_vreg {
	struct udevice			*dev;
	u32				addr;
	const struct rpmh_vreg_hw_data	*hw_data;
	bool				always_wait_for_ack;

	int				enabled;
	bool				bypassed;
	int				uv;
	int			mode;
};

/**
 * struct rpmh_vreg_init_data - initialization data for an RPMh regulator
 * @name:			Name for the regulator which also corresponds
 *				to the device tree subnode name of the regulator
 * @resource_name:		RPMh regulator resource name format string.
 *				This must include exactly one field: '%s' which
 *				is filled at run-time with the PMIC ID provided
 *				by device tree property qcom,pmic-id.  Example:
 *				"ldo%s1" for RPMh resource "ldoa1".
 * @supply_name:		Parent supply regulator name
 * @hw_data:			Configuration data for this PMIC regulator type
 */
struct rpmh_vreg_init_data {
	const char			*name;
	const char			*resource_name;
	const char			*supply_name;
	const struct rpmh_vreg_hw_data	*hw_data;
};

/**
 * rpmh_regulator_send_request() - send the request to RPMh
 * @vreg:		Pointer to the RPMh regulator
 * @cmd:		Pointer to the RPMh command to send
 * @wait_for_ack:	Boolean indicating if execution must wait until the
 *			request has been acknowledged as complete
 *
 * Return: 0 on success, errno on failure
 */
static int rpmh_regulator_send_request(struct rpmh_vreg *vreg,
			struct tcs_cmd *cmd, bool wait_for_ack)
{
	int ret;

	if (wait_for_ack || vreg->always_wait_for_ack)
		ret = rpmh_write(vreg->dev->parent, RPMH_ACTIVE_ONLY_STATE, cmd, 1);
	else
		ret = rpmh_write_async(vreg->dev->parent, RPMH_ACTIVE_ONLY_STATE, cmd,
					1);

	return ret;
}

static int _rpmh_regulator_vrm_set_value(struct udevice *rdev,
				int uv, bool wait_for_ack)
{
	struct rpmh_vreg *vreg = dev_get_priv(rdev);
	struct tcs_cmd cmd = {
		.addr = vreg->addr + RPMH_REGULATOR_REG_VRM_VOLTAGE,
	};
	int ret;

	/* VRM voltage control register is set with voltage in millivolts. */
	uv = (uv / vreg->hw_data->voltage_range.step) *
		(vreg->hw_data->voltage_range.step / 1000);
	cmd.data = uv; // XXX: CHECKME

	ret = rpmh_regulator_send_request(vreg, &cmd, wait_for_ack);
	if (!ret)
		vreg->uv = uv;

	return ret;
}

static int rpmh_regulator_vrm_set_value(struct udevice *rdev,
					int uv)
{
	struct rpmh_vreg *vreg = dev_get_priv(rdev);

	debug("%s: set_value %d (current %d)\n", rdev->name, uv, vreg->uv);

	if (vreg->enabled == -EINVAL) {
		/*
		 * Cache the voltage and send it later when the regulator is
		 * enabled or disabled.
		 */
		vreg->uv = uv;
		return 0;
	}

	return _rpmh_regulator_vrm_set_value(rdev, uv,
					uv > vreg->uv);
}

static int rpmh_regulator_vrm_get_value(struct udevice *rdev)
{
	struct rpmh_vreg *vreg = dev_get_priv(rdev);

	debug("%s: get_value %d\n", rdev->name, vreg->uv);

	return vreg->uv;
}

static int rpmh_regulator_is_enabled(struct udevice *rdev)
{
	struct rpmh_vreg *vreg = dev_get_priv(rdev);

	debug("%s: is_enabled %d\n", rdev->name, vreg->enabled);

	return vreg->enabled;
}

static int rpmh_regulator_set_enable_state(struct udevice *rdev,
					bool enable)
{
	struct rpmh_vreg *vreg = dev_get_priv(rdev);
	struct tcs_cmd cmd = {
		.addr = vreg->addr + RPMH_REGULATOR_REG_ENABLE,
		.data = enable,
	};
	int ret;

	debug("%s: set_enable %d (current %d)\n", rdev->name, enable,
	       vreg->enabled);

	if (vreg->enabled == -EINVAL &&
	    vreg->uv != -ENOTRECOVERABLE) {
		ret = _rpmh_regulator_vrm_set_value(rdev,
						vreg->uv, true);
		if (ret < 0)
			return ret;
	}

	ret = rpmh_regulator_send_request(vreg, &cmd, enable);
	if (!ret)
		vreg->enabled = enable;

	return ret;
}

static int rpmh_regulator_vrm_set_mode_bypass(struct rpmh_vreg *vreg,
					unsigned int mode, bool bypassed)
{
	struct tcs_cmd cmd = {
		.addr = vreg->addr + RPMH_REGULATOR_REG_VRM_MODE,
	};
	struct dm_regulator_mode *pmic_mode;
	int i;

	if (mode > REGULATOR_MODE_HPM)
		return -EINVAL;

	for (i = 0; i < vreg->hw_data->n_modes; i++) {
		pmic_mode = &vreg->hw_data->pmic_mode_map[i];
		if (pmic_mode->id == mode)
			break;
	}
	if (pmic_mode->id != mode) {
		printf("Invalid mode %d\n", mode);
		return -EINVAL;
	}

	if (bypassed)
		cmd.data = PMIC4_BOB_MODE_PASS;
	else
		cmd.data = pmic_mode->id;

	return rpmh_regulator_send_request(vreg, &cmd, true);
}

static int rpmh_regulator_vrm_set_mode(struct udevice *rdev,
					int mode)
{
	struct rpmh_vreg *vreg = dev_get_priv(rdev);
	int ret;

	debug("%s: set_mode %d (current %d)\n", rdev->name, mode, vreg->mode);

	if (mode == vreg->mode)
		return 0;

	ret = rpmh_regulator_vrm_set_mode_bypass(vreg, mode, vreg->bypassed);
	if (!ret)
		vreg->mode = mode;

	return ret;
}

static int rpmh_regulator_vrm_get_mode(struct udevice *rdev)
{
	struct rpmh_vreg *vreg = dev_get_priv(rdev);

	debug("%s: get_mode %d\n", rdev->name, vreg->mode);

	return vreg->mode;
}

static const struct dm_regulator_ops rpmh_regulator_vrm_drms_ops = {
	.get_value = rpmh_regulator_vrm_get_value,
	.set_value = rpmh_regulator_vrm_set_value,
	.set_enable = rpmh_regulator_set_enable_state,
	.get_enable = rpmh_regulator_is_enabled,
	.set_mode = rpmh_regulator_vrm_set_mode,
	.get_mode = rpmh_regulator_vrm_get_mode,
};

static struct dm_regulator_mode pmic_mode_map_pmic5_ldo[] = {
	{
		.id = REGULATOR_MODE_RETENTION,
		.register_value = PMIC5_LDO_MODE_RETENTION,
		.name = "PMIC5_LDO_MODE_RETENTION"
	}, {
		.id = REGULATOR_MODE_LPM,
		.register_value = PMIC5_LDO_MODE_LPM,
		.name = "PMIC5_LDO_MODE_LPM"
	}, {
		.id = REGULATOR_MODE_HPM,
		.register_value = PMIC5_LDO_MODE_HPM,
		.name = "PMIC5_LDO_MODE_HPM"
	},
};

static const struct rpmh_vreg_hw_data pmic5_pldo = {
	.regulator_type = VRM,
	.ops = &rpmh_regulator_vrm_drms_ops,
	.voltage_range = REGULATOR_LINEAR_RANGE(1504000, 0, 255, 8000),
	.n_voltages = 256,
	.hpm_min_load_uA = 10000,
	.pmic_mode_map = pmic_mode_map_pmic5_ldo,
	.n_modes = ARRAY_SIZE(pmic_mode_map_pmic5_ldo),
};

#define RPMH_VREG(_name, _resource_name, _hw_data, _supply_name) \
{ \
	.name		= _name, \
	.resource_name	= _resource_name, \
	.hw_data	= _hw_data, \
	.supply_name	= _supply_name, \
}

static const struct rpmh_vreg_init_data pm8150l_vreg_data[] = {
	RPMH_VREG("ldo11",  "ldo%s11", &pmic5_pldo,      "vdd-l7-l11"),
	{}
};

/* probe an individual regulator */
static int rpmh_regulator_probe(struct udevice *dev)
{
	const struct rpmh_vreg_init_data *init_data;
	struct rpmh_vreg *priv;
	struct dm_regulator_uclass_plat *plat_data;

	init_data = (const struct rpmh_vreg_init_data *)dev_get_driver_data(dev);
	priv = dev_get_priv(dev);
	plat_data = dev_get_uclass_plat(dev);

	priv->dev = dev;
	priv->addr = cmd_db_read_addr(dev->name);
	if (!priv->addr) {
		printf("Failed to read RPMh address for %s\n", dev->name);
		return -ENODEV;
	}

	priv->hw_data = init_data->hw_data;
	priv->enabled = -EINVAL;
	priv->uv = -ENOTRECOVERABLE;
	if (ofnode_read_u32(dev_ofnode(dev), "regulator-initial-mode", &priv->mode))
		priv->mode = -EINVAL;

	// handle stepping
	// almost there!!!
	plat_data->mode = priv->hw_data->pmic_mode_map;
	plat_data->mode_count = priv->hw_data->n_modes;

	return 0;
}

/* for non-drm, xob, or bypass regulators add additional driver definitions */
U_BOOT_DRIVER(rpmh_regulator_drm) = {
	.name = "rpmh_regulator_drm",
	.id = UCLASS_REGULATOR,
	.probe = rpmh_regulator_probe,
	.priv_auto = sizeof(struct rpmh_vreg),
	.ops = &rpmh_regulator_vrm_drms_ops,
};

/* This driver intentionally only supports a subset of the available regulators.
 * This function checks to see if a given regulator node in DT matches a regulator
 * defined in the driver.
 */
static const struct rpmh_vreg_init_data *
vreg_get_init_data(const struct rpmh_vreg_init_data *init_data, ofnode node) {
        const struct rpmh_vreg_init_data *data;

	for(data = init_data; data->name; data++) {
		if (!strncmp(data->name, ofnode_get_name(node), strlen(data->name)))
			return data;
	}

	return NULL;
}

static int rpmh_regulators_bind(struct udevice *dev)
{
	const struct rpmh_vreg_init_data *init_data, *data;
	const char *pmic_id;
	char *name;
	struct driver *drv;
	ofnode node;
	int ret;
	size_t namelen;

	init_data = (const struct rpmh_vreg_init_data *)dev_get_driver_data(dev);
	if (!init_data) {
		printf("No RPMh regulator init data\n");
		return -ENODEV;
	}

	pmic_id = ofnode_read_string(dev_ofnode(dev), "qcom,pmic-id");
	if (!pmic_id) {
		printf("No PMIC ID\n");
		return -ENODEV;
	}

	drv = lists_driver_lookup_name("rpmh_regulator_drm");

	ofnode_for_each_subnode(node, dev_ofnode(dev)) {
		data = vreg_get_init_data(init_data, node);
		if (!data)
			continue;

		/* %s is replaced with pmic_id, so subtract 2, then add 1 for the null terminator */
		namelen = strlen(data->resource_name) + strlen(pmic_id) - 1;
		name = devm_kzalloc(dev, namelen, GFP_KERNEL);
		ret = snprintf(name, namelen, data->resource_name, pmic_id);
		if (ret < 0 || ret >= namelen) {
			printf("Failed to create RPMh regulator name\n");
			return -ENOMEM;
		}

		ret = device_bind_with_driver_data(dev, drv, name, (ulong)data,
						   node, NULL);
	}

	return 0;
}

static const struct udevice_id rpmh_regulator_ids[] = {
	{
		.compatible = "qcom,pm8150l-rpmh-regulators",
		.data = (ulong)pm8150l_vreg_data,
	},
	{ /* sentinal */ },
};

/* Driver for a 'bank' of regulators. This creates devices for each
 * individual regulator
 */
U_BOOT_DRIVER(rpmh_regulators) = {
	.name = "rpmh_regulators",
	.id = UCLASS_MISC,
	.bind = rpmh_regulators_bind,
	.of_match = rpmh_regulator_ids,
	.ops = &rpmh_regulator_vrm_drms_ops,
};
