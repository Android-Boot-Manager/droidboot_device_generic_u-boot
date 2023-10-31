/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Qualcomm common pin control data.
 *
 * Copyright (C) 2023 Linaro Ltd.
 */
#ifndef _QCOM_GPIO_H_
#define _QCOM_GPIO_H_

#include <asm/types.h>
#include <stdbool.h>

struct msm_pin_data {
	int pin_count;
	const unsigned int *pin_offsets;
};

static inline u32 qcom_pin_offset(const unsigned int *offs, unsigned int selector)
{
	u32 out = (selector * 0x1000);

	if (offs)
		return out + offs[selector];

	return out;
}

struct udevice;

/**
 * msm_pinctrl_is_reserved() - Check if a pin lies in a reserved range
 *
 * @dev: pinctrl device
 * @pin: Pin number
 *
 * Returns: true if pin is reserved, otherwise false
 *
 * Call using dev_get_parent() from the GPIO device, it is a child of
 * the pinctrl device.
 */
bool msm_pinctrl_is_reserved(struct udevice *dev, unsigned int pin);

#endif /* _QCOM_GPIO_H_ */
