/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <caf/led_effect.h>
#include "led_state.h"
#include "ei_data_forwarder_event.h"

/* This configuration file is included only once from led_state module and holds
 * information about LED effects associated with data forwarder states and machine
 * learning results.
 */

/* This structure enforces the header file is included only once in the build.
 * Violating this requirement triggers a multiple definition error at link time.
 */
const struct {} led_state_def_include_once;


/* Map function to LED ID */
static const uint8_t led_map[] = {
	[LED_ID_ML_STATE] = 0,
	[LED_ID_SENSOR_SIM] = LED_UNAVAILABLE,
};

static const struct led_effect ei_data_forwarder_led_effects[] = {
	[EI_DATA_FORWARDER_STATE_DISCONNECTED]	=
		LED_EFFECT_LED_BLINK(2000, LED_COLOR(255, 0, 0)),
	[EI_DATA_FORWARDER_STATE_CONNECTED]	=
		LED_EFFECT_LED_BLINK(500, LED_COLOR(255, 0, 0)),
	[EI_DATA_FORWARDER_STATE_TRANSMITTING]	=
		LED_EFFECT_LED_BLINK(50, LED_COLOR(255, 0, 0)),
};

static const struct ml_result_led_effect ml_result_led_effects[] = {
	{
		.label = NULL,
		.effect = LED_EFFECT_LED_BLINK2(1000, 2000, LED_COLOR(255, 255, 255)),
	},
	{
		.label = ANOMALY_LABEL,
		.effect = LED_EFFECT_LED_ON(LED_COLOR(255, 0, 255)),
	},
	{
		.label = "idle",
		.effect = LED_EFFECT_LED_ON(LED_COLOR(255, 255, 255)),
	},
	{
		.label = "zoom_in",
		.effect = LED_EFFECT_LED_ON(LED_COLOR(255, 0, 0)),
	},
	{
		.label = "zoom_out",
		.effect = LED_EFFECT_LED_ON(LED_COLOR(0, 0, 255)),
	},
	{
		.label = "rot_left",
		.effect = LED_EFFECT_LED_ON(LED_COLOR(0, 255, 0)),
	},
	{
		.label = "rot_right",
		.effect = LED_EFFECT_LED_ON(LED_COLOR(127, 0, 127)),
	},
	{
		.label = "move_left",
		.effect = LED_EFFECT_LED_ON(LED_COLOR(127, 127, 127)),
	},
	{
		.label = "move_right",
		.effect = LED_EFFECT_LED_ON(LED_COLOR(127, 255, 127)),
	},
};
