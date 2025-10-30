/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include "led_state_def.h"

#include <caf/events/led_event.h>
#include "ml_app_mode_event.h"
#include "ml_result_event.h"
#include "ei_data_forwarder_event.h"
#include "sensor_sim_event.h"

#define MODULE led_state
#include <caf/events/module_state_event.h>
#include <zephyr/bluetooth/bluetooth.h>
#include "string.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>   // for MIN()
#include <errno.h>             // for -EAGAIN/-EBUSY names (optional)
#include <string.h>            // use angle brackets is fine too
LOG_MODULE_REGISTER(MODULE, CONFIG_ML_APP_LED_STATE_LOG_LEVEL);

#define DISPLAY_ML_RESULTS		IS_ENABLED(CONFIG_ML_APP_ML_RESULT_EVENTS)
#define DISPLAY_SIM_SIGNAL		IS_ENABLED(CONFIG_ML_APP_SENSOR_SIM_EVENTS)
#define DISPLAY_DATA_FORWARDER		IS_ENABLED(CONFIG_ML_APP_EI_DATA_FORWARDER_EVENTS)

#define ANOMALY_THRESH			(CONFIG_ML_APP_LED_STATE_ANOMALY_THRESH / 1000.0)
#define VALUE_THRESH			(CONFIG_ML_APP_LED_STATE_VALUE_THRESH / 1000.0)
#define PREDICTION_STREAK_THRESH	CONFIG_ML_APP_LED_STATE_PREDICTION_STREAK_THRESH

BUILD_ASSERT(PREDICTION_STREAK_THRESH > 0);

#define DEFAULT_EFFECT			(&ml_result_led_effects[0])

static enum ml_app_mode ml_app_mode = ML_APP_MODE_COUNT;
static enum ei_data_forwarder_state forwarder_state = DISPLAY_DATA_FORWARDER ?
	EI_DATA_FORWARDER_STATE_DISCONNECTED : EI_DATA_FORWARDER_STATE_TRANSMITTING;

static const struct led_effect *blocking_led_effect;

static const char *cur_label;
static size_t prediction_streak;

/* Fixed fast non-connectable advertising (50 ms interval) */
static const struct bt_le_adv_param *adv_fast =
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY,
			0x0050, /* 50 ms */
			0x0050, /* 50 ms */
			NULL);

static bool ble_ready;
static bool adv_started;
static int64_t adv_last_update_ms;
#define ADV_MIN_UPDATE_MS 50
// add these just before K_WORK_DELAYABLE_DEFINE(...)
static void adv_retry_fn(struct k_work *w);
static void adv_start_or_update(const uint8_t *svc_data, size_t svc_len);

K_WORK_DELAYABLE_DEFINE(adv_retry_work, adv_retry_fn);
static uint8_t last_svc_buf[31];
static size_t last_svc_len;

//vinh
#define DEVICE_NAME1 "CPS22"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME1) - 1)

/* Advertising payload: Flags + FEAA UUID + FEAA service-data (unchanged format) */
static struct bt_data ad1[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
	BT_DATA_BYTES(BT_DATA_SVC_DATA16,
		      0xaa, 0xfe, /* Eddystone UUID */
		      0x10,       /* Eddystone-URL frame type */
		      0x00,       /* Calibrated Tx power at 0m */
		      0x00,       /* URL Scheme Prefix http://www. */
		      'z','e','p','h','y','r',
		      'p','r','o','j','e','c','t',
		      0x08)       /* .org */
};


//vinh2
//add timer counter

/* Fallback bring-up helpers */
static void start_initial_adv(void);
static void bt_ready_cb(int err);

static void start_initial_adv(void)
{
	static uint8_t init_adata[] = {
		0xaa, 0xfe, 0x10, 0x00, 0x00,
		'i','d','l','e',';','0',';','0',';','-','1'
	};
	adv_start_or_update(init_adata, sizeof(init_adata));
}

static void bt_ready_cb(int err)
{
	if (err) {
		LOG_ERR("bt_enable callback error: %d", err);
		return;
	}
	LOG_INF("Bluetooth ready (callback)");
	ble_ready = true;
	start_initial_adv();
}

static int led_ble_sysinit(const struct device *unused)
{
	ARG_UNUSED(unused);
	int err = bt_enable(bt_ready_cb);
	if (err == -EALREADY) {
		LOG_INF("Bluetooth already enabled (SYS_INIT)");
		ble_ready = true;
		start_initial_adv();
	} else if (err) {
		LOG_ERR("bt_enable (SYS_INIT) err=%d", err);
	}
	return 0;
}
SYS_INIT(led_ble_sysinit, APPLICATION, 50);



uint32_t get_rtc_counter(void)
{
    return NRF_RTC0->COUNTER;
}

//-----------

static void adv_retry_fn(struct k_work *w)
{
	LOG_DBG("adv_retry_fn: attempt to (re)start advertising");
	/* Re-attempt start/update with the last service data */
	adv_start_or_update(last_svc_buf, last_svc_len);
}

static void adv_start_or_update(const uint8_t *svc_data, size_t svc_len)
{
	struct bt_data svc = BT_DATA(BT_DATA_SVC_DATA16, svc_data, svc_len);

	/* cache payload for retries */
	size_t copy_len = MIN(svc_len, sizeof(last_svc_buf));
	memcpy(last_svc_buf, svc_data, copy_len);
	last_svc_len = copy_len;

	if (!ble_ready) {
		k_work_reschedule(&adv_retry_work, K_MSEC(150));
		LOG_DBG("BLE not ready yet; will retry advertising");
		return;
	}

	if (!adv_started) {
		ad1[2] = svc;
		int err = bt_le_adv_start(adv_fast, ad1, ARRAY_SIZE(ad1), NULL, 0);
		if (err) {
			if (err == -EAGAIN || err == -EBUSY) {
				k_work_reschedule(&adv_retry_work, K_MSEC(150));
				LOG_WRN("bt_le_adv_start busy/again (err=%d), will retry", err);
			} else {
				LOG_ERR("bt_le_adv_start err=%d", err);
			}
			return;
		}
		k_work_cancel_delayable(&adv_retry_work);
		LOG_INF("Advertising started (LEGACY NONCONN @ 50 ms), svc_len=%d", (int)svc_len);
		adv_started = true;
		adv_last_update_ms = k_uptime_get();
		return;
	}

	/* already advertising: try to update, else stop/start */
	int64_t now = k_uptime_get();
	if ((now - adv_last_update_ms) < ADV_MIN_UPDATE_MS) {
		return; /* throttle */
	}

	ad1[2] = svc;
	int err = bt_le_adv_update_data(ad1, ARRAY_SIZE(ad1), NULL, 0);
	if (!err) {
		adv_last_update_ms = now;
		return;
	}

	if (err == -EAGAIN || err == -EBUSY) {
		k_work_reschedule(&adv_retry_work, K_MSEC(150));
		LOG_DBG("bt_le_adv_update_data busy/again, will retry");
		return;
	}

	if (err == -ENOTSUP) {
		LOG_WRN("update_data not supported; restarting advertising");
		bt_le_adv_stop();
		adv_started = false;
		k_work_reschedule(&adv_retry_work, K_NO_WAIT);
		return;
	}

	LOG_ERR("bt_le_adv_update_data err=%d", err);
}






static bool is_led_effect_valid(const struct led_effect *le)
{
	const uint8_t zeros[sizeof(struct led_effect)] = {0};

	return memcmp(le, zeros, sizeof(struct led_effect));
}

static bool is_led_effect_blocking(const struct led_effect *le)
{
	return ((!le->loop_forever) && (le->step_count > 1));
}

static void clear_prediction(void)
{
	cur_label = NULL;
	prediction_streak = 0;
}

static void send_led_event(size_t led_id, const struct led_effect *led_effect)
{
	__ASSERT_NO_MSG(led_effect);
	__ASSERT_NO_MSG(led_id < LED_ID_COUNT);

	struct led_event *event = new_led_event();

	event->led_id = led_id;
	event->led_effect = led_effect;
	APP_EVENT_SUBMIT(event);
}

static const struct ml_result_led_effect *get_led_effect(const char *label)
{
	const struct ml_result_led_effect *result = DEFAULT_EFFECT;

	if (!label) {
		return result;
	}

	for (size_t i = 1; i < ARRAY_SIZE(ml_result_led_effects); i++) {
		const struct ml_result_led_effect *t = &ml_result_led_effects[i];

		if ((t->label == label) || !strcmp(t->label, label)) {
			result = t;
			break;
		}
	}

	return result;
}

static void ml_result_set_signin_state(bool state)
{
	struct ml_result_signin_event *event = new_ml_result_signin_event();

	event->module_idx = MODULE_IDX(MODULE);
	event->state = state;
	APP_EVENT_SUBMIT(event);
	LOG_INF("Currently %s result event", state ? "signed in" : "signed off from");
}

static void display_sensor_sim(const char *label)
{
	static const struct ml_result_led_effect *sensor_sim_effect;

	if (label) {
		sensor_sim_effect = get_led_effect(label);

		if (sensor_sim_effect == DEFAULT_EFFECT) {
			LOG_WRN("No LED effect for sensor_sim label %s", label);
		}
	}

	if (sensor_sim_effect) {
		__ASSERT_NO_MSG(!is_led_effect_blocking(&sensor_sim_effect->effect));
		send_led_event(led_map[LED_ID_SENSOR_SIM], &sensor_sim_effect->effect);
	}
}

static void display_ml_result(const char *label, bool force_update)
{
	__ASSERT_NO_MSG(ml_app_mode == ML_APP_MODE_MODEL_RUNNING);

	static const struct ml_result_led_effect *ml_result_effect;
	const struct ml_result_led_effect *new_effect = get_led_effect(label);

	/* Update not needed. */
	if ((ml_result_effect == new_effect) && !force_update) {
		return;
	}

	__ASSERT_NO_MSG(!force_update || !label);

	if (!force_update) {
		if (!label) {
			LOG_INF("Anomaly detected");
		} else if (new_effect == DEFAULT_EFFECT) {
			LOG_INF("No LED effect for label: %s", label);
		} else {
			LOG_INF("Displaying LED effect for label: %s", label);
		}
	}

	/* Synchronize LED effect displayed for simulated signal. */
	if (DISPLAY_SIM_SIGNAL && (new_effect != DEFAULT_EFFECT)) {
		display_sensor_sim(NULL);
	}

	ml_result_effect = new_effect;
	send_led_event(led_map[LED_ID_ML_STATE], &ml_result_effect->effect);

	if (is_led_effect_blocking(&ml_result_effect->effect)) {
		blocking_led_effect = &ml_result_effect->effect;
		ml_result_set_signin_state(false);
	} else {
		blocking_led_effect = NULL;
		ml_result_set_signin_state(true);
	}
}

//vinh

static void update_ml_result(const char *label, float value, float anomaly,
                             int dsptime, int classification_time, int anomaly_time)
{
    /* ------- Build your service-data payload (unchanged logic) ------- */
    static uint8_t adata[64] = {
        0xaa, 0xfe,       /* Eddystone UUID */
        0x10,             /* Eddystone-URL frame type */
        0x00,             /* Calibrated Tx power at 0m */
        0x00,             /* URL Scheme Prefix */
        'z','e','p','h','y','r','p','r','o','j','e','c','t','s',
        0x08              /* .org */
    };
    static uint8_t adatasize = 19;

    /* Compose "<label>;<d>;<c>;<a>" and CAP it so ADV payload stays <= 31 bytes. */
	char rs[40];
	char snum[16];
	rs[0] = '\0';
	if (label) { strncat(rs, label, sizeof(rs) - 1); }
	strncat(rs, ";", sizeof(rs) - 1);

	snprintf(snum, sizeof(snum), "%d", (int)dsptime);
	strncat(rs, snum, sizeof(rs) - 1);
	strncat(rs, ";", sizeof(rs) - 1);

	snprintf(snum, sizeof(snum), "%d", (int)classification_time);
	strncat(rs, snum, sizeof(rs) - 1);
	strncat(rs, ";", sizeof(rs) - 1);

	snprintf(snum, sizeof(snum), "%d", (int)anomaly_time);
	strncat(rs, snum, sizeof(rs) - 1);

	/* !! CRITICAL:  l <= 17 to guarantee 31-byte legacy ADV total size */
	const int RS_MAX_LEN = 17;
	const int l = MIN((int)strlen(rs), RS_MAX_LEN);

	for (int i = 0; i < l; i++) {
		adata[i + 5] = (uint8_t)rs[i];
	}
	adatasize = 5 + (uint8_t)l;


    /* ------- Classification acceptance & LED label selection ------- */
    /* Treat “no anomaly block” as anomaly = -1.0f (sentinel from ml_runner) */
    const bool anomaly_available = (anomaly >= 0.0f);

    const char *new_label = NULL;
    bool accept = false;

    if (anomaly_available && (anomaly > ANOMALY_THRESH)) {
        new_label = ANOMALY_LABEL;
        accept = true;
    } else if (value >= VALUE_THRESH) {
        new_label = label;
        accept = true;
    } else if (!anomaly_available) {
        /* Requested behavior: without anomaly, low-confidence → idle/unknown. */
        new_label = "idle";         /* or set to NULL to use the default effect */
        accept = true;
    } else {
        /* Low confidence with anomaly available: ignore this frame. */
        accept = false;
    }

    if (new_label != cur_label) {
        if ((!new_label || !cur_label) || strcmp(new_label, cur_label)) {
            cur_label = new_label;
            prediction_streak = 0;
        }
    }

    if (accept) {
        prediction_streak++;
    }

    if (prediction_streak >= PREDICTION_STREAK_THRESH) {
        display_ml_result(cur_label, false);
        clear_prediction();
    }

	adv_start_or_update(adata, adatasize);
}


static void validate_configuration(void)
{
	BUILD_ASSERT(ARRAY_SIZE(ml_result_led_effects) >= 1);
	__ASSERT_NO_MSG(!is_led_effect_blocking(&DEFAULT_EFFECT->effect));
	__ASSERT_NO_MSG(!DEFAULT_EFFECT->label);

	size_t anomaly_label_cnt = 0;

	for (size_t i = 1; i < ARRAY_SIZE(ml_result_led_effects); i++) {
		const struct ml_result_led_effect *t = &ml_result_led_effects[i];

		__ASSERT_NO_MSG(is_led_effect_valid(&t->effect));
		__ASSERT_NO_MSG(t->label);

		if (!strcmp(t->label, ANOMALY_LABEL)) {
			anomaly_label_cnt++;
		}
	}

	__ASSERT_NO_MSG(anomaly_label_cnt <= 1);
}

static bool handle_ml_result_event(const struct ml_result_event *event)
{
	if ((ml_app_mode == ML_APP_MODE_MODEL_RUNNING) && !blocking_led_effect) {
		//vinh 
		//update_ml_result(event->label, event->value, event->anomaly);
		update_ml_result(event->label, event->value,event->anomaly,event->dsp_time,event->classification_time,event->anomaly_time);
	}

	return false;
}

static bool handle_sensor_sim_event(const struct sensor_sim_event *event)
{
	display_sensor_sim(event->label);

	return false;
}

static bool handle_ei_data_forwarder_event(const struct ei_data_forwarder_event *event)
{
	__ASSERT_NO_MSG(event->state != EI_DATA_FORWARDER_STATE_DISABLED);
	forwarder_state = event->state;

	__ASSERT_NO_MSG(is_led_effect_valid(&ei_data_forwarder_led_effects[forwarder_state]));

	if (ml_app_mode == ML_APP_MODE_DATA_FORWARDING) {
		send_led_event(led_map[LED_ID_ML_STATE],
			       &ei_data_forwarder_led_effects[forwarder_state]);
	}

	return false;
}

static bool handle_led_ready_event(const struct led_ready_event *event)
{
	if ((event->led_id == led_map[LED_ID_ML_STATE]) &&
	    (ml_app_mode == ML_APP_MODE_MODEL_RUNNING) &&
	    (blocking_led_effect == event->led_effect)) {
		display_ml_result(NULL, true);
	}

	return false;
}

static bool handle_ml_app_mode_event(const struct ml_app_mode_event *event)
{
	ml_app_mode = event->mode;

	if (event->mode == ML_APP_MODE_MODEL_RUNNING) {
		clear_prediction();
		display_ml_result(NULL, true);
	} else if (event->mode == ML_APP_MODE_DATA_FORWARDING) {
		send_led_event(led_map[LED_ID_ML_STATE],
			       &ei_data_forwarder_led_effects[forwarder_state]);
	} else {
		/* Not supported. */
		__ASSERT_NO_MSG(false);
	}

	return false;
}

static bool handle_module_state_event(const struct module_state_event *event)
{
#ifdef CONFIG_CAF_BLE_STATE
	if (check_state(event, MODULE_ID(ble_state), MODULE_STATE_READY)) {
		LOG_INF("ble_state READY");
		ble_ready = true;
		start_initial_adv();
		return false;
	}
#endif

	if (check_state(event, MODULE_ID(main), MODULE_STATE_READY)) {
		if (IS_ENABLED(CONFIG_ASSERT)) {
			validate_configuration();
		} else {
			ARG_UNUSED(is_led_effect_valid);
		}

		static bool initialized;
		__ASSERT_NO_MSG(!initialized);
		module_set_state(MODULE_STATE_READY);
		initialized = true;

		/* tell ml_runner we listen for results (so it starts running) */
		ml_result_set_signin_state(true);

		/* If BT wasn’t enabled yet by SYS_INIT (rare), try here too */
		if (!ble_ready) {
			int err = bt_enable(bt_ready_cb);
			if (err == -EALREADY) {
				LOG_INF("Bluetooth already enabled (module_state)");
				ble_ready = true;
				start_initial_adv();
			} else if (err) {
				LOG_ERR("bt_enable (module_state) err=%d", err);
			}
		}
		return false;
	}

	return false;
}



static bool app_event_handler(const struct app_event_header *aeh)
{
	if (DISPLAY_ML_RESULTS &&
	    is_ml_result_event(aeh)) {
		return handle_ml_result_event(cast_ml_result_event(aeh));
	}

	if (DISPLAY_SIM_SIGNAL &&
	    is_sensor_sim_event(aeh)) {
		return handle_sensor_sim_event(cast_sensor_sim_event(aeh));
	}

	if (DISPLAY_DATA_FORWARDER &&
	    is_ei_data_forwarder_event(aeh)) {
		return handle_ei_data_forwarder_event(cast_ei_data_forwarder_event(aeh));
	}

	if (is_led_ready_event(aeh)) {
		return handle_led_ready_event(cast_led_ready_event(aeh));
	}

	if (is_ml_app_mode_event(aeh)) {
		return handle_ml_app_mode_event(cast_ml_app_mode_event(aeh));
	}

	if (is_module_state_event(aeh)) {
		return handle_module_state_event(cast_module_state_event(aeh));
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, ml_app_mode_event);
APP_EVENT_SUBSCRIBE(MODULE, led_ready_event);
#if DISPLAY_DATA_FORWARDER
APP_EVENT_SUBSCRIBE(MODULE, ei_data_forwarder_event);
#endif /* DISPLAY_DATA_FORWARDER */
#if DISPLAY_ML_RESULTS
APP_EVENT_SUBSCRIBE(MODULE, ml_result_event);
#endif /* DISPLAY_ML_RESULTS */
#if DISPLAY_SIM_SIGNAL
APP_EVENT_SUBSCRIBE(MODULE, sensor_sim_event);
#endif /* DISPLAY_SIM_SIGNAL */
