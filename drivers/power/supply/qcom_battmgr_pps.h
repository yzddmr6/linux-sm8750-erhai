/* SPDX-License-Identifier: GPL-2.0-only */
/* OPD2413 PPS policy. Curve data: Oplus oplus-chg-24924.dtsi (GPL-2.0-only). */
#ifndef __QCOM_BATTMGR_PPS_H
#define __QCOM_BATTMGR_PPS_H

#define OPD_PPS_MAX_MA		4500
#define OPD_PPS_MAX_MW		38000
#define OPD_PPS_IBAT_MAX_MA	7400
#define OPD_PPS_SOFT_FULL_MV	4490
#define OPD_PPS_HARD_FULL_MV	4540
#define OPD_PPS_SOC_STOP		90
#define OPD_PPS_START_MV		5500
#define OPD_PPS_START_MA		1000
/* Stock 10 V VBUS OVP, less the vendor servo's 500 mV headroom. */
#define OPD_PPS_MAX_MV		9500
#define OPD_PPS_POLL_MS		500
/*
 * Let the curve forget transients. The track only ever descends, so one
 * sampling spike on the pack node would otherwise pin the whole session
 * to a lower current band. 20 mV per poll is far slower than real cell
 * rise (tens of mV per minute) and far faster than a switching spike.
 */
#define OPD_PPS_PEAK_DECAY_MV	20
#define OPD_PPS_START_DELAY_MS	500
#define OPD_PPS_TIMEOUT_S	30188
#define OPD_PPS_SLAVE_ON_MA	1900
#define OPD_PPS_SLAVE_OFF_MA	1600
/*
 * Stock uses battery temperature unconditionally through 32 C. Above this,
 * oplus_comm_override_by_shell_temp() can select the software shell_back
 * estimate, which is not available here. Stay in the known temperature range
 * instead of substituting an unrelated thermistor for that estimate.
 */
#define OPD_PPS_TEMP_MAX		320
#define OPD_PPS_TEMP_MIN		50

struct opd_pps_point {
	int vbat_mv;
	int ibus_ma;
};

/* All six SOC bands and seven temperature bands of the third-party curve. */
static const struct opd_pps_point opd_pps_curves[][6] = {
	{
		{ 4150, 3800 }, { 4480, 2650 }, { 4480, 2500 },
		{ 4480, 2000 }, { 4480, 1500 }, { 4490, 1000 }
	},
	{
		{ 4150, 3800 }, { 4200, 2950 }, { 4480, 2500 },
		{ 4480, 2000 }, { 4480, 1500 }, { 4490, 1000 }
	},
	{
		{ 4380, 3800 }, { 4480, 2950 }, { 4480, 2500 },
		{ 4480, 2000 }, { 4480, 1500 }, { 4490, 1000 }
	},
	{
		{ 4380, 3800 }, { 4460, 2950 }, { 4460, 2500 },
		{ 4460, 2000 }, { 4460, 1500 }, { 4470, 1000 }
	},
	{
		{ 4120, 3800 }, { 4120, 2500 }, { 4120, 2000 }, { 4120, 1500 }, { 4130, 1000 }
	},
	{ { 4480, 2000 }, { 4480, 1500 }, { 4490, 1000 } },
	{ { 4480, 2500 }, { 4480, 2000 }, { 4480, 1500 }, { 4490, 1000 } },
	{ { 4480, 1500 }, { 4490, 1000 } },
	{ { 4130, 1000 } },
	{ { 4490, 1000 } },
	{ { 4460, 2500 }, { 4460, 2000 }, { 4460, 1500 }, { 4470, 1000 } },
	{ { 4460, 1500 }, { 4470, 1000 } },
};

static const unsigned char opd_pps_curve_index[6][7] = {
	{ 0, 1, 1, 2, 2, 3, 4 },
	{ 0, 1, 1, 2, 2, 3, 4 },
	{ 5, 6, 1, 2, 2, 3, 4 },
	{ 7, 7, 5, 5, 6, 3, 8 },
	{ 9, 9, 7, 7, 5, 10, 8 },
	{ 9, 9, 9, 9, 7, 11, 8 },
};

static inline int opd_pps_min(int a, int b)
{
	return a < b ? a : b;
}

/* Zero means leave PPS. Advancing at a voltage boundary is conservative. */
static inline int opd_pps_curve_limit(int soc, int temp, int vbat_mv)
{
	static const int soc_max[] = { 20, 35, 55, 75, 85, 90 };
	static const int temp_max[] = { 50, 120, 160, 210, 350, 440, 510 };
	const struct opd_pps_point *curve;
	unsigned int s = 0, t = 0, i;

	if (soc < 0 || soc >= OPD_PPS_SOC_STOP || temp < 0 || temp >= 510 || vbat_mv <= 0)
		return 0;
	while (soc > soc_max[s])
		s++;
	while (temp >= temp_max[t])
		t++;
	curve = opd_pps_curves[opd_pps_curve_index[s][t]];
	for (i = 0; i < 6; i++)
		if (curve[i].vbat_mv && vbat_mv < curve[i].vbat_mv)
			return curve[i].ibus_ma;
	return 0;
}

/* A session may only descend the curve, including after a temperature change. */
static inline int opd_pps_current(int soc, int temp, int vbat_mv, int adapter_ma,
				  int previous_ceiling_ma)
{
	int ma = opd_pps_curve_limit(soc, temp, vbat_mv);

	if (adapter_ma <= 0 || previous_ceiling_ma <= 0)
		return 0;
	ma = opd_pps_min(ma, opd_pps_min(OPD_PPS_MAX_MA, adapter_ma));
	ma = opd_pps_min(ma, previous_ceiling_ma);
	/* PPS current requests have 50 mA granularity. */
	return ma / 50 * 50;
}

/* Vendor current_work steps up every 500 ms. Lower limits apply immediately. */
static inline int opd_pps_step_ma(int previous_ma, int target_ma)
{
	int delta, step;

	if (previous_ma <= 0 || previous_ma > OPD_PPS_MAX_MA ||
	    target_ma <= 0 || target_ma > OPD_PPS_MAX_MA)
		return 0;
	delta = target_ma - previous_ma;
	if (delta <= 0)
		return target_ma;
	step = delta >= 500 ? 500 : delta >= 300 ? 300 : delta >= 100 ? 100 : 50;
	return opd_pps_min(previous_ma + step, target_ma);
}

enum opd_pps_exit {
	OPD_PPS_OK,
	OPD_PPS_INVALID,
	OPD_PPS_TEMPERATURE,
	OPD_PPS_SOC,
	OPD_PPS_FULL,
	OPD_PPS_BATTERY_CURRENT,
	OPD_PPS_INPUT_CURRENT,
	OPD_PPS_INPUT_VOLTAGE,
	OPD_PPS_POWER,
};

static inline enum opd_pps_exit opd_pps_check(int soc, int temp, int vbat_mv,
					    int ibat_ma, int vbus_mv, int ibus_ma,
					    int soc_limit)
{
	if (soc < 0 || soc > 100 || vbat_mv <= 0 || vbus_mv <= 0 || ibus_ma < 0)
		return OPD_PPS_INVALID;
	if (temp < OPD_PPS_TEMP_MIN || temp > OPD_PPS_TEMP_MAX)
		return OPD_PPS_TEMPERATURE;
	if (soc >= opd_pps_min(soc_limit, OPD_PPS_SOC_STOP))
		return OPD_PPS_SOC;
	if (vbat_mv >= OPD_PPS_SOFT_FULL_MV)
		return OPD_PPS_FULL;
	if (ibat_ma >= OPD_PPS_IBAT_MAX_MA)
		return OPD_PPS_BATTERY_CURRENT;
	if (ibus_ma > OPD_PPS_MAX_MA)
		return OPD_PPS_INPUT_CURRENT;
	if (vbus_mv >= 10000)
		return OPD_PPS_INPUT_VOLTAGE;
	if (vbus_mv * ibus_ma > OPD_PPS_MAX_MW * 1000)
		return OPD_PPS_POWER;
	return OPD_PPS_OK;
}

static inline int opd_pps_entry_mv(int vout_mv)
{
	int target;

	if (vout_mv <= 0 || vout_mv >= OPD_PPS_HARD_FULL_MV)
		return 0;
	target = 2 * vout_mv / 100 * 100 + 450;
	return target >= OPD_PPS_START_MV && target <= OPD_PPS_MAX_MV ? target : 0;
}

static inline int opd_pps_ramp_mv(int request_mv, int measured_mv, int target_mv)
{
	int delta = measured_mv - target_mv;
	int step;

	if (request_mv < OPD_PPS_START_MV || request_mv > OPD_PPS_MAX_MV ||
	    measured_mv <= 0 || target_mv < OPD_PPS_START_MV || target_mv > OPD_PPS_MAX_MV)
		return 0;
	if (delta < 0)
		delta = -delta;
	step = delta >= 2000 ? 2000 : delta >= 1000 ? 1000 :
	       delta >= 500 ? 500 : delta >= 200 ? 200 : 100;
	request_mv += measured_mv < target_mv ? step : -step;
	if (request_mv < OPD_PPS_START_MV || request_mv > OPD_PPS_MAX_MV)
		return 0;
	return request_mv;
}

static inline int opd_pps_servo_mv(int request_mv, int vbat_mv, int ibus_ma, int target_ma)
{
	int min_mv, next = request_mv;

	if (vbat_mv <= 0 || vbat_mv >= OPD_PPS_SOFT_FULL_MV || ibus_ma < 0 ||
	    ibus_ma > OPD_PPS_MAX_MA || target_ma <= 0 || target_ma > OPD_PPS_MAX_MA ||
	    request_mv < OPD_PPS_START_MV || request_mv > OPD_PPS_MAX_MV)
		return 0;
	min_mv = (2 * vbat_mv + 100 + 19) / 20 * 20;
	if (ibus_ma > target_ma + 500)
		next -= 100;
	else if (ibus_ma > target_ma + 100)
		next -= 40;
	else if (ibus_ma >= 300 && ibus_ma < target_ma - 100)
		next += 40;
	/* Do not increase voltage on a disconnected/failed pump (<300 mA). */
	if (next < min_mv)
		return 0;
	return opd_pps_min(next, OPD_PPS_MAX_MV);
}

#endif
