#include "obd/oil.h"

#define OIL_IDLE_RPM   800.0f
#define OIL_COLD_IDLE  3.5f
#define OIL_COLD_HI    4.6f
#define OIL_COLD_RPM   2000.0f
#define OIL_HOT_IDLE   1.0f
#define OIL_HOT_HI     4.5f
#define OIL_HOT_RPM    5000.0f
#define OIL_WARM_LO    80.0f
#define OIL_WARM_HI    90.0f

static float oil_bar_at(float rpm, float idle_bar, float hi_rpm,
                        float hi_bar)
{
    if (rpm <= OIL_IDLE_RPM) {
        return idle_bar;
    }
    if (rpm >= hi_rpm) {
        return hi_bar;
    }
    return idle_bar + (hi_bar - idle_bar) * (rpm - OIL_IDLE_RPM) /
                          (hi_rpm - OIL_IDLE_RPM);
}

float obd_oil_bar(float rpm, float water_c, int water_known)
{
    if (rpm < 0.0f) {
        rpm = 0.0f;
    }
    float cold = oil_bar_at(rpm, OIL_COLD_IDLE, OIL_COLD_RPM, OIL_COLD_HI);
    float hot = oil_bar_at(rpm, OIL_HOT_IDLE, OIL_HOT_RPM, OIL_HOT_HI);
    if (!water_known || water_c <= OIL_WARM_LO) {
        return cold;
    }
    if (water_c >= OIL_WARM_HI) {
        return hot;
    }
    float t = (water_c - OIL_WARM_LO) / (OIL_WARM_HI - OIL_WARM_LO);
    return cold + (hot - cold) * t;
}
