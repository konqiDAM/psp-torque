#include "obd/dtc.h"

#include <stddef.h>
#include <string.h>

static char hex_digit(int value)
{
    if (value < 10) {
        return (char)('0' + value);
    }
    return (char)('A' + (value - 10));
}

/* ISO 15031-6: byte 1 carries the letter in bits 7..6 and digit 1 in
 * bits 5..4, masked to two bits, plus digit 2 in bits 3..0. Byte 2 is
 * digits 3 and 4. */
static void format_code(const uint8_t *pair, char *dst)
{
    static const char letters[4] = { 'P', 'C', 'B', 'U' };
    uint8_t hi = pair[0];
    uint8_t lo = pair[1];

    dst[0] = letters[(hi >> 6) & 0x3];
    dst[1] = hex_digit((hi >> 4) & 0x3);
    dst[2] = hex_digit(hi & 0xF);
    dst[3] = hex_digit((lo >> 4) & 0xF);
    dst[4] = hex_digit(lo & 0xF);
    dst[5] = '\0';
}

static int decode_pairs(const uint8_t *pairs, int count, obd_dtc *out,
                        int max)
{
    int written = 0;

    for (int i = 0; i < count; i++) {
        const uint8_t *pair = pairs + i * 2;
        /* An all-zero pair is frame padding, not a code. */
        if (pair[0] == 0 && pair[1] == 0) {
            continue;
        }
        if (written >= max) {
            break;
        }
        format_code(pair, out[written].code);
        written++;
    }

    return written;
}

int obd_dtc_parse_ex(const uint8_t *payload, int len, int has_count,
                     obd_dtc *out, int max)
{
    if (payload == NULL || out == NULL || len < 0 || max < 0) {
        return -1;
    }
    if (len == 0) {
        return 0;
    }

    if (!has_count) {
        /* Even-length answer: codes from the first byte, a lone trailing
         * byte is a truncated pair and cannot be decoded. */
        return decode_pairs(payload, len / 2, out, max);
    }

    /* payload[0] promises N codes; if the payload holds fewer complete
     * pairs, those are still real codes and are decoded. */
    int available = (len - 1) / 2;
    int declared = payload[0];
    int count = (declared < available) ? declared : available;
    return decode_pairs(payload + 1, count, out, max);
}

int obd_dtc_parse(const uint8_t *payload, int len, obd_dtc *out, int max)
{
    if (payload == NULL || out == NULL || len < 0 || max < 0) {
        return -1;
    }
    /* Odd length means one count byte plus whole pairs; even length has
     * no count byte. */
    if ((len % 2) != 0) {
        return obd_dtc_parse_ex(payload, len, 1, out, max);
    }
    return obd_dtc_parse_ex(payload, len, 0, out, max);
}

/* Generic SAE J2012 meanings, P0xxx only: the range a petrol car of the
 * 1990s and 2000s stores. Unknown or manufacturer codes are not listed,
 * so obd_dtc_describe() returns NULL and the screen shows the code
 * alone. Kept sorted by code for reading; the lookup is linear because
 * a few hundred entries cost nothing at one screen refresh. */
typedef struct {
    const char *code;
    const char *text;
} dtc_entry;

static const dtc_entry g_dtc_table[] = {
    { "P0100", "Mass air flow circuit malfunction" },
    { "P0101", "Mass air flow circuit range/performance" },
    { "P0102", "Mass air flow circuit low input" },
    { "P0103", "Mass air flow circuit high input" },
    { "P0104", "Mass air flow circuit intermittent" },
    { "P0105", "MAP/baro circuit malfunction" },
    { "P0106", "MAP/baro circuit range/performance" },
    { "P0107", "MAP/baro circuit low input" },
    { "P0108", "MAP/baro circuit high input" },
    { "P0109", "MAP/baro circuit intermittent" },
    { "P0110", "Intake air temperature circuit malfunction" },
    { "P0111", "Intake air temperature range/performance" },
    { "P0112", "Intake air temperature circuit low input" },
    { "P0113", "Intake air temperature circuit high input" },
    { "P0114", "Intake air temperature circuit intermittent" },
    { "P0115", "Coolant temperature circuit malfunction" },
    { "P0116", "Coolant temperature range/performance" },
    { "P0117", "Coolant temperature circuit low input" },
    { "P0118", "Coolant temperature circuit high input" },
    { "P0119", "Coolant temperature circuit intermittent" },
    { "P0120", "Throttle position sensor A circuit" },
    { "P0121", "Throttle position sensor A range/performance" },
    { "P0122", "Throttle position sensor A circuit low" },
    { "P0123", "Throttle position sensor A circuit high" },
    { "P0124", "Throttle position sensor A intermittent" },
    { "P0125", "Coolant too cold for closed loop control" },
    { "P0126", "Coolant too cold for stable operation" },
    { "P0127", "Intake air temperature too high" },
    { "P0128", "Thermostat below regulating temperature" },
    { "P0130", "O2 sensor circuit (bank 1 sensor 1)" },
    { "P0131", "O2 sensor low voltage (bank 1 sensor 1)" },
    { "P0132", "O2 sensor high voltage (bank 1 sensor 1)" },
    { "P0133", "O2 sensor slow response (bank 1 sensor 1)" },
    { "P0134", "O2 sensor no activity (bank 1 sensor 1)" },
    { "P0135", "O2 sensor heater circuit (bank 1 sensor 1)" },
    { "P0136", "O2 sensor circuit (bank 1 sensor 2)" },
    { "P0137", "O2 sensor low voltage (bank 1 sensor 2)" },
    { "P0138", "O2 sensor high voltage (bank 1 sensor 2)" },
    { "P0139", "O2 sensor slow response (bank 1 sensor 2)" },
    { "P0140", "O2 sensor no activity (bank 1 sensor 2)" },
    { "P0141", "O2 sensor heater circuit (bank 1 sensor 2)" },
    { "P0142", "O2 sensor circuit (bank 1 sensor 3)" },
    { "P0143", "O2 sensor low voltage (bank 1 sensor 3)" },
    { "P0144", "O2 sensor high voltage (bank 1 sensor 3)" },
    { "P0145", "O2 sensor slow response (bank 1 sensor 3)" },
    { "P0146", "O2 sensor no activity (bank 1 sensor 3)" },
    { "P0147", "O2 sensor heater circuit (bank 1 sensor 3)" },
    { "P0150", "O2 sensor circuit (bank 2 sensor 1)" },
    { "P0151", "O2 sensor low voltage (bank 2 sensor 1)" },
    { "P0152", "O2 sensor high voltage (bank 2 sensor 1)" },
    { "P0153", "O2 sensor slow response (bank 2 sensor 1)" },
    { "P0154", "O2 sensor no activity (bank 2 sensor 1)" },
    { "P0155", "O2 sensor heater circuit (bank 2 sensor 1)" },
    { "P0156", "O2 sensor circuit (bank 2 sensor 2)" },
    { "P0157", "O2 sensor low voltage (bank 2 sensor 2)" },
    { "P0158", "O2 sensor high voltage (bank 2 sensor 2)" },
    { "P0159", "O2 sensor slow response (bank 2 sensor 2)" },
    { "P0160", "O2 sensor no activity (bank 2 sensor 2)" },
    { "P0161", "O2 sensor heater circuit (bank 2 sensor 2)" },
    { "P0162", "O2 sensor circuit (bank 2 sensor 3)" },
    { "P0163", "O2 sensor low voltage (bank 2 sensor 3)" },
    { "P0164", "O2 sensor high voltage (bank 2 sensor 3)" },
    { "P0165", "O2 sensor slow response (bank 2 sensor 3)" },
    { "P0166", "O2 sensor no activity (bank 2 sensor 3)" },
    { "P0167", "O2 sensor heater circuit (bank 2 sensor 3)" },
    { "P0170", "Fuel trim malfunction (bank 1)" },
    { "P0171", "System too lean (bank 1)" },
    { "P0172", "System too rich (bank 1)" },
    { "P0173", "Fuel trim malfunction (bank 2)" },
    { "P0174", "System too lean (bank 2)" },
    { "P0175", "System too rich (bank 2)" },
    { "P0176", "Fuel composition sensor circuit" },
    { "P0177", "Fuel composition sensor range/performance" },
    { "P0178", "Fuel composition sensor circuit low" },
    { "P0179", "Fuel composition sensor circuit high" },
    { "P0180", "Fuel temperature sensor A circuit" },
    { "P0181", "Fuel temperature sensor A range/performance" },
    { "P0182", "Fuel temperature sensor A circuit low" },
    { "P0183", "Fuel temperature sensor A circuit high" },
    { "P0184", "Fuel temperature sensor A intermittent" },
    { "P0185", "Fuel temperature sensor B circuit" },
    { "P0186", "Fuel temperature sensor B range/performance" },
    { "P0187", "Fuel temperature sensor B circuit low" },
    { "P0188", "Fuel temperature sensor B circuit high" },
    { "P0189", "Fuel temperature sensor B intermittent" },
    { "P0190", "Fuel rail pressure sensor circuit" },
    { "P0191", "Fuel rail pressure sensor range/performance" },
    { "P0192", "Fuel rail pressure sensor circuit low" },
    { "P0193", "Fuel rail pressure sensor circuit high" },
    { "P0194", "Fuel rail pressure sensor intermittent" },
    { "P0195", "Engine oil temperature sensor circuit" },
    { "P0196", "Engine oil temperature sensor range/performance" },
    { "P0197", "Engine oil temperature sensor low" },
    { "P0198", "Engine oil temperature sensor high" },
    { "P0199", "Engine oil temperature sensor intermittent" },
    { "P0200", "Fuel injector circuit malfunction" },
    { "P0201", "Fuel injector circuit cylinder 1" },
    { "P0202", "Fuel injector circuit cylinder 2" },
    { "P0203", "Fuel injector circuit cylinder 3" },
    { "P0204", "Fuel injector circuit cylinder 4" },
    { "P0205", "Fuel injector circuit cylinder 5" },
    { "P0206", "Fuel injector circuit cylinder 6" },
    { "P0207", "Fuel injector circuit cylinder 7" },
    { "P0208", "Fuel injector circuit cylinder 8" },
    { "P0209", "Fuel injector circuit cylinder 9" },
    { "P0210", "Fuel injector circuit cylinder 10" },
    { "P0211", "Fuel injector circuit cylinder 11" },
    { "P0212", "Fuel injector circuit cylinder 12" },
    { "P0213", "Cold start injector 1 circuit" },
    { "P0214", "Cold start injector 2 circuit" },
    { "P0215", "Engine shutoff solenoid circuit" },
    { "P0216", "Injection timing control circuit" },
    { "P0217", "Engine over temperature" },
    { "P0218", "Transmission over temperature" },
    { "P0219", "Engine overspeed condition" },
    { "P0220", "Throttle position sensor B circuit" },
    { "P0221", "Throttle position sensor B range/performance" },
    { "P0222", "Throttle position sensor B circuit low" },
    { "P0223", "Throttle position sensor B circuit high" },
    { "P0224", "Throttle position sensor B intermittent" },
    { "P0225", "Throttle position sensor C circuit" },
    { "P0226", "Throttle position sensor C range/performance" },
    { "P0227", "Throttle position sensor C circuit low" },
    { "P0228", "Throttle position sensor C circuit high" },
    { "P0229", "Throttle position sensor C intermittent" },
    { "P0230", "Fuel pump primary circuit" },
    { "P0231", "Fuel pump secondary circuit low" },
    { "P0232", "Fuel pump secondary circuit high" },
    { "P0233", "Fuel pump secondary circuit intermittent" },
    { "P0234", "Turbo/supercharger overboost" },
    { "P0235", "Turbo boost sensor A circuit" },
    { "P0236", "Turbo boost sensor A range/performance" },
    { "P0237", "Turbo boost sensor A circuit low" },
    { "P0238", "Turbo boost sensor A circuit high" },
    { "P0239", "Turbo boost sensor B circuit" },
    { "P0240", "Turbo boost sensor B range/performance" },
    { "P0241", "Turbo boost sensor B circuit low" },
    { "P0242", "Turbo boost sensor B circuit high" },
    { "P0243", "Turbo wastegate solenoid A circuit" },
    { "P0244", "Turbo wastegate solenoid A range/performance" },
    { "P0245", "Turbo wastegate solenoid A circuit low" },
    { "P0246", "Turbo wastegate solenoid A circuit high" },
    { "P0247", "Turbo wastegate solenoid B circuit" },
    { "P0248", "Turbo wastegate solenoid B range/performance" },
    { "P0249", "Turbo wastegate solenoid B circuit low" },
    { "P0250", "Turbo wastegate solenoid B circuit high" },
    { "P0251", "Injection pump metering control A" },
    { "P0252", "Injection pump metering control A range/performance" },
    { "P0253", "Injection pump metering control A circuit low" },
    { "P0254", "Injection pump metering control A circuit high" },
    { "P0255", "Injection pump metering control A intermittent" },
    { "P0256", "Injection pump metering control B" },
    { "P0257", "Injection pump metering control B range/performance" },
    { "P0258", "Injection pump metering control B circuit low" },
    { "P0259", "Injection pump metering control B circuit high" },
    { "P0260", "Injection pump metering control B intermittent" },
    { "P0261", "Cylinder 1 injector circuit low" },
    { "P0262", "Cylinder 1 injector circuit high" },
    { "P0263", "Cylinder 1 contribution/balance" },
    { "P0264", "Cylinder 2 injector circuit low" },
    { "P0265", "Cylinder 2 injector circuit high" },
    { "P0266", "Cylinder 2 contribution/balance" },
    { "P0267", "Cylinder 3 injector circuit low" },
    { "P0268", "Cylinder 3 injector circuit high" },
    { "P0269", "Cylinder 3 contribution/balance" },
    { "P0270", "Cylinder 4 injector circuit low" },
    { "P0271", "Cylinder 4 injector circuit high" },
    { "P0272", "Cylinder 4 contribution/balance" },
    { "P0273", "Cylinder 5 injector circuit low" },
    { "P0274", "Cylinder 5 injector circuit high" },
    { "P0275", "Cylinder 5 contribution/balance" },
    { "P0276", "Cylinder 6 injector circuit low" },
    { "P0277", "Cylinder 6 injector circuit high" },
    { "P0278", "Cylinder 6 contribution/balance" },
    { "P0279", "Cylinder 7 injector circuit low" },
    { "P0280", "Cylinder 7 injector circuit high" },
    { "P0281", "Cylinder 7 contribution/balance" },
    { "P0282", "Cylinder 8 injector circuit low" },
    { "P0283", "Cylinder 8 injector circuit high" },
    { "P0284", "Cylinder 8 contribution/balance" },
    { "P0285", "Cylinder 9 injector circuit low" },
    { "P0286", "Cylinder 9 injector circuit high" },
    { "P0287", "Cylinder 9 contribution/balance" },
    { "P0288", "Cylinder 10 injector circuit low" },
    { "P0289", "Cylinder 10 injector circuit high" },
    { "P0290", "Cylinder 10 contribution/balance" },
    { "P0291", "Cylinder 11 injector circuit low" },
    { "P0292", "Cylinder 11 injector circuit high" },
    { "P0293", "Cylinder 11 contribution/balance" },
    { "P0294", "Cylinder 12 injector circuit low" },
    { "P0295", "Cylinder 12 injector circuit high" },
    { "P0296", "Cylinder 12 contribution/balance" },
    { "P0299", "Turbo/supercharger underboost" },
    { "P0300", "Random/multiple cylinder misfire" },
    { "P0301", "Cylinder 1 misfire detected" },
    { "P0302", "Cylinder 2 misfire detected" },
    { "P0303", "Cylinder 3 misfire detected" },
    { "P0304", "Cylinder 4 misfire detected" },
    { "P0305", "Cylinder 5 misfire detected" },
    { "P0306", "Cylinder 6 misfire detected" },
    { "P0307", "Cylinder 7 misfire detected" },
    { "P0308", "Cylinder 8 misfire detected" },
    { "P0309", "Cylinder 9 misfire detected" },
    { "P0310", "Cylinder 10 misfire detected" },
    { "P0311", "Cylinder 11 misfire detected" },
    { "P0312", "Cylinder 12 misfire detected" },
    { "P0313", "Misfire with low fuel level" },
    { "P0314", "Single cylinder misfire, cylinder unknown" },
    { "P0315", "Crankshaft position variation not learned" },
    { "P0316", "Misfire on start-up (first 1000 revs)" },
    { "P0320", "Ignition/engine speed input circuit" },
    { "P0321", "Ignition/engine speed range/performance" },
    { "P0322", "Ignition/engine speed input no signal" },
    { "P0323", "Ignition/engine speed input intermittent" },
    { "P0324", "Knock control system error" },
    { "P0325", "Knock sensor 1 circuit (bank 1 or single)" },
    { "P0326", "Knock sensor 1 range/performance" },
    { "P0327", "Knock sensor 1 circuit low" },
    { "P0328", "Knock sensor 1 circuit high" },
    { "P0329", "Knock sensor 1 circuit intermittent" },
    { "P0330", "Knock sensor 2 circuit (bank 2)" },
    { "P0331", "Knock sensor 2 range/performance" },
    { "P0332", "Knock sensor 2 circuit low" },
    { "P0333", "Knock sensor 2 circuit high" },
    { "P0334", "Knock sensor 2 circuit intermittent" },
    { "P0335", "Crankshaft position sensor A circuit" },
    { "P0336", "Crankshaft position sensor A range/performance" },
    { "P0337", "Crankshaft position sensor A no signal" },
    { "P0338", "Crankshaft position sensor A circuit high" },
    { "P0339", "Crankshaft position sensor A intermittent" },
    { "P0340", "Camshaft position sensor A circuit (bank 1)" },
    { "P0341", "Camshaft position sensor A range/performance" },
    { "P0342", "Camshaft position sensor A circuit low" },
    { "P0343", "Camshaft position sensor A circuit high" },
    { "P0344", "Camshaft position sensor A intermittent" },
    { "P0345", "Camshaft position sensor A circuit (bank 2)" },
    { "P0346", "Camshaft position sensor A range/perf (bank 2)" },
    { "P0347", "Camshaft position sensor A low (bank 2)" },
    { "P0348", "Camshaft position sensor A high (bank 2)" },
    { "P0349", "Camshaft position sensor A intermittent (bank 2)" },
    { "P0350", "Ignition coil primary/secondary circuit" },
    { "P0351", "Ignition coil A primary/secondary circuit" },
    { "P0352", "Ignition coil B primary/secondary circuit" },
    { "P0353", "Ignition coil C primary/secondary circuit" },
    { "P0354", "Ignition coil D primary/secondary circuit" },
    { "P0355", "Ignition coil E primary/secondary circuit" },
    { "P0356", "Ignition coil F primary/secondary circuit" },
    { "P0357", "Ignition coil G primary/secondary circuit" },
    { "P0358", "Ignition coil H primary/secondary circuit" },
    { "P0359", "Ignition coil I primary/secondary circuit" },
    { "P0360", "Ignition coil J primary/secondary circuit" },
    { "P0361", "Ignition coil K primary/secondary circuit" },
    { "P0362", "Ignition coil L primary/secondary circuit" },
    { "P0365", "Camshaft position sensor B circuit (bank 1)" },
    { "P0366", "Camshaft position sensor B range/performance" },
    { "P0367", "Camshaft position sensor B circuit low" },
    { "P0368", "Camshaft position sensor B circuit high" },
    { "P0369", "Camshaft position sensor B intermittent" },
    { "P0370", "Timing reference high resolution signal A" },
    { "P0371", "Timing reference signal A too many pulses" },
    { "P0372", "Timing reference signal A too few pulses" },
    { "P0373", "Timing reference signal A intermittent pulses" },
    { "P0380", "Glow plug/heater circuit A" },
    { "P0381", "Glow plug/heater indicator circuit" },
    { "P0382", "Glow plug/heater circuit B" },
    { "P0385", "Crankshaft position sensor B circuit" },
    { "P0386", "Crankshaft position sensor B range/performance" },
    { "P0387", "Crankshaft position sensor B circuit low" },
    { "P0388", "Crankshaft position sensor B circuit high" },
    { "P0389", "Crankshaft position sensor B intermittent" },
    { "P0390", "Camshaft position sensor B circuit (bank 2)" },
    { "P0391", "Camshaft position sensor B range/perf (bank 2)" },
    { "P0392", "Camshaft position sensor B low (bank 2)" },
    { "P0393", "Camshaft position sensor B high (bank 2)" },
    { "P0394", "Camshaft position sensor B intermittent (bank 2)" },
    { "P0400", "Exhaust gas recirculation flow" },
    { "P0401", "EGR flow insufficient" },
    { "P0402", "EGR flow excessive" },
    { "P0403", "EGR circuit malfunction" },
    { "P0404", "EGR circuit range/performance" },
    { "P0405", "EGR sensor A circuit low" },
    { "P0406", "EGR sensor A circuit high" },
    { "P0407", "EGR sensor B circuit low" },
    { "P0408", "EGR sensor B circuit high" },
    { "P0410", "Secondary air injection system" },
    { "P0411", "Secondary air incorrect flow" },
    { "P0412", "Secondary air valve A circuit" },
    { "P0413", "Secondary air valve A circuit open" },
    { "P0414", "Secondary air valve A circuit shorted" },
    { "P0415", "Secondary air valve B circuit" },
    { "P0416", "Secondary air valve B circuit open" },
    { "P0417", "Secondary air valve B circuit shorted" },
    { "P0418", "Secondary air relay A circuit" },
    { "P0419", "Secondary air relay B circuit" },
    { "P0420", "Catalyst efficiency below threshold (bank 1)" },
    { "P0421", "Warm-up catalyst efficiency (bank 1)" },
    { "P0422", "Main catalyst efficiency low (bank 1)" },
    { "P0423", "Heated catalyst efficiency (bank 1)" },
    { "P0424", "Heated catalyst too cold (bank 1)" },
    { "P0430", "Catalyst efficiency below threshold (bank 2)" },
    { "P0431", "Warm-up catalyst efficiency (bank 2)" },
    { "P0432", "Main catalyst efficiency low (bank 2)" },
    { "P0433", "Heated catalyst efficiency (bank 2)" },
    { "P0434", "Heated catalyst too cold (bank 2)" },
    { "P0440", "EVAP system malfunction" },
    { "P0441", "EVAP incorrect purge flow" },
    { "P0442", "EVAP small leak detected" },
    { "P0443", "EVAP purge valve circuit" },
    { "P0444", "EVAP purge valve circuit open" },
    { "P0445", "EVAP purge valve circuit shorted" },
    { "P0446", "EVAP vent control circuit" },
    { "P0447", "EVAP vent control circuit open" },
    { "P0448", "EVAP vent control circuit shorted" },
    { "P0449", "EVAP vent valve/solenoid circuit" },
    { "P0450", "EVAP pressure sensor circuit" },
    { "P0451", "EVAP pressure sensor range/performance" },
    { "P0452", "EVAP pressure sensor circuit low" },
    { "P0453", "EVAP pressure sensor circuit high" },
    { "P0454", "EVAP pressure sensor intermittent" },
    { "P0455", "EVAP gross leak or no flow" },
    { "P0456", "EVAP very small leak detected" },
    { "P0457", "EVAP leak, fuel cap loose or off" },
    { "P0460", "Fuel level sensor circuit" },
    { "P0461", "Fuel level sensor range/performance" },
    { "P0462", "Fuel level sensor circuit low" },
    { "P0463", "Fuel level sensor circuit high" },
    { "P0464", "Fuel level sensor intermittent" },
    { "P0465", "EVAP purge flow sensor circuit" },
    { "P0466", "EVAP purge flow sensor range/performance" },
    { "P0467", "EVAP purge flow sensor circuit low" },
    { "P0468", "EVAP purge flow sensor circuit high" },
    { "P0469", "EVAP purge flow sensor intermittent" },
    { "P0470", "Exhaust pressure sensor circuit" },
    { "P0471", "Exhaust pressure sensor range/performance" },
    { "P0472", "Exhaust pressure sensor circuit low" },
    { "P0473", "Exhaust pressure sensor circuit high" },
    { "P0474", "Exhaust pressure sensor intermittent" },
    { "P0475", "Exhaust pressure control valve" },
    { "P0476", "Exhaust pressure control valve range/performance" },
    { "P0477", "Exhaust pressure control valve low" },
    { "P0478", "Exhaust pressure control valve high" },
    { "P0479", "Exhaust pressure control valve intermittent" },
    { "P0480", "Cooling fan 1 control circuit" },
    { "P0481", "Cooling fan 2 control circuit" },
    { "P0482", "Cooling fan 3 control circuit" },
    { "P0483", "Cooling fan rationality check" },
    { "P0484", "Cooling fan circuit over current" },
    { "P0485", "Cooling fan power/ground circuit" },
    { "P0491", "Secondary air insufficient flow (bank 1)" },
    { "P0492", "Secondary air insufficient flow (bank 2)" },
    { "P0500", "Vehicle speed sensor circuit" },
    { "P0501", "Vehicle speed sensor range/performance" },
    { "P0502", "Vehicle speed sensor circuit low" },
    { "P0503", "Vehicle speed sensor intermittent/erratic" },
    { "P0505", "Idle control system malfunction" },
    { "P0506", "Idle speed lower than expected" },
    { "P0507", "Idle speed higher than expected" },
    { "P0508", "Idle control circuit low" },
    { "P0509", "Idle control circuit high" },
    { "P0510", "Closed throttle position switch circuit" },
    { "P0511", "Idle air control circuit" },
    { "P0520", "Engine oil pressure switch circuit" },
    { "P0521", "Engine oil pressure range/performance" },
    { "P0522", "Engine oil pressure circuit low" },
    { "P0523", "Engine oil pressure circuit high" },
    { "P0524", "Engine oil pressure too low" },
    { "P0530", "A/C refrigerant pressure sensor circuit" },
    { "P0531", "A/C refrigerant pressure range/performance" },
    { "P0532", "A/C refrigerant pressure sensor low" },
    { "P0533", "A/C refrigerant pressure sensor high" },
    { "P0534", "A/C refrigerant charge loss" },
    { "P0550", "Power steering pressure sensor circuit" },
    { "P0551", "Power steering pressure range/performance" },
    { "P0552", "Power steering pressure circuit low" },
    { "P0553", "Power steering pressure circuit high" },
    { "P0554", "Power steering pressure intermittent" },
    { "P0560", "System voltage malfunction" },
    { "P0561", "System voltage unstable" },
    { "P0562", "System voltage low" },
    { "P0563", "System voltage high" },
    { "P0565", "Cruise control on signal" },
    { "P0566", "Cruise control off signal" },
    { "P0567", "Cruise control resume signal" },
    { "P0568", "Cruise control set signal" },
    { "P0569", "Cruise control coast signal" },
    { "P0570", "Cruise control accelerate signal" },
    { "P0571", "Brake switch A circuit" },
    { "P0572", "Brake switch A circuit low" },
    { "P0573", "Brake switch A circuit high" },
    { "P0574", "Cruise control system malfunction" },
    { "P0600", "Serial communication link" },
    { "P0601", "Control module checksum error" },
    { "P0602", "Control module programming error" },
    { "P0603", "Keep alive memory (KAM) error" },
    { "P0604", "Control module RAM error" },
    { "P0605", "Control module ROM error" },
    { "P0606", "Control module processor fault" },
    { "P0607", "Control module performance" },
    { "P0608", "Control module VSS output A" },
    { "P0609", "Control module VSS output B" },
    { "P0620", "Generator control circuit" },
    { "P0621", "Generator lamp L circuit" },
    { "P0622", "Generator field F circuit" },
    { "P0625", "Generator field circuit low" },
    { "P0626", "Generator field circuit high" },
    { "P0627", "Fuel pump A control circuit open" },
    { "P0628", "Fuel pump A control circuit low" },
    { "P0629", "Fuel pump A control circuit high" },
    { "P0630", "VIN not programmed or mismatched" },
    { "P0631", "VIN not programmed or mismatched - TCM" },
    { "P0633", "Immobiliser key not programmed" },
    { "P0638", "Throttle actuator control (bank 1)" },
    { "P0639", "Throttle actuator control (bank 2)" },
    { "P0641", "Sensor reference voltage A circuit open" },
    { "P0642", "Sensor reference voltage A circuit low" },
    { "P0643", "Sensor reference voltage A circuit high" },
    { "P0645", "A/C clutch relay circuit" },
    { "P0646", "A/C clutch relay circuit low" },
    { "P0647", "A/C clutch relay circuit high" },
    { "P0650", "MIL control circuit" },
    { "P0651", "Sensor reference voltage B circuit open" },
    { "P0652", "Sensor reference voltage B circuit low" },
    { "P0653", "Sensor reference voltage B circuit high" },
    { "P0654", "Engine speed output circuit" },
    { "P0655", "Engine hot lamp output circuit" },
    { "P0656", "Fuel level output circuit" },
    { "P0660", "Intake manifold tuning valve (bank 1)" },
    { "P0661", "Intake manifold tuning valve low" },
    { "P0662", "Intake manifold tuning valve high" },
    { "P0685", "ECM power relay control circuit open" },
    { "P0686", "ECM power relay control circuit low" },
    { "P0687", "ECM power relay control circuit high" },
    { "P0700", "Transmission control system (MIL request)" },
    { "P0701", "Transmission control system range/performance" },
    { "P0702", "Transmission control system electrical" },
    { "P0703", "Torque converter/brake switch B circuit" },
    { "P0704", "Clutch switch input circuit" },
    { "P0705", "Transmission range sensor circuit" },
    { "P0706", "Transmission range sensor range/performance" },
    { "P0707", "Transmission range sensor circuit low" },
    { "P0708", "Transmission range sensor circuit high" },
    { "P0709", "Transmission range sensor intermittent" },
    { "P0710", "Transmission fluid temperature sensor circuit" },
    { "P0711", "Transmission fluid temperature range/performance" },
    { "P0712", "Transmission fluid temperature circuit low" },
    { "P0713", "Transmission fluid temperature circuit high" },
    { "P0714", "Transmission fluid temperature intermittent" },
    { "P0715", "Input/turbine speed sensor circuit" },
    { "P0716", "Input/turbine speed sensor range/performance" },
    { "P0717", "Input/turbine speed sensor no signal" },
    { "P0718", "Input/turbine speed sensor intermittent" },
    { "P0719", "Torque converter/brake switch B circuit low" },
    { "P0720", "Output speed sensor circuit" },
    { "P0721", "Output speed sensor range/performance" },
    { "P0722", "Output speed sensor no signal" },
    { "P0723", "Output speed sensor intermittent" },
    { "P0725", "Engine speed input circuit" },
    { "P0726", "Engine speed input range/performance" },
    { "P0727", "Engine speed input no signal" },
    { "P0728", "Engine speed input intermittent" },
    { "P0730", "Incorrect gear ratio" },
    { "P0731", "Gear 1 incorrect ratio" },
    { "P0732", "Gear 2 incorrect ratio" },
    { "P0733", "Gear 3 incorrect ratio" },
    { "P0734", "Gear 4 incorrect ratio" },
    { "P0735", "Gear 5 incorrect ratio" },
    { "P0736", "Reverse incorrect ratio" },
    { "P0740", "Torque converter clutch circuit" },
    { "P0741", "Torque converter clutch stuck off" },
    { "P0742", "Torque converter clutch stuck on" },
    { "P0743", "Torque converter clutch circuit electrical" },
    { "P0744", "Torque converter clutch intermittent" },
    { "P0745", "Pressure control solenoid A" },
    { "P0746", "Pressure control solenoid A stuck off" },
    { "P0747", "Pressure control solenoid A stuck on" },
    { "P0748", "Pressure control solenoid A electrical" },
    { "P0749", "Pressure control solenoid A intermittent" },
    { "P0750", "Shift solenoid A" },
    { "P0751", "Shift solenoid A performance/stuck off" },
    { "P0752", "Shift solenoid A stuck on" },
    { "P0753", "Shift solenoid A electrical" },
    { "P0754", "Shift solenoid A intermittent" },
    { "P0755", "Shift solenoid B" },
    { "P0756", "Shift solenoid B performance/stuck off" },
    { "P0757", "Shift solenoid B stuck on" },
    { "P0758", "Shift solenoid B electrical" },
    { "P0759", "Shift solenoid B intermittent" },
    { "P0760", "Shift solenoid C" },
    { "P0761", "Shift solenoid C performance/stuck off" },
    { "P0762", "Shift solenoid C stuck on" },
    { "P0763", "Shift solenoid C electrical" },
    { "P0764", "Shift solenoid C intermittent" },
    { "P0765", "Shift solenoid D" },
    { "P0766", "Shift solenoid D performance/stuck off" },
    { "P0767", "Shift solenoid D stuck on" },
    { "P0768", "Shift solenoid D electrical" },
    { "P0769", "Shift solenoid D intermittent" },
    { "P0770", "Shift solenoid E" },
    { "P0771", "Shift solenoid E performance/stuck off" },
    { "P0772", "Shift solenoid E stuck on" },
    { "P0773", "Shift solenoid E electrical" },
    { "P0780", "Shift malfunction" },
    { "P0781", "1-2 shift malfunction" },
    { "P0782", "2-3 shift malfunction" },
    { "P0783", "3-4 shift malfunction" },
    { "P0784", "4-5 shift malfunction" },
    { "P0791", "Intermediate shaft speed sensor circuit" },
    { "P0795", "Pressure control solenoid C" },
    { "P0801", "Reverse inhibit control circuit" },
    { "P0803", "1-4 upshift (skip shift) solenoid" },
    { "P0805", "Clutch position sensor circuit" },
    { "P0810", "Clutch position control error" },
    { "P0830", "Clutch pedal position switch A circuit" },
    { "P0833", "Clutch pedal position switch B circuit" },
    { "P0850", "Park/neutral switch input circuit" },
    { "P0851", "Park/neutral switch input low" },
    { "P0852", "Park/neutral switch input high" },
    { "P0870", "Transmission fluid pressure switch C" },
    { "P0894", "Transmission component slipping" },
};

const char *obd_dtc_describe(const char *code)
{
    if (code == NULL) {
        return NULL;
    }

    for (size_t i = 0; i < sizeof(g_dtc_table) / sizeof(g_dtc_table[0]);
         i++) {
        if (strcmp(g_dtc_table[i].code, code) == 0) {
            return g_dtc_table[i].text;
        }
    }

    return NULL;
}

void obd_dtc_mil(const uint8_t *payload, int len, int *mil_on, int *count)
{
    if (mil_on != NULL) {
        *mil_on = -1;
    }
    if (count != NULL) {
        *count = -1;
    }
    if (payload == NULL || len < 1) {
        return;
    }

    if (mil_on != NULL) {
        *mil_on = (payload[0] & 0x80) ? 1 : 0;
    }
    if (count != NULL) {
        *count = payload[0] & 0x7F;
    }
}
