/*
 * PSP Torque — fase 2
 *
 * Valida en la PSP real el nucleo portable:
 *   - parser ELM327 y decodificacion de PID
 *   - scheduler con prioridades y periodos
 *   - MAP / BOOST / BARO (PID 0x33, captura key-on o fallback EST)
 *   - mascaras 0100 y 0120
 *
 * Pantalla de texto; el dashboard grafico QV llega en la fase 2.
 * Solo lectura: nunca se envian comandos de escritura.
 */

#include <pspkernel.h>
#include <pspdebug.h>
#include <pspdisplay.h>
#include <pspctrl.h>
#include <pspiofilemgr.h>
#include <pspnet.h>
#include <pspnet_inet.h>
#include <pspnet_apctl.h>
#include <pspnet_resolver.h>
#include <psputility.h>
#include <psputility_netconf.h>
#include <psputility_netmodules.h>
#include <psputility_sysparam.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "obd/boost.h"
#include "obd/discovery.h"
#include "obd/dtc.h"
#include "obd/elm327.h"
#include "obd/pid.h"
#include "obd/profile.h"
#include "obd/samples.h"
#include "obd/scheduler.h"
#include "obd/smooth.h"
#include "obd/vehicle.h"
#include "splash_mx5.h"

#include "gfx.h"
#include "audio.h"

PSP_MODULE_INFO("PSPTorqueOBD", 0, 1, 0);
PSP_MAIN_THREAD_ATTR(THREAD_ATTR_USER | THREAD_ATTR_VFPU);
PSP_HEAP_SIZE_KB(4096);

/* Carpeta de datos: la convencion de la consola es SAVEDATA, y el
 * nombre de 9 caracteres es el que usa el firmware para sus partidas
 * (PSPTORQUE = 9). El log, los ajustes y host.txt viven aqui, no junto
 * al EBOOT: asi el usuario puede instalar el EBOOT donde quiera. */
#define SAVEDATA_DIR "ms0:/PSP/SAVEDATA/PSPTORQUE"
#define LOG_DIR SAVEDATA_DIR
#define CONFIG_PATH SAVEDATA_DIR "/host.txt"
#define LOG_PREFIX LOG_DIR "/phase1_"
#define SETTINGS_PATH SAVEDATA_DIR "/settings.txt"
#define SETTINGS_TMP_PATH SAVEDATA_DIR "/settings.tmp"
/* Protocolo forzado, opcional: un numero (3 = ISO 9141-2, 5 = KWP
 * init rapido, 6 = CAN 11/500, 7 = CAN 29/500). Vacio = automatico. */
#define PROTO_PATH SAVEDATA_DIR "/protocol.txt"

/* Instalaciones anteriores guardaban los datos en la carpeta del EBOOT.
 * Se leen de ahi si todavia no hay nada en SAVEDATA, para no perder los
 * ajustes de quien ya tenia la app instalada. */
#define LEGACY_DIR "ms0:/PSP/GAME/PSPTorqueOBD"
#define LEGACY_SETTINGS_PATH LEGACY_DIR "/settings.txt"
#define LEGACY_CONFIG_PATH LEGACY_DIR "/host.txt"
#define DEFAULT_HOST "192.168.0.10"
#define DEFAULT_PORT 35000
#define MAX_TARGETS 4
#define BUILD_TAG "fase2.47 2026-11-18 FASE U3+U6 vista por coche, FALLOS y arranque MX-5 elegible"
#define CMD_TIMEOUT_MS 500
/* Si el adaptador ha demostrado que envia el prompt '>', hay que esperarlo
 * antes de mandar el siguiente comando. El ELM327 hace tareas internas
 * despues de responder y, si le llega otro comando, contesta STOPPED y se
 * queda atascado. Este es el margen de seguridad si el prompt no llega. */
#define ELM_SEARCH_WAIT_MS 20000u
#define ELM_QUIET_MS 60
#define ELM_PROMPT_WAIT_MS 250

/* Reconexion automatica (FASE D3): espera entre intentos y numero de
 * intentos antes de rendirse. La espera es no bloqueante. */
#define RECONN_MAX_TRIES 3
#define RECONN_DELAY_MS 2000

typedef struct {
    char ip[64];
    int port;
} Target;

typedef struct {
    int bytes;
    int data;
    int raw_data;   /* linea con cabecera CAN (escaneo de modulos) */
    int no_data;
    int error;
    int unable;
    int stopped;
    int searching;   /* el adaptador esta buscando el protocolo */
    int prompt;
    elm_frame frame;
    char text[64];
} elm_reply;

static SceUID g_log_fd = -1;
static int g_log_dirty = 0;
static uint32_t g_log_last_sync_ms = 0;
static char g_log_buf[4096];
static int g_log_buf_len = 0;
static char g_log_path[96] = "";

static obd_sample_store g_store;
static boost_state g_boost;
static obd_scheduler g_sched;

static uint32_t g_mask1 = 0;
static uint32_t g_mask2 = 0;
static uint32_t g_mask3 = 0;
static uint8_t g_thr_pid = 0x11;
static const char *g_thr_label = "MARIPOSA";
static int g_oil_supported = 0;
static int g_fuel_supported = 0;
static int g_pedal_supported = 0;
static int g_map_supported = 0;
static int g_maf_supported = 0;
static char g_elm[40] = "?";
static char g_proto[32] = "?";

/* FASE U3: pantalla de arranque elegida y herramientas ocultas. Van aqui
 * arriba porque el selector las consulta antes que nada. */
static int g_start_mode = -1;       /* -1 = la ultima pantalla (last=) */
static uint8_t g_hidden_mask = 0;   /* bits MODE_* que no se ofrecen */
/* Perfil detectado en esta sesion: clave de v.<id>.* y lo que decide si
 * la vista es de este coche o la global. */
static char g_vehicle_id[24] = "";
static char g_vbat[24] = "?";
static char g_ssid[32] = "";
static char g_ip[24] = "";
static int g_elm_prompt_seen = 0;

static float g_peak_rpm = 0.0f;
static float g_peak_speed = 0.0f;
static float g_peak_boost = 0.0f;

/* Suavizado EMA con constante de tiempo fija (FASE E3). */
static obd_smooth g_ema_rpm;
static obd_smooth g_ema_map;
static obd_smooth g_ema_speed;
static obd_smooth g_ema_thr;

/* ------------------------------------------------------------------ */
/* Estado de la UI grafica.                                           */
/* ------------------------------------------------------------------ */

#define DEG "\x7e"   /* gfx.c lo dibuja como simbolo de grado */

#define MODE_QV        0
#define MODE_ENGINE    1
#define MODE_MOTOR     2
#define MODE_TELEMETRY 3
#define MODE_MODULES   4
#define MODE_SNIFFER   5
#define MODE_INFO      6
#define MODE_PROBE     7   /* sonda del coche: solo diagnostico */
#define MODE_FALLOS    8   /* codigos de averia, solo lectura */
#define MODE_COUNT     9

#define HIST_MAX 180

static int g_mode = MODE_QV;
static int g_last_mode = MODE_QV;    /* pantalla recordada entre sesiones */
static int g_picker_open = 0;        /* selector de pantallas (SELECT) */
static int g_picker_sel = 0;
static int g_last_saved_mode = -1;  /* ultima pantalla escrita en last= */

/* FASE K2: capacidad del enlace.
 *
 * En CAN no se mide: la tabla base ya cabe en el presupuesto y ese coche es
 * el de regresion, asi que su comportamiento no puede depender de una
 * medida. En K-line se parte del techo de la clase y se baja a lo medido,
 * que es lo unico que puede hacer util un enlace de 1,9 req/s. */
static float g_link_measured = 0.0f;    /* 0 = sin medida todavia */
static uint16_t g_link_service_ms = 530;
static obd_link_plan g_link_plan;
static float g_plan_demand = 0.0f;      /* req/s que pide el plan actual */
static uint32_t g_cap_window_ms = 0;
static int g_cap_window_reqs = 0;
static uint8_t g_prio_orig[OBD_SCHED_MAX];

/* Capacidad con la que se planifica ahora mismo. */
static float link_capacity(void)
{
    int cls = obd_vehicle_proto_class(g_proto);
    float cap = obd_profile_link_class(cls);
    if (cls == OBD_PROTO_CAN) {
        return cap;
    }
    if (g_link_measured > 0.0f && g_link_measured < cap) {
        return g_link_measured;
    }
    return cap;
}

static int scan_can(void);
static const char *tr(const char *key);
static unsigned int g_prev_buttons = 0;

/* Nombre visible de cada pantalla, en la barra y en el selector. */
static const char *mode_name(int mode)
{
    switch (mode) {
    case MODE_QV:        return tr("GENERAL");
    case MODE_ENGINE:    return tr("MOTOR");
    case MODE_MOTOR:     return tr("DIAG");
    case MODE_TELEMETRY: return tr("TELEM");
    case MODE_MODULES:   return tr("MODULOS");
    case MODE_SNIFFER:   return tr("SNIFFER");
    case MODE_INFO:      return tr("INFO");
    case MODE_PROBE:     return tr("SONDA");
    case MODE_FALLOS:    return tr("FALLOS");
    default:             return "";
    }
}

/* Una sola linea de para que sirve cada pantalla, para el selector. */
static const char *mode_purpose(int mode)
{
    switch (mode) {
    case MODE_QV:        return tr("Dashboard principal");
    case MODE_ENGINE:    return tr("Detalle del motor");
    case MODE_MOTOR:     return tr("Mezcla y encendido");
    case MODE_TELEMETRY: return tr("Series de telemetria");
    case MODE_MODULES:   return tr("Centralitas del coche");
    case MODE_SNIFFER:   return tr("Trafico del bus");
    case MODE_INFO:      return tr("Que es cada dato");
    case MODE_PROBE:     return tr("Probar un coche");
    case MODE_FALLOS:    return tr("Codigos de averia");
    default:             return "";
    }
}

/* Pantallas de conduccion: las unicas que recorre L/R y las unicas que
 * salen en la barra superior. */
static int mode_is_driving(int mode)
{
    return mode == MODE_QV || mode == MODE_ENGINE ||
           mode == MODE_MOTOR || mode == MODE_TELEMETRY;
}

/* Una pantalla que este coche no puede usar no se ofrece en la barra y
 * sale en gris en el selector con el motivo. Mientras no se conoce el
 * protocolo (antes de conectar) no se oculta nada. */
static int screen_available(int mode)
{
    /* MODULOS funciona en las dos clases: en CAN con el sondeo UDS y en
     * K-line con las mascaras de OBD-II, que es todo lo que hay. SNIFFER
     * decodifica tramas CAN y en un bus K-line no tiene nada que capturar. */
    if (mode != MODE_SNIFFER) {
        return 1;
    }
    if (g_proto[0] == 0 || strcmp(g_proto, "?") == 0 ||
        strcmp(g_proto, "A0") == 0) {
        return 1;
    }
    return scan_can();
}

/* Una pantalla oculta con hidden= no se ofrece en el selector, pero sigue
 * siendo alcanzable si es la que estaba puesta. */
static int screen_offered(int mode)
{
    if (mode >= 0 && mode < MODE_COUNT && (g_hidden_mask & (1u << mode))) {
        return 0;
    }
    return screen_available(mode);
}

/* Motivo por el que una pantalla no esta disponible, o cadena vacia. */
static const char *screen_reason(int mode)
{
    if (screen_available(mode)) {
        return "";
    }
    return tr("Solo CAN");
}

/* Orden del selector: primero lo que se usa conduciendo, despues las
 * herramientas. */
static const int g_picker_order[MODE_COUNT] = {
    MODE_QV, MODE_ENGINE, MODE_MOTOR, MODE_TELEMETRY,
    MODE_MODULES, MODE_FALLOS, MODE_SNIFFER, MODE_INFO, MODE_PROBE
};

static int picker_index_of(int mode)
{
    for (int i = 0; i < MODE_COUNT; i++) {
        if (g_picker_order[i] == mode) {
            return i;
        }
    }
    return 0;
}

/* Mueve la seleccion saltando las pantallas que este coche no tiene. */
static int picker_step(int idx, int dir)
{
    for (int k = 1; k <= MODE_COUNT; k++) {
        int ni = (idx + dir * k) % MODE_COUNT;
        if (ni < 0) {
            ni += MODE_COUNT;
        }
        if (screen_offered(g_picker_order[ni])) {
            return ni;
        }
    }
    return idx;
}

/* L/R solo recorre las pantallas de conduccion. Desde una herramienta
 * vuelve al dashboard: pasar por SNIFFER o MODULOS sin querer, con el
 * coche en marcha, no es aceptable. */
static int mode_step(int mode, int dir)
{
    static const int driving[4] = { MODE_QV, MODE_ENGINE, MODE_MOTOR,
                                    MODE_TELEMETRY };
    int idx = -1;

    if (!mode_is_driving(mode)) {
        return MODE_QV;
    }
    for (int i = 0; i < 4; i++) {
        if (driving[i] == mode) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return MODE_QV;
    }
    return driving[(idx + dir + 4) % 4];
}

static int g_link_connected = 0;

/* FASE F2: respuestas STOPPED seguidas del adaptador.
 *
 * El ELM327 puede quedarse atascado en STOPPED y no sale de ahi: en un
 * viaje real se vio 99+ respuestas seguidas igual. Como STOPPED no marca
 * "error" en el parser, antes la app martilleaba al adaptador sin
 * llegar nunca a la reconexion. */
#define ELM_STUCK_MIN 3      /* para mostrar "ELM ATASCADO" */
#define ELM_STUCK_RECOVER 8  /* para escalar a reconexion */

static int g_elm_stuck = 0;

/* SELECT durante una conexion en curso la cancela (FASE D3). */
static int g_connect_cancel = 0;

/* Modo demo: datos simulados de un coche en movimiento. Se activa y se
 * desactiva manteniendo L+R durante 1,5 s (ver run_dashboard). */
static int g_demo = 0;
static uint32_t g_demo_last = 0;
static int g_fps = 0;
static float g_demo_speed = 0.0f;
static float g_demo_rpm = 850.0f;
static float g_demo_t = 0.0f;
static float g_demo_dist_mil = 698.0f;
static float g_demo_fuel = 78.0f;
static uint32_t g_boost_learn_saved = 0;

/* Efectos derivados de los datos reales de velocidad (no son lecturas
 * nuevas del coche): aceleracion, nitro de NFS, skill de Forza y la G
 * de Gran Turismo. Todo se calcula con la velocidad ya leida. */
static float g_fx_speed_prev = 0.0f;
static uint32_t g_fx_speed_ts = 0;
static float g_fx_accel = 0.0f;     /* km/h por segundo */
static float g_fx_nitro = 0.0f;     /* 0..1 */
static float g_fx_skill = 0.0f;
static uint32_t g_fx_skill_until = 0;
static int g_chime_on = 0;

/* ------------------------------------------------------------------ */
/* Descubrimiento de modulos (solo lectura).                          */
/* ------------------------------------------------------------------ */

#define SCAN_MAX_MODULES 12
#define SCAN_MAX_LINES   64
#define SCAN_LINE_LEN    128

typedef struct {
    char header[16];   /* cabecera de respuesta: 7E8, 18DAF110... */
    char req[16];      /* cabecera de peticion UDS: 7E0, 18DA15F1... */
    int obd;           /* responde a 0100/0120/0140 */
    int uds;           /* responde a 3E00 (Tester Present) */
    uint32_t mask1;
    uint32_t mask2;
    uint32_t mask3;
    char name[28];     /* F197: nombre del sistema */
    char part[24];     /* F187: referencia de pieza */
    char sw[24];       /* F188: version de software */
    char serial[24];   /* F18C: numero de serie */
    char vin[20];      /* F190: VIN */
} scan_module;

static scan_module g_scan[SCAN_MAX_MODULES];
static int g_scan_count = 0;
static int g_scan_state = 0;   /* 0 = sin escanear, 1 = escaneando, 2 = listo */
static int g_scan_sel = 0;
static int g_scan_request = 0;
static char g_scan_status[48] = "";
static char g_scan_proto[32] = "";

static char g_cap[SCAN_MAX_LINES][SCAN_LINE_LEN];
static int g_cap_count = 0;
static int g_cap_on = 0;

/* 1 si el protocolo activo es CAN de 11 bits (donde funcionan 7E0-7E7). */
static int scan_can_11bit(void);

/* ------------------------------------------------------------------ */
/* Sniffer CAN: captura todo el trafico del bus.                       */
/* ------------------------------------------------------------------ */

#define SNIF_LINES 300
#define SNIF_LEN   96

static char g_snif_buf[SNIF_LINES][SNIF_LEN];
static int g_snif_total = 0;
static int g_snif_view = 0;   /* 0 = cola; >0 = desplazamiento */
static int g_snif_running = 0;
static int g_snif_request = 0;

static void run_sniff(int sock);
static void draw_sniffer(void);

/* Relee ATDPN y actualiza g_proto. */
static void scan_refresh_proto(int sock, const char *tag);

static void run_scan(int sock);
static void draw_modules(void);
static void build_scheduler(void);
static void demo_apply_flags(int on);
static float clamp_float(float v, float lo, float hi);

/* Ajustes de tema persistidos en settings.txt. */
typedef struct {
    int theme;      /* 0 = NORMAL (color elegible), >=1 juegos */
    int color;      /* color base del tema NORMAL: 0..4 */
    int lang;       /* 0 = auto, 1 = espanol, 2 = ingles */
    int contrast;   /* 0 = bajo, 1 = normal, 2 = alto */
    int anim;       /* 1 = animacion de arranque */
    int fast;       /* 0 = completo, 1 = modo rapido */
    int peak_hold;  /* 1 = mostrar pico maximo */
    int smooth;     /* 1 = suavizado de datos */
    int shift_rpm;  /* RPM para shift light */
    int rpm_max;    /* fondo de escala del tacometro */
    int units;      /* 0 = metrico, 1 = imperial */
    int boost_auto; /* 1 = fondo de escala aprendido del boost real */
    int boost_peak; /* maximo de boost aprendido, en decimas de bar */
    int boost_max;  /* fondo de escala manual, en decimas de bar */
    int chime_kmh;  /* fijo 100 km/h; sin ajuste */
    int chime_gap;  /* fijo 9 decimas = 0,90 s; sin ajuste */
    int show_fps;   /* 1 = mostrar FPS en pantalla */
    int debug;      /* 1 = panel de diagnostico de rendimiento */
    char profile[24]; /* perfil forzado (vehicle.h); vacio = autodetectar */
} app_settings;

static app_settings g_cfg = { 0, 0, 0, 1, 1, 0, 1, 0,
                              4000, 7000, 0, 1, 0, 15, 100, 9, 1, 0, "" };

/* FASE F1: hueco minimo entre el fin de una respuesta y el envio del
 * siguiente comando al ELM327. Medido: con 0 ms el clon retrasa su
 * respuesta un tick de 16,7 ms y la capacidad cae de ~12 a ~9,25
 * peticiones/s. 8 ms es el valor conocido-bueno; el ajuste del menu
 * permite calibrar el umbral real de cada adaptador. */
#define ELM_GAP_DEFAULT_MS 8
static const int g_elm_gap_values[7] = { 0, 2, 4, 6, 8, 12, 17 };
static int g_elm_min_gap_ms = ELM_GAP_DEFAULT_MS;
static uint32_t g_last_resp_ms = 0;   /* anclaje del hueco */

/* Lo que aporta el perfil del coche (FASE U4): si esta a 0 se usa el
 * criterio de siempre, para que un coche sin perfil no cambie nada. */
static int g_profile_redline = 0;   /* inicio de la zona roja */
static int g_profile_cut = 0;       /* corte de inyeccion */
static int g_profile_tach = 0;      /* fondo de escala, o 0 = el del ajuste */

static float tach_max_rpm(void)
{
    return (float)((g_profile_tach > 0) ? g_profile_tach : g_cfg.rpm_max);
}

/* Lo que aporta el coche no se escribe en settings.txt: son hechos del
 * vehiculo, no preferencias. Si el usuario toca el tacometro en el menu,
 * manda el suyo hasta la proxima conexion. */
static void vehicle_release_tacho(void)
{
    g_profile_tach = 0;
    g_profile_redline = 0;
    g_profile_cut = 0;
}

static float redline_rpm(void)
{
    int tach = (int)tach_max_rpm();
    if (g_profile_redline > 0 && g_profile_redline < tach) {
        return (float)g_profile_redline;
    }
    /* Zona roja automatica: 500 rpm antes del fondo de escala. Con el
     * ajuste por defecto (7000) queda en 6500, como el corte habitual. */
    int r = tach - 500;
    if (r < 2000) {
        r = 2000;
    }
    if (r > tach - 200) {
        r = tach - 200;
    }
    return (float)r;
}

/* ------------------------------------------------------------------ */
/* Unidades de medida.                                                */
/* ------------------------------------------------------------------ */

static int units_imperial(void)
{
    return g_cfg.units == 1;
}

static const char *units_speed_name(void)
{
    return units_imperial() ? "mph" : "km/h";
}

static const char *units_temp_name(void)
{
    return units_imperial() ? DEG "F" : DEG "C";
}

static const char *units_press_name(void)
{
    return units_imperial() ? "psi" : "bar";
}

static float units_speed_from_kmh(float kmh)
{
    return units_imperial() ? kmh * 0.621371f : kmh;
}

static float units_speed_scale(void)
{
    /* Fondo de escala del dial de velocidad: 200 km/h = 125 mph. */
    return units_imperial() ? 125.0f : 200.0f;
}

static float units_temp_from_c(float c)
{
    return units_imperial() ? c * 1.8f + 32.0f : c;
}

/* Presion: se guarda siempre en bar; solo cambia la presentacion. */
static float units_press_from_bar(float bar)
{
    return units_imperial() ? bar * 14.5038f : bar;
}

/* MAP se guarda en kPa; en bar se ve como bar y en imperial como psi. */
static float units_map_from_kpa(float kpa)
{
    return units_imperial() ? kpa * 0.145038f : kpa / 100.0f;
}

/* MAF: g/s en metrico, lb/min en imperial. */
static const char *units_maf_name(void)
{
    return units_imperial() ? "lb/min" : "g/s";
}

static float units_maf_from_gs(float gs)
{
    return units_imperial() ? gs * 0.132277f : gs;
}

/* Fondo de escala del boost. Con el aprendizaje activo se usa el mayor
 * boost observado a plena carga y se deja un 15% de margen; el valor
 * manual queda como respaldo mientras no haya aprendizaje. */
static float boost_max_bar(void)
{
    float manual = (float)g_cfg.boost_max / 10.0f;

    if (g_cfg.boost_auto && g_cfg.boost_peak > 0) {
        float learned = ((float)g_cfg.boost_peak / 10.0f) * 1.15f;
        if (learned < 0.5f) {
            learned = 0.5f;
        }
        if (learned > 3.0f) {
            learned = 3.0f;
        }
        return learned;
    }

    return manual;
}

/* Elementos que se pueden pintar y estilo de cada hueco del dashboard. */
enum {
    W_RPM = 0, W_SPEED, W_BOOST, W_ECT, W_IAT, W_MAP, W_MAF,
    W_THR, W_LOAD, W_PLATE, W_VOLT, W_OIL, W_FUEL,
    W_STFT, W_LTFT, W_TIMING, W_O2_1, W_O2_2, W_DIST_MIL, W_BARO,
    W_GEAR,
    W_COUNT
};
#define W_OFF (-1)

enum { STYLE_GAUGE = 0, STYLE_BAR, STYLE_TEXT, STYLE_OFF, STYLE_COUNT };

#define DASH_SLOTS 6
#define TEL_SLOTS  5

static const char *const g_widget_names[W_COUNT] = {
    "RPM", "VELOCIDAD", "BOOST", "ECT", "IAT", "MAP", "MAF",
    "ACELERADOR", "CARGA", "MARIPOSA", "VOLT", "ACEITE", "COMBUSTIBLE",
    "STFT", "LTFT", "AVANCE", "O2-1", "O2-2", "MIL", "BARO", "MARCHA"
};

/* Nombre visible de cada dato: pasa por tr() para que el dashboard siga el
 * idioma como el resto de la interfaz. */
static const char *widget_name(int item)
{
    if (item < 0 || item >= W_COUNT) {
        return "";
    }
    return tr(g_widget_names[item]);
}

static const char *const g_style_names[STYLE_COUNT] = {
    "RELOJ", "BARRA", "TEXTO", "OCULTO"
};

/* Valores por defecto; settings_load los reinicia y luego lee el archivo. */
static int g_slot_item[DASH_SLOTS] = {
    W_RPM, W_SPEED, W_BOOST, W_ECT, W_IAT, W_MAP
};
static int g_slot_style[DASH_SLOTS] = {
    STYLE_GAUGE, STYLE_GAUGE, STYLE_BAR,
    STYLE_TEXT, STYLE_TEXT, STYLE_TEXT
};

static int g_tel_item[TEL_SLOTS] = {
    W_RPM, W_BOOST, W_ECT, W_SPEED, W_OFF
};

/* FASE U4b: lo que el coche puede o no llenar.
 *
 * La vista no se toca ni se guarda: g_slot_item sigue siendo la del
 * usuario y settings.txt escribe esa. Lo unico que cambia es lo que se
 * PINTA: si el coche no tiene MAP, los huecos que iban de BOOST y de MAP
 * ensenan MAF y carga en vez de dos N/D. En cuanto el usuario toca la
 * vista a mano manda la suya y el coche no opina. */
static int g_vehicle_centre = OBD_CENTRE_BOOST;
static int g_layout_user = 0;        /* 1 = el usuario ha tocado la vista */

/* FASE U3: preferencias v2.
 *
 * Tres cosas que van por coche o por persona y no por sesion:
 *   - la vista de cada coche (v.<id>.slotN_item / telN_item), para que
 *     ordenarla en la MX-5 no descoloque el MiTo;
 *   - la pantalla de arranque (start=), un nombre o "last";
 *   - las herramientas que no se usan (hidden=), que no salen en el
 *     selector.
 * Un fichero viejo sigue valiendo: lo que no aparece se queda como esta. */
#define VEHICLE_PREFS_MAX 4

typedef struct {
    char id[24];
    int has_layout;
    int slot_item[DASH_SLOTS];
    int slot_style[DASH_SLOTS];
    int tel_item[TEL_SLOTS];
    int has_tacho;
    int rpm_max;
    int shift_rpm;
} vehicle_prefs;


static vehicle_prefs g_car_prefs[VEHICLE_PREFS_MAX];
static int g_car_prefs_count = 0;

/* La vista global tal cual viene del fichero, para poder volver a ella
 * cuando el coche no trae la suya. */
static int g_global_slot_item[DASH_SLOTS];
static int g_global_slot_style[DASH_SLOTS];
static int g_global_tel_item[TEL_SLOTS];

static int slot_item_eff(int i)
{
    if (i < 0 || i >= DASH_SLOTS) {
        return W_OFF;
    }
    if (g_layout_user) {
        return g_slot_item[i];
    }
    if (g_vehicle_centre == OBD_CENTRE_MAF) {
        if (i == 2) {
            return W_MAF;
        }
        if (i == 3) {
            return W_GEAR;
        }
        if (i == 5) {
            return W_LOAD;
        }
    } else if (g_vehicle_centre == OBD_CENTRE_LOAD) {
        if (i == 2) {
            return W_LOAD;
        }
        if (i == 5) {
            return W_MAF;
        }
    }
    return g_slot_item[i];
}

/* El estilo acompana al hueco cuando el coche lo cambia. */
static int slot_style_eff(int i)
{
    if (i < 0 || i >= DASH_SLOTS) {
        return STYLE_OFF;
    }
    if (!g_layout_user && slot_item_eff(i) != g_slot_item[i]) {
        return (i == 2) ? STYLE_BAR : STYLE_TEXT;
    }
    return g_slot_style[i];
}

/* La serie de telemetria que pintaba boost en el grafico. */
static int tel_item_eff(int i)
{
    if (i < 0 || i >= TEL_SLOTS) {
        return W_OFF;
    }
    if (!g_layout_user && g_tel_item[i] == W_BOOST) {
        if (g_vehicle_centre == OBD_CENTRE_MAF) {
            return W_MAF;
        }
        if (g_vehicle_centre == OBD_CENTRE_LOAD) {
            return W_LOAD;
        }
    }
    return g_tel_item[i];
}

/* 1 mientras la vista sea la de fabrica: es lo unico que el perfil del
 * coche puede sustituir. En cuanto el usuario cambia un hueco, su vista
 * manda y no se toca ni al cambiar de coche. */
static int layout_is_factory(void)
{
    static const int fit[DASH_SLOTS] = { W_RPM, W_SPEED, W_BOOST,
                                         W_ECT, W_IAT, W_MAP };
    static const int fit_style[DASH_SLOTS] = { STYLE_GAUGE, STYLE_GAUGE,
                                               STYLE_BAR, STYLE_TEXT,
                                               STYLE_TEXT, STYLE_TEXT };
    static const int fit_tel[TEL_SLOTS] = { W_RPM, W_BOOST, W_ECT,
                                            W_SPEED, W_OFF };
    for (int i = 0; i < DASH_SLOTS; i++) {
        if (g_slot_item[i] != fit[i] || g_slot_style[i] != fit_style[i]) {
            return 0;
        }
    }
    for (int i = 0; i < TEL_SLOTS; i++) {
        if (g_tel_item[i] != fit_tel[i]) {
            return 0;
        }
    }
    return 1;
}

/* Preferencias de un coche por id; con 'create' se dan de alta. */
static vehicle_prefs *car_prefs_for(const char *id, int create)
{
    if (id == NULL || id[0] == '\0') {
        return NULL;
    }
    for (int i = 0; i < g_car_prefs_count; i++) {
        if (strcmp(g_car_prefs[i].id, id) == 0) {
            return &g_car_prefs[i];
        }
    }
    if (!create || g_car_prefs_count >= VEHICLE_PREFS_MAX) {
        return NULL;
    }
    vehicle_prefs *p = &g_car_prefs[g_car_prefs_count++];
    memset(p, 0, sizeof(*p));
    snprintf(p->id, sizeof(p->id), "%s", id);
    return p;
}

/* Guarda la vista global y pone la del coche, si la tiene. */
static void layout_apply_for_car(void)
{
    memcpy(g_slot_item, g_global_slot_item, sizeof(g_slot_item));
    memcpy(g_slot_style, g_global_slot_style, sizeof(g_slot_style));
    memcpy(g_tel_item, g_global_tel_item, sizeof(g_tel_item));

    vehicle_prefs *p = car_prefs_for(g_vehicle_id, 0);
    if (p != NULL && p->has_layout) {
        memcpy(g_slot_item, p->slot_item, sizeof(g_slot_item));
        memcpy(g_slot_style, p->slot_style, sizeof(g_slot_style));
        memcpy(g_tel_item, p->tel_item, sizeof(g_tel_item));
        /* La vista es de este coche: el overlay por capacidad no opina. */
        g_layout_user = 1;
    } else {
        g_layout_user = !layout_is_factory();
    }
}

static int g_edit_open = 0;
static int g_edit_sel = 0;
static int g_tel_edit_open = 0;
static int g_tel_edit_sel = 0;
static int g_info_scroll = 0;

static const char *const g_color_names[5] = {
    "ROSSO", "VERDE", "AMBAR", "AZZURRO", "MONO"
};

static const char *const g_units_names[2] = { "METRICO", "IMPERIAL" };

static int g_lang = 0;        /* 0 = espanol, 1 = ingles */
static int g_psp_lang = PSP_SYSTEMPARAM_LANGUAGE_ENGLISH;

static const char *tr(const char *key)
{
    static const struct { const char *es; const char *en; } t[] = {
        { "TEMA", "THEME" },
        { "COLOR", "COLOR" },
        { "CONTRASTE", "CONTRAST" },
        { "ANIMACION", "ANIMATION" },
        { "MUESTREO", "SAMPLING" },
        { "EDITAR VISTA", "EDIT VIEW" },
        { "PICO MAX", "PEAK HOLD" },
        { "SUAVIZADO", "SMOOTHING" },
        { "SHIFT RPM", "SHIFT RPM" },
        { "RPM MAX", "RPM MAX" },
        { "UNIDADES", "UNITS" },
        { "TURBO AUTO", "TURBO AUTO" },
        { "BOOST MAX", "BOOST MAX" },
        { "IDIOMA", "LANGUAGE" },
        { "HUECO ELM", "ELM GAP" },
        { "ELM ATASCADO", "ELM STUCK" },
        { "DATOS VIEJOS", "OLD DATA" },
        { "GENERAL", "GENERAL" },
        { "MOTOR", "ENGINE" },
        { "DIAG", "DIAG" },
        { "TELEM", "TELEM" },
        { "TELEMETRIA", "TELEMETRY" },
        { "MODULOS", "MODULES" },
        { "SNIFFER", "SNIFFER" },
        { "INFO", "INFO" },
        { "ABRIR", "OPEN" },
        { "SI", "YES" },
        { "NO", "NO" },
        { "RAPIDO", "FAST" },
        { "COMPLETO", "FULL" },
        { "METRICO", "METRIC" },
        { "IMPERIAL", "IMPERIAL" },
        { "OFF", "OFF" },
        { "CERRAR APP", "CLOSE APP" },
        { "ARRIBA/ABAJO: ELEGIR   IZQ/DER: CAMBIAR",
          "UP/DOWN: SELECT   LEFT/RIGHT: CHANGE" },
        { "X: GUARDAR   CIRCLE: BORRAR TURBO   START: SALIR",
          "X: SAVE   CIRCLE: CLEAR TURBO   START: EXIT" },
        { "PANTALLAS", "SCREENS" },
        { "HERRAMIENTAS", "TOOLS" },
        { "L/R: VOLVER", "L/R: BACK" },
        { "X: ABRIR   SELECT: CERRAR", "X: OPEN   SELECT: CLOSE" },
        { "Dashboard principal", "Main dashboard" },
        { "Detalle del motor", "Engine detail" },
        { "Mezcla y encendido", "Mixture and ignition" },
        { "Series de telemetria", "Telemetry traces" },
        { "Centralitas del coche", "ECU discovery" },
        { "Trafico del bus", "Bus capture" },
        { "Que es cada dato", "What each value is" },
        { "Probar un coche", "Try a car" },
        { "Solo CAN", "CAN only" },
        { "K-line: pendiente", "K-line: pending" },
        { "ACTUAL", "CURRENT" },
        { "PANTALLA INICIAL", "START SCREEN" },
        { "VELOCIDAD", "SPEED" },
        { "MARIPOSA", "THROTTLE" },
        { "ACELERADOR", "ACCELERATOR" },
        { "CARGA", "LOAD" },
        { "ACEITE", "OIL" },
        { "COMBUSTIBLE", "FUEL" },
        { "AVANCE", "TIMING" },
        { "MARCHA", "GEAR" },
        { "TACOMETRO", "TACHO" },
        { "TELEMETRIA", "TELEMETRY" },
        { "Sin datos del motor.", "No engine data." },
        { "Sin datos avanzados soportados.", "No advanced data supported." },
        { "Comprueba el contacto y la conexion.", "Check the ignition and the connection." },
        { "X: ELEGIR SERIES", "X: CHOOSE SERIES" },
        { "EDITAR VISTA", "EDIT LAYOUT" },
        { "SERIES DE TELEMETRIA", "TELEMETRY SERIES" },
        { "SONDA DEL COCHE", "CAR PROBE" },
        { "PULSA X PARA SONDEAR EL COCHE.", "PRESS X TO PROBE THE CAR." },
        { "ULTIMA", "LAST" },
        { "FALLOS", "FAULTS" },
        { "Codigos de averia", "Fault codes" },
        { "GUARDADOS", "STORED" },
        { "PENDIENTES", "PENDING" },
        { "PERMANENTES", "PERMANENT" },
        { "SIN CODIGOS", "NO CODES" },
        { "LAMPARA", "LAMP" },
        { "X: LEER   CIRCLE: LIMPIAR PANTALLA",
          "X: READ   CIRCLE: CLEAR SCREEN" },
        { "SI", "ON" },
        { "NO", "OFF" },
    };
    if (g_lang == 0 || key == NULL) {
        return key;
    }
    for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
        if (strcmp(t[i].es, key) == 0) {
            return t[i].en;
        }
    }
    return key;
}

#define SET_COUNT 18
#define SET_EXIT  17

static int g_settings_open = 0;
static int g_settings_sel = 0;
static int g_exit_request = 0;

/* Estado grafico y de conexion en segundo plano. */
static int g_gfx_active = 0;
static int g_screen_ready = 0;
static int g_net_busy = 0;
static char g_net_msg[64] = "";
static uint32_t g_net_spin = 0;
static uint32_t g_net_ui_last = 0;

/* Resumen que se imprime en texto al salir de la GUI. */
typedef struct {
    int requests;
    int data_count;
    int no_data_count;
    int timeouts;
    int link_losses;   /* numero de veces que se perdio el enlace */
    int reconnects;    /* reconexiones correctas durante la sesion */
} session_summary;

static session_summary g_summary;

static float g_hist_rpm[HIST_MAX];
static float g_hist_boost[HIST_MAX];
static float g_hist_ect[HIST_MAX];
static float g_hist_speed[HIST_MAX];
static float g_hist_iat[HIST_MAX];
static float g_hist_thr[HIST_MAX];
static float g_hist_map[HIST_MAX];
static float g_hist_maf[HIST_MAX];
static float g_hist_load[HIST_MAX];
static float g_hist_plate[HIST_MAX];
static float g_hist_volt[HIST_MAX];
static float g_hist_oil[HIST_MAX];
static float g_hist_fuel[HIST_MAX];
static float g_hist_stft[HIST_MAX];
static float g_hist_ltft[HIST_MAX];
static float g_hist_timing[HIST_MAX];
static float g_hist_o2_1[HIST_MAX];
static float g_hist_o2_2[HIST_MAX];
static float g_hist_dist_mil[HIST_MAX];
static float g_hist_baro[HIST_MAX];
static uint32_t g_hist_valid[HIST_MAX];
static int g_hist_pos = 0;
static int g_hist_count = 0;
static unsigned int g_hist_last = 0;

static uint32_t now_ms(void)
{
    return (uint32_t)(sceKernelGetSystemTimeWide() / 1000);
}

/* Dibujo de la pantalla de conexion; se define mas abajo. */
static void net_ui_tick(void);
static void net_ui_set(const char *msg);

/* Cede CPU, refresca la pantalla de conexion y atiende la cancelacion;
 * se define mas abajo (FASE D3). */
static void connect_yield(void);

/* Scheduler; se define mas abajo. */
static void build_scheduler(void);

/* ------------------------------------------------------------------ */
/* Log: pantalla + archivo, con sincronizacion periodica.              */
/* ------------------------------------------------------------------ */

static int log_exists(const char *path)
{
    SceIoStat st;
    return sceIoGetstat(path, &st) >= 0;
}

/*
 * Un archivo por sesion:
 *   - Con reloj valido: phase1_AAAAMMDD_HHMMSS.log
 *   - Sin reloj valido: phase1_001.log, phase1_002.log, ...
 * Nunca se borra el log de una sesion anterior.
 */
static void log_pick_path(void)
{
    char base[80];

    time_t t = time(NULL);
    struct tm *lt = localtime(&t);

    if (lt != NULL && lt->tm_year >= 100) {
        snprintf(base, sizeof(base),
                 LOG_PREFIX "%04d%02d%02d_%02d%02d%02d",
                 lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
                 lt->tm_hour, lt->tm_min, lt->tm_sec);
        snprintf(g_log_path, sizeof(g_log_path), "%s.log", base);

        /* Dos arranques en el mismo segundo: anadir sufijo. */
        if (log_exists(g_log_path)) {
            for (int i = 1; i <= 99; i++) {
                snprintf(g_log_path, sizeof(g_log_path),
                         "%s_%02d.log", base, i);
                if (!log_exists(g_log_path)) {
                    break;
                }
            }
        }
        return;
    }

    for (int i = 1; i <= 999; i++) {
        snprintf(g_log_path, sizeof(g_log_path), LOG_PREFIX "%03d.log", i);
        if (!log_exists(g_log_path)) {
            return;
        }
    }
}

static int g_log_broken = 0;

static void log_open(void)
{
    g_log_path[0] = '\0';
    log_pick_path();
    g_log_fd = sceIoOpen(g_log_path,
                         PSP_O_WRONLY | PSP_O_CREAT | PSP_O_TRUNC,
                         0777);
    g_log_dirty = 0;
    g_log_broken = (g_log_fd < 0);
}

static void log_flush(void);

static void log_close(void)
{
    if (g_log_fd >= 0) {
        log_flush();
        sceIoSync("ms0:", 0);
        sceIoClose(g_log_fd);
        g_log_fd = -1;
        g_log_buf_len = 0;
    }
}

static void log_flush(void)
{
    if (g_log_fd >= 0 && g_log_buf_len > 0) {
        sceIoWrite(g_log_fd, g_log_buf, (unsigned int)g_log_buf_len);
        g_log_buf_len = 0;
        g_log_dirty = 1;
    }
}

static void log_sync(void)
{
    if (g_log_fd >= 0) {
        log_flush();
        if (g_log_dirty) {
            uint32_t now = now_ms();
            if (g_log_last_sync_ms == 0 ||
                (uint32_t)(now - g_log_last_sync_ms) >= 2000u) {
                sceIoSync("ms0:", 0);
                g_log_dirty = 0;
                g_log_last_sync_ms = now;
            }
        }
    }
}

static void log_vwrite(const char *fmt, va_list ap, int screen)
{
    char buf[256];

    vsnprintf(buf, sizeof(buf), fmt, ap);
    if (screen && g_screen_ready) {
        pspDebugScreenPrintf("%s", buf);
    }
    if (g_log_fd >= 0) {
        int len = (int)strlen(buf);
        if (len >= (int)sizeof(g_log_buf)) {
            log_flush();
            sceIoWrite(g_log_fd, buf, (unsigned int)len);
            g_log_dirty = 1;
        } else {
            if (g_log_buf_len + len > (int)sizeof(g_log_buf)) {
                log_flush();
            }
            memcpy(g_log_buf + g_log_buf_len, buf, (size_t)len);
            g_log_buf_len += len;
            g_log_dirty = 1;
        }
    }
}

static void log_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_vwrite(fmt, ap, 1);
    va_end(ap);
    /* Las lineas de arranque interesan aunque la app muera despues. */
    log_sync();
}

static void log_file_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_vwrite(fmt, ap, 0);
    va_end(ap);
}

/* ------------------------------------------------------------------ */
/* Ajustes de tema (settings.txt).                                    */
/* ------------------------------------------------------------------ */

static char *trim_in_place(char *s)
{
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {
        s++;
    }
    char *end = s + strlen(s);
    while (end > s && (end[-1] == ' ' || end[-1] == '\t' ||
                       end[-1] == '\r' || end[-1] == '\n')) {
        end--;
    }
    *end = '\0';
    return s;
}

static void settings_apply(void)
{
    g_cfg.smooth = 1;   /* suavizado siempre activo */
    /* Idioma efectivo. */
    if (g_cfg.lang == 0) {
        g_lang = (g_psp_lang == PSP_SYSTEMPARAM_LANGUAGE_SPANISH) ? 0 : 1;
    } else {
        g_lang = g_cfg.lang - 1;
    }

    int preset = (g_cfg.theme == 0) ? g_cfg.color : g_cfg.theme;
    gfx_theme_apply(preset, g_cfg.contrast);
    audio_set_gap_ms(g_cfg.chime_gap * 100);
    if (!g_cfg.smooth) {
        obd_smooth_reset(&g_ema_rpm);
        obd_smooth_reset(&g_ema_map);
        obd_smooth_reset(&g_ema_speed);
        obd_smooth_reset(&g_ema_thr);
    }
    if (g_link_connected) {
        build_scheduler();
        obd_sched_reset_due(&g_sched, now_ms());
    }
    if (g_demo) {
        demo_apply_flags(1);
    }

    /* Al cambiar de tema se reinician los efectos y el campaneo. */
    if (g_chime_on) {
        g_chime_on = 0;
        audio_chime(0);
    }
    g_fx_nitro = 0.0f;
    g_fx_skill = 0.0f;
    g_fx_skill_until = 0;
}

/* Protocolo forzado desde protocol.txt. 0 = automatico. */
static int g_proto_force = 0;

static void proto_load(void)
{
    char buf[16];
    SceUID fd = sceIoOpen(PROTO_PATH, PSP_O_RDONLY, 0777);

    if (fd < 0) {
        return;
    }
    int n = sceIoRead(fd, buf, sizeof(buf) - 1);
    sceIoClose(fd);
    if (n <= 0) {
        return;
    }
    buf[n] = '\0';
    int v = atoi(buf);
    if (v == 3 || v == 5 || v == 6 || v == 7 || v == 8 || v == 9) {
        g_proto_force = v;
        log_file_printf("Protocolo forzado: %d\n", v);
    }
}

/* Nombre de pantalla a modo. Acepta los nombres que se ven en la barra y
 * tambien el numero, que es lo que escribe el propio programa. */
static int mode_from_name(const char *name)
{
    if (name == NULL || name[0] == '\0') {
        return -1;
    }
    if (name[0] >= '0' && name[1] == '\0') {
        int m = name[0] - '0';
        return (m >= 0 && m < MODE_COUNT) ? m : -1;
    }
    for (int m = 0; m < MODE_COUNT; m++) {
        const char *a = mode_name(m);
        const char *b = name;
        int same = 1;
        while (*a != '\0' || *b != '\0') {
            char ca = *a, cb = *b;
            if (ca >= 'a' && ca <= 'z') ca = (char)(ca - 'a' + 'A');
            if (cb >= 'a' && cb <= 'z') cb = (char)(cb - 'a' + 'A');
            if (ca != cb) {
                same = 0;
                break;
            }
            a++;
            b++;
        }
        if (same) {
            return m;
        }
    }
    if (strcmp(name, "last") == 0 || strcmp(name, "ultima") == 0) {
        return -1;
    }
    return -1;
}

/* "hidden=SNIFFER,SONDA" -> mascara de modos. */
static uint8_t mode_hidden_parse(const char *val)
{
    uint8_t mask = 0;
    char buf[96];
    snprintf(buf, sizeof(buf), "%s", val);

    char *p = buf;
    while (*p != '\0') {
        while (*p == ',' || *p == ' ' || *p == ';') {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        char *start = p;
        while (*p != '\0' && *p != ',' && *p != ' ' && *p != ';') {
            p++;
        }
        char saved = *p;
        *p = '\0';
        int m = mode_from_name(start);
        if (m >= 0 && m < MODE_COUNT) {
            mask = (uint8_t)(mask | (1u << m));
        }
        if (saved == '\0') {
            break;
        }
        p++;
    }
    return mask;
}

/* v.<id>.<clave>=valor */
static void prefs_parse_key(const char *key, const char *val)
{
    char id[24];
    const char *rest = strchr(key + 2, '.');
    if (rest == NULL) {
        return;
    }
    size_t idlen = (size_t)(rest - (key + 2));
    if (idlen == 0 || idlen >= sizeof(id)) {
        return;
    }
    memcpy(id, key + 2, idlen);
    id[idlen] = '\0';

    vehicle_prefs *vp = car_prefs_for(id, 1);
    if (vp == NULL) {
        return;
    }
    const char *sk = rest + 1;

    if (strncmp(sk, "slot", 4) == 0 && sk[4] >= '0' && sk[4] <= '5') {
        int si = sk[4] - '0';
        if (strcmp(sk + 5, "_item") == 0) {
            vp->slot_item[si] = atoi(val);
            vp->has_layout = 1;
        } else if (strcmp(sk + 5, "_style") == 0) {
            vp->slot_style[si] = atoi(val);
            vp->has_layout = 1;
        }
    } else if (strncmp(sk, "tel", 3) == 0 && sk[3] >= '0' && sk[3] <= '4') {
        int ti = sk[3] - '0';
        if (strcmp(sk + 4, "_item") == 0) {
            vp->tel_item[ti] = atoi(val);
            vp->has_layout = 1;
        }
    } else if (strcmp(sk, "rpm_max") == 0) {
        vp->rpm_max = atoi(val);
        vp->has_tacho = 1;
    } else if (strcmp(sk, "shift_rpm") == 0) {
        vp->shift_rpm = atoi(val);
        vp->has_tacho = 1;
    }
}

/* Nombres de las pantallas ocultas, para escribir hidden=. */
static const char *hidden_names_join(void)
{
    static char buf[96];
    buf[0] = '\0';
    for (int m = 0; m < MODE_COUNT; m++) {
        if ((g_hidden_mask & (1u << m)) == 0) {
            continue;
        }
        if (buf[0] != '\0') {
            strncat(buf, ",", sizeof(buf) - strlen(buf) - 1);
        }
        strncat(buf, mode_name(m), sizeof(buf) - strlen(buf) - 1);
    }
    return buf;
}

/* Antes de escribir: lo que el usuario acaba de cambiar va al coche si
 * hay coche, y si no a la vista global. Asi la vista de un coche no se
 * cuela en el otro al guardar. */
static void prefs_capture(void)
{
    if (g_vehicle_id[0] != '\0') {
        vehicle_prefs *p = car_prefs_for(g_vehicle_id, 1);
        if (p != NULL) {
            memcpy(p->slot_item, g_slot_item, sizeof(g_slot_item));
            memcpy(p->slot_style, g_slot_style, sizeof(g_slot_style));
            memcpy(p->tel_item, g_tel_item, sizeof(g_tel_item));
            p->has_layout = 1;
        }
    } else {
        memcpy(g_global_slot_item, g_slot_item, sizeof(g_slot_item));
        memcpy(g_global_slot_style, g_slot_style, sizeof(g_slot_style));
        memcpy(g_global_tel_item, g_tel_item, sizeof(g_tel_item));
    }
}

static void settings_load(void)
{
    int psp_lang = PSP_SYSTEMPARAM_LANGUAGE_ENGLISH;
    sceUtilityGetSystemParamInt(PSP_SYSTEMPARAM_ID_INT_LANGUAGE, &psp_lang);
    g_psp_lang = psp_lang;

    g_cfg.theme = 0;
    g_cfg.color = 0;
    g_cfg.lang = 0;
    g_cfg.contrast = 1;
    g_cfg.anim = 1;
    g_cfg.fast = 0;
    g_cfg.peak_hold = 1;
    g_cfg.smooth = 0;
    g_cfg.shift_rpm = 4000;
    g_cfg.rpm_max = 7000;
    g_cfg.units = 0;
    g_cfg.boost_auto = 1;
    g_cfg.boost_peak = 0;
    g_cfg.boost_max = 15;
    g_cfg.smooth = 1;   /* suavizado forzado */
    g_cfg.chime_kmh = 100;
    g_cfg.chime_gap = 9;
    g_cfg.debug = 0;

    /* Vista personalizada: valores de fabrica de los seis huecos. */
    g_slot_item[0] = W_RPM;   g_slot_style[0] = STYLE_GAUGE;
    g_slot_item[1] = W_SPEED; g_slot_style[1] = STYLE_GAUGE;
    g_slot_item[2] = W_BOOST; g_slot_style[2] = STYLE_BAR;
    g_slot_item[3] = W_ECT;   g_slot_style[3] = STYLE_TEXT;
    g_slot_item[4] = W_IAT;   g_slot_style[4] = STYLE_TEXT;
    g_slot_item[5] = W_MAP;   g_slot_style[5] = STYLE_TEXT;
    g_tel_item[0] = W_RPM;
    g_tel_item[1] = W_BOOST;
    g_tel_item[2] = W_ECT;
    g_tel_item[3] = W_SPEED;
    g_tel_item[4] = W_OFF;

    g_car_prefs_count = 0;
    memset(g_car_prefs, 0, sizeof(g_car_prefs));
    g_start_mode = -1;
    g_hidden_mask = 0;

    SceUID fd = sceIoOpen(SETTINGS_PATH, PSP_O_RDONLY, 0777);
    if (fd < 0) {
        fd = sceIoOpen(LEGACY_SETTINGS_PATH, PSP_O_RDONLY, 0777);
    }
    if (fd >= 0) {
        char buf[2048];
        int n = sceIoRead(fd, buf, sizeof(buf) - 1);
        sceIoClose(fd);

        if (n > 0) {
            buf[n] = '\0';
            char *p = buf;
            while (*p && *p != '\0') {
                char *line = p;
                while (*p && *p != '\n') {
                    p++;
                }
                if (*p == '\n') {
                    *p++ = '\0';
                }

                char *eq = strchr(line, '=');
                if (eq == NULL) {
                    continue;
                }
                *eq = '\0';
                char *key = trim_in_place(line);
                char *val = trim_in_place(eq + 1);

                if (strcmp(key, "theme") == 0 ||
                    strcmp(key, "tema") == 0) {
                    g_cfg.theme = atoi(val);
                } else if (strcmp(key, "contraste") == 0 ||
                           strcmp(key, "contrast") == 0) {
                    g_cfg.contrast = atoi(val);
                } else if (strcmp(key, "animacion") == 0 ||
                           strcmp(key, "anim") == 0) {
                    int v = atoi(val);
                    g_cfg.anim = (v < 0) ? 0 : (v > 2 ? 2 : v);
                } else if (strcmp(key, "rapido") == 0 ||
                           strcmp(key, "fast") == 0 ||
                           strcmp(key, "modo") == 0) {
                    g_cfg.fast = atoi(val) ? 1 : 0;
                } else if (strcmp(key, "pico") == 0 ||
                           strcmp(key, "peak") == 0) {
                    g_cfg.peak_hold = atoi(val) ? 1 : 0;
                } else if (strcmp(key, "suavizado") == 0 ||
                           strcmp(key, "smooth") == 0) {
                    g_cfg.smooth = atoi(val) ? 1 : 0;
                } else if (strcmp(key, "shift") == 0 ||
                           strcmp(key, "shift_rpm") == 0) {
                    g_cfg.shift_rpm = atoi(val);
                    if (g_cfg.shift_rpm < 2000) g_cfg.shift_rpm = 2000;
                    if (g_cfg.shift_rpm > 7000) g_cfg.shift_rpm = 7000;
                } else if (strcmp(key, "rpm_max") == 0 ||
                           strcmp(key, "rpm_maximo") == 0 ||
                           strcmp(key, "rpm") == 0) {
                    g_cfg.rpm_max = atoi(val);
                } else if (strcmp(key, "unidades") == 0 ||
                           strcmp(key, "units") == 0) {
                    g_cfg.units = atoi(val) ? 1 : 0;
                } else if (strcmp(key, "turbo_auto") == 0 ||
                           strcmp(key, "boost_auto") == 0) {
                    g_cfg.boost_auto = atoi(val) ? 1 : 0;
                } else if (strcmp(key, "turbo_peak") == 0 ||
                           strcmp(key, "boost_peak") == 0) {
                    g_cfg.boost_peak = atoi(val);
                } else if (strcmp(key, "boost_max") == 0 ||
                           strcmp(key, "boost") == 0) {
                    g_cfg.boost_max = atoi(val);
                } else if (strcmp(key, "color") == 0 ||
                           strcmp(key, "tema_color") == 0) {
                    g_cfg.color = atoi(val);
                } else if (strcmp(key, "idioma") == 0 ||
                           strcmp(key, "lang") == 0 ||
                           strcmp(key, "language") == 0) {
                    g_cfg.lang = atoi(val);
                } else if (strcmp(key, "fps") == 0 ||
                           strcmp(key, "fps_show") == 0 ||
                           strcmp(key, "show_fps") == 0) {
                    g_cfg.show_fps = atoi(val) ? 1 : 0;
                } else if (strcmp(key, "profile") == 0 ||
                           strcmp(key, "perfil") == 0 ||
                           strcmp(key, "vehicle") == 0) {
                    snprintf(g_cfg.profile, sizeof(g_cfg.profile), "%.23s", val);
                } else if (strcmp(key, "start") == 0 ||
                           strcmp(key, "pantalla_inicial") == 0) {
                    g_start_mode = mode_from_name(val);
                } else if (strcmp(key, "hidden") == 0 ||
                           strcmp(key, "ocultas") == 0) {
                    g_hidden_mask = mode_hidden_parse(val);
                } else if (key[0] == 'v' && key[1] == '.') {
                    prefs_parse_key(key, val);
                } else if (strcmp(key, "last") == 0 ||
                           strcmp(key, "pantalla") == 0) {
                    int v = atoi(val);
                    if (v >= 0 && v < MODE_COUNT) {
                        g_last_mode = v;
                    }
                } else if (strcmp(key, "debug") == 0) {
                    g_cfg.debug = atoi(val) ? 1 : 0;
                } else if (strcmp(key, "elm_gap") == 0 ||
                           strcmp(key, "hueco_elm") == 0) {
                    int v = atoi(val);
                    int valid = 0;
                    for (int k = 0; k < 7; k++) {
                        if (g_elm_gap_values[k] == v) {
                            valid = 1;
                            break;
                        }
                    }
                    g_elm_min_gap_ms = valid ? v : ELM_GAP_DEFAULT_MS;
                } else if (strncmp(key, "slot", 4) == 0 &&
                           key[4] >= '0' && key[4] <= '5') {
                    int si = key[4] - '0';
                    if (strcmp(key + 5, "_item") == 0) {
                        g_slot_item[si] = atoi(val);
                    } else if (strcmp(key + 5, "_style") == 0) {
                        g_slot_style[si] = atoi(val);
                    }
                } else if (strncmp(key, "tel", 3) == 0 &&
                           key[3] >= '0' && key[3] <= '4') {
                    int ti = key[3] - '0';
                    if (strcmp(key + 4, "_item") == 0) {
                        g_tel_item[ti] = atoi(val);
                    }
                }
            }
        }
    }

    if (g_cfg.theme < 0 || g_cfg.theme >= GFX_THEME_COUNT) {
        g_cfg.theme = 0;
    }
    if (g_cfg.contrast < 0 || g_cfg.contrast > 2) {
        g_cfg.contrast = 1;
    }
    /* 0 = sin animacion, 1 = la del MiTo, 2 = la del MX-5. */
    if (g_cfg.anim < 0 || g_cfg.anim > 2) {
        g_cfg.anim = 1;
    }
    g_cfg.fast = g_cfg.fast ? 1 : 0;
    g_cfg.peak_hold = g_cfg.peak_hold ? 1 : 0;
    g_cfg.smooth = g_cfg.smooth ? 1 : 0;
    g_cfg.units = g_cfg.units ? 1 : 0;
    g_cfg.boost_auto = g_cfg.boost_auto ? 1 : 0;
    if (g_cfg.boost_peak < 0) g_cfg.boost_peak = 0;
    if (g_cfg.boost_peak > 30) g_cfg.boost_peak = 30;

    /* Escalas universales: el usuario puede adaptarlas a cualquier coche. */
    if (g_cfg.rpm_max < 4000) g_cfg.rpm_max = 4000;
    if (g_cfg.rpm_max > 9000) g_cfg.rpm_max = 9000;
    if (g_cfg.shift_rpm < 2000) g_cfg.shift_rpm = 2000;
    if (g_cfg.shift_rpm > g_cfg.rpm_max) {
        g_cfg.shift_rpm = g_cfg.rpm_max;
    }
    if (g_cfg.boost_max < 5) g_cfg.boost_max = 5;
    if (g_cfg.boost_max > 30) g_cfg.boost_max = 30;

    /* Compatibilidad: los antiguos temas de color pasan al tema
     * NORMAL con su color base seleccionado. */
    if (g_cfg.theme >= 1 && g_cfg.theme <= 4) {
        g_cfg.color = g_cfg.theme;
        g_cfg.theme = 0;
    }
    if (g_cfg.theme < 0 || g_cfg.theme >= GFX_THEME_COUNT) {
        g_cfg.theme = 0;
    }
    if (g_cfg.color < 0 || g_cfg.color > 4) {
        g_cfg.color = 0;
    }
    if (g_cfg.lang < 0 || g_cfg.lang > 2) {
        g_cfg.lang = 0;
    }
    g_cfg.show_fps = g_cfg.show_fps ? 1 : 0;
    g_cfg.debug = g_cfg.debug ? 1 : 0;

    /* El aviso INITIAL D es fijo... */
    g_cfg.chime_kmh = 100;
    g_cfg.chime_gap = 9;

    for (int i = 0; i < DASH_SLOTS; i++) {
        if (g_slot_item[i] < 0 || g_slot_item[i] >= W_COUNT) {
            g_slot_item[i] = W_RPM;
        }
        if (g_slot_style[i] < 0 || g_slot_style[i] >= STYLE_COUNT) {
            g_slot_style[i] = STYLE_TEXT;
        }
    }
    for (int i = 0; i < TEL_SLOTS; i++) {
        if (g_tel_item[i] < W_OFF || g_tel_item[i] >= W_COUNT) {
            g_tel_item[i] = W_OFF;
        }
    }

    settings_apply();

    /* La vista del fichero es la global: es la que se usa cuando el coche
     * no trae la suya y la que se vuelve a poner al cambiar de coche. */
    memcpy(g_global_slot_item, g_slot_item, sizeof(g_slot_item));
    memcpy(g_global_slot_style, g_slot_style, sizeof(g_slot_style));
    memcpy(g_global_tel_item, g_tel_item, sizeof(g_tel_item));
    g_layout_user = !layout_is_factory();

    log_file_printf("Ajustes: tema=%s contraste=%d animacion=%s%s\n",
                    gfx_theme_name(g_cfg.theme), g_cfg.contrast,
                    g_cfg.anim ? "SI" : "NO",
                    g_layout_user ? " vista=PROPIA" : " vista=FABRICA");
}

static void settings_save(void)
{
    prefs_capture();

    /* Se escribe al lado y se renombra: si se corta la corriente a mitad,
     * el archivo bueno sigue entero. Con PSP_O_TRUNC sobre el de verdad,
     * un corte dejaba settings.txt en cero y se perdian tema, idioma,
     * huecos y perfil de golpe. */
    SceUID fd = sceIoOpen(SETTINGS_TMP_PATH,
                          PSP_O_WRONLY | PSP_O_CREAT | PSP_O_TRUNC,
                          0777);
    if (fd < 0) {
        log_file_printf("Ajustes: no se pudo guardar (0x%08X)\n", fd);
        return;
    }

    char buf[2048];
    int n = snprintf(buf, sizeof(buf),
                     "# PSP Torque - ajustes de tema\n"
                     "# theme: 0=QV ROSSO 1=VERDE 2=AMBAR "
                     "3=AZZURRO 4=MONO 5=FORZA 6=NFS "
                     "7=GRAN TURISMO 8=INITIAL D\n"
                     "# contraste: 0=BAJO 1=NORMAL 2=ALTO\n"
                     "# animacion: 0=NO 1=SI\n"
                     "# rapido: 0=COMPLETO 1=RAPIDO\n"
                     "# pico: 0=NO 1=SI\n"
                     "# suavizado: 0=NO 1=SI\n"
                     "# shift: RPM para shift light\n"
                     "# rpm_max: fondo de escala del tacometro\n"
                     "# (la zona roja es automatica: rpm_max - 500)\n"
                     "# unidades: 0=METRICO 1=IMPERIAL\n"
                     "# turbo_auto: 0=NO 1=SI (fondo aprendido)\n"
                     "# turbo_peak: maximo de boost aprendido (decimas de bar)\n"
                     "# boost_max: fondo de escala manual (decimas de bar)\n"
                     "# slotN_item/style: que dato y como se pinta cada hueco\n"
                     "# telN_item: series de la pestana TELEMETRIA\n"
                     "# elm_gap: hueco minimo entre comandos al ELM327 (ms)\n"
                     "#   0/2/4/6/8/12/17; 8 es el valor recomendado\n"
                     "theme=%d\n"
                     "color=%d\n"
                     "idioma=%d\n"
                     "contraste=%d\n"
                     "animacion=%d\n"
                     "rapido=%d\n"
                     "pico=%d\n"
                     "suavizado=%d\n"
                     "shift=%d\n"
                     "rpm_max=%d\n"
                     "unidades=%d\n"
                     "turbo_auto=%d\n"
                     "turbo_peak=%d\n"
                     "boost_max=%d\n"
                     "slot0_item=%d\nslot0_style=%d\n"
                     "slot1_item=%d\nslot1_style=%d\n"
                     "slot2_item=%d\nslot2_style=%d\n"
                     "slot3_item=%d\nslot3_style=%d\n"
                     "slot4_item=%d\nslot4_style=%d\n"
                     "slot5_item=%d\nslot5_style=%d\n"
                     "tel0_item=%d\n"
                     "tel1_item=%d\n"
                     "tel2_item=%d\n"
                     "tel3_item=%d\n"
                     "tel4_item=%d\n"
                     "fps_show=%d\n"
                     "debug=%d\n"
                     "elm_gap=%d\n"
                     "last=%d\n"
                     "profile=%s\n"
                     "version=2\n"
                     "start=%s\n"
                     "hidden=%s\n",
                     g_cfg.theme, g_cfg.color, g_cfg.lang, g_cfg.contrast, g_cfg.anim,
                     g_cfg.fast, g_cfg.peak_hold,
                     g_cfg.smooth, g_cfg.shift_rpm, g_cfg.rpm_max,
                     g_cfg.units, g_cfg.boost_auto,
                     g_cfg.boost_peak, g_cfg.boost_max,
                     g_slot_item[0], g_slot_style[0],
                     g_slot_item[1], g_slot_style[1],
                     g_slot_item[2], g_slot_style[2],
                     g_slot_item[3], g_slot_style[3],
                     g_slot_item[4], g_slot_style[4],
                     g_slot_item[5], g_slot_style[5],
                     g_tel_item[0], g_tel_item[1], g_tel_item[2],
                     g_tel_item[3], g_tel_item[4],
                     g_cfg.show_fps, g_cfg.debug, g_elm_min_gap_ms, g_mode,
                     g_cfg.profile,
                     g_start_mode >= 0 ? mode_name(g_start_mode) : "last",
                     hidden_names_join());
    if (n > (int)sizeof(buf) - 1) {
        n = (int)sizeof(buf) - 1;
    }
    if (n > 0) {
        sceIoWrite(fd, buf, (unsigned int)n);
    }
    sceIoSync("ms0:", 0);
    sceIoClose(fd);

    sceIoRemove(SETTINGS_PATH);
    int renamed = sceIoRename(SETTINGS_TMP_PATH, SETTINGS_PATH);
    if (renamed < 0) {
        log_file_printf("Ajustes: renombrar fallo (0x%08X)\n", renamed);
        return;
    }

    log_file_printf("Ajustes guardados: tema=%s contraste=%d animacion=%s\n",
                    gfx_theme_name(g_cfg.theme), g_cfg.contrast,
                    g_cfg.anim ? "SI" : "NO");
    log_sync();
}

/* ------------------------------------------------------------------ */
/* Salida y utilidades.                                                */
/* ------------------------------------------------------------------ */

static int exit_callback(int arg1, int arg2, void *common)
{
    (void)arg1;
    (void)arg2;
    (void)common;
    sceKernelExitGame();
    return 0;
}

static int callback_thread(SceSize args, void *argp)
{
    (void)args;
    (void)argp;
    int cbid = sceKernelCreateCallback("Exit Callback", exit_callback, NULL);
    sceKernelRegisterExitCallback(cbid);
    sceKernelSleepThreadCB();
    return 0;
}

static void setup_exit_callback(void)
{
    int thid = sceKernelCreateThread("update_thread", callback_thread,
                                     0x11, 0xFA0, 0, 0);
    if (thid >= 0) {
        sceKernelStartThread(thid, 0, 0);
    }
}

static void fmt_0(char *buf, size_t size, float value)
{
    int v = (int)(value + (value >= 0.0f ? 0.5f : -0.5f));
    snprintf(buf, size, "%d", v);
}

static void fmt_1(char *buf, size_t size, float value)
{
    int negative = value < 0.0f;
    float abs_value = negative ? -value : value;
    int scaled = (int)(abs_value * 10.0f + 0.5f);
    snprintf(buf, size, "%s%d.%d", negative ? "-" : "",
             scaled / 10, scaled % 10);
}

static int sample_value(uint8_t pid, uint32_t now, float *out)
{
    const obd_sample *sample = obd_store_get(&g_store, pid);
    if (sample == NULL || !sample->valid) {
        return 0;
    }
    if ((uint32_t)(now - sample->ts_ms) > 6000u) {
        return 0;
    }
    *out = sample->value;
    return 1;
}

/* ------------------------------------------------------------------ */
/* Aprendizaje del fondo de escala del turbo.                         */
/* ------------------------------------------------------------------ */

/* Solo se aprende a plena carga (carga o acelerador >= 90%) y con el
 * motor por encima de 3000 rpm. Asi el maximo guardado corresponde a
 * una aceleracion real y no a una lectura suelta en retencion. */
static void boost_learn_update(uint32_t now)
{
    if (!g_cfg.boost_auto || g_demo || !g_boost.boost_valid) {
        return;
    }

    float load = 0.0f;
    float thr = 0.0f;
    float rpm = 0.0f;
    int wot = 0;

    if (sample_value(0x04, now, &load) && load >= 90.0f) {
        wot = 1;
    }
    if (sample_value(0x11, now, &thr) && thr >= 90.0f) {
        wot = 1;
    }
    if (g_thr_pid != 0x11 && sample_value(g_thr_pid, now, &thr) &&
        thr >= 90.0f) {
        wot = 1;
    }
    if (!wot || !sample_value(0x0C, now, &rpm) || rpm < 3000.0f) {
        return;
    }

    int peak = (int)(g_boost.boost_bar * 10.0f + 0.5f);
    if (peak < 1 || peak <= g_cfg.boost_peak) {
        return;
    }

    g_cfg.boost_peak = peak;
    if (g_cfg.boost_peak > 30) {
        g_cfg.boost_peak = 30;
    }

    char val[16];
    fmt_1(val, sizeof(val), (float)g_cfg.boost_peak / 10.0f);
    log_file_printf("TURBO aprendido: %s bar a %.0f rpm (carga %.0f%%)\n",
                    val, rpm, load);

    /* Se guarda como mucho cada 15 s para no castigar la Memory Stick. */
    if (g_boost_learn_saved == 0 ||
        (uint32_t)(now - g_boost_learn_saved) > 15000u) {
        settings_save();
        g_boost_learn_saved = now;
    }
}

/* ------------------------------------------------------------------ */
/* Modo demo: coche simulado en movimiento.                           */
/* ------------------------------------------------------------------ */

static void demo_reset(void)
{
    g_demo_speed = 0.0f;
    g_demo_rpm = 850.0f;
    g_demo_t = 0.0f;
    g_demo_last = 0;
    g_demo_dist_mil = 698.0f;
    g_demo_fuel = 78.0f;
    obd_store_init(&g_store);
    boost_init(&g_boost, PROFILE_BARO_FALLBACK_KPA);
    g_hist_count = 0;
    g_hist_pos = 0;
    g_hist_last = 0;
    g_fx_speed_prev = 0.0f;
    g_fx_speed_ts = 0;
    g_fx_accel = 0.0f;
    g_fx_nitro = 0.0f;
    g_fx_skill = 0.0f;
    g_fx_skill_until = 0;
}

static void demo_update(uint32_t now)
{
    if (g_demo_last != 0 && (uint32_t)(now - g_demo_last) < 100u) {
        return;
    }

    float dt = (g_demo_last == 0) ? 0.1f
                                  : (float)(now - g_demo_last) / 1000.0f;
    if (dt > 0.5f) {
        dt = 0.5f;
    }
    g_demo_last = now;
    g_demo_t += dt;

    /* Perfil de conduccion normal de ~130 s, basado en el uso real del
     * MiTo: ralenti, salida ciudad, crucero, adelantamiento y vuelta a
     * ralenti. Se repite sin cortes. */
    /* Ciclo de 130 s: ralenti, aceleracion, crucero, estirada y frenada. */
    static const float pts_t[] = {
        0.0f, 6.0f, 21.0f, 33.0f, 43.0f, 57.0f, 66.0f,
        78.0f, 88.0f, 98.0f, 108.0f, 118.0f, 130.0f
    };
    static const float pts_v[] = {
        0.0f, 0.0f, 100.0f, 100.0f, 50.0f, 50.0f, 90.0f,
        90.0f, 130.0f, 130.0f, 80.0f, 80.0f, 0.0f
    };
    static const int pts_n = 13;

    float cycle = fmodf(g_demo_t, 130.0f);
    if (cycle < 0.0f) {
        cycle = 0.0f;
    }

    float target_speed = 0.0f;
    float slope = 0.0f;
    for (int i = 0; i < pts_n - 1; i++) {
        if (cycle >= pts_t[i] && cycle < pts_t[i + 1]) {
            float span = pts_t[i + 1] - pts_t[i];
            float frac = (span > 0.0f)
                             ? (cycle - pts_t[i]) / span
                             : 0.0f;
            target_speed = pts_v[i] + (pts_v[i + 1] - pts_v[i]) * frac;
            slope = (pts_v[i + 1] - pts_v[i]) / span;
            break;
        }
    }

    g_demo_speed = target_speed +
                   sinf(g_demo_t * 0.7f) * 1.2f +
                   sinf(g_demo_t * 2.3f) * 0.4f;
    if (g_demo_speed < 0.0f) {
        g_demo_speed = 0.0f;
    }

    static const float g_lo_spd[6] = { 0.0f, 25.0f, 45.0f, 65.0f,
                                       90.0f, 115.0f };
    static const float g_hi_spd[6] = { 30.0f, 50.0f, 70.0f, 95.0f,
                                       120.0f, 160.0f };
    static const float g_lo_rpm[6] = { 900.0f, 1500.0f, 1800.0f,
                                       1900.0f, 2000.0f, 2200.0f };
    static const float g_hi_rpm[6] = { 4500.0f, 4200.0f, 4000.0f,
                                       3700.0f, 3400.0f, 3200.0f };

    int gear = 5;
    for (int i = 0; i < 5; i++) {
        if (g_demo_speed < g_hi_spd[i]) {
            gear = i;
            break;
        }
    }

    if (g_demo_speed < 1.0f) {
        g_demo_rpm = 850.0f + sinf(g_demo_t * 2.0f) * 25.0f;
    } else {
        float span = g_hi_spd[gear] - g_lo_spd[gear];
        float within = 0.0f;
        if (span > 0.0f) {
            within = (g_demo_speed - g_lo_spd[gear]) / span;
            within = clamp_float(within, 0.0f, 1.0f);
        }
        g_demo_rpm = g_lo_rpm[gear] +
                     within * (g_hi_rpm[gear] - g_lo_rpm[gear]);
    }

    float accel_req = slope / 3.6f;
    float v_mps = g_demo_speed / 3.6f;
    float drag = 0.0f;
    if (g_demo_speed > 0.5f) {
        drag = 0.0008f * v_mps * v_mps + 0.15f;
    }
    float pedal = (accel_req + drag) / 3.2f * 100.0f;
    pedal += sinf(g_demo_t * 3.1f) * 1.5f;
    pedal = clamp_float(pedal, 0.0f, 95.0f);

    if (slope > 5.5f) {
        pedal += 12.0f;
        pedal = clamp_float(pedal, 0.0f, 95.0f);
    }

    float plate = 12.5f + pedal * 0.77f;
    plate = clamp_float(plate, 12.5f, 89.5f);

    float baro_kpa = 97.6f + sinf(g_demo_t / 300.0f) * 0.6f;
    float vac = 0.0f;
    if (g_demo_speed > 5.0f && pedal < 8.0f) {
        vac = 15.0f;
    }
    float boost = 0.0f;
    if (pedal > 25.0f) {
        boost = powf((pedal - 25.0f) / 75.0f, 1.2f) * 200.0f;
    }
    float map_kpa = 45.0f - (pedal / 100.0f) * 18.0f - vac + boost;
    map_kpa = clamp_float(map_kpa, 20.0f, 236.0f);

    float load = 10.0f + pedal * 0.9f;
    if (g_demo_speed < 1.0f) {
        load = 15.0f;
    }
    if (g_demo_speed > 5.0f && pedal < 3.0f) {
        load = 5.0f;
    }
    load = clamp_float(load, 0.0f, 100.0f);

    float adv = 0.0f;
    if (g_demo_speed < 1.0f) {
        adv = 5.0f + sinf(g_demo_t * 0.5f) * 2.0f;
    } else {
        adv = 12.0f + (1.0f - load / 100.0f) * 30.0f -
              (boost / 100.0f) * 10.0f;
    }
    adv += sinf(g_demo_t * 1.3f) * 1.5f;
    adv = clamp_float(adv, 0.0f, 60.0f);

    float ect = 80.0f + 4.0f * (1.0f - expf(-g_demo_t / 240.0f)) +
                sinf(g_demo_t / 45.0f) * 1.2f;
    ect = clamp_float(ect, 27.0f, 90.0f);

    float iat = 28.0f + 4.0f * sinf(g_demo_t / 90.0f) +
                (1.0f - g_demo_speed / 130.0f) * 2.0f;
    iat = clamp_float(iat, 20.0f, 45.0f);

    float o2_1 = 0.45f + 0.38f * sinf(g_demo_t * 8.8f) +
                 (pedal / 100.0f) * 0.08f;
    o2_1 = clamp_float(o2_1, 0.05f, 0.95f);
    float o2_2 = 0.55f + 0.12f * sinf(g_demo_t * 2.5f) +
                 (pedal / 100.0f) * 0.05f;
    o2_2 = clamp_float(o2_2, 0.10f, 0.85f);

    float stft = 4.0f * sinf(g_demo_t * 0.15f) +
                 2.0f * sinf(g_demo_t * 0.9f);
    stft = clamp_float(stft, -20.0f, 20.0f);
    float ltft = -4.0f + 5.0f * sinf(g_demo_t / 40.0f);
    ltft = clamp_float(ltft, -25.0f, 25.0f);

    float volt = 13.8f + 0.25f * (g_demo_rpm / 3000.0f) +
                 sinf(g_demo_t * 1.3f) * 0.10f;
    volt = clamp_float(volt, 12.5f, 14.4f);

    g_demo_fuel = 78.0f - fmodf(g_demo_t, 3600.0f) * 0.0015f;
    if (g_demo_fuel < 10.0f) {
        g_demo_fuel = 10.0f;
    }

    g_demo_dist_mil += g_demo_speed * dt / 3600.0f;
    if (g_demo_dist_mil < 600.0f) {
        g_demo_dist_mil = 698.0f;
    }

    obd_store_set(&g_store, 0x0C, g_demo_rpm, OBD_UNIT_RPM, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x0D, g_demo_speed, OBD_UNIT_KPH, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x04, load, OBD_UNIT_PERCENT, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x05, ect, OBD_UNIT_CELSIUS, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x0F, iat, OBD_UNIT_CELSIUS, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x0B, map_kpa, OBD_UNIT_KPA, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x11, plate, OBD_UNIT_PERCENT, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x49, pedal, OBD_UNIT_PERCENT, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x42, volt, OBD_UNIT_VOLT, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x2F, g_demo_fuel, OBD_UNIT_PERCENT, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x33, baro_kpa, OBD_UNIT_KPA, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x06, stft, OBD_UNIT_PERCENT, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x07, ltft, OBD_UNIT_PERCENT, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x0E, adv, OBD_UNIT_DEGREES, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x14, o2_1, OBD_UNIT_VOLT, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x15, o2_2, OBD_UNIT_VOLT, now,
                  OBD_SRC_SIMULATED);
    obd_store_set(&g_store, 0x21, g_demo_dist_mil, OBD_UNIT_KM, now,
                  OBD_SRC_SIMULATED);

    boost_set_map(&g_boost, map_kpa, now);
}

/* Al entrar en demo se muestran todos los campos disponibles; al salir
 * se recupera el estado real del coche. */
static void demo_apply_flags(int on)
{
    if (on) {
        /* Demo basado en el MiTo 1.4 170cv: MAP (sin MAF), pedal 0x49,
         * combustible y trims/avance con la codificacion no estandar. */
        g_oil_supported = 0;
        g_fuel_supported = 1;
        g_map_supported = 1;
        g_maf_supported = 0;
        g_pedal_supported = 1;
        g_thr_pid = 0x49;
        g_thr_label = "ACELERADOR";
        obd_pid_set_mito_encoding(1);
    } else {
        g_oil_supported = 0;
        g_fuel_supported = 0;
        g_map_supported = 0;
        g_maf_supported = 0;
        g_pedal_supported = 0;
        g_thr_pid = 0x11;
        g_thr_label = "MARIPOSA";
        obd_pid_set_mito_encoding(
            g_mask1 == 0xBE3EB811u &&
            g_mask2 == 0x80022001u &&
            g_mask3 == 0xC0C00000u);
    }
}

static void demo_toggle(void)
{
    g_demo = !g_demo;

    if (g_demo) {
        demo_apply_flags(1);
        demo_reset();
        log_file_printf("MODO DEMO activado (datos SIM)\n");
    } else {
        demo_apply_flags(0);
        obd_store_init(&g_store);
        boost_init(&g_boost, PROFILE_BARO_FALLBACK_KPA);
        if (g_link_connected) {
            build_scheduler();
        }
        log_file_printf("MODO DEMO desactivado\n");
    }

    g_peak_rpm = 0.0f;
    g_peak_speed = 0.0f;
    g_peak_boost = 0.0f;
    log_sync();
}

/* ------------------------------------------------------------------ */
/* Red.                                                               */
/* ------------------------------------------------------------------ */

static void add_target(Target *list, int *count, int max,
                       const char *ip, int port)
{
    if (*count >= max || ip == NULL || ip[0] == '\0') {
        return;
    }
    for (int i = 0; i < *count; i++) {
        if (list[i].port == port && strcmp(list[i].ip, ip) == 0) {
            return;
        }
    }
    strncpy(list[*count].ip, ip, sizeof(list[*count].ip) - 1);
    list[*count].ip[sizeof(list[*count].ip) - 1] = '\0';
    list[*count].port = port;
    (*count)++;
}

static int load_targets(Target *list, int max)
{
    int count = 0;

    SceUID fd = sceIoOpen(CONFIG_PATH, PSP_O_RDONLY, 0777);
    if (fd < 0) {
        fd = sceIoOpen(LEGACY_CONFIG_PATH, PSP_O_RDONLY, 0777);
    }
    if (fd >= 0) {
        char buf[768];
        int n = sceIoRead(fd, buf, sizeof(buf) - 1);
        sceIoClose(fd);
        if (n > 0) {
            buf[n] = '\0';
            char *p = buf;
            while (*p && count < max) {
                while (*p == ' ' || *p == '\t' ||
                       *p == '\r' || *p == '\n') {
                    p++;
                }
                if (*p == '#' || *p == ';') {
                    while (*p && *p != '\n') {
                        p++;
                    }
                    continue;
                }
                if (*p == '\0') {
                    break;
                }

                char *start = p;
                while (*p && *p != ' ' && *p != '\t' &&
                       *p != '\r' && *p != '\n') {
                    p++;
                }
                size_t len = (size_t)(p - start);
                if (len >= sizeof(list[0].ip)) {
                    len = sizeof(list[0].ip) - 1;
                }

                char ip[64];
                memcpy(ip, start, len);
                ip[len] = '\0';

                int port = DEFAULT_PORT;
                while (*p == ' ' || *p == '\t') {
                    p++;
                }
                if (*p >= '0' && *p <= '9') {
                    int value = 0;
                    while (*p >= '0' && *p <= '9') {
                        value = value * 10 + (*p - '0');
                        p++;
                    }
                    if (value > 0 && value < 65536) {
                        port = value;
                    }
                }
                while (*p && *p != '\n') {
                    p++;
                }

                add_target(list, &count, max, ip, port);
            }
        }
    }

    add_target(list, &count, max, DEFAULT_HOST, DEFAULT_PORT);
    return count;
}

static int wait_for_ip(int timeout_ms)
{
    int waited = 0;
    while (waited < timeout_ms) {
        int state = 0;
        if (sceNetApctlGetState(&state) == 0 &&
            state == PSP_NET_APCTL_STATE_GOT_IP) {
            return 0;
        }
        connect_yield();
        if (g_connect_cancel) {
            return -2;
        }
        sceKernelDelayThread(100 * 1000);
        waited += 100;
    }
    return -1;
}

static void read_net_info(void)
{
    SceNetApctlInfo info;
    memset(&info, 0, sizeof(info));

    if (sceNetApctlGetInfo(PSP_NET_APCTL_INFO_IP, &info) == 0) {
        strncpy(g_ip, info.ip, sizeof(g_ip) - 1);
        g_ip[sizeof(g_ip) - 1] = '\0';
        log_printf("IP PSP : %s\n", g_ip);
    }
    if (sceNetApctlGetInfo(PSP_NET_APCTL_INFO_SSID, &info) == 0) {
        strncpy(g_ssid, (const char *)info.ssid, sizeof(g_ssid) - 1);
        g_ssid[sizeof(g_ssid) - 1] = '\0';
        log_printf("SSID   : %s\n", g_ssid);
    }
    if (sceNetApctlGetInfo(PSP_NET_APCTL_INFO_SECURITY_TYPE, &info) == 0) {
        log_printf("Segur. : %u (0=abierta,1=WEP,2=WPA)\n",
                             info.securityType);
    }
    if (sceNetApctlGetInfo(PSP_NET_APCTL_INFO_STRENGTH, &info) == 0) {
        log_printf("Senal  : %u %%\n", info.strength);
    }
    if (sceNetApctlGetInfo(PSP_NET_APCTL_INFO_GATEWAY, &info) == 0) {
        log_printf("Gateway: %s\n", info.gateway);
    }
}

static int get_gateway(char *out, size_t out_size)
{
    SceNetApctlInfo info;
    memset(&info, 0, sizeof(info));
    if (sceNetApctlGetInfo(PSP_NET_APCTL_INFO_GATEWAY, &info) != 0) {
        return -1;
    }
    strncpy(out, info.gateway, out_size - 1);
    out[out_size - 1] = '\0';
    return (out[0] != '\0') ? 0 : -1;
}

/*
 * El dialogo de red pasa por NONE -> INIT -> VISIBLE -> ... -> FINISHED.
 * Hay que ignorar el NONE inicial o el dialogo se cerraria sin mostrarse.
 */
static int run_netconf_dialog(void)
{
    pspUtilityNetconfData data;
    struct pspUtilityNetconfAdhoc adhocparam;

    memset(&adhocparam, 0, sizeof(adhocparam));
    memset(&data, 0, sizeof(data));
    data.base.size = sizeof(data);
    data.base.language = PSP_SYSTEMPARAM_LANGUAGE_SPANISH;
    data.base.buttonSwap = PSP_UTILITY_ACCEPT_CROSS;
    data.base.graphicsThread = 17;
    data.base.accessThread = 19;
    data.base.fontThread = 18;
    data.base.soundThread = 16;
    data.action = PSP_NETCONF_ACTION_CONNECTAP;
    data.adhocparam = &adhocparam;
    data.hotspot = 0;
    data.wifisp = 0;

    int ret = sceUtilityNetconfInitStart(&data);
    if (ret < 0) {
        log_printf("netconf init: 0x%08X\n", ret);
        return ret;
    }

    int saw_dialog = 0;
    int saw_any = 0;
    int last_status = -1;
    unsigned long long start = (unsigned long long)sceKernelGetSystemTimeWide();

    for (;;) {
        int status = sceUtilityNetconfGetStatus();
        unsigned long long now = (unsigned long long)sceKernelGetSystemTimeWide();
        int elapsed_ms = (int)((now - start) / 1000);

        if (status != last_status) {
            log_file_printf("netconf estado: %d\n", status);
            last_status = status;
        }
        if (status == PSP_UTILITY_DIALOG_INIT ||
            status == PSP_UTILITY_DIALOG_VISIBLE ||
            status == PSP_UTILITY_DIALOG_QUIT ||
            status == PSP_UTILITY_DIALOG_FINISHED) {
            saw_any = 1;
        }

        if (status == PSP_UTILITY_DIALOG_VISIBLE) {
            saw_dialog = 1;
            sceUtilityNetconfUpdate(1);
        } else if (status == PSP_UTILITY_DIALOG_QUIT) {
            sceUtilityNetconfShutdownStart();
        } else if (status == PSP_UTILITY_DIALOG_FINISHED) {
            break;
        } else if (status == PSP_UTILITY_DIALOG_NONE) {
            if (saw_dialog) {
                break;
            }
            if (!saw_any && elapsed_ms > 10000) {
                log_printf("El dialogo de red no arranco.\n");
                break;
            }
        }

        if (elapsed_ms > 120000) {
            log_printf("Dialogo de red: tiempo agotado.\n");
            break;
        }
        sceDisplayWaitVblankStart();
    }

    return 0;
}

static int tcp_connect(const char *ip, int port, int timeout_ms)
{
    int sock = sceNetInetSocket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        return -1;
    }

    int nonblock = 1;
    sceNetInetSetsockopt(sock, SOL_SOCKET, SO_NONBLOCK,
                         &nonblock, sizeof(nonblock));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((unsigned short)port);
    addr.sin_addr.s_addr = inet_addr(ip);

    int ret = sceNetInetConnect(sock, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        int err = sceNetInetGetErrno();
        if (err != EINPROGRESS && err != EWOULDBLOCK && err != EAGAIN) {
            sceNetInetClose(sock);
            return -2;
        }

        SceNetInetPollfd pfd;
        pfd.fd = sock;
        pfd.events = SCE_NET_INET_POLLOUT;

        int remaining = timeout_ms;
        int ready = 0;
        while (remaining > 0) {
            int slice = (remaining > 100) ? 100 : remaining;
            pfd.revents = 0;
            int polled = sceNetInetPoll(&pfd, 1, slice);
            if (polled < 0) {
                break;
            }
            if (polled > 0) {
                ready = 1;
                break;
            }
            remaining -= slice;
            connect_yield();
            if (g_connect_cancel) {
                break;
            }
        }

        if (g_connect_cancel) {
            sceNetInetClose(sock);
            return -5;
        }

        if (!ready) {
            sceNetInetClose(sock);
            return -3;
        }

        int so_error = 0;
        socklen_t len = sizeof(so_error);
        if (sceNetInetGetsockopt(sock, SOL_SOCKET, SO_ERROR,
                                 &so_error, &len) < 0 || so_error != 0) {
            sceNetInetClose(sock);
            return -4;
        }
    }

    return sock;
}

static int send_all(int sock, const char *data, int len)
{
    int sent = 0;
    while (sent < len) {
        int n = sceNetInetSend(sock, data + sent, len - sent, 0);
        if (n <= 0) {
            int err = sceNetInetGetErrno();
            if (err == EWOULDBLOCK || err == EAGAIN) {
                SceNetInetPollfd pfd;
                pfd.fd = sock;
                pfd.events = SCE_NET_INET_POLLOUT;
                pfd.revents = 0;
                if (sceNetInetPoll(&pfd, 1, 200) <= 0) {
                    return -1;
                }
                continue;
            }
            return -1;
        }
        sent += n;
    }
    return sent;
}

/* ------------------------------------------------------------------ */
/* Protocolo ELM327.                                                  */
/* ------------------------------------------------------------------ */

static void trim_copy(char *dst, size_t size, const char *src)
{
    while (*src == ' ' || *src == '\t' || *src == '\r' || *src == '\n') {
        src++;
    }
    size_t len = strlen(src);
    while (len > 0) {
        char c = src[len - 1];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            len--;
        } else {
            break;
        }
    }
    if (len >= size) {
        len = size - 1;
    }
    memcpy(dst, src, len);
    dst[len] = '\0';
}

static int is_noise_line(const char *text)
{
    return strncmp(text, "SEARCHING", 9) == 0 ||
           strncmp(text, "BUS INIT", 8) == 0 ||
           strncmp(text, "STOPPED", 7) == 0;
}

static int handle_line(elm_reply *reply, const char *line)
{
    char text[128];
    trim_copy(text, sizeof(text), line);
    if (text[0] == '\0') {
        return 0;
    }
    if (strcmp(text, ">") == 0) {
        reply->prompt = 1;
        g_elm_prompt_seen = 1;
        return 1;
    }

    /* Un bus caido puede generar miles de "CAN ERROR" seguidos. Se
     * registran los primeros y luego solo uno de cada 200 para no
     * llenar la tarjeta ni ralentizar la escritura. */
    static int can_err_streak = 0;
    int log_line = 1;
    if (strncmp(text, "CAN ERROR", 9) == 0) {
        can_err_streak++;
        log_line = (can_err_streak <= 5) || ((can_err_streak % 200) == 0);
    } else {
        can_err_streak = 0;
    }
    if (log_line) {
        log_file_printf("  %s\n", text);
    }

    /* Captura para el escaneo de modulos. Se guarda la linea tal cual
     * para poder ensenarla en pantalla y volcarla al log. */
    if (g_cap_on && g_cap_count < SCAN_MAX_LINES) {
        snprintf(g_cap[g_cap_count], SCAN_LINE_LEN, "%s", text);
        g_cap_count++;
    }

    obd_disc_line disc;
    if (obd_disc_parse_line(text, &disc) && disc.data_len > 0) {
        reply->raw_data = 1;
    }

    if (strncmp(text, "STOPPED", 7) == 0) {
        reply->stopped = 1;
    }

    /* El ELM327 esta buscando el protocolo. En K-line esa busqueda
     * puede tardar 10-15 s y cortarla lo deja en STOPPED, asi que se
     * trata como "sigue esperando", no como respuesta. */
    if (strncmp(text, "SEARCHING", 9) == 0) {
        reply->searching = 1;
    }

    if (reply->text[0] == '\0' && !is_noise_line(text)) {
        strncpy(reply->text, text, sizeof(reply->text) - 1);
        reply->text[sizeof(reply->text) - 1] = '\0';
    }

    elm_frame frame;
    elm_parse_result result = elm_parse_line(text, &frame);
    if (result == ELM_PARSE_DATA) {
        reply->data = 1;
        reply->frame = frame;
    } else if (result == ELM_PARSE_NO_DATA) {
        reply->no_data = 1;
    } else if (result == ELM_PARSE_ERROR) {
        reply->error = 1;
    } else if (result == ELM_PARSE_UNABLE) {
        reply->unable = 1;
    }

    return 1;
}

/* ------------------------------------------------------------------ */
/* Lectura ELM327 no bloqueante (FASE D1).                            */
/*                                                                    */
/* El socket ya es no bloqueante, asi que la espera de la respuesta se */
/* modela como una maquina de estados que avanza una vez por iteracion */
/* del bucle principal. El render y el pad dejan de congelarse durante */
/* los ~200 ms de cada PID. Solo hay una peticion en vuelo: nunca se   */
/* envia un comando nuevo mientras hay una pendiente.                  */
/* ------------------------------------------------------------------ */

#define ELM_LINE_MAX 160

typedef enum {
    ELM_RX_PENDING = 0,
    ELM_RX_DONE,
    ELM_RX_ERROR
} elm_rx_status;

typedef struct {
    int active;          /* peticion en curso */
    int sock;
    int wait_ms;
    int attempt;         /* 0 = primero, 1 = reintento tras STOPPED */
    int retry_pending;
    uint32_t retry_at;
    uint32_t start;
    uint32_t last_rx;
    int done_response;
    int line_len;
    char line[ELM_LINE_MAX];
    char cmd[16];
    elm_reply reply;
} elm_rx;

/* Descarta lo que quede pendiente en el socket (restos de una conexion
 * anterior o de un comando sin prompt). Evita el desincronizado. */
static void elm_flush_input(int sock)
{
    char buf[256];
    for (int i = 0; i < 64; i++) {
        SceNetInetPollfd pfd;
        pfd.fd = sock;
        pfd.events = SCE_NET_INET_POLLIN;
        pfd.revents = 0;
        if (sceNetInetPoll(&pfd, 1, 0) <= 0) {
            break;
        }
        if (sceNetInetRecv(sock, buf, sizeof(buf), 0) <= 0) {
            break;
        }
    }
}

/* Envia el comando de la peticion y arranca los temporizadores. */
static int elm_rx_send(elm_rx *rx, uint32_t now)
{
    char tx[64];
    snprintf(tx, sizeof(tx), "%s\r", rx->cmd);
    log_file_printf("> %s%s\n", rx->cmd, rx->attempt ? " (reintento)" : "");

    if (send_all(rx->sock, tx, (int)strlen(tx)) < 0) {
        return -1;
    }
    rx->start = now;
    rx->last_rx = now;
    rx->done_response = 0;
    rx->line_len = 0;
    return 0;
}

/* Arranca una peticion. Devuelve -1 si no se pudo enviar. */
static int elm_rx_start(elm_rx *rx, int sock, const char *cmd, int wait_ms)
{
    uint32_t now = now_ms();
    memset(rx, 0, sizeof(*rx));
    rx->sock = sock;
    rx->wait_ms = wait_ms;
    snprintf(rx->cmd, sizeof(rx->cmd), "%s", cmd);

    if (elm_rx_send(rx, now) < 0) {
        rx->reply.error = 1;
        return -1;
    }
    rx->active = 1;
    return 0;
}

/* Procesa los bytes recibidos: parte lineas y las pasa al parser. */
static void elm_rx_feed(elm_rx *rx, const char *buf, int n)
{
    for (int i = 0; i < n; i++) {
        char c = buf[i];

        /* El prompt puede llegar sin CR/LF: lo detectamos al vuelo. */
        if (c == '>') {
            if (rx->line_len > 0) {
                rx->line[rx->line_len] = '\0';
                handle_line(&rx->reply, rx->line);
                rx->line_len = 0;
            }
            rx->reply.prompt = 1;
            g_elm_prompt_seen = 1;
            break;
        }

        if (c == '\r' || c == '\n') {
            rx->line[rx->line_len] = '\0';
            if (handle_line(&rx->reply, rx->line)) {
                rx->done_response = rx->reply.data || rx->reply.raw_data ||
                                    rx->reply.no_data || rx->reply.unable ||
                                    rx->reply.error;
            }
            rx->line_len = 0;
            if (rx->reply.prompt) {
                break;
            }
        } else if (rx->line_len < (int)sizeof(rx->line) - 1) {
            rx->line[rx->line_len++] = c;
        }
    }
}

/* Avanza la maquina de estados sin bloquear. */
static elm_rx_status elm_rx_step(elm_rx *rx)
{
    if (!rx->active) {
        return ELM_RX_DONE;
    }

    uint32_t now = now_ms();

    /* Reintento tras STOPPED: espera 100 ms y reenvia una vez. */
    if (rx->retry_pending) {
        if ((int32_t)(now - rx->retry_at) < 0) {
            return ELM_RX_PENDING;
        }
        rx->retry_pending = 0;
        rx->attempt = 1;
        log_file_printf("  STOPPED: reintento\n");
        memset(&rx->reply, 0, sizeof(rx->reply));
        if (elm_rx_send(rx, now) < 0) {
            rx->reply.error = 1;
            rx->active = 0;
            return ELM_RX_ERROR;
        }
        return ELM_RX_PENDING;
    }

    /* Drena el socket sin bloquear. */
    for (;;) {
        SceNetInetPollfd pfd;
        pfd.fd = rx->sock;
        pfd.events = SCE_NET_INET_POLLIN;
        pfd.revents = 0;

        int polled = sceNetInetPoll(&pfd, 1, 0);
        if (polled < 0) {
            rx->reply.error = 1;
            rx->active = 0;
            return ELM_RX_ERROR;
        }
        if (polled == 0) {
            break;
        }

        char buf[128];
        int n = sceNetInetRecv(rx->sock, buf, sizeof(buf), 0);
        if (n <= 0) {
            int err = sceNetInetGetErrno();
            if (err == EWOULDBLOCK || err == EAGAIN) {
                break;
            }
            rx->reply.error = 1;
            rx->active = 0;
            return ELM_RX_ERROR;
        }

        rx->reply.bytes += n;
        rx->last_rx = now_ms();
        elm_rx_feed(rx, buf, n);
        if (rx->reply.prompt) {
            break;
        }
    }

    now = now_ms();

    /* Prompt: respuesta completa. */
    if (rx->reply.prompt) {
        goto complete;
    }

    /* Respuesta valida y enlace en silencio: la damos por completa.
     * Algunos ELM327 WiFi no mandan '>'. */
    if (rx->done_response) {
        int limit = g_elm_prompt_seen ? ELM_PROMPT_WAIT_MS : ELM_QUIET_MS;
        if ((int)(now - rx->last_rx) >= limit) {
            goto complete;
        }
    }

    /* Timeout global. Mientras el adaptador dice SEARCHING se le da un
     * margen largo: la busqueda de protocolo en K-line tarda 10-15 s y
     * abortarla a los 3 s deja al ELM327 atascado en STOPPED para el
     * resto de la sesion. */
    {
        uint32_t limit = (uint32_t)rx->wait_ms;
        if (rx->reply.searching && limit < ELM_SEARCH_WAIT_MS) {
            limit = ELM_SEARCH_WAIT_MS;
        }
        if ((uint32_t)(now - rx->start) >= limit) {
            goto complete;
        }
    }

    return ELM_RX_PENDING;

complete:
    if (rx->line_len > 0) {
        rx->line[rx->line_len] = '\0';
        handle_line(&rx->reply, rx->line);
        rx->line_len = 0;
    }

    log_file_printf("  [%s %dms%s]\n",
                    rx->reply.prompt ? "prompt" : "sin prompt",
                    (int)(now - rx->start),
                    rx->reply.stopped ? " STOPPED" : "");

    /* STOPPED: el ELM se atasco; reintentar una vez. */
    if (rx->reply.stopped && rx->attempt == 0) {
        rx->retry_pending = 1;
        rx->retry_at = now + 100;
        return ELM_RX_PENDING;
    }

    rx->active = 0;
    return ELM_RX_DONE;
}

/* Envoltorio bloqueante sobre la maquina de estados, para la conexion,
 * el escaneo y los comandos AT (no criticos en latencia). */
static void elm_send_and_read(int sock, const char *cmd, int wait_ms,
                              elm_reply *reply)
{
    elm_rx rx;

    if (elm_rx_start(&rx, sock, cmd, wait_ms) < 0) {
        memset(reply, 0, sizeof(*reply));
        reply->error = 1;
        return;
    }

    for (;;) {
        net_ui_tick();
        if (elm_rx_step(&rx) != ELM_RX_PENDING) {
            break;
        }
        /* Cede la CPU; el socket se sondea sin bloquear. */
        sceKernelDelayThread(1000);
    }

    *reply = rx.reply;
}

/* ------------------------------------------------------------------ */
/* Envio de la siguiente peticion OBD (FASE C).                       */
/* ------------------------------------------------------------------ */

/* Envia el PID que toque segun el planificador. Devuelve 1 si se ha
 * enviado, 0 si no tocaba nada y -1 si el envio ha fallado.
 *
 * Se separa del bucle principal para poder llamarla en dos momentos:
 * al principio de la iteracion (cuando no hay nada en vuelo) y justo
 * despues de procesar una respuesta (FASE C: el comando siguiente sale
 * de inmediato, sin esperar al frame ni al hueco de espera). */
static int obd_send_next(int sock, elm_rx *rx, int *rx_idx, uint8_t *rx_pid,
                         uint32_t *rx_t0)
{
    if (sock < 0 || rx->active) {
        return 0;
    }

    int idx = obd_sched_next(&g_sched, now_ms());
    if (idx < 0) {
        return 0;
    }

    obd_poll_item *item = &g_sched.items[idx];
    uint8_t pid = item->pid;
    char cmd[8];

    elm_build_request(cmd, sizeof(cmd), 0x01, pid);
    obd_sched_begin(&g_sched, idx, now_ms());

    *rx_idx = idx;
    *rx_pid = pid;
    *rx_t0 = now_ms();

    if (elm_rx_start(rx, sock, cmd, CMD_TIMEOUT_MS) < 0) {
        /* El socket esta muerto. Se libera el PID como si fuera un
         * timeout (no como "no soportado") para que el planificador no
         * se quede colgado con la peticion en vuelo. */
        obd_sched_end(&g_sched, idx, now_ms(), 1);
        memset(rx, 0, sizeof(*rx));
        return -1;
    }

    return 1;
}


/* ------------------------------------------------------------------ */
/* Suavizado exponencial con constante de tiempo fija (FASE E3).      */
/* ------------------------------------------------------------------ */

/* El retardo que introduce el filtro esta en el nucleo (obd_smooth):
 * tau = 150 ms. Con un factor de mezcla fijo (0,3) el retardo
 * dependia de la cadencia de muestreo y el tacometro tardaba 2,1 s en
 * seguir al aceleron. Con tau fijo el retardo es siempre el mismo. */

static void store_value(uint8_t pid, const elm_frame *frame, uint32_t ts)
{
    obd_pid_value value;
    if (obd_pid_decode(pid, frame->data, frame->data_len, &value) != 0 ||
        !value.valid) {
        return;
    }

    float v = value.value;

    /* Suavizado EMA opcional (constante de tiempo fija). */
    if (g_cfg.smooth) {
        if (pid == 0x0C) {
            v = obd_smooth_apply(&g_ema_rpm, v, ts);
        } else if (pid == 0x0B) {
            v = obd_smooth_apply(&g_ema_map, v, ts);
        } else if (pid == 0x0D) {
            v = obd_smooth_apply(&g_ema_speed, v, ts);
        } else if (pid == g_thr_pid) {
            v = obd_smooth_apply(&g_ema_thr, v, ts);
        }
    }

    obd_store_set(&g_store, pid, v, value.unit, ts, OBD_SRC_REAL);

    if (pid == 0x0B) {
        boost_set_map(&g_boost, v, ts);
    } else if (pid == 0x33) {
        boost_set_baro_pid(&g_boost, value.value, ts);
    }
}

static void maybe_capture_key_on(uint32_t now, uint32_t session_ms)
{
    float map_value = 0.0f;
    float rpm_value = 0.0f;
    float speed_value = 0.0f;

    if (!sample_value(0x0B, now, &map_value) ||
        !sample_value(0x0C, now, &rpm_value)) {
        return;
    }

    int speed_valid = sample_value(0x0D, now, &speed_value);
    if (boost_capture_key_on(&g_boost, map_value,
                             1, rpm_value,
                             speed_valid, speed_value,
                             now, session_ms)) {
        char baro[16];
        fmt_1(baro, sizeof(baro), g_boost.baro_kpa);
        log_file_printf("BARO key-on capturado: %s kPa\n", baro);
    }
}

static void apply_support_masks(void)
{
    if (g_mask1 != 0) {
        obd_sched_apply_mask(&g_sched, g_mask1, 0x01);
    }
    if (g_mask2 != 0) {
        obd_sched_apply_mask(&g_sched, g_mask2, 0x21);
    }
    if (g_mask3 != 0) {
        obd_sched_apply_mask(&g_sched, g_mask3, 0x41);
    }
}

/* FASE U4: que coche es. Se decide una vez por conexion, con la evidencia
 * que ya tenemos (ATDPN y las tres mascaras), y se escribe en el log: sin
 * esa linea, un cuadro equivocado no se puede explicar despues. */
static int g_vehicle_seen = 0;

static void vehicle_detect(void)
{
    if (g_vehicle_seen) {
        return;
    }
    g_vehicle_seen = 1;

    int exact = 0;
    int proto = obd_vehicle_proto_class(g_proto);
    const obd_vehicle_profile *p = obd_vehicle_match(
        proto, g_mask1, g_mask2, g_mask3, g_cfg.profile, &exact);

    if (p == NULL) {
        log_file_printf("Vehiculo: sin perfil (proto %s, mascaras %08X/%08X/%08X)\n",
                        g_proto, g_mask1, g_mask2, g_mask3);
        return;
    }

    /* Un perfil generico no toca nada: el tacometro y la zona roja se
     * quedan como los tenga el usuario. Y lo detectado no se escribe en
     * settings.txt: si un dia no hay coche, sigue valiendo lo suyo. */
    vehicle_release_tacho();
    if (!p->generic) {
        g_profile_tach = p->rpm_max;
        g_profile_redline = p->redline;
        g_profile_cut = p->fuel_cut;
    }

    /* El centro del cuadro, por capacidad: con MAP y baro hay boost; sin
     * MAP pero con MAF se pinta MAF; sin ninguno de los dos, carga. Un
     * perfil puede decirlo (la MX-5 dice MAF); si no, se deduce de lo que
     * el coche declara. */
    if (p->centre == OBD_CENTRE_FROM_CAPS) {
        /* 0x33 (baro) vive en el bloque 0x20: mascara 2, bit 0x13. */
        if (g_map_supported && obd_mask_supports(g_mask2, 0x13)) {
            g_vehicle_centre = OBD_CENTRE_BOOST;
        } else if (g_maf_supported) {
            g_vehicle_centre = OBD_CENTRE_MAF;
        } else {
            g_vehicle_centre = OBD_CENTRE_LOAD;
        }
    } else {
        g_vehicle_centre = p->centre;
    }

    /* Identidad para las preferencias por coche y su vista, si tiene. */
    snprintf(g_vehicle_id, sizeof(g_vehicle_id), "%s",
             p->generic ? "" : p->id);
    layout_apply_for_car();

    log_file_printf("Vehiculo: %s (%s, proto %s, mascaras %08X/%08X/%08X)"
                    " tacometro=%d rojo=%d corte=%d\n",
                    p->name,
                    (g_cfg.profile[0] &&
                     obd_vehicle_by_id(g_cfg.profile) != NULL)
                        ? "forzado" : (exact ? "detectado" : "generico"),
                    g_proto, g_mask1, g_mask2, g_mask3,
                    (int)tach_max_rpm(),
                    p->generic ? g_profile_redline : p->redline,
                    p->generic ? g_profile_cut : p->fuel_cut);
}

/* ------------------------------------------------------------------ */
/* Reparto del ancho de banda por pantalla (FASE F3).                  */
/* ------------------------------------------------------------------ */

/* PID que pinta cada elemento del panel. W_BOOST es derivado (MAP +
 * baro), asi que no tiene PID propio. */
static uint8_t widget_pid(int item)
{
    switch (item) {
    case W_RPM:      return 0x0C;
    case W_SPEED:    return 0x0D;
    case W_ECT:      return 0x05;
    case W_IAT:      return 0x0F;
    case W_MAP:      return 0x0B;
    case W_MAF:      return 0x10;
    case W_THR:      return (uint8_t)g_thr_pid;
    case W_LOAD:     return 0x04;
    case W_PLATE:    return 0x0D;
    case W_VOLT:     return 0x42;
    case W_OIL:      return 0x5C;
    case W_FUEL:     return 0x2F;
    case W_STFT:     return 0x06;
    case W_LTFT:     return 0x07;
    case W_TIMING:   return 0x0E;
    case W_O2_1:     return 0x14;
    case W_O2_2:     return 0x15;
    case W_DIST_MIL: return 0x21;
    case W_BARO:     return 0x33;
    case W_GEAR:     return 0;   /* derivado de RPM y velocidad */
    default:         return 0;
    }
}

/* Recalcula el periodo de cada PID segun la pantalla activa. Lo que se
 * esta mirando va a su periodo base; lo demas se ralentiza. Los PID que
 * acaban de aparecer pasan a estar pendientes de inmediato.
 *
 * Los huecos del GENERAL y las series de TELEMETRIA son configurables,
 * asi que cualquier PID puede acabar en pantalla: se fuerza su periodo
 * base con un conjunto de bytes que cubre todo el rango de OBD-II. */
static void apply_screen_schedule(uint32_t now)
{
    uint8_t mode_mask;
    float demand = 0.0f;

    switch (g_mode) {
    case MODE_ENGINE:
        mode_mask = 0x02;
        break;
    case MODE_MOTOR:
        mode_mask = 0x04;
        break;
    case MODE_TELEMETRY:
        mode_mask = 0x08;
        break;
    default:
        mode_mask = 0x01;   /* GENERAL, MODULOS, SNIFFER e INFO */
        break;
    }

    /* Conjunto de PIDs forzados a pantalla. Es un array de bytes, no
     * una mascara: PIDs como 0x5C (92) no caben en 64 bits y
     * `1u << pid` seria comportamiento indefinido. */
    obd_forced_set forced;
    memset(&forced, 0, sizeof(forced));
    if (g_mode == MODE_QV) {
        for (int i = 0; i < DASH_SLOTS; i++) {
            uint8_t pid = widget_pid(slot_item_eff(i));
            if (pid != 0) {
                obd_forced_set_add(&forced, pid);
            }
        }
    } else if (g_mode == MODE_TELEMETRY) {
        for (int i = 0; i < TEL_SLOTS; i++) {
            uint8_t pid = widget_pid(tel_item_eff(i));
            if (pid != 0) {
                obd_forced_set_add(&forced, pid);
            }
        }
    }

    /* FASE K2: si la tabla base no cabe en el presupuesto del enlace se
     * reparte por capacidad; si cabe, obd_plan_period devuelve la tabla tal
     * cual y el coche rapido no cambia de comportamiento. */
    obd_plan_init(&g_link_plan, link_capacity(), g_link_service_ms,
                  obd_profile_demand(mode_mask));
    obd_sched_set_strict(&g_sched, g_link_plan.compressed);

    /* Cuantos valores se reparten el presupuesto: el RPM va aparte y los
     * que no estan soportados no cuentan. */
    int value_count = 0;
    for (int i = 0; i < g_sched.count; i++) {
        uint8_t pid = g_sched.items[i].pid;
        uint8_t mask = obd_profile_visible_mask(pid);
        int visible = (mask & mode_mask) != 0 ||
                      obd_forced_set_has(&forced, pid);
        if (visible && pid != 0x0C && g_sched.items[i].supported != 2) {
            value_count++;
        }
    }

    int rank = 0;
    for (int i = 0; i < g_sched.count; i++) {
        uint8_t pid = g_sched.items[i].pid;
        uint8_t mask = obd_profile_visible_mask(pid);
        int visible = (mask & mode_mask) != 0 ||
                      obd_forced_set_has(&forced, pid);
        int my_rank = 0;
        if (visible && pid != 0x0C) {
            my_rank = ++rank;
        }
        uint16_t period = obd_plan_period(&g_link_plan, pid, visible,
                                          my_rank, value_count);
        if (period == 0) {
            /* PID sin entrada en la tabla: periodo lento, nunca 10 Hz. */
            period = 10000;
        }
        int was_due = (int32_t)(now - g_sched.items[i].next_due_ms) >= 0;
        obd_sched_set_period(&g_sched, i, period,
                             visible && !was_due ? 1 : 0);

        /* Con el enlace comprimido la prioridad pasa a ser el nivel del
         * plan (RPM, valores en pantalla, fondo). Sin comprimir se devuelve
         * la de la tabla, que es la que guarda build_scheduler. */
        if (g_link_plan.compressed) {
            uint8_t tier = (pid == 0x0C) ? 0 : (visible ? 1 : 2);
            if (g_sched.items[i].priority != tier) {
                obd_sched_set_priority(&g_sched, i, tier);
            }
        } else if (g_sched.items[i].priority != g_prio_orig[i]) {
            obd_sched_set_priority(&g_sched, i, g_prio_orig[i]);
        }

        if (g_sched.items[i].supported != 2 && period > 0) {
            demand += 1000.0f / (float)period;
        }
    }
    g_plan_demand = demand;
}

static void build_scheduler(void)
{
    obd_sched_init(&g_sched);
    /* Soporte real declarado por las mascaras de la ECU. */
    g_map_supported = obd_mask_supports(g_mask1, 0x0B);
    g_maf_supported = obd_mask_supports(g_mask1, 0x10);

    for (int i = 0; i < PROFILE_QV_POLL_COUNT; i++) {
        uint8_t pid = PROFILE_QV_POLL[i].pid;
        obd_sched_add(&g_sched, pid,
                      obd_profile_base_period(pid),
                      PROFILE_QV_POLL[i].priority);
    }
    for (int i = 0; i < PROFILE_EXTRA_POLL_COUNT; i++) {
        if (obd_mask_supports(g_mask1, PROFILE_EXTRA_POLL[i].pid)) {
            /* Modo rapido: se salta la carga para acortar el barrido. */
            if (g_cfg.fast && PROFILE_EXTRA_POLL[i].pid == 0x04) {
                continue;
            }
            uint8_t pid = PROFILE_EXTRA_POLL[i].pid;
            obd_sched_add(&g_sched, pid, obd_profile_base_period(pid),
                          PROFILE_EXTRA_POLL[i].priority);
        }
    }

    /* Si el coche no tiene MAP pero si MAF, se usa MAF como medida
     * principal. El boost no se puede calcular sin MAP. */
    if (!g_map_supported && g_maf_supported) {
        obd_sched_add(&g_sched, 0x10, obd_profile_base_period(0x10), 1);
    }

    /* 0x42: tension del modulo, solo si el bloque 0x40 lo soporta y
     * no estamos en modo rapido. */
    if (!g_cfg.fast && (g_mask3 & 0x40000000u)) {
        obd_sched_add(&g_sched, 0x42, obd_profile_base_period(0x42), 3);
    }

    /* Aceite y combustible, solo si estan soportados. */
    g_oil_supported = (g_mask3 & 0x00000010u) ? 1 : 0;
    g_fuel_supported = (g_mask2 & 0x00020000u) ? 1 : 0;
    if (!g_cfg.fast && g_oil_supported) {
        obd_sched_add(&g_sched, 0x5C, obd_profile_base_period(0x5C), 4);
    }
    if (!g_cfg.fast && g_fuel_supported) {
        obd_sched_add(&g_sched, 0x2F, obd_profile_base_period(0x2F), 4);
    }

    /* Motor avanzado: trims, avance, lambda, distancia con MIL. */
    if (!g_cfg.fast) {
        for (int i = 0; i < PROFILE_MOTOR_POLL_COUNT; i++) {
            uint8_t pid = PROFILE_MOTOR_POLL[i].pid;
            obd_sched_add(&g_sched, pid, obd_profile_base_period(pid),
                          PROFILE_MOTOR_POLL[i].priority);
        }
    }

    /* Pedal del acelerador: 0x49 (posicion D) o 0x4A (posicion E)
     * cuando la ECU los soporta. 0x5A es la posicion relativa y
     * 0x11 es la mariposa, que se mantiene siempre como referencia. */
    g_thr_pid = 0x11;
    g_thr_label = "MARIPOSA";
    g_pedal_supported = 0;
    if (g_mask3 & 0x00800000u) {          /* bit 0x49 */
        obd_sched_add(&g_sched, 0x49, obd_profile_base_period(0x49), 1);
        g_thr_pid = 0x49;
        g_thr_label = "ACELERADOR";
        g_pedal_supported = 1;
    } else if (g_mask3 & 0x00400000u) {   /* bit 0x4A */
        obd_sched_add(&g_sched, 0x4A, obd_profile_base_period(0x4A), 1);
        g_thr_pid = 0x4A;
        g_thr_label = "ACELERADOR";
        g_pedal_supported = 1;
    } else if (g_mask3 & 0x00000040u) {   /* bit 0x5A */
        obd_sched_add(&g_sched, 0x5A, obd_profile_base_period(0x5A), 1);
        g_thr_pid = 0x5A;
        g_thr_label = "ACELERADOR";
        g_pedal_supported = 1;
    }

    apply_support_masks();
    vehicle_detect();

    /* Si la pantalla en la que estabamos no vale para este coche (p. ej.
     * SNIFFER elegido antes de conectar con un K-line), se vuelve a
     * GENERAL: quedarse ahi ofrece un boton que habla CAN en un bus que no
     * lo es. */
    if (!mode_is_driving(g_mode) && !screen_available(g_mode)) {
        g_mode = MODE_QV;
        g_picker_open = 0;
    }

    log_file_printf("Acelerador: 0x%02X (%s), pedal=%s\n",
                    g_thr_pid, g_thr_label,
                    g_pedal_supported ? "SI" : "NO");
    log_file_printf("Soporte: aceite=%s combustible=%s mariposa=0x11\n",
                    g_oil_supported ? "SI" : "NO",
                    g_fuel_supported ? "SI" : "NO");

    /* FASE K2: guardar las prioridades de la tabla, que son a las que se
     * vuelve en cuanto el enlace deja de estar comprimido. */
    for (int i = 0; i < g_sched.count && i < OBD_SCHED_MAX; i++) {
        g_prio_orig[i] = g_sched.items[i].priority;
    }

    /* FASE F3: los periodos dependen de la pantalla activa. */
    apply_screen_schedule(now_ms());
}

/* ------------------------------------------------------------------ */
/* Conexion WiFi + ELM327, con avisos sobre el dashboard.             */
/* ------------------------------------------------------------------ */

static int connect_network_and_elm(Target *targets, int *target_count,
                                   int allow_dialog)
{
    g_net_busy = 1;
    g_net_ui_last = 0;
    g_link_connected = 0;
    g_connect_cancel = 0;
    g_elm_stuck = 0;

    obd_store_init(&g_store);
    boost_init(&g_boost, PROFILE_BARO_FALLBACK_KPA);
    g_mask1 = 0;
    g_mask2 = 0;
    g_mask3 = 0;
    g_vehicle_seen = 0;
    g_thr_pid = 0x11;
    g_thr_label = "MARIPOSA";
    g_oil_supported = 0;
    g_fuel_supported = 0;
    g_pedal_supported = 0;
    g_map_supported = 0;
    g_maf_supported = 0;

    int state = 0;
    sceNetApctlGetState(&state);
    int got_ip = (state == PSP_NET_APCTL_STATE_GOT_IP);

    if (allow_dialog) {
        /* SQUARE: configuracion WiFi de la PSP, conectado o no. */
        net_ui_set("ABRIENDO WIFI");
        connect_yield();

        run_netconf_dialog();
        sceKernelDelayThread(500 * 1000);

        net_ui_set("CONECTANDO WIFI");
        connect_yield();
        got_ip = (wait_for_ip(20000) == 0);

        if (!got_ip && !g_connect_cancel) {
            sceNetApctlConnect(1);
            got_ip = (wait_for_ip(20000) == 0);
        }
    } else if (!got_ip) {
        /* TRIANGLE / arranque: perfil guardado, sin dialogo. */
        net_ui_set("CONECTANDO WIFI");
        connect_yield();

        int cret = sceNetApctlConnect(1);
        log_file_printf("apctl connect(1): 0x%08X\n", cret);
        got_ip = (wait_for_ip(15000) == 0);
    }

    if (g_connect_cancel) {
        log_file_printf("Conexion cancelada por el usuario.\n");
        net_ui_set("CANCELADO");
        g_net_busy = 0;
        return -1;
    }

    if (!got_ip) {
        net_ui_set("SIN WIFI");
        connect_yield();
        log_file_printf("Sin IP (estado %d).\n", state);
        g_net_busy = 0;
        return -1;
    }

    read_net_info();

    char gateway[32];
    if (get_gateway(gateway, sizeof(gateway)) == 0) {
        add_target(targets, target_count, MAX_TARGETS,
                   gateway, DEFAULT_PORT);
    }

    int sock = -1;

    /* FASE D3: hasta 2 rondas. Si tenemos IP pero ningun ELM responde
     * por TCP, la asociacion WiFi puede estar obsoleta (es el fallo
     * tipico del primer reintento tras perder el enlace: "TCP error
     * -3"). Se reconecta el AP y se reintenta una vez. */
    for (int net_try = 0; net_try < 2 && sock < 0; net_try++) {
        if (net_try > 0) {
            log_file_printf("TCP sin respuesta; reconectando WiFi...\n");
            log_sync();
            net_ui_set("RECONECTANDO WIFI");
            connect_yield();
            sceNetApctlDisconnect();
            sceKernelDelayThread(500 * 1000);
            int cret = sceNetApctlConnect(1);
            log_file_printf("apctl reconnect: 0x%08X\n", cret);
            if (wait_for_ip(10000) != 0) {
                log_file_printf("Sin IP tras reconexion.\n");
                break;
            }
            read_net_info();
        }

        net_ui_set("BUSCANDO ELM327");
        connect_yield();

        for (int i = 0; i < *target_count; i++) {
            log_file_printf("Probando %s:%d ...\n",
                            targets[i].ip, targets[i].port);
            int s = tcp_connect(targets[i].ip, targets[i].port, 5000);
            if (s < 0) {
                log_file_printf("TCP error %d (errno %d)\n",
                                s, sceNetInetGetErrno());
                if (g_connect_cancel) {
                    break;
                }
                continue;
            }

            /* D2: drenar restos y desactivar el eco antes de identificar,
             * para no capturar "ATI" como nombre del adaptador. */
            elm_flush_input(s);

            elm_reply reply;
            elm_send_and_read(s, "ATE0", 1000, &reply);
            elm_send_and_read(s, "ATI", 2500, &reply);
            if (reply.bytes > 0) {
                strncpy(g_elm, reply.text, sizeof(g_elm) - 1);
                g_elm[sizeof(g_elm) - 1] = '\0';
                sock = s;
                log_file_printf("Adaptador: %s\n", g_elm);
                break;
            }

            log_file_printf("Sin respuesta en %s\n", targets[i].ip);
            sceNetInetClose(s);
        }

        if (g_connect_cancel) {
            break;
        }
    }

    if (g_connect_cancel) {
        log_file_printf("Conexion cancelada por el usuario.\n");
        net_ui_set("CANCELADO");
        g_net_busy = 0;
        return -1;
    }

    if (sock < 0) {
        net_ui_set("SIN ADAPTADOR");
        connect_yield();
        log_file_printf("Sin adaptador ELM327.\n");
        g_net_busy = 0;
        return -1;
    }

    /* D2: limpiar restos antes de configurar; el desincronizado hacia
     * que cada comando leyera la respuesta del anterior. */
    elm_flush_input(sock);

    net_ui_set("CONFIGURANDO ELM327");
    connect_yield();

    elm_reply reply;

    /* Handshake validado: si 0100 no devuelve mascara (00000000) el
     * adaptador no esta listo; se repite la secuencia AT completa tras
     * drenar el socket (hasta 2 intentos). */
    for (int hs = 0; hs < 2 && !g_connect_cancel; hs++) {
        if (hs > 0) {
            log_file_printf("Handshake: reintento %d/2\n", hs + 1);
            elm_flush_input(sock);
        }

        elm_send_and_read(sock, "ATE0", 1000, &reply);
        elm_send_and_read(sock, "ATL0", 1000, &reply);
        elm_send_and_read(sock, "ATS0", 1000, &reply);
        elm_send_and_read(sock, "ATH0", 1000, &reply);
        {
            char sp[16];
            snprintf(sp, sizeof(sp), "ATSP%d", g_proto_force);
            log_file_printf("Protocolo: %s (%s)\n",
                            g_proto_force ? "forzado" : "automatico", sp);
            elm_send_and_read(sock, sp, 3000, &reply);
        }

        elm_send_and_read(sock, "ATDPN", 2000, &reply);
        strncpy(g_proto, reply.text, sizeof(g_proto) - 1);
        g_proto[sizeof(g_proto) - 1] = '\0';
        log_file_printf("ATDPN: %s\n", g_proto);

        elm_send_and_read(sock, "ATRV", 2000, &reply);
        strncpy(g_vbat, reply.text, sizeof(g_vbat) - 1);
        g_vbat[sizeof(g_vbat) - 1] = '\0';
        log_file_printf("ATRV : %s\n", g_vbat);

        net_ui_set("LEYENDO PIDs");
        connect_yield();
        if (g_connect_cancel) {
            break;
        }

        g_mask1 = 0;
        elm_send_and_read(sock, "0100", 3000, &reply);
        if (reply.data && reply.frame.data_len >= 4) {
            g_mask1 = obd_pid_mask(reply.frame.data, reply.frame.data_len);
        }
        log_file_printf("mascara 0100: %08X\n", (unsigned)g_mask1);

        if (g_mask1 != 0) {
            break;
        }
        log_file_printf("Handshake: 0100 sin mascara, se reintenta\n");
    }

    if (g_connect_cancel) {
        log_file_printf("Conexion cancelada por el usuario.\n");
        sceNetInetClose(sock);
        net_ui_set("CANCELADO");
        g_net_busy = 0;
        return -1;
    }

    if (g_mask1 == 0) {
        log_file_printf("Handshake: adaptador sin datos OBD (0100=00000000)\n");
    }

    if (obd_mask_supports(g_mask1, 0x20)) {
        elm_send_and_read(sock, "0120", 3000, &reply);
        if (reply.data && reply.frame.data_len >= 4) {
            g_mask2 = obd_pid_mask(reply.frame.data,
                                   reply.frame.data_len);
        }
        log_file_printf("mascara 0120: %08X\n", (unsigned)g_mask2);
    } else {
        log_file_printf("0120: bloque no soportado\n");
    }

    /* Mascara 0140: PIDs 0x41-0x60 (acelerador 0x49/0x4A/0x5A, etc.). */
    if (g_mask2 & 0x00000001u) {
        elm_send_and_read(sock, "0140", 3000, &reply);
        if (reply.data && reply.frame.data_len >= 4) {
            g_mask3 = obd_pid_mask(reply.frame.data,
                                   reply.frame.data_len);
        }
        log_file_printf("mascara 0140: %08X\n", (unsigned)g_mask3);
    } else {
        log_file_printf("0140: bloque no soportado\n");
    }

    /* Codificacion no estandar del MiTo 1.4 170cv: se detecta por sus
     * mascaras reales para no alterar la decodificacion de otros coches. */
    int mito_encoding =
        (g_mask1 == 0xBE3EB811u &&
         g_mask2 == 0x80022001u &&
         g_mask3 == 0xC0C00000u);
    obd_pid_set_mito_encoding(mito_encoding);
    log_file_printf("Codificacion: %s\n",
                    mito_encoding ? "MiTo (trims con signo, avance directo)"
                                  : "OBD estandar");

    build_scheduler();

    /* ATDPN antes de la primera peticion OBD puede devolver "A0"
     * (automatico, protocolo aun sin detectar). Se relee ya con el
     * protocolo real para que el escaneo UDS y la restauracion usen
     * el valor correcto. */
    scan_refresh_proto(sock, "conexion");

    g_link_connected = 1;
    g_net_busy = 0;
    log_file_printf("Conexion lista: %d PID.\n", g_sched.count);
    log_sync();
    return sock;
}

/* ------------------------------------------------------------------ */
/* Descubrimiento de modulos.                                         */
/*                                                                    */
/* Solo lectura: se usan servicios OBD-II (01/09) y UDS de lectura     */
/* (3E Tester Present y 22 ReadDataByIdentifier). Nunca se escribe,    */
/* no se borran codigos y no se envian actuadores.                    */
/* ------------------------------------------------------------------ */

static unsigned scan_hex(const char *text)
{
    unsigned value = 0;
    for (; text != NULL && *text != '\0'; text++) {
        int d = -1;
        if (*text >= '0' && *text <= '9') {
            d = *text - '0';
        } else if (*text >= 'A' && *text <= 'F') {
            d = *text - 'A' + 10;
        } else if (*text >= 'a' && *text <= 'f') {
            d = *text - 'a' + 10;
        } else {
            break;
        }
        value = (value << 4) | (unsigned)d;
    }
    return value;
}

static void scan_refresh_proto(int sock, const char *tag)
{
    if (sock < 0) {
        return;
    }

    elm_reply reply;
    elm_send_and_read(sock, "ATDPN", 2000, &reply);
    if (reply.text[0] == '\0') {
        return;
    }

    snprintf(g_proto, sizeof(g_proto), "%.31s", reply.text);
    log_file_printf("ATDPN (%s): %s\n", tag, g_proto);
}

static int scan_can_11bit(void)
{
    size_t n = strlen(g_proto);
    if (n == 0) {
        return 0;
    }
    char c = g_proto[n - 1];
    return c == '6' || c == '8';
}

static int scan_can_29bit(void)
{
    size_t n = strlen(g_proto);
    if (n == 0) {
        return 0;
    }
    char c = g_proto[n - 1];
    return c == '7' || c == '9';
}

static int scan_can(void)
{
    return scan_can_11bit() || scan_can_29bit();
}

static scan_module *scan_module_for(const char *header, const char *req)
{
    const char *hdr = (header != NULL && header[0] != '\0')
                          ? header : "???";

    for (int i = 0; i < g_scan_count; i++) {
        if (strcmp(g_scan[i].header, hdr) == 0) {
            if (req != NULL && req[0] != '\0' && g_scan[i].req[0] == '\0') {
                strncpy(g_scan[i].req, req, sizeof(g_scan[i].req) - 1);
                g_scan[i].req[sizeof(g_scan[i].req) - 1] = '\0';
            }
            return &g_scan[i];
        }
    }

    if (g_scan_count >= SCAN_MAX_MODULES) {
        return NULL;
    }

    scan_module *m = &g_scan[g_scan_count++];
    memset(m, 0, sizeof(*m));
    strncpy(m->header, hdr, sizeof(m->header) - 1);
    m->header[sizeof(m->header) - 1] = '\0';
    if (req != NULL && req[0] != '\0') {
        strncpy(m->req, req, sizeof(m->req) - 1);
        m->req[sizeof(m->req) - 1] = '\0';
    }
    return m;
}

static void scan_status(const char *msg)
{
    snprintf(g_scan_status, sizeof(g_scan_status), "%s", msg);
    net_ui_set(g_scan_status);
    net_ui_tick();
}

static int scan_cancelled(void)
{
    SceCtrlData pad;
    memset(&pad, 0, sizeof(pad));
    sceCtrlReadBufferPositive(&pad, 1);
    return (pad.Buttons & PSP_CTRL_CIRCLE) != 0;
}

/* Ejecuta un comando y guarda todas sus lineas de respuesta. */
static void scan_send(int sock, const char *cmd, int wait_ms)
{
    elm_reply reply;
    g_cap_count = 0;
    g_cap_on = 1;
    elm_send_and_read(sock, cmd, wait_ms, &reply);
    g_cap_on = 0;
}

static void scan_collect_masks(void)
{
    for (int i = 0; i < g_cap_count; i++) {
        obd_disc_line line;
        if (!obd_disc_parse_line(g_cap[i], &line)) {
            continue;
        }

        const uint8_t *payload = NULL;
        int len = obd_disc_payload(&line, &payload);
        if (len < 6 || payload[0] != 0x41) {
            continue;
        }

        uint8_t pid = payload[1];
        if (pid != 0x00 && pid != 0x20 && pid != 0x40) {
            continue;
        }

        scan_module *m = scan_module_for(line.header, NULL);
        if (m == NULL) {
            continue;
        }

        m->obd = 1;
        uint32_t mask = obd_disc_mask(payload + 2, len - 2);
        if (pid == 0x00) {
            m->mask1 = mask;
        } else if (pid == 0x20) {
            m->mask2 = mask;
        } else {
            m->mask3 = mask;
        }
    }
}

static void scan_obd(int sock)
{
    if (scan_cancelled()) {
        return;
    }

    scan_status("OBD-II: 0100");
    scan_send(sock, "0100", 2500);
    scan_collect_masks();

    int need20 = 0;
    for (int i = 0; i < g_scan_count; i++) {
        if (g_scan[i].mask1 != 0 &&
            obd_mask_supports(g_scan[i].mask1, 0x20)) {
            need20 = 1;
        }
    }
    if (need20 && !scan_cancelled()) {
        scan_status("OBD-II: 0120");
        scan_send(sock, "0120", 2500);
        scan_collect_masks();
    }

    int need40 = 0;
    for (int i = 0; i < g_scan_count; i++) {
        if (g_scan[i].mask2 != 0 &&
            obd_mask_supports(g_scan[i].mask2, 0x20)) {
            need40 = 1;
        }
    }
    if (need40 && !scan_cancelled()) {
        scan_status("OBD-II: 0140");
        scan_send(sock, "0140", 2500);
        scan_collect_masks();
    }
}

/*
 * Igual que obd_disc_payload pero sin descartar las tramas que empiezan
 * por 0x3N: en texto ASCII un VIN puede empezar por '0' (0x30) en una
 * linea de continuacion. Un flow control real no lo envia la ECU.
 */
static int scan_line_payload(const obd_disc_line *line,
                             const uint8_t **payload)
{
    if (line->data_len <= 0) {
        return 0;
    }

    const uint8_t *d = line->data;
    int n = line->data_len;
    int type = (d[0] >> 4) & 0x0F;

    if (type == 0) {
        int len = d[0] & 0x0F;
        if (len <= 0 || n < 2) {
            return 0;
        }
        if (len > n - 1) {
            len = n - 1;
        }
        *payload = d + 1;
        return len;
    }
    if (type == 1) {
        if (n < 3) {
            return 0;
        }
        *payload = d + 2;
        return n - 2;
    }
    if (type == 2) {
        *payload = d + 1;
        return n - 1;
    }

    *payload = d;
    return n;
}

/*
 * Extrae el texto ASCII de una respuesta 62 F1 xx. Sirve tanto para
 * tramas single frame como para ISO-TP multi-frame y para los clones
 * que imprimen prefijos "0:", "1:".
 */
static void scan_extract_did_text(uint8_t did, const char *want_resp,
                                  char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) {
        return;
    }
    out[0] = '\0';

    int active = 0;
    size_t used = 0;

    for (int i = 0; i < g_cap_count && used + 1 < out_size; i++) {
        obd_disc_line line;
        if (!obd_disc_parse_line(g_cap[i], &line)) {
            continue;
        }

        /* Solo se aceptan lineas del modulo pedido: con la direccion
         * funcional pueden contestar varias ECUs a la vez. */
        if (want_resp != NULL && want_resp[0] != '\0' &&
            line.header[0] != '\0' &&
            strcmp(line.header, want_resp) != 0) {
            continue;
        }

        const uint8_t *payload = NULL;
        int len = scan_line_payload(&line, &payload);
        if (len <= 0) {
            continue;
        }

        int start = -1;
        for (int j = 0; j + 2 < len; j++) {
            if (payload[j] == 0x62 && payload[j + 1] == 0xF1 &&
                payload[j + 2] == did) {
                start = j + 3;
                active = 1;
                break;
            }
        }
        if (start < 0) {
            if (!active) {
                continue;
            }
            start = 0;
        }

        for (int j = start; j < len && used + 1 < out_size; j++) {
            uint8_t c = payload[j];
            if (c >= 0x20 && c <= 0x7E) {
                if (!(c == ' ' && used == 0)) {
                    out[used++] = (char)c;
                }
            } else if (used > 0) {
                break;
            }
        }
    }

    out[used] = '\0';
    while (used > 0 && out[used - 1] == ' ') {
        out[--used] = '\0';
    }
}

static void scan_read_did(int sock, const char *req, uint8_t low,
                          const char *want_resp,
                          char *out, size_t out_size)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "22F1%02X", (unsigned)low);
    scan_send(sock, cmd, 1200);
    scan_extract_did_text(low, want_resp, out, out_size);

    if (out[0] == '\0') {
        for (int i = 0; i < g_cap_count; i++) {
            obd_disc_line line;
            if (!obd_disc_parse_line(g_cap[i], &line)) {
                continue;
            }
            const uint8_t *payload = NULL;
            int len = obd_disc_payload(&line, &payload);
            if (len >= 3 && payload[0] == 0x7F && payload[1] == 0x22) {
                log_file_printf("UDS %s %s: NRC %02X\n",
                                req, cmd, (unsigned)payload[2]);
                break;
            }
        }
    }
}

/* 1 si la ultima respuesta del ELM327 fue un error. */
static int scan_cap_error(void)
{
    for (int i = 0; i < g_cap_count; i++) {
        const char *s = g_cap[i];
        while (*s == ' ' || *s == '\t') {
            s++;
        }
        if (strcmp(s, "?") == 0 ||
            strncmp(s, "ERROR", 5) == 0 ||
            strncmp(s, "UNABLE", 6) == 0) {
            return 1;
        }
    }
    return 0;
}

/* Cabecera CAN. Los ELM327 antiguos solo aceptan los 3 bytes finales de
 * una ID de 29 bits en ATSH; el byte de prioridad se fija con AT CP.
 * Se intenta primero ese formato y luego el de 8 digitos completo. */
static int scan_set_header(int sock, const char *id)
{
    char cmd[24];

    if (strlen(id) == 8) {
        char cp[16];
        snprintf(cp, sizeof(cp), "ATCP %c%c", id[0], id[1]);
        scan_send(sock, cp, 500);

        snprintf(cmd, sizeof(cmd), "ATSH %s", id + 2);
        scan_send(sock, cmd, 500);
        if (!scan_cap_error()) {
            return 1;
        }

        snprintf(cmd, sizeof(cmd), "ATSH %s", id);
        scan_send(sock, cmd, 500);
        if (!scan_cap_error()) {
            return 1;
        }
        return 0;
    }

    snprintf(cmd, sizeof(cmd), "ATSH %s", id);
    scan_send(sock, cmd, 500);
    return !scan_cap_error();
}

/* Cabecera de respuesta esperada para una peticion. */
static void scan_expected_resp(const char *req, char *out, size_t out_size)
{
    out[0] = '\0';

    if (strlen(req) == 8 && strncmp(req, "18DA", 4) == 0) {
        /* 18 DA xx F1 -> 18 DA F1 xx. */
        snprintf(out, out_size, "18DAF1%.2s", req + 4);
    } else if (strlen(req) == 3) {
        unsigned r = scan_hex(req);
        snprintf(out, out_size, "%03X", (r + 8u) & 0xFFFu);
    }
}

static int scan_uds_probe(int sock, const char *req,
                          char *resp, size_t resp_size)
{
    char want[16];
    scan_expected_resp(req, want, sizeof(want));

    scan_set_header(sock, req);
    scan_send(sock, "3E00", 800);

    int present = 0;
    resp[0] = '\0';

    for (int i = 0; i < g_cap_count; i++) {
        obd_disc_line line;
        if (!obd_disc_parse_line(g_cap[i], &line)) {
            continue;
        }
        const uint8_t *payload = NULL;
        int len = obd_disc_payload(&line, &payload);
        int ok = (len >= 2 && payload[0] == 0x7E && payload[1] == 0x00) ||
                 (len >= 3 && payload[0] == 0x7F && payload[1] == 0x3E);
        if (!ok) {
            continue;
        }

        /* En direccion fisica solo vale la respuesta de esa ECU. Si la
         * cabecera no se pudo fijar, el ELM327 responde de forma
         * funcional: se descarta para no inventar modulos. */
        if (want[0] != '\0' &&
            (line.header[0] == '\0' || strcmp(line.header, want) != 0)) {
            continue;
        }

        present = 1;
        if (line.header[0] != '\0') {
            snprintf(resp, resp_size, "%s", line.header);
        }
        break;
    }

    if (present && resp[0] == '\0') {
        snprintf(resp, resp_size, "%s", want);
    }

    return present;
}

/* Lee los DIDs estandar de identificacion de un modulo ya detectado. */
static void scan_uds_fill(int sock, const char *req, const char *resp)
{
    scan_module *m = scan_module_for(resp, req);
    if (m == NULL) {
        return;
    }
    m->uds = 1;

    /* Asegura la cabecera fisica antes de leer los DIDs. */
    scan_set_header(sock, req);

    log_file_printf("UDS %s -> %s detectado\n", req, resp);

    scan_status("UDS: F190 VIN");
    scan_read_did(sock, req, 0x90, resp, m->vin, sizeof(m->vin));
    scan_status("UDS: F197 SISTEMA");
    scan_read_did(sock, req, 0x97, resp, m->name, sizeof(m->name));
    scan_status("UDS: F187 PIEZA");
    scan_read_did(sock, req, 0x87, resp, m->part, sizeof(m->part));
    scan_status("UDS: F188 SW");
    scan_read_did(sock, req, 0x88, resp, m->sw, sizeof(m->sw));
    scan_status("UDS: F18C SERIE");
    scan_read_did(sock, req, 0x8C, resp, m->serial, sizeof(m->serial));

    log_file_printf("  VIN='%s' sistema='%s' pieza='%s' sw='%s' serie='%s'\n",
                    m->vin, m->name, m->part, m->sw, m->serial);
}

/* 1 si ya hay un modulo con esa cabecera de peticion. */
static int scan_req_known(const char *req)
{
    for (int i = 0; i < g_scan_count; i++) {
        if (g_scan[i].req[0] != '\0' &&
            strcmp(g_scan[i].req, req) == 0) {
            return 1;
        }
    }
    return 0;
}

/* Saca la direccion de ECU de una respuesta 18DAF1xx. Devuelve -1 si no
 * tiene ese formato. */
static int scan_ecu_from_header(const char *header)
{
    if (header == NULL || strlen(header) != 8 ||
        strncmp(header, "18DAF1", 6) != 0) {
        return -1;
    }
    return (int)scan_hex(header + 6);
}

static void scan_uds_11(int sock)
{
    for (int a = 0; a < 8; a++) {
        if (scan_cancelled()) {
            log_file_printf("Escaneo cancelado por el usuario\n");
            return;
        }
        if (g_scan_count >= SCAN_MAX_MODULES) {
            log_file_printf("Escaneo: limite de modulos alcanzado\n");
            return;
        }

        char req[8];
        snprintf(req, sizeof(req), "%03X", 0x7E0 + a);
        char status[32];
        snprintf(status, sizeof(status), "UDS: SONDEANDO %s", req);
        scan_status(status);

        char resp[16];
        int present = scan_uds_probe(sock, req, resp, sizeof(resp));
        if (!present) {
            continue;
        }

        scan_uds_fill(sock, req, resp);
    }
}

/* Sondeo UDS en CAN de 29 bits. Las respuestas OBD llegan como
 * 18 DA F1 xx, asi que la ECU xx se direcciona con 18 DA xx F1.
 * Primero se prueba la direccion funcional 18DB33F1 (una sola peticion
 * puede contestar varios modulos) y despues se sondean las direcciones
 * fisicas 0x00-0xFF para encontrar modulos que no responden a OBD-II. */
static void scan_uds_29(int sock)
{
    /* Direccion funcional: puede responder mas de un modulo. Primero se
     * recogen las respuestas en local, porque scan_uds_fill vuelve a
     * usar g_cap para leer los DIDs. */
    scan_status("UDS: FUNCIONAL 18DB33F1");
    int hdr_ok = scan_set_header(sock, "18DB33F1");
    log_file_printf("UDS29: cabecera funcional %s\n",
                    hdr_ok ? "OK" : "ERROR (se usa la de por defecto)");
    scan_send(sock, "3E00", 1500);

    char found_req[16][16];
    char found_resp[16][16];
    int found_count = 0;

    for (int i = 0; i < g_cap_count && found_count < 16; i++) {
        obd_disc_line line;
        if (!obd_disc_parse_line(g_cap[i], &line)) {
            continue;
        }
        const uint8_t *payload = NULL;
        int len = obd_disc_payload(&line, &payload);
        if (len < 2) {
            continue;
        }
        if (!((payload[0] == 0x7E && payload[1] == 0x00) ||
              (payload[0] == 0x7F && payload[1] == 0x3E))) {
            continue;
        }
        int ecu = scan_ecu_from_header(line.header);
        if (ecu < 0) {
            continue;
        }
        char req[16];
        snprintf(req, sizeof(req), "18DA%02XF1", (unsigned)ecu);
        if (scan_req_known(req)) {
            continue;
        }
        int dup = 0;
        for (int k = 0; k < found_count; k++) {
            if (strcmp(found_req[k], req) == 0) {
                dup = 1;
                break;
            }
        }
        if (dup) {
            continue;
        }
        snprintf(found_req[found_count], sizeof(found_req[0]), "%s", req);
        snprintf(found_resp[found_count], sizeof(found_resp[0]), "%s",
                 line.header);
        found_count++;
    }

    for (int k = 0; k < found_count; k++) {
        scan_uds_fill(sock, found_req[k], found_resp[k]);
    }

    /* Direcciones fisicas 0x00-0xFF. */
    for (int addr = 0; addr <= 0xFF; addr++) {
        if (scan_cancelled()) {
            log_file_printf("Escaneo cancelado por el usuario\n");
            return;
        }
        if (g_scan_count >= SCAN_MAX_MODULES) {
            log_file_printf("Escaneo: limite de modulos alcanzado\n");
            return;
        }

        char req[16];
        snprintf(req, sizeof(req), "18DA%02XF1", (unsigned)addr);
        if (scan_req_known(req)) {
            continue;
        }

        if ((addr & 0x0F) == 0) {
            char status[40];
            snprintf(status, sizeof(status), "UDS: SONDEANDO %02X", addr);
            scan_status(status);
        }

        char resp[16];
        int present = scan_uds_probe(sock, req, resp, sizeof(resp));
        if (!present) {
            continue;
        }

        scan_uds_fill(sock, req, resp);
    }
}

static void scan_uds(int sock)
{
    if (!scan_can()) {
        log_file_printf("UDS: se omite (protocolo no CAN)\n");
        return;
    }

    elm_reply reply;
    /* Formato automatico CAN: el ELM327 anade el PCI de ISO-TP. */
    elm_send_and_read(sock, "ATCAF1", 1000, &reply);
    elm_send_and_read(sock, "ATAL", 1000, &reply);

    if (scan_can_11bit()) {
        scan_uds_11(sock);
    } else {
        scan_uds_29(sock);
    }
}

static void scan_restore(int sock)
{
    elm_reply reply;

    /* No se usa ATD: resetearia el protocolo y obligaria al ELM327 a
     * buscarlo otra vez, con el riesgo de timeouts al volver al
     * dashboard. Solo se deshacen los cambios del escaneo. */
    if (scan_can_11bit()) {
        scan_set_header(sock, "7DF");
        elm_send_and_read(sock, "ATNL", 800, &reply);
    } else if (scan_can_29bit()) {
        scan_set_header(sock, "18DB33F1");
        elm_send_and_read(sock, "ATNL", 800, &reply);
    }
    elm_send_and_read(sock, "ATH0", 800, &reply);
    elm_send_and_read(sock, "ATS0", 800, &reply);
    elm_send_and_read(sock, "ATE0", 800, &reply);
    elm_send_and_read(sock, "ATL0", 800, &reply);
}

static void run_scan(int sock)
{
    if (g_scan_state == 1) {
        return;
    }

    g_scan_state = 1;
    g_scan_count = 0;
    g_scan_sel = 0;
    g_scan_status[0] = '\0';
    snprintf(g_scan_proto, sizeof(g_scan_proto), "%s", g_proto);

    log_file_printf("=== ESCANEO DE MODULOS ===\n");
    log_file_printf("Protocolo: %s\n", g_proto);

    if (sock < 0) {
        g_scan_state = 2;
        snprintf(g_scan_status, sizeof(g_scan_status), "SIN ADAPTADOR");
        log_file_printf("Escaneo: sin adaptador\n");
        log_sync();
        return;
    }

    g_net_busy = 1;
    g_net_ui_last = 0;
    scan_status("PREPARANDO ESCANEO");
    net_ui_tick();

    elm_reply reply;
    elm_send_and_read(sock, "ATE0", 1000, &reply);
    elm_send_and_read(sock, "ATL0", 1000, &reply);
    elm_send_and_read(sock, "ATH1", 1000, &reply);
    elm_send_and_read(sock, "ATS1", 1000, &reply);
    if (scan_can()) {
        /* Formato automatico CAN: el ELM327 anade el PCI de ISO-TP. */
        elm_send_and_read(sock, "ATCAF1", 1000, &reply);
        elm_send_and_read(sock, "ATAL", 1000, &reply);
    }

    scan_obd(sock);

    /* El protocolo puede no haberse detectado antes de scan_obd (ATDPN
     * devolvio "A0"). Tras 0100 ya esta detectado: se relee antes de
     * sondear modulos UDS y de restaurar el ELM327. */
    scan_refresh_proto(sock, "escaneo");
    snprintf(g_scan_proto, sizeof(g_scan_proto), "%s", g_proto);

    /* El sondeo UDS pide cabeceras CAN concretas (ATSH 7E0, 3E00, 22F1xx):
     * en un coche de K-line eso no significa nada y ademas sale de lo que
     * la app promete (01/02/03/06/07/09/0A y comandos AT). Ahi el escaneo
     * se queda en las mascaras de OBD-II, que es lo que identifica a la
     * centralita en ISO 9141-2. */
    if (scan_can()) {
        scan_uds(sock);
    } else {
        log_file_printf("Escaneo: K-line, sondeo UDS omitido\n");
    }

    scan_status("RESTAURANDO ELM327");
    scan_restore(sock);

    g_net_busy = 0;
    g_scan_state = 2;
    g_scan_status[0] = '\0';
    g_net_ui_last = 0;

    for (int i = 0; i < g_scan_count; i++) {
        scan_module *m = &g_scan[i];
        log_file_printf("MOD %-8s req=%-4s obd=%d uds=%d "
                        "m1=%08X m2=%08X m3=%08X "
                        "sistema='%s' pieza='%s' sw='%s' "
                        "vin='%s' serie='%s'\n",
                        m->header, m->req, m->obd, m->uds,
                        (unsigned)m->mask1, (unsigned)m->mask2,
                        (unsigned)m->mask3, m->name, m->part, m->sw,
                        m->vin, m->serial);
    }
    log_file_printf("=== FIN ESCANEO: %d modulos ===\n", g_scan_count);
    log_sync();
}

/* ------------------------------------------------------------------ */
/* Pantalla.                                                          */
/* ------------------------------------------------------------------ */

typedef struct {
    char rpm[16];
    int rpm_valid;
    float rpm_val;

    char speed[16];
    int speed_valid;
    float speed_val;

    char boost[24];
    int boost_valid;
    int boost_est;
    float boost_val;

    char ect[16];
    int ect_valid;
    float ect_val;

    char iat[16];
    int iat_valid;
    float iat_val;

    char map_kpa[16];
    char map_bar[16];
    int map_valid;
    float map_val;      /* kPa absolutos */

    char maf[16];
    int maf_valid;
    float maf_val;      /* g/s */

    char thr[16];
    int thr_valid;
    float thr_val;

    char plate[16];
    int plate_valid;
    float plate_val;

    char load[16];
    int load_valid;
    float load_val;

    char volt[16];
    int volt_valid;
    float volt_val;

    char oil[16];
    int oil_valid;
    float oil_val;

    char fuel[16];
    int fuel_valid;
    float fuel_val;

    char stft[16];
    int stft_valid;
    float stft_val;

    char ltft[16];
    int ltft_valid;
    float ltft_val;

    char timing[16];
    int timing_valid;
    float timing_val;

    char o2_1[16];
    int o2_1_valid;
    float o2_1_val;

    char o2_2[16];
    int o2_2_valid;
    float o2_2_val;

    char dist_mil[16];
    int dist_mil_valid;
    float dist_mil_val;

    char baro_hpa[16];
    int baro_valid;
    float baro_kpa_val;
    const char *baro_src;
} dash_data;

static unsigned int g_session_start_ms = 0;

static float clamp_float(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void fmt_2(char *buf, size_t size, float value)
{
    int neg = (value < 0.0f);
    if (neg) {
        value = -value;
    }
    int scaled = (int)(value * 100.0f + 0.5f);
    if (scaled == 0) {
        neg = 0;
    }
    snprintf(buf, size, "%s%d.%02d", neg ? "-" : "",
             scaled / 100, scaled % 100);
}

static void fmt_signed_2(char *buf, size_t size, float value)
{
    int neg = (value < 0.0f);
    if (neg) {
        value = -value;
    }
    int scaled = (int)(value * 100.0f + 0.5f);
    if (scaled == 0) {
        neg = 0;
    }
    snprintf(buf, size, "%c%d.%02d", neg ? '-' : '+',
             scaled / 100, scaled % 100);
}

/* FASE F2: edad del ultimo dato utilizable, en ms.
 *
 * Devuelve 0xFFFFFFFF si no hay ninguna muestra valida todavia. */
static uint32_t data_age_ms(uint32_t now)
{
    if (g_demo) {
        return 0;
    }
    uint32_t ts = obd_store_newest_ts(&g_store);
    if (ts == 0u) {
        return 0xFFFFFFFFu;
    }
    return (uint32_t)(now - ts);
}

/* FASE F2: umbrales de la edad del dato.
 *
 * OLD: a partir de aqui el contenido se atenua y la cabecera dice
 *      "DATOS VIEJOS". Con el RPM a ~330 ms son tres periodos, asi que
 *      no parpadea con un timeout puntual.
 * DEAD: a partir de aqui el dato ya no sirve y se muestra "SIN DATOS". */
#define DATA_OLD_MS  1000u
#define DATA_DEAD_MS 3000u

static int data_is_stale(uint32_t now)
{
    if (g_demo) {
        return 0;
    }
    return data_age_ms(now) > DATA_DEAD_MS;
}

/* Historial por canal: cada elemento declara su PID de origen. Los
 * calculados (boost/baro) se resuelven aparte. Asi anadir un canal
 * nuevo solo requiere ampliar el enum y esta tabla. */
static const uint8_t g_hist_pid[W_COUNT] = {
    0x0C, 0x0D, 0x00, 0x05, 0x0F, 0x0B, 0x10,
    0x11, 0x04, 0x11, 0x42, 0x5C, 0x2F,
    0x06, 0x07, 0x0E, 0x14, 0x15, 0x21, 0x33
};

static float *tel_values(int item);

static void history_update(uint32_t now)
{
    if (g_hist_count > 0 && (uint32_t)(now - g_hist_last) < 200u) {
        return;
    }
    g_hist_last = now;

    uint32_t mask = 0;

    for (int i = 0; i < W_COUNT; i++) {
        float *hist = tel_values(i);
        if (hist == NULL) {
            continue;
        }

        float v = 0.0f;
        int ok = 0;

        if (i == W_BOOST) {
            if (g_boost.boost_valid) {
                v = g_boost.boost_bar;
                ok = 1;
            }
        } else if (i == W_BARO) {
            if (g_boost.baro_valid) {
                v = g_boost.baro_kpa;
                ok = 1;
            }
        } else if (i == W_THR) {
            ok = sample_value(g_thr_pid, now, &v);
        } else {
            ok = sample_value(g_hist_pid[i], now, &v);
        }

        hist[g_hist_pos] = ok ? v : 0.0f;
        if (ok) {
            mask |= (uint32_t)1u << i;
        }
    }

    g_hist_valid[g_hist_pos] = mask;
    g_hist_pos = (g_hist_pos + 1) % HIST_MAX;
    if (g_hist_count < HIST_MAX) {
        g_hist_count++;
    }
}

/* ------------------------------------------------------------------ */
/* Estilos de aguja y arco por tema.                                  */
/* ------------------------------------------------------------------ */

static int theme_is_game(void)
{
    return g_cfg.theme >= 5;
}

static uint32_t theme_accent(void)
{
    switch (g_cfg.theme) {
    case 5:  return QV_RED;    /* Forza: rosa/festival */
    case 6:  return QV_GREEN;  /* NFS: neon verde */
    case 7:  return QV_CYAN;   /* GT: azul de competicion */
    case 8:  return QV_RED;    /* Initial D: rojo del AE86 */
    case 9:  return QV_CYAN;   /* Cyberpunk: cian neon */
    case 10: return QV_GREEN;  /* Retro: fosforo verde */
    case 11: return QV_RED;    /* Nothing: acento rojo */
    case 12: return QV_CYAN;   /* Back to the Future: cian */
    case 13: return QV_AMBER;  /* Todo gas: naranja */
    default: return QV_RED;
    }
}

/* Fondo del reloj: cada tema dibuja la escala de forma distinta. */
static void theme_track(float cx, float cy, float r, uint32_t color)
{
    switch (g_cfg.theme) {
    case 5: /* Forza: bloques separados tipo festival. */
        for (int i = 0; i < 18; i++) {
            float a0 = 135.0f + 270.0f * (float)i / 18.0f;
            float a1 = a0 + 270.0f / 18.0f - 4.0f;
            uint32_t c = (i % 3 == 0) ? gfx_shade(color, 150) : color;
            gfx_arc(cx, cy, r, 11.0f, a0, a1, 5, c);
        }
        break;

    case 6: /* NFS: dos aros finos con el centro vacio. */
        gfx_arc(cx, cy, r - 4.0f, 2.0f, 135.0f, 405.0f, 54, color);
        gfx_arc(cx, cy, r + 4.0f, 1.0f, 135.0f, 405.0f, 54,
                gfx_shade(color, 70));
        break;

    case 7: /* GT: linea fina con aro tecnico exterior. */
        gfx_arc(cx, cy, r, 2.0f, 135.0f, 405.0f, 54, color);
        gfx_arc(cx, cy, r + 6.0f, 1.0f, 135.0f, 405.0f, 54,
                gfx_shade(color, 160));
        break;

    case 8: /* Initial D: bezel grueso con doble aro retro. */
        gfx_arc(cx, cy, r, 10.0f, 135.0f, 405.0f, 54, color);
        gfx_arc(cx, cy, r - 6.0f, 1.0f, 135.0f, 405.0f, 54,
                gfx_shade(color, 170));
        gfx_arc(cx, cy, r + 6.0f, 1.0f, 135.0f, 405.0f, 54,
                gfx_shade(color, 120));
        break;

    case 9: /* Cyberpunk: doble aro neon. */
        gfx_arc(cx, cy, r + 4.0f, 2.0f, 135.0f, 405.0f, 54,
                gfx_shade(QV_CYAN, 40));
        gfx_arc(cx, cy, r, 3.0f, 135.0f, 405.0f, 54, QV_CYAN);
        gfx_arc(cx, cy, r - 5.0f, 1.0f, 135.0f, 405.0f, 54,
                gfx_shade(QV_CRIT, 60));
        break;

    case 10: /* Retro: bloques LCD separados. */
        for (int i = 0; i < 18; i++) {
            float a0 = 135.0f + 270.0f * (float)i / 18.0f;
            float a1 = a0 + 270.0f / 18.0f - 6.0f;
            gfx_arc(cx, cy, r, 9.0f, a0, a1, 4, QV_GREEN);
        }
        break;

    case 11: /* Nothing: aro fino y limpio. */
        gfx_arc(cx, cy, r, 2.0f, 135.0f, 405.0f, 54, QV_DIM);
        gfx_arc(cx, cy, r - 8.0f, 1.0f, 135.0f, 405.0f, 54,
                gfx_shade(QV_DIM, 40));
        break;

    case 12: /* Back to the Future: doble aro cian/violeta. */
        gfx_arc(cx, cy, r + 5.0f, 2.0f, 135.0f, 405.0f, 54,
                gfx_shade(QV_CYAN, 35));
        gfx_arc(cx, cy, r, 4.0f, 135.0f, 405.0f, 54, QV_CYAN);
        gfx_arc(cx, cy, r - 6.0f, 1.0f, 135.0f, 405.0f, 54,
                gfx_shade(QV_CRIT, 60));
        break;

    case 13: /* Todo gas: aro grueso naranja/azul. */
        gfx_arc(cx, cy, r, 9.0f, 135.0f, 405.0f, 54, QV_AMBER);
        gfx_arc(cx, cy, r - 5.0f, 2.0f, 135.0f, 405.0f, 54,
                gfx_shade(QV_CYAN, 50));
        for (int i = 0; i < 12; i++) {
            float a0 = 135.0f + 270.0f * (float)i / 12.0f;
            float a1 = a0 + 270.0f / 12.0f - 5.0f;
            gfx_arc(cx, cy, r + 4.0f, 3.0f, a0, a1, 3, QV_CYAN);
        }
        break;

    default:
        gfx_arc(cx, cy, r, 8.0f, 135.0f, 405.0f, 54, color);
        break;
    }
}

/* Zona roja. */
static void theme_redline(float cx, float cy, float r, float a0, float a1)
{
    switch (g_cfg.theme) {
    case 5: /* Forza: bloques rojos con halo. */
        for (float a = a0; a < a1; a += 12.0f) {
            float e = a + 8.0f;
            if (e > a1) e = a1;
            gfx_arc(cx, cy, r, 17.0f, a, e, 3,
                    gfx_shade(QV_RED, 18));
            gfx_arc(cx, cy, r, 11.0f, a, e, 3, QV_RED);
        }
        break;

    case 6: /* NFS: neon rojo fino. */
        gfx_arc(cx, cy, r, 15.0f, a0, a1, 14,
                gfx_shade(QV_RED, 18));
        gfx_arc(cx, cy, r - 4.0f, 2.0f, a0, a1, 14, QV_RED);
        break;

    case 7: /* GT: zona roja rayada como en circuito. */
        for (float a = a0; a <= a1; a += 7.0f) {
            float rad = a * 0.0174532925f;
            gfx_line(cx + (r - 9.0f) * cosf(rad),
                     cy + (r - 9.0f) * sinf(rad),
                     cx + (r + 9.0f) * cosf(rad),
                     cy + (r + 9.0f) * sinf(rad),
                     1.5f, QV_RED);
        }
        break;

    case 8: /* Initial D: rojo macizo con filo blanco. */
        gfx_arc(cx, cy, r, 12.0f, a0, a1, 14, QV_RED);
        gfx_arc(cx, cy, r - 7.0f, 1.0f, a0, a1, 14, QV_TEXT);
        break;

    case 9: /* Cyberpunk: magenta neon. */
        gfx_arc(cx, cy, r, 14.0f, a0, a1, 14, gfx_shade(QV_CRIT, 30));
        gfx_arc(cx, cy, r, 6.0f, a0, a1, 14, QV_CRIT);
        break;

    case 10: /* Retro: bloques rojos. */
        for (float a = a0; a < a1; a += 8.0f) {
            float e = a + 5.0f;
            if (e > a1) e = a1;
            gfx_arc(cx, cy, r, 10.0f, a, e, 2, QV_RED);
        }
        break;

    case 11: /* Nothing: linea roja fina. */
        gfx_arc(cx, cy, r, 3.0f, a0, a1, 14, QV_RED);
        break;

    case 12: /* Back to the Future: magenta con brillo. */
        gfx_arc(cx, cy, r, 12.0f, a0, a1, 14, gfx_shade(QV_CRIT, 30));
        gfx_arc(cx, cy, r, 5.0f, a0, a1, 14, QV_CRIT);
        break;

    case 13: /* Todo gas: bloques naranjas. */
        for (float a = a0; a < a1; a += 7.0f) {
            float e = a + 4.0f;
            if (e > a1) e = a1;
            gfx_arc(cx, cy, r, 9.0f, a, e, 2, QV_AMBER);
        }
        break;

    default:
        gfx_arc(cx, cy, r, 8.0f, a0, a1, 10, QV_RED);
        break;
    }
}

/* Arco del valor actual. */
static void theme_value(float cx, float cy, float r, float a1,
                        uint32_t color, float t)
{
    switch (g_cfg.theme) {
    case 5: /* Forza: trazo muy grueso con brillo. */
        gfx_arc(cx, cy, r, t + 10.0f, 135.0f, a1, 54,
                gfx_shade(color, 20));
        gfx_arc(cx, cy, r, t + 6.0f, 135.0f, a1, 54, color);
        gfx_arc(cx, cy, r + 2.0f, 1.0f, 135.0f, a1, 54, QV_TEXT);
        break;

    case 6: /* NFS: nucleo blanco con halo neon. */
        gfx_arc(cx, cy, r, t + 12.0f, 135.0f, a1, 54,
                gfx_shade(color, 14));
        gfx_arc(cx, cy, r, t + 5.0f, 135.0f, a1, 54,
                gfx_shade(color, 35));
        gfx_arc(cx, cy, r, t, 135.0f, a1, 54, color);
        gfx_arc(cx, cy, r, 1.0f, 135.0f, a1, 54, QV_TEXT);
        break;

    case 7: /* GT: trazo fino con aro exterior. */
        gfx_arc(cx, cy, r, t, 135.0f, a1, 54, color);
        gfx_arc(cx, cy, r + 5.0f, 1.0f, 135.0f, a1, 54,
                gfx_shade(color, 50));
        break;

    case 8: /* Initial D: contorno negro y relleno. */
        gfx_arc(cx, cy, r, t + 5.0f, 135.0f, a1, 54, QV_BG);
        gfx_arc(cx, cy, r, t, 135.0f, a1, 54, color);
        gfx_arc(cx, cy, r - t * 0.5f, 1.0f, 135.0f, a1, 54,
                gfx_shade(color, 45));
        break;

    case 9: /* Cyberpunk: glow grueso. */
        gfx_arc(cx, cy, r, t + 10.0f, 135.0f, a1, 54,
                gfx_shade(color, 18));
        gfx_arc(cx, cy, r, t + 4.0f, 135.0f, a1, 54, color);
        gfx_arc(cx, cy, r, t, 135.0f, a1, 54, QV_TEXT);
        break;

    case 10: /* Retro: segmentos verdes. */
        for (float a = 135.0f; a < a1; a += 6.0f) {
            float e = a + 4.0f;
            if (e > a1) e = a1;
            gfx_arc(cx, cy, r, t, a, e, 2, QV_GREEN);
        }
        break;

    case 11: /* Nothing: linea fina. */
        gfx_arc(cx, cy, r, 2.0f, 135.0f, a1, 54, QV_TEXT);
        break;

    case 12: /* Back to the Future: cian glow. */
        gfx_arc(cx, cy, r, t + 8.0f, 135.0f, a1, 54,
                gfx_shade(QV_CYAN, 20));
        gfx_arc(cx, cy, r, t + 3.0f, 135.0f, a1, 54, QV_CYAN);
        gfx_arc(cx, cy, r, t, 135.0f, a1, 54, QV_TEXT);
        break;

    case 13: /* Todo gas: naranja. */
        gfx_arc(cx, cy, r, t + 6.0f, 135.0f, a1, 54, QV_AMBER);
        gfx_arc(cx, cy, r, t, 135.0f, a1, 54, QV_CYAN);
        break;

    default:
        gfx_arc(cx, cy, r, t, 135.0f, a1, 54, color);
        break;
    }
}

/* Marcas de la escala. */
static void theme_ticks(float cx, float cy, float r, int count, int major_every)
{
    if (!theme_is_game() || count < 2) {
        return;
    }

    /* Forza usa los bloques del aro como escala: sin marcas. */
    if (g_cfg.theme == 5) {
        return;
    }

    const float deg2rad = 0.017453292519943295f;
    for (int i = 0; i <= count; i++) {
        float ang = (135.0f + 270.0f * (float)i / (float)count) * deg2rad;
        int major = (major_every > 0) && ((i % major_every) == 0);
        float in = r - (major ? 11.0f : 5.0f);
        float out = r + 4.0f;
        uint32_t c = major ? QV_DIM : gfx_shade(QV_DIM, 55);
        float th = major ? 2.0f : 1.0f;

        if (g_cfg.theme == 6) {
            /* NFS: marcas cortas solo hacia fuera. */
            in = r + 5.0f;
            out = r + (major ? 13.0f : 9.0f);
            c = major ? QV_CYAN : gfx_shade(QV_CYAN, 40);
        } else if (g_cfg.theme == 8) {
            /* Initial D: marcas largas blancas. */
            in = r - (major ? 14.0f : 8.0f);
            out = r + 8.0f;
            c = major ? QV_TEXT : gfx_shade(QV_TEXT, 45);
        } else if (g_cfg.theme == 9) {
            /* Cyberpunk: marcas neon hacia fuera. */
            in = r + 4.0f;
            out = r + (major ? 12.0f : 8.0f);
            c = major ? QV_CYAN : gfx_shade(QV_CRIT, 45);
        } else if (g_cfg.theme == 10) {
            /* Retro: marcas cortas verdes. */
            in = r - (major ? 8.0f : 4.0f);
            out = r + 4.0f;
            c = major ? QV_GREEN : gfx_shade(QV_GREEN, 40);
        } else if (g_cfg.theme == 11) {
            /* Nothing: puntos minimos. */
            in = r - 2.0f;
            out = r + 2.0f;
            c = major ? QV_TEXT : gfx_shade(QV_TEXT, 25);
            th = 1.0f;
        } else if (g_cfg.theme == 12) {
            /* Back to the Future: marcas cian. */
            in = r - (major ? 10.0f : 5.0f);
            out = r + (major ? 6.0f : 3.0f);
            c = major ? QV_CYAN : gfx_shade(QV_CRIT, 40);
        } else if (g_cfg.theme == 13) {
            /* Todo gas: marcas naranjas. */
            in = r - (major ? 9.0f : 4.0f);
            out = r + 5.0f;
            c = major ? QV_AMBER : gfx_shade(QV_CYAN, 40);
        }

        gfx_line(cx + in * cosf(ang), cy + in * sinf(ang),
                 cx + out * cosf(ang), cy + out * sinf(ang),
                 th, c);
    }
}

/* Numeros de la escala en GT e Initial D. */
static void theme_scale_labels(float cx, float cy, float r, float max_rpm)
{
    if (g_cfg.theme != 7 && g_cfg.theme != 8 &&
        g_cfg.theme != 9 && g_cfg.theme != 10 &&
        g_cfg.theme != 12) {
        return;
    }

    int top = (int)(max_rpm / 1000.0f);
    if (top < 2 || top > 10) {
        return;
    }

    for (int i = 0; i <= top; i++) {
        float ang = (135.0f + 270.0f * (float)i / (float)top) *
                    0.0174532925f;
        char lbl[12];
        snprintf(lbl, sizeof(lbl), "%d", i);
        int tx = (int)(cx + (r + 13.0f) * cosf(ang)) - 4;
        int ty = (int)(cy + (r + 13.0f) * sinf(ang)) - 4;
        gfx_text(tx, ty, lbl,
                 (g_cfg.theme == 8 || g_cfg.theme == 11) ? QV_TEXT :
                 (g_cfg.theme == 9 ? QV_CYAN :
                  (g_cfg.theme == 10 ? QV_GREEN :
                   (g_cfg.theme == 12 ? QV_CYAN : QV_DIM))), 1);
    }
}

/* Aguja grande del tacometro. */
static void theme_needle(float cx, float cy, float r, float ratio,
                         uint32_t color)
{
    if (!theme_is_game()) {
        return;
    }

    float ang = (135.0f + 270.0f * clamp_float(ratio, 0.0f, 1.0f)) *
                0.0174532925f;
    float ca = cosf(ang);
    float sa = sinf(ang);
    float tipr = r - 12.0f;
    float tipx = cx + tipr * ca;
    float tipy = cy + tipr * sa;
    float perpx = -sa;
    float perpy = ca;

    switch (g_cfg.theme) {
    case 5: /* Forza: aguja triangular ancha con punta blanca. */
        gfx_line(cx - 10.0f * ca, cy - 10.0f * sa, tipx, tipy,
                 10.0f, gfx_shade(color, 45));
        gfx_line(cx, cy, tipx, tipy, 7.0f, color);
        gfx_line(cx + 6.0f * ca, cy + 6.0f * sa,
                 tipx, tipy, 3.0f, QV_TEXT);
        gfx_circle(tipx, tipy, 4.0f, QV_TEXT);
        gfx_circle(tipx, tipy, 2.0f, color);
        break;

    case 6: /* NFS: aguja doble paralela con punta de flecha. */
        gfx_line(cx, cy, tipx, tipy, 12.0f, gfx_shade(color, 18));
        gfx_line(cx + perpx * 3.0f, cy + perpy * 3.0f,
                 tipx + perpx * 2.0f, tipy + perpy * 2.0f,
                 2.0f, color);
        gfx_line(cx - perpx * 3.0f, cy - perpy * 3.0f,
                 tipx - perpx * 2.0f, tipy - perpy * 2.0f,
                 2.0f, color);
        gfx_line(cx, cy, tipx, tipy, 1.0f, QV_TEXT);
        gfx_line(tipx, tipy,
                 tipx - 8.0f * ca + perpx * 5.0f,
                 tipy - 8.0f * sa + perpy * 5.0f, 2.0f, color);
        gfx_line(tipx, tipy,
                 tipx - 8.0f * ca - perpx * 5.0f,
                 tipy - 8.0f * sa - perpy * 5.0f, 2.0f, color);
        break;

    case 7: /* GT: aguja fina con contrapeso. */
        gfx_line(cx - 11.0f * ca, cy - 11.0f * sa, tipx, tipy,
                 6.0f, gfx_shade(QV_RED, 40));
        gfx_line(cx, cy, tipx, tipy, 3.0f, QV_RED);
        gfx_line(cx - 14.0f * ca, cy - 14.0f * sa,
                 cx - 3.0f * ca, cy - 3.0f * sa, 4.0f, QV_TEXT);
        gfx_circle(tipx, tipy, 2.5f, QV_RED);
        break;

    case 8: /* Initial D: espada blanca con filo rojo. */
        gfx_line(cx - 13.0f * ca, cy - 13.0f * sa, tipx, tipy,
                 9.0f, QV_BG);
        gfx_line(cx - 13.0f * ca, cy - 13.0f * sa, tipx, tipy,
                 6.0f, QV_TEXT);
        gfx_line(cx + (r - 34.0f) * ca, cy + (r - 34.0f) * sa,
                 tipx, tipy, 4.0f, QV_RED);
        gfx_line(cx, cy, tipx, tipy, 1.0f, QV_TEXT);
        break;

    case 9: /* Cyberpunk: laser fino con glow. */
        gfx_line(cx, cy, tipx, tipy, 10.0f, gfx_shade(QV_CYAN, 18));
        gfx_line(cx, cy, tipx, tipy, 4.0f, QV_CYAN);
        gfx_line(cx, cy, tipx, tipy, 1.0f, QV_TEXT);
        gfx_circle(tipx, tipy, 3.0f, QV_CRIT);
        break;

    case 10: /* Retro: bloque grueso pixelado. */
        gfx_line(cx - 6.0f * ca, cy - 6.0f * sa, tipx, tipy,
                 8.0f, QV_GREEN);
        gfx_line(cx - 6.0f * ca, cy - 6.0f * sa, tipx, tipy,
                 4.0f, QV_BG);
        gfx_rect(tipx - 2.0f, tipy - 2.0f, 4.0f, 4.0f, QV_AMBER);
        break;

    case 11: /* Nothing: linea fina negra. */
        gfx_line(cx, cy, tipx, tipy, 2.0f, QV_TEXT);
        gfx_circle(tipx, tipy, 2.0f, QV_TEXT);
        break;

    case 12: /* Back to the Future: cian con brillo. */
        gfx_line(cx, cy, tipx, tipy, 10.0f, gfx_shade(QV_CYAN, 18));
        gfx_line(cx, cy, tipx, tipy, 4.0f, QV_CYAN);
        gfx_line(cx, cy, tipx, tipy, 1.0f, QV_TEXT);
        gfx_circle(tipx, tipy, 3.0f, QV_CRIT);
        break;

    case 13: /* Todo gas: bloque naranja. */
        gfx_line(cx - 5.0f * ca, cy - 5.0f * sa, tipx, tipy,
                 9.0f, QV_AMBER);
        gfx_line(cx - 5.0f * ca, cy - 5.0f * sa, tipx, tipy,
                 4.0f, QV_BG);
        gfx_rect(tipx - 3.0f, tipy - 3.0f, 6.0f, 6.0f, QV_CYAN);
        break;

    }

}

/* Cubo central propio de cada aguja. */
static void theme_hub(float cx, float cy, uint32_t color)
{
    if (!theme_is_game()) {
        return;
    }

    switch (g_cfg.theme) {
    case 5: /* Forza */
        gfx_circle(cx, cy, 11.0f, gfx_shade(color, 60));
        gfx_circle(cx, cy, 8.0f, QV_BG);
        gfx_circle(cx, cy, 6.0f, QV_TEXT);
        gfx_circle(cx, cy, 3.0f, color);
        break;

    case 6: /* NFS: cubo neon */
        gfx_circle(cx, cy, 12.0f, gfx_shade(color, 18));
        gfx_circle(cx, cy, 8.0f, QV_BG);
        gfx_circle(cx, cy, 6.5f, color);
        gfx_circle(cx, cy, 3.0f, QV_TEXT);
        break;

    case 7: /* GT: cubo de competicion */
        gfx_circle(cx, cy, 8.0f, QV_TEXT);
        gfx_circle(cx, cy, 5.0f, QV_RED);
        gfx_circle(cx, cy, 2.0f, QV_TEXT);
        break;

    case 8: /* Initial D: cubo retro */
        gfx_circle(cx, cy, 9.0f, QV_BG);
        gfx_circle(cx, cy, 8.0f, QV_TEXT);
        gfx_circle(cx, cy, 5.0f, QV_BG);
        gfx_circle(cx, cy, 3.5f, QV_RED);
        break;

    case 9: /* Cyberpunk: anillo neon. */
        gfx_circle(cx, cy, 11.0f, gfx_shade(QV_CYAN, 25));
        gfx_circle(cx, cy, 8.0f, QV_BG);
        gfx_circle(cx, cy, 6.0f, QV_CYAN);
        gfx_circle(cx, cy, 3.0f, QV_CRIT);
        break;

    case 10: /* Retro: cuadrado verde. */
        gfx_rect(cx - 6.0f, cy - 6.0f, 12.0f, 12.0f, QV_GREEN);
        gfx_rect(cx - 4.0f, cy - 4.0f, 8.0f, 8.0f, QV_BG);
        gfx_rect(cx - 2.0f, cy - 2.0f, 4.0f, 4.0f, QV_AMBER);
        break;

    case 11: /* Nothing: punto negro. */
        gfx_circle(cx, cy, 4.0f, QV_TEXT);
        gfx_circle(cx, cy, 2.0f, QV_BG);
        break;

    case 12: /* Back to the Future: anillo cian. */
        gfx_circle(cx, cy, 11.0f, gfx_shade(QV_CYAN, 25));
        gfx_circle(cx, cy, 8.0f, QV_BG);
        gfx_circle(cx, cy, 6.0f, QV_CYAN);
        gfx_circle(cx, cy, 3.0f, QV_CRIT);
        break;

    case 13: /* Todo gas: cuadrado naranja. */
        gfx_rect(cx - 6.0f, cy - 6.0f, 12.0f, 12.0f, QV_AMBER);
        gfx_rect(cx - 4.0f, cy - 4.0f, 8.0f, 8.0f, QV_BG);
        gfx_rect(cx - 2.0f, cy - 2.0f, 4.0f, 4.0f, QV_CYAN);
        break;

    }

}

/* Puntero de borde para los diales pequenos. */
static void theme_rim_pointer(float cx, float cy, float r, float ratio,
                              uint32_t color)
{
    if (!theme_is_game()) {
        return;
    }

    float ang = (135.0f + 270.0f * clamp_float(ratio, 0.0f, 1.0f)) *
                0.0174532925f;
    float ca = cosf(ang);
    float sa = sinf(ang);

    switch (g_cfg.theme) {
    case 5: { /* Forza: bloque grueso en el borde. */
        float x1 = cx + (r - 2.0f) * ca;
        float y1 = cy + (r - 2.0f) * sa;
        float x2 = cx + (r + 14.0f) * ca;
        float y2 = cy + (r + 14.0f) * sa;
        gfx_line(x1, y1, x2, y2, 12.0f, gfx_shade(color, 35));
        gfx_line(x1, y1, x2, y2, 7.0f, color);
        break;
    }

    case 6: { /* NFS: punta de flecha neon. */
        float x1 = cx + (r + 4.0f) * ca;
        float y1 = cy + (r + 4.0f) * sa;
        float x2 = cx + (r + 13.0f) * ca;
        float y2 = cy + (r + 13.0f) * sa;
        gfx_line(x1, y1, x2, y2, 9.0f, gfx_shade(color, 20));
        gfx_line(x1, y1, x2, y2, 3.0f, color);
        gfx_line(x2, y2,
                 x2 - 6.0f * ca - sa * 5.0f,
                 y2 - 6.0f * sa + ca * 5.0f, 2.0f, color);
        gfx_line(x2, y2,
                 x2 - 6.0f * ca + sa * 5.0f,
                 y2 - 6.0f * sa - ca * 5.0f, 2.0f, color);
        break;
    }

    case 7: { /* GT: marca fina con punto rojo. */
        float x1 = cx + (r + 4.0f) * ca;
        float y1 = cy + (r + 4.0f) * sa;
        float x2 = cx + (r + 12.0f) * ca;
        float y2 = cy + (r + 12.0f) * sa;
        gfx_line(x1, y1, x2, y2, 2.0f, QV_TEXT);
        gfx_circle(x2, y2, 2.5f, QV_RED);
        break;
    }

    case 8: { /* Initial D: marca doble blanca/roja. */
        float x1 = cx + (r + 2.0f) * ca;
        float y1 = cy + (r + 2.0f) * sa;
        float x2 = cx + (r + 13.0f) * ca;
        float y2 = cy + (r + 13.0f) * sa;
        gfx_line(x1, y1, x2, y2, 7.0f, QV_BG);
        gfx_line(x1, y1, x2, y2, 5.0f, QV_TEXT);
        gfx_line(cx + (r + 8.0f) * ca, cy + (r + 8.0f) * sa,
                 x2, y2, 3.0f, QV_RED);
        break;
    }

    case 9: { /* Cyberpunk: punta neon. */
        float x1 = cx + (r + 3.0f) * ca;
        float y1 = cy + (r + 3.0f) * sa;
        float x2 = cx + (r + 14.0f) * ca;
        float y2 = cy + (r + 14.0f) * sa;
        gfx_line(x1, y1, x2, y2, 8.0f, gfx_shade(QV_CYAN, 25));
        gfx_line(x1, y1, x2, y2, 3.0f, QV_CYAN);
        gfx_circle(x2, y2, 3.0f, QV_CRIT);
        break;
    }

    case 10: { /* Retro: bloque verde. */
        float x1 = cx + (r + 4.0f) * ca;
        float y1 = cy + (r + 4.0f) * sa;
        float x2 = cx + (r + 12.0f) * ca;
        float y2 = cy + (r + 12.0f) * sa;
        gfx_rect(x1 - 3.0f, y1 - 3.0f, 6.0f, 6.0f, QV_GREEN);
        gfx_rect(x2 - 3.0f, y2 - 3.0f, 6.0f, 6.0f, QV_AMBER);
        break;
    }

    case 11: { /* Nothing: punto negro. */
        float x2 = cx + (r + 10.0f) * ca;
        float y2 = cy + (r + 10.0f) * sa;
        gfx_circle(x2, y2, 3.0f, QV_TEXT);
        break;
    }

    case 12: { /* Back to the Future: punta cian. */
        float x1 = cx + (r + 3.0f) * ca;
        float y1 = cy + (r + 3.0f) * sa;
        float x2 = cx + (r + 14.0f) * ca;
        float y2 = cy + (r + 14.0f) * sa;
        gfx_line(x1, y1, x2, y2, 8.0f, gfx_shade(QV_CYAN, 25));
        gfx_line(x1, y1, x2, y2, 3.0f, QV_CYAN);
        gfx_circle(x2, y2, 3.0f, QV_CRIT);
        break;
    }

    case 13: { /* Todo gas: punta naranja. */
        float x2 = cx + (r + 12.0f) * ca;
        float y2 = cy + (r + 12.0f) * sa;
        gfx_rect(x2 - 4.0f, y2 - 4.0f, 8.0f, 8.0f, QV_AMBER);
        break;
    }
    }

    gfx_circle(cx, cy, 4.0f, QV_TEXT);
    gfx_circle(cx, cy, 2.0f, color);
}

static void qv_panel(float x, float y, float w, float h)
{
    gfx_rect(x, y, w, h, QV_PANEL);
    gfx_border(x, y, w, h, 1.0f, QV_TRACK);

    /* Los temas de juego marcan las esquinas del panel con su acento. */
    if (g_cfg.theme >= 5) {
        uint32_t ac = theme_accent();
        float l = 12.0f;
        gfx_rect(x, y, l, 2.0f, ac);
        gfx_rect(x, y, 2.0f, l, ac);
        gfx_rect(x + w - l, y, l, 2.0f, ac);
        gfx_rect(x + w - 2.0f, y, 2.0f, l, ac);
        gfx_rect(x, y + h - 2.0f, l, 2.0f, ac);
        gfx_rect(x, y + h - l, 2.0f, l, ac);
        gfx_rect(x + w - l, y + h - 2.0f, l, 2.0f, ac);
        gfx_rect(x + w - 2.0f, y + h - l, 2.0f, l, ac);
    }
}

static void qv_tile(float x, float y, float w, float h,
                    const char *label, const char *value, const char *unit,
                    uint32_t vcolor, int vscale)
{
    qv_panel(x, y, w, h);

    /* Indicador con el acento de cada tema de juego. */
    if (g_cfg.theme == 5) {
        gfx_rect(x, y + 5.0f, 3.0f, h - 10.0f, theme_accent());
    } else if (g_cfg.theme == 6) {
        gfx_rect(x + 8.0f, y + h - 3.0f, w - 16.0f, 2.0f,
                 gfx_shade(theme_accent(), 55));
    } else if (g_cfg.theme == 7) {
        gfx_rect(x + 8.0f, y + 3.0f, w - 16.0f, 1.0f,
                 gfx_shade(QV_TEXT, 35));
    } else if (g_cfg.theme == 8) {
        gfx_rect(x, y, 4.0f, h, QV_TEXT);
        gfx_rect(x, y + 6.0f, 4.0f, 7.0f, QV_RED);
    }


    /* En filas bajas (menos de 36 px) no caben etiqueta, valor y unidad
     * en tres lineas: se dibujan etiqueta a la izquierda y valor a la
     * derecha en la misma linea para que no se solapen. */
    if (h < 36.0f) {
        gfx_text((int)x + 8, (int)(y + (h - 8.0f) * 0.5f),
                 label, QV_DIM, 1);

        char buf[40];
        if (unit != NULL && unit[0] != '\0') {
            snprintf(buf, sizeof(buf), "%s %s", value, unit);
        } else {
            snprintf(buf, sizeof(buf), "%s", value);
        }
        gfx_text_right((int)(x + w) - 8,
                       (int)(y + (h - 8.0f) * 0.5f), buf, vcolor, 1);
        return;
    }

    gfx_text((int)x + 8, (int)y + 6, label, QV_DIM, 1);
    gfx_text((int)x + 8, (int)y + 20, value, vcolor, vscale);
    if (unit != NULL && unit[0] != '\0') {
        gfx_text((int)x + 8, (int)(y + h - 12), unit, QV_DIM, 1);
    }
}

static void draw_top_bar(int connected, int stale, uint32_t age_ms)
{
    /* La cabecera no se atenua: es donde vive el aviso de que el dato
     * esta viejo, asi que tiene que seguir leyendose siempre. */
    int saved_dim = g_gfx_dim;
    g_gfx_dim = 100;

    gfx_rect(0, 0, GFX_W, 25, QV_PANEL);
    gfx_rect(0, 25, GFX_W, 1,
             theme_is_game() ? theme_accent() : QV_RED);

    static const int drive_mode[4] = { MODE_QV, MODE_ENGINE, MODE_MOTOR,
                                       MODE_TELEMETRY };
    static const int drive_x[4] = { 6, 80, 140, 196 };

    if (mode_is_driving(g_mode)) {
        for (int i = 0; i < 4; i++) {
            const char *name = mode_name(drive_mode[i]);
            int x = drive_x[i];
            gfx_text(x, 8, name,
                     (g_mode == drive_mode[i]) ? QV_TEXT : QV_DIM, 1);
            if (g_mode == drive_mode[i]) {
                gfx_rect((float)x, 20.0f,
                         (float)gfx_text_width(name, 1), 2.0f,
                         theme_is_game() ? theme_accent() : QV_RED);
            }
        }
    } else {
        /* En una herramienta la barra dice donde estas y como volver. */
        int w = gfx_text_width(mode_name(g_mode), 1);
        gfx_text(6, 8, mode_name(g_mode), QV_TEXT, 1);
        gfx_text(6 + w + 10, 8, tr("HERRAMIENTAS"), QV_DIM, 1);
        gfx_text(6 + w + 10 + gfx_text_width(tr("HERRAMIENTAS"), 1) + 10, 8,
                 tr("L/R: VOLVER"), QV_DIM, 1);
    }

    uint32_t dot = QV_CRIT;
    const char *state = "DESCONECT";
    uint32_t state_color = QV_CRIT;
    if (g_demo) {
        dot = QV_AMBER;
        state = "DEMO";
        state_color = QV_AMBER;
    } else if (g_net_busy) {
        dot = QV_AMBER;
        state = "CONECT";
        state_color = QV_AMBER;
    } else if (connected) {
        if (g_elm_stuck >= ELM_STUCK_MIN) {
            /* FASE F2: el adaptador contesta pero se queda atascado en
             * STOPPED. No es lo mismo que no tener datos: el enlace
             * esta vivo y el ELM327 no responde. */
            dot = QV_CRIT;
            state = tr("ELM ATASCADO");
            state_color = QV_CRIT;
        } else if (stale) {
            dot = QV_AMBER;
            state = tr("SIN DATOS");
            state_color = QV_AMBER;
        } else if (age_ms >= DATA_OLD_MS) {
            /* FASE F2: el dato envejece. Se avisa en vez de dejar el
             * panel congelado sin explicacion. */
            dot = QV_AMBER;
            state = tr("DATOS VIEJOS");
            state_color = QV_AMBER;
        } else {
            dot = QV_OK;
            state = "ONLINE";
            state_color = QV_OK;
        }
    }

    char tb[16];
    uint32_t session_s = (now_ms() - g_session_start_ms) / 1000u;
    if (session_s >= 3600u) {
        snprintf(tb, sizeof(tb), "%u:%02u:%02u",
                 (unsigned)(session_s / 3600u),
                 (unsigned)((session_s % 3600u) / 60u),
                 (unsigned)(session_s % 60u));
    } else {
        snprintf(tb, sizeof(tb), "%02u:%02u",
                 (unsigned)(session_s / 60u),
                 (unsigned)(session_s % 60u));
    }

    char righttop[40];
    snprintf(righttop, sizeof(righttop), "%s  %s", state, tb);
    int right_x = GFX_W - 8 - gfx_text_width(righttop, 1);
    /* Con la octava pestana la cabecera va justa: si el estado largo
     * chocaria con las pestanas, se deja solo el reloj. */
    if (right_x < 392) {
        snprintf(righttop, sizeof(righttop), "%s", tb);
        right_x = GFX_W - 8 - gfx_text_width(righttop, 1);
    }
    gfx_circle((float)(right_x - 9), 12.0f, 4.0f, dot);
    gfx_text(right_x, 8, righttop, state_color, 1);

    /* Si el log no se puede abrir hay que decirlo: antes se perdian
     * todas las lineas sin avisar. */
    if (g_log_broken) {
        gfx_text(392 - gfx_text_width("SIN LOG", 1), 8, "SIN LOG",
                 QV_CRIT, 1);
    }

    g_gfx_dim = saved_dim;
}

static void draw_tacho(const dash_data *d, float x, float y, float w, float h)
{
    qv_panel(x, y, w, h);
    gfx_text((int)x + 10, (int)y + 7, tr("TACOMETRO"), QV_DIM, 1);

    float cx = x + w * 0.5f;
    float cy = y + h * 0.5f + 7.0f;
    float r = 54.0f;

    theme_track(cx, cy, r, QV_TRACK);
    theme_ticks(cx, cy, r, 14, 2);
    theme_scale_labels(cx, cy, r, tach_max_rpm());

    float red_start = 135.0f + 270.0f *
        (redline_rpm() / tach_max_rpm());
    theme_redline(cx, cy, r, red_start, 405.0f);

    if (d->rpm_valid) {
        float ratio = clamp_float(d->rpm_val / tach_max_rpm(), 0.0f, 1.0f);
        uint32_t c = QV_TEXT;
        if (d->rpm_val >= redline_rpm()) {
            c = QV_RED;
        } else if (d->rpm_val >= redline_rpm() - 300.0f) {
            c = QV_AMBER;
        }
        if (g_cfg.shift_rpm > 0 && d->rpm_val >= (float)g_cfg.shift_rpm) {
            c = QV_RED;
        }
        theme_value(cx, cy, r, 135.0f + 270.0f * ratio, c, 6.0f);
        theme_needle(cx, cy, r, ratio, theme_is_game() ? theme_accent() : c);
        theme_hub(cx, cy, theme_accent());
        gfx_text_center((int)cx, (int)cy - 16, d->rpm, QV_TEXT, 3);

        if (g_cfg.shift_rpm > 0 && d->rpm_val >= (float)g_cfg.shift_rpm) {
            int sy = theme_is_game() ? 50 : 34;
            gfx_text_center((int)cx, (int)cy + sy, "SHIFT", QV_RED, 1);
        }
    } else {
        gfx_text_center((int)cx, (int)cy - 12, "N/D", QV_DIM, 3);
    }

    /* Remate inferior propio del tema. */
    if (theme_is_game()) {
        float pr = d->rpm_valid ?
            clamp_float(d->rpm_val / tach_max_rpm(), 0.0f, 1.0f) : 0.0f;

        if (g_cfg.theme == 5) {
            gfx_rect(cx - 44.0f, (float)((int)cy + 22), 22.0f, 3.0f,
                     theme_accent());
            gfx_rect(cx + 22.0f, (float)((int)cy + 22), 22.0f, 3.0f,
                     theme_accent());
        } else if (g_cfg.theme == 6) {
            /* Barra digital segmentada. */
            float bx = cx - 60.0f;
            float by = (float)((int)cy + 33);
            int lit = (int)(pr * 20.0f + 0.5f);
            for (int i = 0; i < 20; i++) {
                uint32_t c = (i < lit) ? QV_GREEN : QV_TRACK;
                if (i >= 15) c = (i < lit) ? QV_RED : QV_TRACK;
                gfx_rect(bx + (float)i * 6.0f, by, 4.0f, 5.0f, c);
            }
        } else if (g_cfg.theme == 7) {
            gfx_rect(cx - 34.0f, (float)((int)cy + 20), 3.0f, 11.0f,
                     QV_RED);
        } else if (g_cfg.theme == 8) {
            /* Banda a cuadros del AE86. */
            float bx = cx - 60.0f;
            float by = (float)((int)cy + 31);
            for (int i = 0; i < 20; i++) {
                uint32_t c = (i & 1) ? QV_TEXT : QV_BG;
                gfx_rect(bx + (float)i * 6.0f, by, 6.0f, 4.0f, c);
            }
        }
    }


    /* Pico maximo retenido. */
    if (g_cfg.peak_hold && g_peak_rpm > 0.0f) {
        float pr = clamp_float(g_peak_rpm / tach_max_rpm(), 0.0f, 1.0f);
        float ang = (135.0f + 270.0f * pr) * 0.0174532925f;
        float x1 = cx + (r - 3.0f) * cosf(ang);
        float y1 = cy + (r - 3.0f) * sinf(ang);
        float x2 = cx + (r + 5.0f) * cosf(ang);
        float y2 = cy + (r + 5.0f) * sinf(ang);
        gfx_line(x1, y1, x2, y2, 2.0f, QV_AMBER);
    }

    gfx_text_center((int)cx, (int)cy + 22, "RPM", QV_DIM, 1);
}

static void draw_speed(const dash_data *d, float x, float y, float w, float h)
{
    qv_panel(x, y, w, h);
    gfx_text((int)x + 10, (int)y + 7, tr("VELOCIDAD"), QV_DIM, 1);

    if (d->speed_valid) {
        gfx_text((int)x + 10, (int)y + 22, d->speed, QV_TEXT, 5);
    } else {
        gfx_text((int)x + 10, (int)y + 22, "N/D", QV_DIM, 5);
    }
    gfx_text((int)(x + w - 72), (int)y + 44, units_speed_name(), QV_DIM, 2);
}

static void draw_boost(const dash_data *d, float x, float y, float w, float h)
{
    qv_panel(x, y, w, h);
    gfx_text((int)x + 10, (int)y + 7, "BOOST", QV_DIM, 1);
    if (d->boost_est) {
        gfx_text_right((int)(x + w) - 10, (int)y + 7, "EST", QV_AMBER, 1);
    }

    if (d->boost_valid) {
        gfx_text((int)x + 10, (int)y + 19, d->boost,
                 d->boost_val >= 0.0f ? QV_TEXT : QV_DIM, 3);
    } else {
        gfx_text((int)x + 10, (int)y + 19, "N/D", QV_DIM, 3);
    }
    gfx_text_right((int)(x + w) - 10, (int)y + 27,
                   units_press_name(), QV_DIM, 1);

    float bx = x + 10.0f;
    float by = y + h - 25.0f;
    float bw = w - 20.0f;
    float bh = (g_cfg.theme == 7) ? 5.0f : 10.0f;

    gfx_rect(bx, by, bw, bh, QV_TRACK);

    if (g_cfg.theme == 5) {
        /* Forza: barra troceada en bloques. */
        for (int i = 1; i < 24; i++) {
            gfx_rect(bx + bw * (float)i / 24.0f, by, 1.0f, bh, QV_BG);
        }
    } else if (g_cfg.theme == 6) {
        /* NFS: carriles neon separados. */
        for (int i = 1; i < 24; i++) {
            gfx_rect(bx + bw * (float)i / 24.0f, by, 2.0f, bh, QV_BG);
        }
        gfx_rect(bx, by - 1.0f, bw, 1.0f, gfx_shade(QV_GREEN, 45));
        gfx_rect(bx, by + bh, bw, 1.0f, gfx_shade(QV_GREEN, 45));
    } else if (g_cfg.theme == 7) {
        /* GT: barra fina con marcas de escala. */
        for (int i = 0; i <= 10; i++) {
            float tx = bx + bw * (float)i / 10.0f;
            gfx_rect(tx, by - 3.0f, 1.0f, bh + 6.0f,
                     gfx_shade(QV_DIM, 55));
        }
    } else if (g_cfg.theme == 8) {
        /* Initial D: barra con marco blanco. */
        gfx_border(bx - 1.0f, by - 1.0f, bw + 2.0f, bh + 2.0f,
                   1.0f, QV_TEXT);
    } else if (g_cfg.theme == 9) {
        /* Cyberpunk: marco neon y segmentos. */
        gfx_border(bx - 1.0f, by - 1.0f, bw + 2.0f, bh + 2.0f,
                   1.0f, QV_CYAN);
        for (int i = 1; i < 24; i++) {
            gfx_rect(bx + bw * (float)i / 24.0f, by, 1.0f, bh,
                     gfx_shade(QV_CYAN, 25));
        }
    } else if (g_cfg.theme == 10) {
        /* Retro: bloques verdes. */
        for (int i = 1; i < 12; i++) {
            gfx_rect(bx + bw * (float)i / 12.0f, by, 2.0f, bh, QV_BG);
        }
    } else if (g_cfg.theme == 11) {
        /* Nothing: barra fina. */
        bh = 4.0f;
        gfx_rect(bx, by, bw, bh, QV_TRACK);
    } else if (g_cfg.theme == 12) {
        /* Back to the Future: marco cian. */
        gfx_border(bx - 1.0f, by - 1.0f, bw + 2.0f, bh + 2.0f,
                   1.0f, QV_CYAN);
    } else if (g_cfg.theme == 13) {
        /* Todo gas: bloques naranjas. */
        for (int i = 1; i < 12; i++) {
            gfx_rect(bx + bw * (float)i / 12.0f, by, 2.0f, bh, QV_BG);
        }
    }

    float span = boost_max_bar() - (-1.0f);
    float zero = bx + bw * (0.0f - (-1.0f)) / span;
    gfx_rect(zero - 1.0f, by - 2.0f, 2.0f, bh + 4.0f, QV_DIM);

    if (d->boost_valid) {
        float v = clamp_float(d->boost_val, -1.0f, boost_max_bar());
        float bv = bx + bw * (v - (-1.0f)) / span;
        float x0 = (bv < zero) ? bv : zero;
        float x1 = (bv < zero) ? zero : bv;
        uint32_t fill = (v >= 0.0f) ? QV_GREEN : QV_AMBER;

        if (g_cfg.theme == 5) {
            fill = (v >= 0.0f) ? theme_accent() : QV_AMBER;
        } else if (g_cfg.theme == 6) {
            fill = (v >= 0.0f) ? QV_GREEN : QV_AMBER;
        } else if (g_cfg.theme == 7) {
            fill = (v >= 0.0f) ? QV_RED : QV_AMBER;
        } else if (g_cfg.theme == 8) {
            fill = (v >= 0.0f) ? QV_TEXT : QV_AMBER;
        } else if (g_cfg.theme == 9) {
            fill = (v >= 0.0f) ? QV_CYAN : QV_CRIT;
        } else if (g_cfg.theme == 10) {
            fill = (v >= 0.0f) ? QV_GREEN : QV_AMBER;
        } else if (g_cfg.theme == 11) {
            fill = (v >= 0.0f) ? QV_TEXT : QV_RED;
        } else if (g_cfg.theme == 12) {
            fill = (v >= 0.0f) ? QV_CYAN : QV_CRIT;
        } else if (g_cfg.theme == 13) {
            fill = (v >= 0.0f) ? QV_AMBER : QV_RED;
        }

        if (g_cfg.theme == 6) {
            /* Solo se encienden las celdas recorridas. */
            for (int i = 0; i < 24; i++) {
                float cx0 = bx + bw * (float)i / 24.0f + 1.0f;
                float cx1 = bx + bw * (float)(i + 1) / 24.0f - 1.0f;
                if (cx1 < x0 || cx0 > x1) {
                    continue;
                }
                uint32_t c = (i >= 18) ? QV_RED : fill;
                gfx_rect(cx0, by + 1.0f, cx1 - cx0, bh - 2.0f, c);
            }
        } else {
            if (theme_is_game() && g_cfg.theme != 7) {
                gfx_rect(x0 - 2.0f, by - 3.0f, x1 - x0 + 4.0f,
                         bh + 6.0f, gfx_shade(fill, 20));
            }
            gfx_rect(x0, by + 1.0f, x1 - x0, bh - 2.0f, fill);
        }

        if (theme_is_game()) {
            gfx_rect(bv - 1.5f, by - 4.0f, 3.0f, bh + 8.0f, QV_TEXT);
        }
    }

    char scale_lo[16];
    char scale_hi[16];
    fmt_1(scale_lo, sizeof(scale_lo), units_press_from_bar(-1.0f));
    fmt_1(scale_hi, sizeof(scale_hi), units_press_from_bar(boost_max_bar()));
    gfx_text((int)bx, (int)(by + 12), scale_lo, QV_DIM, 1);
    gfx_text_right((int)(bx + bw), (int)(by + 12), scale_hi, QV_DIM, 1);
}

static void draw_dial(float x, float y, float w, float h,
                      const char *value_text, int valid,
                      float ratio, float redline_ratio,
                      const char *label, const char *unit)
{
    qv_panel(x, y, w, h);
    gfx_text((int)x + 8, (int)y + 6, label, QV_DIM, 1);

    float cx = x + w * 0.5f;
    float cy = y + h * 0.5f + 6.0f;
    float r = (w < h ? w : h) * 0.34f;
    if (r < 18.0f) r = 18.0f;

    theme_track(cx, cy, r, QV_TRACK);
    theme_ticks(cx, cy, r, 10, 5);

    if (redline_ratio > 0.0f && redline_ratio < 1.0f) {
        float rs = 135.0f + 270.0f * redline_ratio;
        theme_redline(cx, cy, r, rs, 405.0f);
    }

    if (valid) {
        float rr = clamp_float(ratio, 0.0f, 1.0f);
        uint32_t c = QV_TEXT;
        if (redline_ratio > 0.0f && rr >= redline_ratio) {
            c = QV_RED;
        } else if (redline_ratio > 0.0f &&
                   rr >= redline_ratio - 0.08f) {
            c = QV_AMBER;
        }
        theme_value(cx, cy, r, 135.0f + 270.0f * rr, c, 4.0f);
        theme_rim_pointer(cx, cy, r, rr,
                          theme_is_game() ? theme_accent() : c);
    }

    if (valid) {
        gfx_text_center((int)cx, (int)cy - 8, value_text, QV_TEXT, 2);
    } else {
        gfx_text_center((int)cx, (int)cy - 8, "N/D", QV_DIM, 2);
    }

    if (unit != NULL && unit[0] != '\0') {
        gfx_text_center((int)cx, (int)cy + 12, unit, QV_DIM, 1);
    }
}

typedef struct {
    char value[24];
    const char *label;
    const char *unit;
    uint32_t color;
    uint32_t fill;
    int valid;
    float ratio;
    float center;    /* posicion del cero en la barra; <0 = desde 0 */
    float redline;   /* zona roja del reloj; <0 = ninguna */
    int tacho;
} widget_view;

/* Prepara valor, unidad, color y proporcion de un elemento del panel. */
/* Estado de mezcla para una sonda lambda de banda estrecha. No es una
 * medicion de AFR real: la sonda solo conmuta rico/pobre alrededor de
 * lambda = 1 (unas 0,45 V). */
static const char *o2_state_str(float v)
{
    if (v < 0.30f) {
        return "POBRE";
    }
    if (v > 0.70f) {
        return "RICA";
    }
    return "ESTEQUIO.";
}

static void widget_view_fill(const dash_data *d, int item, widget_view *wv)
{
    memset(wv, 0, sizeof(*wv));
    wv->label = widget_name(item);
    wv->unit = "";
    wv->color = QV_TEXT;
    wv->fill = QV_GREEN;
    wv->center = -1.0f;
    wv->redline = -1.0f;
    snprintf(wv->value, sizeof(wv->value), "N/D");

    switch (item) {
    case W_RPM:
        wv->label = "RPM";
        wv->unit = "rpm";
        wv->fill = QV_RED;
        wv->redline = redline_rpm() / tach_max_rpm();
        wv->tacho = 1;
        if (d->rpm_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->rpm);
            wv->valid = 1;
            wv->ratio = clamp_float(d->rpm_val / tach_max_rpm(), 0.0f, 1.0f);
            if (d->rpm_val >= redline_rpm()) {
                wv->color = QV_RED;
            } else if (d->rpm_val >= redline_rpm() - 300.0f) {
                wv->color = QV_AMBER;
            }
            if (g_cfg.shift_rpm > 0 &&
                d->rpm_val >= (float)g_cfg.shift_rpm) {
                wv->color = QV_RED;
            }
        }
        break;

    case W_GEAR: {
        /* La marcha sale de rpm y velocidad, con las relaciones del perfil
         * del coche. Sin perfil, o parado, N/D: no se adivina. */
        wv->label = "MARCHA";
        wv->unit = "";
        wv->fill = QV_CYAN;
        const obd_vehicle_profile *vp = obd_vehicle_by_id(g_vehicle_id);
        int gear = 0;
        if (vp != NULL && d->rpm_valid && d->speed_valid) {
            gear = obd_vehicle_gear(vp->gear_kmh, d->rpm_val, d->speed_val);
        }
        if (gear > 0) {
            snprintf(wv->value, sizeof(wv->value), "%d", gear);
            wv->valid = 1;
        }
        break;
    }

    case W_SPEED:
        wv->label = "VELOCIDAD";
        wv->unit = units_speed_name();
        wv->fill = QV_CYAN;
        if (d->speed_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->speed);
            wv->valid = 1;
            wv->ratio = clamp_float(d->speed_val / units_speed_scale(),
                                    0.0f, 1.0f);
        }
        break;

    case W_BOOST:
        wv->label = "BOOST";
        wv->unit = units_press_name();
        wv->center = 1.0f / (boost_max_bar() + 1.0f);
        wv->fill = QV_GREEN;
        if (d->boost_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->boost);
            wv->valid = 1;
            wv->ratio = clamp_float((d->boost_val + 1.0f) /
                                    (boost_max_bar() + 1.0f), 0.0f, 1.0f);
            if (d->boost_val < 0.0f) {
                wv->color = QV_DIM;
                wv->fill = QV_AMBER;
            }
        }
        break;

    case W_ECT:
        wv->label = "ECT";
        wv->unit = units_temp_name();
        if (d->ect_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->ect);
            wv->valid = 1;
            wv->ratio = clamp_float(d->ect_val / 120.0f, 0.0f, 1.0f);
            if (d->ect_val >= PROFILE_ECT_CRIT_C) {
                wv->color = QV_CRIT;
                wv->fill = QV_CRIT;
            } else if (d->ect_val >= PROFILE_ECT_WARN_C) {
                wv->color = QV_AMBER;
                wv->fill = QV_AMBER;
            }
        }
        break;

    case W_IAT:
        wv->label = "IAT";
        wv->unit = units_temp_name();
        if (d->iat_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->iat);
            wv->valid = 1;
            wv->ratio = clamp_float(d->iat_val / 120.0f, 0.0f, 1.0f);
            if (d->iat_val >= PROFILE_IAT_WARN_C) {
                wv->color = QV_AMBER;
                wv->fill = QV_AMBER;
            }
        }
        break;

    case W_MAP:
        if (g_map_supported) {
            wv->label = "MAP ABS";
            wv->unit = units_press_name();
            if (d->map_valid) {
                snprintf(wv->value, sizeof(wv->value), "%s", d->map_bar);
                wv->valid = 1;
                wv->ratio = clamp_float(d->map_val / 300.0f, 0.0f, 1.0f);
            }
        } else if (g_maf_supported) {
            wv->label = "MAF";
            wv->unit = units_maf_name();
            if (d->maf_valid) {
                snprintf(wv->value, sizeof(wv->value), "%s", d->maf);
                wv->valid = 1;
                wv->ratio = clamp_float(d->maf_val / 300.0f, 0.0f, 1.0f);
            }
        } else {
            wv->label = "MAP ABS";
            wv->unit = units_press_name();
        }
        break;

    case W_MAF:
        wv->label = "MAF";
        wv->unit = units_maf_name();
        if (d->maf_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->maf);
            wv->valid = 1;
            wv->ratio = clamp_float(d->maf_val / 300.0f, 0.0f, 1.0f);
        }
        break;

    case W_THR:
        wv->label = "ACELERADOR";
        wv->unit = "%";
        if (d->thr_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->thr);
            wv->valid = 1;
            wv->ratio = clamp_float(d->thr_val / 100.0f, 0.0f, 1.0f);
        }
        break;

    case W_LOAD:
        wv->label = "CARGA";
        wv->unit = "%";
        if (d->load_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->load);
            wv->valid = 1;
            wv->ratio = clamp_float(d->load_val / 100.0f, 0.0f, 1.0f);
        }
        break;

    case W_PLATE:
        wv->label = "MARIPOSA";
        wv->unit = "%";
        if (d->plate_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->plate);
            wv->valid = 1;
            wv->ratio = clamp_float(d->plate_val / 100.0f, 0.0f, 1.0f);
        }
        break;

    case W_VOLT:
        wv->label = "VOLT";
        wv->unit = "V";
        if (d->volt_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->volt);
            wv->valid = 1;
            wv->ratio = clamp_float((d->volt_val - 8.0f) / 8.0f,
                                    0.0f, 1.0f);
        }
        break;

    case W_OIL:
        wv->label = "ACEITE";
        wv->unit = units_temp_name();
        if (d->oil_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->oil);
            wv->valid = 1;
            wv->ratio = clamp_float(d->oil_val / 120.0f, 0.0f, 1.0f);
        }
        break;

    case W_FUEL:
        wv->label = "COMBUSTIBLE";
        wv->unit = "%";
        if (d->fuel_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->fuel);
            wv->valid = 1;
            wv->ratio = clamp_float(d->fuel_val / 100.0f, 0.0f, 1.0f);
        }
        break;

    case W_STFT:
        wv->label = "STFT";
        wv->unit = "%";
        if (d->stft_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->stft);
            wv->valid = 1;
            wv->ratio = clamp_float((d->stft_val + 25.0f) / 50.0f,
                                    0.0f, 1.0f);
            wv->center = 0.5f;
        }
        break;

    case W_LTFT:
        wv->label = "LTFT";
        wv->unit = "%";
        if (d->ltft_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->ltft);
            wv->valid = 1;
            wv->ratio = clamp_float((d->ltft_val + 25.0f) / 50.0f,
                                    0.0f, 1.0f);
            wv->center = 0.5f;
        }
        break;

    case W_TIMING:
        wv->label = "AVANCE";
        wv->unit = "deg";
        if (d->timing_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->timing);
            wv->valid = 1;
            wv->ratio = clamp_float((d->timing_val + 20.0f) / 80.0f,
                                    0.0f, 1.0f);
        }
        break;

    case W_O2_1:
        wv->label = "O2-1";
        wv->unit = "V";
        if (d->o2_1_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->o2_1);
            wv->valid = 1;
            wv->ratio = clamp_float(d->o2_1_val / 1.2f, 0.0f, 1.0f);
            wv->unit = o2_state_str(d->o2_1_val);
        }
        break;

    case W_O2_2:
        wv->label = "O2-2";
        wv->unit = "V";
        if (d->o2_2_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->o2_2);
            wv->valid = 1;
            wv->ratio = clamp_float(d->o2_2_val / 1.2f, 0.0f, 1.0f);
            wv->unit = o2_state_str(d->o2_2_val);
        }
        break;

    case W_DIST_MIL:
        wv->label = "MIL";
        wv->unit = "km";
        if (d->dist_mil_valid) {
            snprintf(wv->value, sizeof(wv->value), "%s", d->dist_mil);
            wv->valid = 1;
            wv->ratio = clamp_float(d->dist_mil_val / 500.0f, 0.0f, 1.0f);
        }
        break;

    case W_BARO:
        wv->label = "BARO";
        wv->unit = units_press_name();
        if (d->baro_valid) {
            fmt_2(wv->value, sizeof(wv->value),
                  units_map_from_kpa(d->baro_kpa_val));
            wv->valid = 1;
            wv->ratio = clamp_float((d->baro_kpa_val - 80.0f) / 30.0f,
                                    0.0f, 1.0f);
        }
        break;

    default:
        break;
    }
}

static void draw_widget_bar(float x, float y, float w, float h,
                            const widget_view *wv)
{
    qv_panel(x, y, w, h);
    gfx_text((int)x + 8, (int)y + 6, wv->label, QV_DIM, 1);

    char buf[32];
    if (wv->valid) {
        if (wv->unit[0] != '\0') {
            snprintf(buf, sizeof(buf), "%s %s", wv->value, wv->unit);
        } else {
            snprintf(buf, sizeof(buf), "%s", wv->value);
        }
        gfx_text_right((int)(x + w) - 8, (int)y + 6, buf, wv->color, 1);
    } else {
        gfx_text_right((int)(x + w) - 8, (int)y + 6, "N/D", QV_DIM, 1);
    }

    float bx = x + 8.0f;
    float bw = w - 16.0f;
    float by = y + h - 17.0f;
    float bh = 8.0f;

    gfx_rect(bx, by, bw, bh, QV_TRACK);
    gfx_rect(bx, by - 1.0f, bw, 1.0f, gfx_shade(wv->fill, 45));

    if (wv->valid) {
        float r = clamp_float(wv->ratio, 0.0f, 1.0f);
        float x0, x1;

        if (wv->center >= 0.0f) {
            float c = clamp_float(wv->center, 0.0f, 1.0f);
            x0 = bx + bw * (r < c ? r : c);
            x1 = bx + bw * (r < c ? c : r);
            gfx_rect(bx + bw * c - 1.0f, by - 2.0f,
                     2.0f, bh + 4.0f, QV_DIM);
        } else {
            x0 = bx;
            x1 = bx + bw * r;
        }

        if (x1 > x0) {
            gfx_rect(x0, by + 1.0f, x1 - x0, bh - 2.0f, wv->fill);
        }
        gfx_rect(bx + bw * r - 1.5f, by - 3.0f,
                 3.0f, bh + 6.0f, QV_TEXT);
    }
}

static void draw_dash_widget(const dash_data *d, int item, int style,
                             float x, float y, float w, float h)
{
    if (style == STYLE_OFF || item == W_OFF) {
        return;
    }

    widget_view wv;
    widget_view_fill(d, item, &wv);

    if (style == STYLE_TEXT) {
        if (item == W_SPEED) {
            draw_speed(d, x, y, w, h);
            return;
        }
        int sc = (h >= 100.0f) ? 3 : 2;
        qv_tile(x, y, w, h, wv.label,
                wv.valid ? wv.value : "N/D", wv.unit,
                wv.valid ? wv.color : QV_DIM, sc);
        return;
    }

    if (style == STYLE_BAR) {
        if (item == W_BOOST) {
            draw_boost(d, x, y, w, h);
            return;
        }
        draw_widget_bar(x, y, w, h, &wv);
        return;
    }

    if (item == W_RPM && w >= 180.0f && h >= 120.0f) {
        draw_tacho(d, x, y, w, h);
    } else {
        draw_dial(x, y, w, h, wv.value, wv.valid, wv.ratio,
                  wv.redline, wv.label, wv.unit);
    }
}

static void draw_qv(const dash_data *d)
{
    static const float rx[DASH_SLOTS] = {  8.0f, 228.0f, 228.0f,
                                           8.0f, 168.0f, 328.0f };
    static const float ry[DASH_SLOTS] = { 32.0f,  32.0f, 104.0f,
                                         186.0f, 186.0f, 186.0f };
    static const float rw[DASH_SLOTS] = { 212.0f, 244.0f, 244.0f,
                                          152.0f, 152.0f, 144.0f };
    static const float rh[DASH_SLOTS] = { 146.0f,  64.0f,  74.0f,
                                           80.0f,  80.0f,  80.0f };

    for (int i = 0; i < DASH_SLOTS; i++) {
        draw_dash_widget(d, slot_item_eff(i), slot_style_eff(i),
                         rx[i], ry[i], rw[i], rh[i]);
    }
}

static void draw_engine(const dash_data *d)
{
    char bval[16] = "N/D";
    if (d->boost_valid) {
        fmt_signed_2(bval, sizeof(bval),
                     units_press_from_bar(d->boost_val));
    }

    struct { const char *label; const char *value; const char *unit;
             uint32_t color; int valid; } tiles[12];
    int n = 0;

    if (g_map_supported) {
        tiles[n].label = "MAP"; tiles[n].value = d->map_valid ? d->map_kpa : "N/D";
        tiles[n].unit = units_imperial() ? "psi ABS" : "kPa ABS";
        tiles[n].color = d->map_valid ? QV_TEXT : QV_DIM;
        tiles[n].valid = d->map_valid; n++;
    } else if (g_maf_supported) {
        tiles[n].label = "MAF"; tiles[n].value = d->maf_valid ? d->maf : "N/D";
        tiles[n].unit = units_maf_name();
        tiles[n].color = d->maf_valid ? QV_TEXT : QV_DIM;
        tiles[n].valid = d->maf_valid; n++;
    }

    if (g_map_supported) {
        tiles[n].label = "BOOST"; tiles[n].value = bval;
        tiles[n].unit = units_press_name();
        tiles[n].color = d->boost_valid ? QV_TEXT : QV_DIM;
        tiles[n].valid = d->boost_valid; n++;
    }

    tiles[n].label = "ECT"; tiles[n].value = d->ect_valid ? d->ect : "N/D";
    tiles[n].unit = units_temp_name();
    tiles[n].color = d->ect_valid ? QV_TEXT : QV_DIM;
    tiles[n].valid = d->ect_valid; n++;

    tiles[n].label = "IAT"; tiles[n].value = d->iat_valid ? d->iat : "N/D";
    tiles[n].unit = units_temp_name();
    tiles[n].color = d->iat_valid ? QV_TEXT : QV_DIM;
    tiles[n].valid = d->iat_valid; n++;

    tiles[n].label = g_thr_label; tiles[n].value = d->thr_valid ? d->thr : "N/D";
    tiles[n].unit = "%";
    tiles[n].color = d->thr_valid ? QV_TEXT : QV_DIM;
    tiles[n].valid = d->thr_valid; n++;

    if (d->load_valid) {
        tiles[n].label = "LOAD"; tiles[n].value = d->load;
        tiles[n].unit = "%"; tiles[n].color = QV_TEXT; tiles[n].valid = 1; n++;
    }

    if (g_pedal_supported) {
        tiles[n].label = "MARIPOSA"; tiles[n].value = d->plate_valid ? d->plate : "N/D";
        tiles[n].unit = "%"; tiles[n].color = d->plate_valid ? QV_TEXT : QV_DIM;
        tiles[n].valid = d->plate_valid; n++;
    }

    if (g_mask3 & 0x40000000u) {
        tiles[n].label = "VOLT"; tiles[n].value = d->volt_valid ? d->volt : "N/D";
        tiles[n].unit = "V"; tiles[n].color = d->volt_valid ? QV_TEXT : QV_DIM;
        tiles[n].valid = d->volt_valid; n++;
    }

    if (g_oil_supported && d->oil_valid) {
        tiles[n].label = "ACEITE"; tiles[n].value = d->oil;
        tiles[n].unit = units_temp_name(); tiles[n].color = QV_TEXT; tiles[n].valid = 1; n++;
    }

    if (g_fuel_supported && d->fuel_valid) {
        tiles[n].label = "COMBUSTIBLE"; tiles[n].value = d->fuel;
        tiles[n].unit = "%"; tiles[n].color = QV_TEXT; tiles[n].valid = 1; n++;
    }

    int m = 0;
    for (int i = 0; i < n; i++) {
        if (!tiles[i].valid) continue;
        tiles[m++] = tiles[i];
    }
    n = m;

    if (n == 0) {
        gfx_text(16, 64, tr("Sin datos del motor."), QV_DIM, 1);
        return;
    }

    const int cols = 4;
    const int rows = (n + cols - 1) / cols;
    const float tw = 110.0f;
    const float pitchx = 118.0f;

    /* Con 9-10 datos (p. ej. MiTo: MAP, boost, ECT, IAT, acelerador,
     * carga, mariposa, voltaje y combustible) salen 3 filas. Se ajusta
     * el alto para que la ultima fila no se salga de la pantalla. */
    float th = 78.0f;
    float gapy = 8.0f;
    if (rows > 2) {
        th = (234.0f - gapy * (float)(rows - 1)) / (float)rows;
    }
    const float pitchy = th + gapy;

    for (int i = 0; i < n; i++) {
        int col = i % cols;
        int row = i / cols;
        float x = 8.0f + (float)col * pitchx;
        float y = 32.0f + (float)row * pitchy;
        qv_tile(x, y, tw, th, tiles[i].label,
                tiles[i].value, tiles[i].unit,
                tiles[i].color, 2);
    }
}



static void draw_motor(const dash_data *d)
{
    struct { const char *label; const char *value; const char *unit;
             uint32_t color; int valid; } tiles[8];
    int n = 0;

    if (d->stft_valid) {
        tiles[n].label = "STFT"; tiles[n].value = d->stft;
        tiles[n].unit = "%"; tiles[n].color = QV_TEXT; tiles[n].valid = 1; n++;
    }
    if (d->ltft_valid) {
        tiles[n].label = "LTFT"; tiles[n].value = d->ltft;
        tiles[n].unit = "%"; tiles[n].color = QV_TEXT; tiles[n].valid = 1; n++;
    }
    if (d->timing_valid) {
        tiles[n].label = "AVANCE"; tiles[n].value = d->timing;
        tiles[n].unit = "deg"; tiles[n].color = QV_AMBER; tiles[n].valid = 1; n++;
    }
    if (d->o2_1_valid) {
        tiles[n].label = "O2-1"; tiles[n].value = d->o2_1;
        tiles[n].unit = o2_state_str(d->o2_1_val);
        tiles[n].color = QV_GREEN; tiles[n].valid = 1; n++;
    }
    if (d->o2_2_valid) {
        tiles[n].label = "O2-2"; tiles[n].value = d->o2_2;
        tiles[n].unit = o2_state_str(d->o2_2_val);
        tiles[n].color = QV_GREEN; tiles[n].valid = 1; n++;
    }
    if (d->fuel_valid) {
        tiles[n].label = "COMBUSTIBLE"; tiles[n].value = d->fuel;
        tiles[n].unit = "%"; tiles[n].color = QV_TEXT; tiles[n].valid = 1; n++;
    }
    if (d->volt_valid) {
        tiles[n].label = "VOLT"; tiles[n].value = d->volt;
        tiles[n].unit = "V"; tiles[n].color = QV_TEXT; tiles[n].valid = 1; n++;
    }
    if (d->dist_mil_valid) {
        tiles[n].label = "MIL"; tiles[n].value = d->dist_mil;
        tiles[n].unit = "km"; tiles[n].color = QV_AMBER; tiles[n].valid = 1; n++;
    }

    if (n == 0) {
        gfx_text(16, 64, tr("Sin datos avanzados soportados."), QV_DIM, 1);
        gfx_text(16, 84, tr("Comprueba el contacto y la conexion."), QV_DIM, 1);
        return;
    }

    int m = 0;
    for (int i = 0; i < n; i++) {
        if (!tiles[i].valid) continue;
        tiles[m++] = tiles[i];
    }
    n = m;

    const int cols = 4;
    const float tw = 110.0f;
    const float th = 74.0f;
    const float pitchx = 118.0f;
    const float pitchy = 82.0f;

    for (int i = 0; i < n; i++) {
        int col = i % cols;
        int row = i / cols;
        float x = 8.0f + (float)col * pitchx;
        float y = 32.0f + (float)row * pitchy;
        qv_tile(x, y, tw, th, tiles[i].label,
                tiles[i].value, tiles[i].unit,
                tiles[i].color, 2);
    }
}

static void series_stats(const float *hist, uint32_t bit, float *mn, float *mx, int *n)
{
    *mn = 0.0f;
    *mx = 0.0f;
    *n = 0;
    int first = 1;

    for (int i = 0; i < HIST_MAX; i++) {
        if ((g_hist_valid[i] & bit) == 0) {
            continue;
        }
        float v = hist[i];
        if (first) {
            *mn = v;
            *mx = v;
            first = 0;
        } else {
            if (v < *mn) *mn = v;
            if (v > *mx) *mx = v;
        }
        (*n)++;
    }
}

static void graph_series(const float *hist, float lo, float hi, uint32_t bit,
                         uint32_t color, float gx, float gy, float gw, float gh)
{
    if (g_hist_count < 2 || hist == NULL) {
        return;
    }
    int start = (g_hist_count < HIST_MAX) ? 0 : g_hist_pos;
    int prev_valid = 0;
    float prev_x = 0.0f, prev_y = 0.0f;

    for (int k = 0; k < g_hist_count; k++) {
        int idx = (start + k) % HIST_MAX;
        if ((g_hist_valid[idx] & bit) == 0) {
            prev_valid = 0;
            continue;
        }
        float v = hist[idx];
        if (v < lo) v = lo;
        if (v > hi) v = hi;
        float x = gx + gw * (float)k / (float)(g_hist_count - 1);
        float y = gy + gh - (v - lo) / (hi - lo) * gh;
        if (prev_valid) {
            gfx_line(prev_x, prev_y, x, y, 2.0f, color);
        }
        prev_x = x;
        prev_y = y;
        prev_valid = 1;
    }
}

static float *tel_values(int item)
{
    switch (item) {
    case W_RPM:      return g_hist_rpm;
    case W_SPEED:    return g_hist_speed;
    case W_BOOST:    return g_hist_boost;
    case W_ECT:      return g_hist_ect;
    case W_IAT:      return g_hist_iat;
    case W_MAP:      return g_hist_map;
    case W_MAF:      return g_hist_maf;
    case W_THR:      return g_hist_thr;
    case W_LOAD:     return g_hist_load;
    case W_PLATE:    return g_hist_plate;
    case W_VOLT:     return g_hist_volt;
    case W_OIL:      return g_hist_oil;
    case W_FUEL:     return g_hist_fuel;
    case W_STFT:     return g_hist_stft;
    case W_LTFT:     return g_hist_ltft;
    case W_TIMING:   return g_hist_timing;
    case W_O2_1:     return g_hist_o2_1;
    case W_O2_2:     return g_hist_o2_2;
    case W_DIST_MIL: return g_hist_dist_mil;
    case W_BARO:     return g_hist_baro;
    default:         return NULL;
    }
}

/* Bit del historial: un bit por elemento para poder marcar los que
 * realmente traen dato en cada muestra. */
static uint32_t tel_bit(int item)
{
    if (item < 0 || item >= W_COUNT) {
        return 0u;
    }
    return (uint32_t)1u << item;
}

/* Soporte real declarado por las mascaras OBD de la ECU. */
static int pid_supported(uint8_t pid)
{
    if (pid >= 0x01 && pid <= 0x20) {
        return g_mask1 != 0 && obd_mask_supports(g_mask1, pid);
    }
    if (pid >= 0x21 && pid <= 0x40) {
        return g_mask2 != 0 && obd_mask_supports(g_mask2, pid);
    }
    if (pid >= 0x41 && pid <= 0x60) {
        return g_mask3 != 0 && obd_mask_supports(g_mask3, pid);
    }
    return 0;
}

/* Un elemento esta disponible si el coche lo soporta. Sin mascaras
 * (aun sin conectar) se muestran todos para poder configurar la vista. */
static int widget_available(int item)
{
    if (item == W_OFF) {
        return 1;
    }
    if (item < 0 || item >= W_COUNT) {
        return 0;
    }
    if (g_demo) {
        return 1;
    }
    if (g_mask1 == 0 && g_mask2 == 0 && g_mask3 == 0) {
        return 1;
    }

    switch (item) {
    case W_RPM:   return 1;
    case W_GEAR:  return 1;
    case W_BOOST: return g_map_supported;
    case W_MAP:   return g_map_supported;
    case W_MAF:   return g_maf_supported;
    case W_PLATE: return g_pedal_supported;
    case W_OIL:   return g_oil_supported;
    case W_FUEL:  return g_fuel_supported;
    default:      return pid_supported(g_hist_pid[item]);
    }
}

/* Siguiente elemento valido para los editores. Si include_off es 1 se
 * permite OCULTO. */
static int widget_cycle(int item, int dir, int include_off)
{
    int lo = include_off ? W_OFF : 0;
    int n = W_COUNT - lo;
    int idx = item - lo;
    if (idx < 0) {
        idx = 0;
    }

    for (int k = 1; k <= n; k++) {
        int ni = (idx + dir * k) % n;
        if (ni < 0) {
            ni += n;
        }
        int cand = ni + lo;
        if (cand == W_OFF || widget_available(cand)) {
            return cand;
        }
    }
    return item;
}

static void tel_range(int item, float *lo, float *hi)
{
    switch (item) {
    case W_RPM:      *lo = 0.0f;   *hi = tach_max_rpm(); break;
    case W_BOOST:    *lo = -1.0f;  *hi = boost_max_bar(); break;
    case W_ECT:      *lo = 0.0f;   *hi = 120.0f; break;
    case W_SPEED:    *lo = 0.0f;   *hi = 240.0f; break;
    case W_IAT:      *lo = 0.0f;   *hi = 120.0f; break;
    case W_MAP:      *lo = 0.0f;   *hi = 300.0f; break;
    case W_MAF:      *lo = 0.0f;   *hi = 300.0f; break;
    case W_THR:      *lo = 0.0f;   *hi = 100.0f; break;
    case W_LOAD:     *lo = 0.0f;   *hi = 100.0f; break;
    case W_PLATE:    *lo = 0.0f;   *hi = 100.0f; break;
    case W_VOLT:     *lo = 8.0f;   *hi = 16.0f; break;
    case W_OIL:      *lo = 0.0f;   *hi = 150.0f; break;
    case W_FUEL:     *lo = 0.0f;   *hi = 100.0f; break;
    case W_STFT:     *lo = -25.0f; *hi = 25.0f; break;
    case W_LTFT:     *lo = -25.0f; *hi = 25.0f; break;
    case W_TIMING:   *lo = -20.0f; *hi = 60.0f; break;
    case W_O2_1:     *lo = 0.0f;   *hi = 1.2f; break;
    case W_O2_2:     *lo = 0.0f;   *hi = 1.2f; break;
    case W_DIST_MIL: *lo = 0.0f;   *hi = 500.0f; break;
    case W_BARO:     *lo = 80.0f;  *hi = 110.0f; break;
    default:         *lo = 0.0f;   *hi = 1.0f; break;
    }
}

static uint32_t tel_color(int item)
{
    switch (item) {
    case W_RPM:      return QV_TEXT;
    case W_BOOST:    return QV_RED;
    case W_ECT:      return QV_GREEN;
    case W_SPEED:    return QV_CYAN;
    case W_IAT:      return QV_AMBER;
    case W_MAP:      return QV_OK;
    case W_MAF:      return QV_DIM;
    case W_THR:      return QV_CRIT;
    case W_LOAD:     return QV_AMBER;
    case W_PLATE:    return QV_CYAN;
    case W_VOLT:     return QV_GREEN;
    case W_OIL:      return QV_AMBER;
    case W_FUEL:     return QV_OK;
    case W_STFT:     return QV_CYAN;
    case W_LTFT:     return QV_OK;
    case W_TIMING:   return QV_AMBER;
    case W_O2_1:     return QV_GREEN;
    case W_O2_2:     return QV_OK;
    case W_DIST_MIL: return QV_CRIT;
    case W_BARO:     return QV_DIM;
    default:         return QV_DIM;
    }
}

static const char *tel_name(int item)
{
    switch (item) {
    case W_SPEED: return "VELOCIDAD";
    case W_THR:   return "ACELERADOR";
    default:
        return widget_name(item);
    }
}

static const char *tel_short(int item)
{
    switch (item) {
    case W_SPEED:    return "VEL";
    case W_THR:      return "THR";
    case W_LOAD:     return "CARG";
    case W_PLATE:    return "MARIP";
    case W_OIL:      return "ACEI";
    case W_FUEL:     return "COMB";
    case W_TIMING:   return "AVZ";
    default:
        return widget_name(item);
    }
}

static const char *tel_unit(int item)
{
    switch (item) {
    case W_RPM:      return "rpm";
    case W_BOOST:    return units_press_name();
    case W_ECT:
    case W_IAT:
    case W_OIL:      return units_temp_name();
    case W_SPEED:    return units_speed_name();
    case W_MAP:
    case W_BARO:     return units_press_name();
    case W_MAF:      return units_maf_name();
    case W_THR:
    case W_LOAD:
    case W_PLATE:
    case W_FUEL:
    case W_STFT:
    case W_LTFT:     return "%";
    case W_VOLT:
    case W_O2_1:
    case W_O2_2:     return "V";
    case W_TIMING:   return "deg";
    case W_DIST_MIL: return "km";
    default:         return "";
    }
}

static void tel_print(int item, float raw, char *buf, size_t size)
{
    switch (item) {
    case W_RPM:
        fmt_0(buf, size, raw);
        break;
    case W_BOOST:
        fmt_signed_2(buf, size, units_press_from_bar(raw));
        break;
    case W_ECT:
    case W_IAT:
    case W_OIL:
        fmt_0(buf, size, units_temp_from_c(raw));
        break;
    case W_SPEED:
        fmt_0(buf, size, units_speed_from_kmh(raw));
        break;
    case W_MAP:
    case W_BARO:
        fmt_2(buf, size, units_map_from_kpa(raw));
        break;
    case W_MAF:
        fmt_2(buf, size, units_maf_from_gs(raw));
        break;
    case W_VOLT:
    case W_O2_1:
    case W_O2_2:
    case W_STFT:
    case W_LTFT:
    case W_TIMING:
        fmt_signed_2(buf, size, raw);
        break;
    case W_THR:
    case W_LOAD:
    case W_PLATE:
    case W_FUEL:
    case W_DIST_MIL:
        fmt_0(buf, size, raw);
        break;
    default:
        snprintf(buf, size, "N/D");
        break;
    }
}

static void draw_tel_stat(int item, float x, float y, float w, float h)
{
    qv_panel(x, y, w, h);

    if (item == W_OFF) {
        gfx_text((int)x + 8, (int)y + 6, "---", QV_DIM, 1);
        return;
    }

    char title[28];
    snprintf(title, sizeof(title), "%s MAX %s", tel_short(item),
             tel_unit(item));
    gfx_text((int)x + 8, (int)y + 6, title, QV_DIM, 1);

    float mn, mx;
    int n;
    char buf[24];
    char small[24];

    series_stats(tel_values(item), tel_bit(item), &mn, &mx, &n);

    if (n > 0) {
        tel_print(item, mx, buf, sizeof(buf));
        gfx_text((int)x + 8, (int)y + 20, buf,
                 tel_color(item), 2);
        tel_print(item, mn, small, sizeof(small));
        gfx_text((int)x + 8, (int)y + h - 12, "MIN", QV_DIM, 1);
        gfx_text((int)x + 40, (int)y + h - 12, small, QV_DIM, 1);
    } else {
        gfx_text((int)x + 8, (int)y + 20, "N/D", QV_DIM, 2);
    }
}
static void draw_telemetry(void)
{
    float px = 8.0f;
    float py = 32.0f;
    float pw = 464.0f;
    float ph = 158.0f;

    qv_panel(px, py, pw, ph);
    gfx_text((int)px + 10, (int)py + 8, tr("TELEMETRIA"), QV_DIM, 1);
    gfx_text((int)px + 96, (int)py + 8, tr("X: ELEGIR SERIES"), QV_DIM, 1);

    /* Leyenda: solo las series que el usuario haya activado. */
    for (int i = 0; i < TEL_SLOTS; i++) {
        int item = tel_item_eff(i);
        if (item == W_OFF) {
            continue;
        }
        float lx = px + 196.0f + (float)i * 56.0f;
        gfx_rect(lx, py + 9.0f, 8.0f, 8.0f, tel_color(item));
        gfx_text((int)lx + 11, (int)py + 8, tel_short(item), QV_DIM, 1);
    }

    float gx = px + 12.0f;
    float gy = py + 26.0f;
    float gw = pw - 24.0f;
    float gh = ph - 38.0f;

    for (int i = 1; i < 4; i++) {
        float y = gy + gh * (float)i / 4.0f;
        gfx_rect(gx, y, gw, 1.0f, QV_TRACK);
    }
    gfx_rect(gx, gy + gh - 1.0f, gw, 1.0f, QV_TRACK);

    for (int i = 0; i < TEL_SLOTS; i++) {
        int item = tel_item_eff(i);
        if (item == W_OFF) {
            continue;
        }
        float lo, hi;
        tel_range(item, &lo, &hi);
        graph_series(tel_values(item), lo, hi, tel_bit(item),
                     tel_color(item), gx, gy, gw, gh);
    }

    /* Cinco casillas con margen simetrico (8 px) y separacion de 6 px:
     * 5*88 + 4*6 = 464, de x=8 a x=472. Antes la ultima quedaba 3 px
     * mas cerca del borde y la fila se solapaba con el panel superior. */
    static const float sx[TEL_SLOTS] = {
        8.0f, 102.0f, 196.0f, 290.0f, 384.0f
    };
    for (int i = 0; i < TEL_SLOTS; i++) {
        draw_tel_stat(tel_item_eff(i), sx[i], 196.0f, 88.0f, 70.0f);
    }
}

/* ------------------------------------------------------------------ */
/* Ventana de ajustes.                                                */
/* ------------------------------------------------------------------ */

static const char *const g_contrast_names[3] = { "BAJO", "NORMAL", "ALTO" };

static void draw_settings_overlay(void)
{
    const float x = 8.0f;
    const float y = 30.0f;
    const float w = 464.0f;
    const float h = 238.0f;
    const int items = SET_COUNT;
    const int per_col = 9;
    const float colw = 218.0f;

    /* Sombra y panel. */
    gfx_rect(x + 4.0f, y + 4.0f, w, h, QV_BG);
    gfx_rect(x, y, w, h, QV_PANEL2);
    gfx_border(x, y, w, h, 2.0f, QV_RED);

    gfx_text((int)x + 16, (int)y + 9, "AJUSTES", QV_TEXT, 2);
    gfx_rect(x + 16.0f, y + 30.0f, w - 32.0f, 1.0f, QV_TRACK);

    char value[32];
    for (int i = 0; i < items; i++) {
        int col = i / per_col;
        int row = i % per_col;
        float cx = x + 12.0f + (float)col * (colw + 8.0f);
        float ry = y + 42.0f + (float)row * 19.0f;
        int selected = (g_settings_sel == i);

        if (selected) {
            gfx_rect(cx - 2.0f, ry - 4.0f, colw, 17.0f, QV_PANEL);
            gfx_rect(cx - 2.0f, ry - 4.0f, 3.0f, 17.0f, QV_RED);
        }

        const char *label = "";
        if (i == 0) {
            label = tr("TEMA");
            snprintf(value, sizeof(value), "%s", gfx_theme_name(g_cfg.theme));
        } else if (i == 1) {
            label = tr("COLOR");
            if (g_cfg.theme == 0) {
                snprintf(value, sizeof(value), "%s",
                         g_color_names[g_cfg.color]);
            } else {
                snprintf(value, sizeof(value), "N/A");
            }
        } else if (i == 2) {
            label = tr("CONTRASTE");
            snprintf(value, sizeof(value), "%s",
                     g_contrast_names[g_cfg.contrast]);
        } else if (i == 3) {
            label = tr("ANIMACION");
            snprintf(value, sizeof(value), "%s",
                     g_cfg.anim == 0 ? tr("NO")
                                     : (g_cfg.anim == 1 ? "MITO" : "MX-5"));
        } else if (i == 4) {
            label = tr("MUESTREO");
            snprintf(value, sizeof(value), "%s",
                     g_cfg.fast ? "RAPIDO" : "COMPLETO");
        } else if (i == 5) {
            label = tr("HUECO ELM");
            snprintf(value, sizeof(value), "%d ms", g_elm_min_gap_ms);
        } else if (i == 6) {
            label = tr("EDITAR VISTA");
            snprintf(value, sizeof(value), "ABRIR");
        } else if (i == 7) {
            label = tr("PICO MAX");
            snprintf(value, sizeof(value), "%s",
                     g_cfg.peak_hold ? "SI" : "NO");
        } else if (i == 8) {
            label = tr("SHIFT RPM");
            snprintf(value, sizeof(value), "%d", g_cfg.shift_rpm);
        } else if (i == 9) {
            label = tr("RPM MAX");
            snprintf(value, sizeof(value), "%d", g_cfg.rpm_max);
        } else if (i == 10) {
            label = tr("UNIDADES");
            snprintf(value, sizeof(value), "%s",
                     g_units_names[g_cfg.units]);
        } else if (i == 11) {
            label = tr("TURBO AUTO");
            snprintf(value, sizeof(value), "%s",
                     g_cfg.boost_auto ? "SI" : "NO");
        } else if (i == 12) {
            label = tr("BOOST MAX");
            char v[16];
            fmt_1(v, sizeof(v), boost_max_bar());
            if (g_cfg.boost_auto) {
                snprintf(value, sizeof(value), "AUTO %s %s", v,
                         units_press_name());
            } else {
                snprintf(value, sizeof(value), "%s %s", v,
                         units_press_name());
            }
        } else if (i == 13) {
            label = tr("IDIOMA");
            const char *names[3] = { "AUTO", "ESPANOL", "ENGLISH" };
            snprintf(value, sizeof(value), "%s", names[g_cfg.lang]);
        } else if (i == 14) {
            label = "FPS";
            snprintf(value, sizeof(value), "%s",
                     g_cfg.show_fps ? "SI" : "NO");
        } else if (i == 15) {
            label = tr("PANTALLA INICIAL");
            snprintf(value, sizeof(value), "%s",
                     (g_start_mode < 0) ? tr("ULTIMA")
                                        : mode_name(g_start_mode));
        } else if (i == 16) {
            label = "DEBUG";
            snprintf(value, sizeof(value), "%s",
                     g_cfg.debug ? "SI" : "NO");
        } else {
            label = tr("SALIR");
            snprintf(value, sizeof(value), "CERRAR APP");
        }

        uint32_t label_color = selected ? QV_TEXT : QV_DIM;
        uint32_t value_color = selected ? QV_AMBER : QV_DIM;
        if (i == SET_EXIT) {
            label_color = selected ? QV_CRIT : QV_DIM;
            value_color = selected ? QV_CRIT : QV_DIM;
        }

        gfx_text((int)cx + 8, (int)ry, label, label_color, 1);
        gfx_text_right((int)(cx + colw) - 8, (int)ry, value,
                       value_color, 1);
    }

    gfx_text((int)x + 16, (int)(y + h) - 28,
             "ARRIBA/ABAJO: ELEGIR   IZQ/DER: CAMBIAR", QV_DIM, 1);
    gfx_text((int)x + 16, (int)(y + h) - 16,
             "X: GUARDAR   CIRCLE: BORRAR TURBO   START: SALIR",
             QV_DIM, 1);
}

/* Editor de la vista principal: que dato y como se pinta cada hueco. */
static void draw_edit_overlay(void)
{
    const float x = 8.0f;
    const float y = 26.0f;
    const float w = 464.0f;
    const float h = 242.0f;

    gfx_rect(x + 4.0f, y + 4.0f, w, h, QV_BG);
    gfx_rect(x, y, w, h, QV_PANEL2);
    gfx_border(x, y, w, h, 2.0f, theme_is_game() ? theme_accent() : QV_RED);

    gfx_text((int)x + 16, (int)y + 8, tr("EDITAR VISTA"), QV_TEXT, 2);
    gfx_rect(x + 16.0f, y + 30.0f, w - 32.0f, 1.0f, QV_TRACK);

    char line[48];
    for (int i = 0; i < DASH_SLOTS; i++) {
        float ry = y + 40.0f + (float)i * 26.0f;
        int selected = (g_edit_sel == i);

        if (selected) {
            gfx_rect(x + 10.0f, ry - 4.0f, w - 20.0f, 22.0f, QV_PANEL);
            gfx_rect(x + 10.0f, ry - 4.0f, 3.0f, 22.0f,
                     theme_is_game() ? theme_accent() : QV_RED);
        }

        snprintf(line, sizeof(line), "%d  %s", i + 1,
                 widget_name(g_slot_item[i]));
        gfx_text((int)x + 20, (int)ry, line,
                 selected ? QV_TEXT : QV_DIM, 1);

        gfx_text_right((int)(x + w) - 20, (int)ry,
                       g_style_names[g_slot_style[i]],
                       selected ? QV_AMBER : QV_DIM, 1);
    }

    gfx_text((int)x + 16, (int)(y + h) - 34,
             "ARRIBA/ABAJO: HUECO   IZQ/DER: RELOJ/BARRA/TEXTO/OCULTO",
             QV_DIM, 1);
    gfx_text((int)x + 16, (int)(y + h) - 22,
             "TRIANGLE/SQUARE: CAMBIAR DATO", QV_DIM, 1);
    gfx_text((int)x + 16, (int)(y + h) - 11,
             "X o START: GUARDAR", QV_TEXT, 1);
}

/* Series disponibles en TELEMETRIA: cualquier canal soportado por la
 * ECU, mas OCULTO. El orden del enum es el orden de seleccion. */
static int tel_cycle_step(int item, int dir)
{
    return widget_cycle(item, dir, 1);
}

static void draw_tel_edit_overlay(void)
{
    const float x = 8.0f;
    const float y = 34.0f;
    const float w = 464.0f;
    const float h = 214.0f;

    gfx_rect(x + 4.0f, y + 4.0f, w, h, QV_BG);
    gfx_rect(x, y, w, h, QV_PANEL2);
    gfx_border(x, y, w, h, 2.0f, theme_is_game() ? theme_accent() : QV_RED);

    gfx_text((int)x + 16, (int)y + 8, tr("SERIES DE TELEMETRIA"), QV_TEXT, 2);
    gfx_rect(x + 16.0f, y + 30.0f, w - 32.0f, 1.0f, QV_TRACK);

    char line[40];
    for (int i = 0; i < TEL_SLOTS; i++) {
        float ry = y + 42.0f + (float)i * 26.0f;
        int selected = (g_tel_edit_sel == i);

        if (selected) {
            gfx_rect(x + 10.0f, ry - 4.0f, w - 20.0f, 24.0f, QV_PANEL);
            gfx_rect(x + 10.0f, ry - 4.0f, 3.0f, 24.0f,
                     theme_is_game() ? theme_accent() : QV_RED);
        }

        int item = g_tel_item[i];
        snprintf(line, sizeof(line), "SERIE %d", i + 1);
        gfx_text((int)x + 20, (int)ry, line,
                 selected ? QV_TEXT : QV_DIM, 1);

        const char *name = (item == W_OFF) ? "OCULTO" : tel_name(item);
        gfx_text_right((int)(x + w) - 20, (int)ry, name,
                       selected ? tel_color(item) : QV_DIM, 1);
    }

    gfx_text((int)x + 16, (int)(y + h) - 24,
             "ARRIBA/ABAJO: SERIE   IZQ/DER: CAMBIAR", QV_DIM, 1);
    gfx_text((int)x + 16, (int)(y + h) - 12,
             "X o START: GUARDAR", QV_TEXT, 1);
}

static void draw_net_banner(void)
{
    if (!g_net_busy || g_mode == MODE_MODULES ||
        g_mode == MODE_SNIFFER) {
        return;
    }

    const float x = 110.0f;
    const float y = 102.0f;
    const float w = 260.0f;
    const float h = 66.0f;

    gfx_rect(x + 4.0f, y + 4.0f, w, h, QV_BG);
    gfx_rect(x, y, w, h, QV_PANEL2);
    gfx_border(x, y, w, h, 2.0f, QV_RED);

    /* FASE F2: el aviso de red no se atenua nunca: es justo el
     * momento en que el usuario necesita leerlo. */
    int saved_dim = g_gfx_dim;
    g_gfx_dim = 100;

    gfx_text_center(GFX_W / 2, (int)y + 14, g_net_msg, QV_TEXT, 1);

    /* Puntos animados mientras se espera a la red. */
    char dots[8] = "";
    int ndots = (int)((g_net_spin / 4u) % 4u);
    for (int i = 0; i < ndots; i++) {
        dots[i] = '.';
    }
    dots[ndots] = '\0';
    gfx_text_center(GFX_W / 2, (int)y + 36, dots, QV_AMBER, 2);

    g_gfx_dim = saved_dim;
}

static void run_sniff(int sock)
{
    if (g_snif_running) {
        return;
    }

    g_snif_running = 1;
    g_snif_total = 0;
    g_snif_view = 0;
    for (int i = 0; i < SNIF_LINES; i++) {
        g_snif_buf[i][0] = '\0';
    }

    log_file_printf("=== SNIFFER CAN ===\n");
    log_file_printf("Protocolo: %s\n", g_proto);

    if (sock < 0) {
        g_snif_running = 0;
        log_file_printf("Sniffer: sin adaptador\n");
        log_sync();
        return;
    }

    g_net_busy = 1;
    g_net_ui_last = 0;

    elm_reply reply;
    elm_send_and_read(sock, "ATE0", 800, &reply);
    elm_send_and_read(sock, "ATL0", 800, &reply);
    elm_send_and_read(sock, "ATH1", 800, &reply);
    elm_send_and_read(sock, "ATS1", 800, &reply);
    if (scan_can()) {
        elm_send_and_read(sock, "ATCAF0", 800, &reply);
        elm_send_and_read(sock, "ATAL", 800, &reply);
    }

    /* Iniciar monitor: ATMA no devuelve prompt hasta que se detiene. */
    const char *cmd = "ATMA\r";
    sceNetInetSend(sock, cmd, (int)strlen(cmd), 0);

    log_file_printf("Sniffer: monitor activo (CIRCLE para parar)\n");
    log_sync();

    char line[256];
    int llen = 0;
    uint32_t start = now_ms();
    int running = 1;

    while (running) {
        SceCtrlData pad;
        memset(&pad, 0, sizeof(pad));
        sceCtrlReadBufferPositive(&pad, 1);
        if (pad.Buttons & (PSP_CTRL_CIRCLE | PSP_CTRL_CROSS)) {
            running = 0;
            continue;
        }

        SceNetInetPollfd pfd;
        pfd.fd = sock;
        pfd.events = SCE_NET_INET_POLLIN;
        pfd.revents = 0;
        int polled = sceNetInetPoll(&pfd, 1, 50);
        if (polled > 0) {
            char buf[256];
            int n = sceNetInetRecv(sock, buf, sizeof(buf), 0);
            if (n > 0) {
                for (int i = 0; i < n; i++) {
                    char c = buf[i];
                    if (c == '\r' || c == '\n') {
                        if (llen > 0) {
                            line[llen] = '\0';
                            char text[SNIF_LEN];
                            trim_copy(text, sizeof(text), line);
                            if (text[0] != '\0') {
                                log_file_printf("SNIFF %s\n", text);
                                if (g_snif_total < SNIF_LINES) {
                                    snprintf(g_snif_buf[g_snif_total],
                                             SNIF_LEN, "%s", text);
                                } else {
                                    /* buffer circular: desplazar */
                                    for (int j = 1; j < SNIF_LINES; j++) {
                                        memcpy(g_snif_buf[j - 1],
                                               g_snif_buf[j], SNIF_LEN);
                                    }
                                    snprintf(g_snif_buf[SNIF_LINES - 1],
                                             SNIF_LEN, "%s", text);
                                }
                                g_snif_total++;
                                if (g_snif_total > SNIF_LINES) {
                                    g_snif_total = SNIF_LINES;
                                }
                            }
                            llen = 0;
                        }
                    } else if (llen < (int)sizeof(line) - 1) {
                        line[llen++] = c;
                    }
                }
            } else if (n < 0) {
                int err = sceNetInetGetErrno();
                if (err != EWOULDBLOCK && err != EAGAIN) {
                    running = 0;
                }
            }
        }

        if ((uint32_t)(now_ms() - start) > 120000u) {
            running = 0;
        }
    }

    /* Detener monitor: cualquier caracter. */
    char stop = ' ';
    sceNetInetSend(sock, &stop, 1, 0);
    sceKernelDelayThread(300 * 1000);

    /* Drenar lo que quede. */
    for (int k = 0; k < 20; k++) {
        SceNetInetPollfd pfd;
        pfd.fd = sock;
        pfd.events = SCE_NET_INET_POLLIN;
        pfd.revents = 0;
        if (sceNetInetPoll(&pfd, 1, 50) <= 0) {
            break;
        }
        char buf[256];
        int n = sceNetInetRecv(sock, buf, sizeof(buf), 0);
        if (n <= 0) {
            break;
        }
    }

    /* Restaurar ELM327. */
    elm_send_and_read(sock, "ATE0", 800, &reply);
    elm_send_and_read(sock, "ATL0", 800, &reply);
    elm_send_and_read(sock, "ATS0", 800, &reply);
    elm_send_and_read(sock, "ATH0", 800, &reply);
    if (scan_can()) {
        elm_send_and_read(sock, "ATNL", 800, &reply);
        elm_send_and_read(sock, "ATCAF1", 800, &reply);
    }

    g_net_busy = 0;
    g_snif_running = 0;
    g_net_ui_last = 0;

    log_file_printf("=== FIN SNIFFER: %d tramas ===\n",
                    g_snif_total);
    log_sync();
}

/* ------------------------------------------------------------------ */
/* SONDA DEL COCHE (pantalla PROBE).                                  */
/*                                                                    */
/* Para probar la app en un coche que no es el de referencia. No es un */
/* dashboard: lee la identidad del adaptador y de la ECU, las mascaras */
/* de PID y despues pide cada PID una vez para dejar escrito que       */
/* contesta la centralita y que no. Solo lectura: modo 01 y modo 09,   */
/* ningun servicio de escritura.                                      */
/*                                                                    */
/* Es una herramienta de diagnostico, no parte del producto: lo que    */
/* vale de verdad es el log de la sesion, donde quedan todas las       */
/* respuestas crudas. En pantalla cabe el resumen.                    */
/* ------------------------------------------------------------------ */

#define PROBE_ROWS    24
#define PROBE_LABEL   18
#define PROBE_VALUE   18
#define PROBE_VISIBLE 12
#define PROBE_AT_MS   2000
#define PROBE_PID_MS  2000

enum {
    PROBE_IDLE = 0,
    PROBE_RUNNING,
    PROBE_DONE,
    PROBE_NOLINK
};

typedef struct {
    uint8_t pid;
    int item;                     /* elemento del panel, para tel_print */
    char label[PROBE_LABEL];
    char value[PROBE_VALUE];
    uint8_t answered;
    uint8_t no_data;
} probe_row;

static probe_row g_probe[PROBE_ROWS];
static int g_probe_count = 0;
static int g_probe_ok = 0;
static int g_probe_state = PROBE_IDLE;
static int g_probe_scroll = 0;
static int g_probe_request = 0;
static char g_probe_hint[48] = "";
static char g_probe_proto[24] = "";
static char g_probe_ati[24] = "";
static char g_probe_vbat[16] = "";
static char g_probe_header[48] = "";
static char g_probe_vin[24] = "";
static char g_probe_cal[24] = "";
static char g_probe_cvn[24] = "";
static char g_probe_ecu[24] = "";
static uint32_t g_probe_mask[3];

/* Copia acotada, sin depender de snprintf para no pelearse con
 * -Wformat-truncation. */
static void probe_copy(char *dst, size_t size, const char *src)
{
    size_t n;

    if (size == 0) {
        return;
    }
    if (src == NULL) {
        src = "";
    }
    n = strlen(src);
    if (n >= size) {
        n = size - 1;
    }
    memcpy(dst, src, n);
    dst[n] = '\0';
}

/* "a b" en un buffer acotado. */
static void probe_pair(char *dst, size_t size, const char *a, const char *b)
{
    size_t used;

    if (size == 0) {
        return;
    }
    dst[0] = '\0';
    probe_copy(dst, size, a);
    used = strlen(dst);
    if (used + 2 < size) {
        dst[used] = ' ';
        dst[used + 1] = '\0';
        used++;
    }
    if (b != NULL && b[0] != '\0' && used + 1 < size) {
        probe_copy(dst + used, size - used, b);
    }
}

/* Comando AT cuya respuesta no interesa. */
static void probe_send(int sock, const char *cmd, int wait_ms)
{
    elm_reply reply;

    elm_send_and_read(sock, cmd, wait_ms, &reply);
}

static void probe_status(const char *msg)
{
    probe_copy(g_probe_hint, sizeof(g_probe_hint), msg);
    net_ui_set(g_probe_hint);
    net_ui_tick();
}

static void probe_clear(void)
{
    memset(g_probe, 0, sizeof(g_probe));
    g_probe_count = 0;
    g_probe_ok = 0;
    g_probe_mask[0] = 0;
    g_probe_mask[1] = 0;
    g_probe_mask[2] = 0;
    g_probe_proto[0] = '\0';
    g_probe_ati[0] = '\0';
    g_probe_vbat[0] = '\0';
    g_probe_header[0] = '\0';
    g_probe_vin[0] = '\0';
    g_probe_cal[0] = '\0';
    g_probe_cvn[0] = '\0';
    g_probe_ecu[0] = '\0';
    g_probe_hint[0] = '\0';
    g_probe_scroll = 0;
}

/* Un comando AT, su respuesta de texto y al log. */
static void probe_at(int sock, const char *cmd, char *out, size_t out_size)
{
    elm_reply reply;

    elm_send_and_read(sock, cmd, PROBE_AT_MS, &reply);
    probe_copy(out, out_size, reply.text);
    log_file_printf("PROBE %-8s -> '%s'\n", cmd, reply.text);
}

/* Busca "41 <pid> xxxx" dentro de las lineas capturadas y devuelve la
 * mascara. En K-line con ATH1 delante van los tres bytes de cabecera,
 * asi que se busca el marcador 0x41 en vez de asumir la posicion: la
 * sonda tiene que funcionar en cualquier protocolo, aunque el parser de
 * descubrimiento del nucleo solo entienda cabeceras CAN. */
static uint32_t probe_mask_from_capture(int *found)
{
    *found = 0;
    for (int i = 0; i < g_cap_count; i++) {
        obd_disc_line line;
        if (!obd_disc_parse_line(g_cap[i], &line)) {
            continue;
        }
        for (int b = 0; b + 6 <= line.data_len; b++) {
            if (line.data[b] != 0x41) {
                continue;
            }
            uint8_t pid = line.data[b + 1];
            if (pid != 0x00 && pid != 0x20 && pid != 0x40) {
                continue;
            }
            *found = 1;
            return obd_pid_mask(&line.data[b + 2],
                                line.data_len - (b + 2));
        }
    }
    return 0;
}

/* Lee el bloque de mascara indicado (0x00, 0x20 o 0x40). */
static uint32_t probe_mask_block(int sock, uint8_t block)
{
    char cmd[8];
    int found = 0;
    uint32_t mask;

    snprintf(cmd, sizeof(cmd), "01%02X", (unsigned)block);
    scan_send(sock, cmd, PROBE_PID_MS);
    mask = probe_mask_from_capture(&found);
    log_file_printf("PROBE %s: mascara %08X%s\n", cmd, (unsigned)mask,
                    found ? "" : " (sin respuesta)");
    for (int i = 0; i < g_cap_count; i++) {
        log_file_printf("  %s\n", g_cap[i]);
    }
    return found ? mask : 0;
}

/* Texto ASCII de una respuesta de modo 09 (VIN, CalID, CVN, nombre de
 * la ECU). Las respuestas largas llegan en varias lineas y algunos
 * clones las prefieren con "0:", "1:"; el 0x49 se busca en cualquier
 * posicion para no depender del protocolo. */
static void probe_mode09(int sock, const char *cmd, const char *tag,
                         char *out, size_t out_size)
{
    size_t written = 0;

    if (out_size == 0) {
        return;
    }
    out[0] = '\0';
    scan_send(sock, cmd, PROBE_PID_MS);
    log_file_printf("PROBE %s (%s):\n", tag, cmd);
    for (int i = 0; i < g_cap_count; i++) {
        log_file_printf("  %s\n", g_cap[i]);
    }

    for (int i = 0; i < g_cap_count && written + 1 < out_size; i++) {
        obd_disc_line line;
        if (!obd_disc_parse_line(g_cap[i], &line)) {
            continue;
        }
        for (int b = 0; b + 3 < line.data_len; b++) {
            if (line.data[b] != 0x49) {
                continue;
            }
            /* 49 <pid> <n>: se salta la cabecera de respuesta y el
             * numero de trama. */
            for (int k = b + 3;
                 k < line.data_len && written + 1 < out_size; k++) {
                unsigned char c = line.data[k];
                if (c >= 0x20 && c < 0x7F) {
                    out[written++] = (char)c;
                }
            }
            break;
        }
    }
    out[written] = '\0';
    log_file_printf("PROBE %s texto: '%s'\n", tag, out);
}

static int probe_row_for_pid(uint8_t pid)
{
    for (int i = 0; i < g_probe_count; i++) {
        if (g_probe[i].pid == pid) {
            return i;
        }
    }
    return -1;
}

/* Pide un PID una vez y apunta que ha pasado: respondio, NO DATA o
 * timeout. Es la parte que dice que PID tiene de verdad el coche. */
static void probe_pid(int sock, int item, uint8_t pid, const char *name)
{
    char cmd[8];
    char label[PROBE_LABEL];
    elm_reply reply;
    probe_row *row;

    if (g_probe_count >= PROBE_ROWS) {
        return;
    }
    if (probe_row_for_pid(pid) >= 0) {
        return;   /* MARIPOSA y VELOCIDAD comparten 0x0D */
    }

    row = &g_probe[g_probe_count++];
    memset(row, 0, sizeof(*row));
    row->pid = pid;
    row->item = item;

    snprintf(cmd, sizeof(cmd), "01%02X", (unsigned)pid);
    probe_pair(label, sizeof(label), cmd, name);
    probe_copy(row->label, sizeof(row->label), label);

    elm_send_and_read(sock, cmd, PROBE_PID_MS, &reply);

    if (reply.data && elm_frame_matches(&reply.frame, 0x01, pid)) {
        obd_pid_value value;
        row->answered = 1;
        g_probe_ok++;
        if (obd_pid_decode(pid, reply.frame.data, reply.frame.data_len,
                           &value) == 0) {
            tel_print(item, value.value, row->value, sizeof(row->value));
        } else {
            probe_copy(row->value, sizeof(row->value), "OK");
        }
    } else if (reply.no_data) {
        row->no_data = 1;
        probe_copy(row->value, sizeof(row->value), "NO DATA");
    } else if (reply.unable) {
        probe_copy(row->value, sizeof(row->value), "SIN COCHE");
    } else if (reply.stopped) {
        probe_copy(row->value, sizeof(row->value), "STOPPED");
    } else {
        probe_copy(row->value, sizeof(row->value), "TIMEOUT");
    }

    log_file_printf("PROBE %s %-11s = %s\n", cmd, name, row->value);
}

static void probe_sweep(int sock)
{
    for (int item = 0; item < W_COUNT; item++) {
        uint8_t pid = widget_pid(item);
        if (pid == 0) {
            continue;   /* BOOST es derivado de MAP y baro */
        }
        probe_pid(sock, item, pid, g_widget_names[item]);
        if (scan_cancelled()) {
            log_file_printf("PROBE: cancelada por el usuario\n");
            return;
        }
        net_ui_tick();
    }
}

/* Mascaras: deja en el log la lista de PID que el coche declara. */
static void probe_log_supported(void)
{
    log_file_printf("PROBE mascaras: %08X %08X %08X\n",
                    (unsigned)g_probe_mask[0], (unsigned)g_probe_mask[1],
                    (unsigned)g_probe_mask[2]);
    log_file_printf("PROBE declara:");
    for (int pid = 1; pid <= 0x60; pid++) {
        int block = (pid - 1) / 0x20;
        int bit = (pid - 1) % 0x20;
        if (block < 0 || block > 2 || bit < 0 || bit > 31) {
            break;
        }
        if ((g_probe_mask[block] >> (31 - bit)) & 1u) {
            log_file_printf(" %02X", pid);
        }
    }
    log_file_printf("\n");
}

static void run_probe(int sock)
{
    probe_clear();

    if (sock < 0) {
        g_probe_state = PROBE_NOLINK;
        log_file_printf("SONDA: sin adaptador, no se puede sondear\n");
        return;
    }

    log_file_printf("=== SONDA DEL COCHE ===\n");
    g_probe_state = PROBE_RUNNING;
    g_net_busy = 1;
    g_net_ui_last = 0;

    probe_status("SONDA: ADAPTADOR");
    probe_send(sock, "ATE0", 1000);
    probe_send(sock, "ATL0", 1000);
    probe_send(sock, "ATS0", 1000);
    probe_send(sock, "ATH0", 1000);
    probe_at(sock, "ATI", g_probe_ati, sizeof(g_probe_ati));
    probe_at(sock, "ATDPN", g_probe_proto, sizeof(g_probe_proto));
    probe_at(sock, "ATRV", g_probe_vbat, sizeof(g_probe_vbat));
    {
        char proto_desc[48];
        probe_at(sock, "ATDP", proto_desc, sizeof(proto_desc));
    }

    if (scan_cancelled()) {
        goto probe_done;
    }

    probe_status("SONDA: MASCARAS DE PID");
    g_probe_mask[0] = probe_mask_block(sock, 0x00);
    if (obd_mask_supports(g_probe_mask[0], 0x20)) {
        g_probe_mask[1] = probe_mask_block(sock, 0x20);
        if (obd_mask_supports(g_probe_mask[1], 0x40)) {
            g_probe_mask[2] = probe_mask_block(sock, 0x40);
        }
    }
    probe_log_supported();

    if (scan_cancelled()) {
        goto probe_done;
    }

    /* Cabecera cruda: es la evidencia que hace falta para arreglar el
     * descubrimiento de modulos en K-line. */
    probe_status("SONDA: CABECERA");
    probe_send(sock, "ATH1", 1000);
    probe_send(sock, "ATS1", 1000);
    scan_send(sock, "0100", PROBE_PID_MS);
    if (g_cap_count > 0) {
        probe_copy(g_probe_header, sizeof(g_probe_header), g_cap[0]);
    }
    log_file_printf("PROBE cabecera 0100: '%s'\n", g_probe_header);
    probe_send(sock, "ATH0", 1000);
    probe_send(sock, "ATS0", 1000);

    if (scan_cancelled()) {
        goto probe_done;
    }

    probe_status("SONDA: VIN");
    probe_mode09(sock, "0902", "VIN", g_probe_vin, sizeof(g_probe_vin));
    probe_status("SONDA: CALID");
    probe_mode09(sock, "0904", "CALID", g_probe_cal, sizeof(g_probe_cal));
    probe_status("SONDA: CVN");
    probe_mode09(sock, "0906", "CVN", g_probe_cvn, sizeof(g_probe_cvn));
    probe_status("SONDA: ECU");
    probe_mode09(sock, "090A", "ECU", g_probe_ecu, sizeof(g_probe_ecu));

    if (scan_cancelled()) {
        goto probe_done;
    }

    probe_status("SONDA: PID UNO A UNO");
    probe_sweep(sock);

probe_done:
    /* El dashboard necesita el adaptador como lo dejo el handshake. */
    probe_send(sock, "ATH0", 800);
    probe_send(sock, "ATS0", 800);
    probe_send(sock, "ATE0", 800);
    probe_send(sock, "ATL0", 800);
    scan_refresh_proto(sock, "sonda");
    probe_copy(g_probe_proto, sizeof(g_probe_proto), g_proto);

    log_file_printf("PROBE resumen: %d/%d PID responden, protocolo '%s'\n",
                    g_probe_ok, g_probe_count, g_probe_proto);
    log_sync();

    g_probe_state = PROBE_DONE;
    g_probe_hint[0] = '\0';
    g_net_busy = 0;
    g_net_ui_last = 0;
}

static void draw_probe(void)
{
    char txt[80];
    int max_scroll = g_probe_count - PROBE_VISIBLE;

    if (max_scroll < 0) {
        max_scroll = 0;
    }
    if (g_probe_scroll > max_scroll) {
        g_probe_scroll = max_scroll;
    }
    if (g_probe_scroll < 0) {
        g_probe_scroll = 0;
    }

    gfx_text(8, 32, tr("SONDA DEL COCHE"), QV_TEXT, 1);

    if (g_probe_state == PROBE_IDLE) {
        gfx_text(8, 56, tr("PULSA X PARA SONDEAR EL COCHE."), QV_TEXT, 1);
        gfx_text(8, 72, "Solo lectura: modo 01 y modo 09.", QV_DIM, 1);
        gfx_text(8, 88, "El resultado completo queda en el log.", QV_DIM, 1);
        return;
    }
    if (g_probe_state == PROBE_NOLINK) {
        gfx_text(8, 56, "SIN ADAPTADOR.", QV_AMBER, 1);
        gfx_text(8, 72, "TRIANGLE REINTENTA LA CONEXION.", QV_DIM, 1);
        return;
    }
    if (g_probe_state == PROBE_RUNNING) {
        gfx_text(8, 56, g_probe_hint, QV_AMBER, 1);
        gfx_text(8, 72, "CIRCLE CANCELA.", QV_DIM, 1);
        return;
    }

    /* Adaptador y protocolo. */
    gfx_text(8, 44, "PROT", QV_DIM, 1);
    gfx_text(40, 44, g_probe_proto[0] ? g_probe_proto : "-", QV_TEXT, 1);
    gfx_text(96, 44, "BAT", QV_DIM, 1);
    gfx_text(122, 44, g_probe_vbat[0] ? g_probe_vbat : "-", QV_TEXT, 1);
    gfx_text(176, 44, "ADAPTADOR", QV_DIM, 1);
    gfx_text(238, 44, g_probe_ati[0] ? g_probe_ati : "-", QV_TEXT, 1);

    snprintf(txt, sizeof(txt), "MASC %08X %08X %08X",
             (unsigned)g_probe_mask[0], (unsigned)g_probe_mask[1],
             (unsigned)g_probe_mask[2]);
    gfx_text(8, 56, txt, QV_DIM, 1);

    snprintf(txt, sizeof(txt), "PID %d/%d", g_probe_ok, g_probe_count);
    gfx_text(8, 68, txt, g_probe_ok > 0 ? QV_OK : QV_AMBER, 1);
    if (g_probe_vin[0] != '\0') {
        snprintf(txt, sizeof(txt), "VIN %s", g_probe_vin);
        gfx_text(96, 68, txt, QV_TEXT, 1);
    }

    gfx_rect(8.0f, 80.0f, GFX_W - 16.0f, 1.0f, QV_TRACK);
    gfx_text(10, 84, "PID  DATO", QV_DIM, 1);
    gfx_text_right(400, 84, "RESPUESTA", QV_DIM, 1);

    for (int i = 0; i < PROBE_VISIBLE; i++) {
        int idx = g_probe_scroll + i;
        uint32_t color;
        if (idx >= g_probe_count) {
            break;
        }
        if (g_probe[idx].answered) {
            color = QV_OK;
        } else if (g_probe[idx].no_data) {
            color = QV_AMBER;
        } else {
            color = QV_DIM;
        }
        int y = 96 + i * 13;
        gfx_text(10, y, g_probe[idx].label, QV_TEXT, 1);
        gfx_text_right(400, y, g_probe[idx].value, color, 1);
    }

    if (g_probe_ecu[0] != '\0') {
        snprintf(txt, sizeof(txt), "ECU %s", g_probe_ecu);
        gfx_text(8, 258, txt, QV_DIM, 1);
    }

    if (max_scroll > 0) {
        float tx = GFX_W - 8.0f;
        float ty = 92.0f;
        float th = 160.0f;
        float bh;
        gfx_rect(tx, ty, 2.0f, th, QV_TRACK);
        bh = th * (float)PROBE_VISIBLE / (float)g_probe_count;
        gfx_rect(tx, ty + (th - bh) * (float)g_probe_scroll /
                     (float)max_scroll, 2.0f, bh, QV_DIM);
    }
}

/* Selector de pantallas (SELECT). Las lista todas con su proposito y su
 * estado en este coche: lo que no se puede usar sale en gris y con el
 * motivo, porque ocultarlo no ensena nada. */
static void draw_picker(void)
{
    int y = 44;

    gfx_rect(0.0f, 0.0f, (float)GFX_W, (float)GFX_H, QV_BG);
    gfx_rect(0.0f, 0.0f, (float)GFX_W, 26.0f, QV_PANEL);
    gfx_text(8, 8, tr("PANTALLAS"), QV_TEXT, 1);
    gfx_text_right(GFX_W - 8, 8, tr("X: ABRIR   SELECT: CERRAR"), QV_DIM, 1);
    gfx_rect(8.0f, 30.0f, (float)GFX_W - 16.0f, 1.0f, QV_TRACK);

    for (int i = 0; i < MODE_COUNT; i++) {
        int mode = g_picker_order[i];
        int avail = screen_offered(mode);
        int sel = (i == g_picker_sel);

        if (!avail && !screen_available(mode)) {
            /* oculta: no ocupa sitio en la lista */
            continue;
        }
        if (i == 4) {
            gfx_text(14, y, tr("HERRAMIENTAS"), QV_DIM, 1);
            y += 14;
        }
        if (sel) {
            gfx_rect(4.0f, (float)y - 3.0f, (float)GFX_W - 8.0f, 20.0f,
                     QV_PANEL2);
            gfx_rect(4.0f, (float)y - 3.0f, 3.0f, 20.0f,
                     theme_is_game() ? theme_accent() : QV_RED);
        }
        gfx_text(14, y, mode_name(mode), avail ? QV_TEXT : QV_DIM, 1);
        gfx_text(120, y, mode_purpose(mode), QV_DIM, 1);
        if (!avail) {
            gfx_text_right(GFX_W - 12, y, screen_reason(mode), QV_AMBER, 1);
        } else if (mode == g_mode) {
            gfx_text_right(GFX_W - 12, y, tr("ACTUAL"), QV_OK, 1);
        }
        y += 24;
    }
}

/* ------------------------------------------------------------------ */
/* FALLOS: codigos de averia (modos 03, 07 y 0A). Solo lectura.        */
/* ------------------------------------------------------------------ */

#define DTC_MAX 12

static obd_dtc g_dtc_stored[DTC_MAX];
static obd_dtc g_dtc_pending[DTC_MAX];
static obd_dtc g_dtc_permanent[DTC_MAX];
static int g_dtc_stored_n = 0;
static int g_dtc_pending_n = 0;
static int g_dtc_permanent_n = 0;
static int g_dtc_mil = -1;        /* -1 = no se ha leido */
static int g_dtc_count = -1;
static int g_dtc_state = 0;       /* 0 sin leer, 1 leyendo, 2 leido */
static char g_dtc_note[32] = "";
static int g_dtc_request = 0;

static int dtc_hex_digit(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

/* Pasa a bytes la respuesta en texto del ELM327 ("43 01 33 00 00 00 00").
 * Devuelve cuantos bytes hay, servicio incluido; 0 si no habia datos. */
static int dtc_read_hex(int sock, const char *cmd, uint8_t *out, int max,
                        elm_reply *reply)
{
    elm_send_and_read(sock, cmd, 3000, reply);
    if (!reply->data) {
        return 0;
    }

    int n = 0;
    const char *p = reply->text;
    while (*p != '\0' && n < max) {
        while (*p == ' ') {
            p++;
        }
        if (dtc_hex_digit(p[0]) < 0 || dtc_hex_digit(p[1]) < 0) {
            break;
        }
        out[n++] = (uint8_t)((dtc_hex_digit(p[0]) << 4) | dtc_hex_digit(p[1]));
        p += 2;
    }
    return n;
}

static void run_fallos(int sock)
{
    if (g_dtc_state == 1) {
        return;
    }

    g_dtc_state = 1;
    g_dtc_stored_n = 0;
    g_dtc_pending_n = 0;
    g_dtc_permanent_n = 0;
    g_dtc_mil = -1;
    g_dtc_count = -1;
    snprintf(g_dtc_note, sizeof(g_dtc_note), "%s", tr("LEYENDO"));

    if (sock < 0) {
        g_dtc_state = 2;
        snprintf(g_dtc_note, sizeof(g_dtc_note), "%s", tr("SIN ADAPTADOR"));
        return;
    }

    /* Con las cabeceras apagadas, que es como trabaja el dashboard: la
     * respuesta empieza por el byte de servicio (41, 43, 47, 4A). */
    elm_reply reply;
    uint8_t buf[32];

    log_file_printf("=== CODIGOS DE AVERIA ===\n");

    int n = dtc_read_hex(sock, "0101", buf, (int)sizeof(buf), &reply);
    if (n >= 3 && buf[0] == 0x41 && buf[1] == 0x01) {
        obd_dtc_mil(&buf[2], n - 2, &g_dtc_mil, &g_dtc_count);
        log_file_printf("MIL: %d, codigos: %d\n", g_dtc_mil, g_dtc_count);
    } else {
        log_file_printf("MIL: sin respuesta (0101)\n");
    }

    n = dtc_read_hex(sock, "03", buf, (int)sizeof(buf), &reply);
    if (n >= 2 && buf[0] == 0x43) {
        g_dtc_stored_n = obd_dtc_parse(&buf[1], n - 1, g_dtc_stored, DTC_MAX);
        if (g_dtc_stored_n < 0) {
            g_dtc_stored_n = 0;
        }
    }

    n = dtc_read_hex(sock, "07", buf, (int)sizeof(buf), &reply);
    if (n >= 2 && buf[0] == 0x47) {
        g_dtc_pending_n = obd_dtc_parse(&buf[1], n - 1, g_dtc_pending, DTC_MAX);
        if (g_dtc_pending_n < 0) {
            g_dtc_pending_n = 0;
        }
    }

    n = dtc_read_hex(sock, "0A", buf, (int)sizeof(buf), &reply);
    if (n >= 2 && buf[0] == 0x4A) {
        g_dtc_permanent_n = obd_dtc_parse(&buf[1], n - 1, g_dtc_permanent,
                                          DTC_MAX);
        if (g_dtc_permanent_n < 0) {
            g_dtc_permanent_n = 0;
        }
    }

    log_file_printf("Codigos: guardados=%d pendientes=%d permanentes=%d\n",
                    g_dtc_stored_n, g_dtc_pending_n, g_dtc_permanent_n);
    log_sync();

    g_dtc_state = 2;
    g_dtc_note[0] = '\0';
}

/* Una fila de codigo: el codigo y, si la tabla lo conoce, que significa. */
static void dtc_draw_row(const obd_dtc *d, int x, int y)
{
    const char *desc = obd_dtc_describe(d->code);
    gfx_text(x, y, d->code, QV_TEXT, 1);
    if (desc != NULL) {
        gfx_text(x + 56, y, desc, QV_DIM, 1);
    } else {
        gfx_text(x + 56, y, tr("sin descripcion"), QV_DIM, 1);
    }
}

static void draw_fallos(void)
{
    int y = 40;

    gfx_text(8, y, tr("FALLOS"), QV_TEXT, 2);
    char line[48];
    const char *mil = (g_dtc_mil < 0) ? "N/D"
                                      : (g_dtc_mil ? tr("SI") : tr("NO"));
    const char *cnt = (g_dtc_count < 0) ? "N/D" : "";
    if (g_dtc_count >= 0) {
        snprintf(line, sizeof(line), "%s %s   %s %s %d", tr("LAMPARA"), mil,
                 tr("CODIGOS"), cnt, g_dtc_count);
    } else {
        snprintf(line, sizeof(line), "%s %s   %s %s", tr("LAMPARA"), mil,
                 tr("CODIGOS"), cnt);
    }
    gfx_text_right(GFX_W - 8, y + 4, line, QV_DIM, 1);
    y += 24;

    if (g_dtc_state == 1) {
        gfx_text(8, y, g_dtc_note, QV_AMBER, 1);
        return;
    }

    static const int group_n[3] = { 0, 0, 0 };
    (void)group_n;

    const char *names[3] = { tr("GUARDADOS"), tr("PENDIENTES"),
                             tr("PERMANENTES") };
    const obd_dtc *sets[3] = { g_dtc_stored, g_dtc_pending,
                               g_dtc_permanent };
    const int counts[3] = { g_dtc_stored_n, g_dtc_pending_n,
                            g_dtc_permanent_n };

    if (g_dtc_state == 0) {
        gfx_text(8, y, tr("X: LEER   CIRCLE: LIMPIAR PANTALLA"), QV_DIM, 1);
        return;
    }

    for (int g = 0; g < 3; g++) {
        if (y > 216) {
            break;
        }
        snprintf(line, sizeof(line), "%s (%d)", names[g], counts[g]);
        gfx_text(8, y, line, QV_CYAN, 1);
        y += 14;
        if (counts[g] == 0) {
            gfx_text(20, y, tr("SIN CODIGOS"), QV_DIM, 1);
            y += 14;
            continue;
        }
        for (int i = 0; i < counts[g] && y <= 216; i++) {
            dtc_draw_row(&sets[g][i], 20, y);
            y += 14;
        }
        y += 4;
    }

    gfx_text(8, 254, tr("X: LEER   CIRCLE: LIMPIAR PANTALLA"), QV_DIM, 1);
}

static void draw_modules(void)
{
    /* En la lista de resultados la cabecera la forma la linea de
     * protocolo/modulos; el titulo solo se dibuja cuando no hay lista. */
    if (!(g_scan_state == 2 && g_scan_count > 0)) {
        gfx_text(16, 34, "DESCUBRIMIENTO DE MODULOS", QV_TEXT, 1);
    }
    gfx_rect(16.0f, 48.0f, GFX_W - 32.0f, 1.0f, QV_TRACK);

    if (g_scan_state == 0) {
        gfx_text(16, 62, "Pulsa X para escanear el bus.", QV_AMBER, 1);
        gfx_text(16, 82, "OBD-II: modulos que responden a 0100/0120/0140.",
                 QV_DIM, 1);
        gfx_text(16, 98, "UDS: deteccion con 3E00 y lectura de DIDs",
                 QV_DIM, 1);
        gfx_text(16, 114, "F190 VIN, F197 sistema, F187 pieza,",
                 QV_DIM, 1);
        gfx_text(16, 130, "F188 software y F18C numero de serie.",
                 QV_DIM, 1);
        gfx_text(16, 158, "Solo lectura: no se escribe nada en la ECU.",
                 QV_GREEN, 1);
        gfx_text(16, 186, "La centralita de suspension puede no aparecer:",
                 QV_DIM, 1);
        gfx_text(16, 202, "muchas no hablan OBD-II ni UDS estandar.",
                 QV_DIM, 1);
        gfx_text(16, 238, "L/R: PANTALLA   TRIANGLE: RECONECTAR",
                 QV_DIM, 1);
        return;
    }

    if (g_scan_state == 1) {
        char dots[6] = "";
        int ndots = (int)((g_net_spin / 4u) % 4u);
        for (int i = 0; i < ndots; i++) {
            dots[i] = '.';
        }
        dots[ndots] = '\0';

        gfx_text(16, 64, g_scan_status, QV_AMBER, 1);
        gfx_text(16, 80, dots, QV_TEXT, 2);

        char found[40];
        snprintf(found, sizeof(found), "Modulos encontrados: %d",
                 g_scan_count);
        gfx_text(16, 110, found, QV_TEXT, 1);

        gfx_text(16, 140, "CIRCLE: cancelar el escaneo", QV_DIM, 1);
        gfx_text(16, 164, "No apagues el contacto.", QV_DIM, 1);
        return;
    }

    if (g_scan_count == 0) {
        gfx_text(16, 64, "No responde ningun modulo.", QV_CRIT, 1);
        gfx_text(16, 84, "Comprueba el contacto y la conexion.",
                 QV_DIM, 1);
        gfx_text(16, 104, g_scan_status, QV_DIM, 1);
        gfx_text(16, 238, "X: REESCANEAR   L/R: PANTALLA",
                 QV_DIM, 1);
        return;
    }

    char head[64];
    snprintf(head, sizeof(head), "PROTOCOLO %s   MODULOS %d",
             g_scan_proto[0] ? g_scan_proto : "?", g_scan_count);
    gfx_text(16, 34, head, QV_DIM, 1);

    const int rows = 8;
    int first = g_scan_sel - rows / 2;
    if (first < 0) {
        first = 0;
    }
    if (first > g_scan_count - rows) {
        first = g_scan_count - rows;
    }
    if (first < 0) {
        first = 0;
    }

    for (int i = 0; i < rows && first + i < g_scan_count; i++) {
        int idx = first + i;
        scan_module *m = &g_scan[idx];
        float y = 56.0f + (float)i * 20.0f;
        int selected = (idx == g_scan_sel);

        if (selected) {
            gfx_rect(12.0f, y - 2.0f, GFX_W - 24.0f, 18.0f, QV_PANEL);
            gfx_rect(12.0f, y - 2.0f, 3.0f, 18.0f, QV_RED);
        }

        const char *type = "OBD";
        uint32_t type_color = QV_GREEN;
        if (m->obd && m->uds) {
            type = "OBD+UDS";
            type_color = QV_AMBER;
        } else if (m->uds) {
            type = "UDS";
            type_color = QV_CYAN;
        }
        gfx_text(20, (int)y, type, type_color, 1);
        gfx_text(88, (int)y, m->header, QV_TEXT, 1);

        char info[64];
        if (m->name[0] != '\0') {
            snprintf(info, sizeof(info), "%s", m->name);
        } else if (m->part[0] != '\0') {
            snprintf(info, sizeof(info), "%s", m->part);
        } else if (m->mask1 != 0) {
            snprintf(info, sizeof(info), "PIDs %08X",
                     (unsigned)m->mask1);
        } else if (m->req[0] != '\0') {
            snprintf(info, sizeof(info), "REQ %s", m->req);
        } else {
            snprintf(info, sizeof(info), "-");
        }
        gfx_text(160, (int)y, info, selected ? QV_TEXT : QV_DIM, 1);

        if (m->mask1 != 0) {
            char mask[12];
            snprintf(mask, sizeof(mask), "%08X", (unsigned)m->mask1);
            gfx_text_right(GFX_W - 20, (int)y, mask, QV_DIM, 1);
        }
    }

    /* Detalle del modulo seleccionado. */
    scan_module *sel = &g_scan[g_scan_sel];
    gfx_rect(0.0f, 220.0f, GFX_W, 1.0f, QV_TRACK);

    char detail[96];
    if (sel->part[0] != '\0' || sel->sw[0] != '\0') {
        snprintf(detail, sizeof(detail), "PIEZA %.18s   SW %.10s",
                 sel->part[0] ? sel->part : "-",
                 sel->sw[0] ? sel->sw : "-");
        gfx_text(8, 226, detail, QV_DIM, 1);
    }
    if (sel->vin[0] != '\0') {
        snprintf(detail, sizeof(detail), "VIN %s", sel->vin);
        gfx_text(8, 238, detail, QV_DIM, 1);
    }
    if (sel->serial[0] != '\0') {
        snprintf(detail, sizeof(detail), "SERIE %s", sel->serial);
        gfx_text(8, 250, detail, QV_DIM, 1);
    }

    gfx_text(8, 262, "X: REESCANEAR   ARRIBA/ABAJO: ELEGIR   L/R: PANTALLA",
             QV_DIM, 1);
}

static void draw_sniffer(void)
{
    gfx_text(16, 34, "SNIFFER CAN", QV_TEXT, 1);
    gfx_rect(16.0f, 48.0f, GFX_W - 32.0f, 1.0f, QV_TRACK);

    if (g_snif_running) {
        gfx_text(16, 62, "CAPTURANDO... CIRCLE o X para parar",
                 QV_AMBER, 1);
    } else if (g_snif_total == 0) {
        gfx_text(16, 62, "Pulsa X para iniciar la captura.", QV_AMBER, 1);
        gfx_text(16, 82, "Captura todo el trafico CAN del bus.", QV_DIM, 1);
        gfx_text(16, 98, "Util para interceptar AlfaOBD/MES.", QV_DIM, 1);
        gfx_text(16, 114, "Todo se guarda en el log de la sesion.", QV_DIM, 1);
    } else {
        char head[64];
        snprintf(head, sizeof(head), "CAPTURADAS %d TRAMAS",
                 g_snif_total);
        gfx_text(16, 62, head, QV_GREEN, 1);
        gfx_text(16, 78, "X: NUEVA CAPTURA   ARRIBA/ABAJO: SCROLL",
                 QV_DIM, 1);
    }

    const int rows = 12;
    int total = g_snif_total;
    int start = total - rows - g_snif_view;
    if (start < 0) {
        start = 0;
    }
    if (start > total - rows && total > rows) {
        start = total - rows;
    }
    if (start < 0) {
        start = 0;
    }

    for (int i = 0; i < rows && start + i < total; i++) {
        int idx = start + i;
        if (idx < 0 || idx >= SNIF_LINES) {
            continue;
        }
        float y = 100.0f + (float)i * 13.0f;
        gfx_text(8, (int)y, g_snif_buf[idx], QV_TEXT, 1);
    }

    gfx_text(8, 262, "X: INICIAR/PARAR   ARRIBA/ABAJO: SCROLL   L/R: PANTALLA",
             QV_DIM, 1);
}

/* ------------------------------------------------------------------ */
/* Efectos de tema derivados de la velocidad.                         */
/* ------------------------------------------------------------------ */

static void theme_effects_update(uint32_t now)
{
    const obd_sample *sv = obd_store_get(&g_store, 0x0D);

    if (g_net_busy || sv == NULL || !sv->valid ||
        (uint32_t)(now - sv->ts_ms) > 6000u) {
        g_fx_accel *= 0.85f;
        if (g_fx_nitro > 0.02f) g_fx_nitro -= 0.04f;
        else g_fx_nitro = 0.0f;
        if (g_fx_skill_until != 0 && now > g_fx_skill_until) {
            g_fx_skill = 0.0f;
            g_fx_skill_until = 0;
        }
        if (g_chime_on) {
            g_chime_on = 0;
            audio_chime(0);
        }
        g_fx_speed_ts = 0;
        return;
    }

    float spd = sv->value;

    /* Aviso de velocidad INITIAL D: se evalua siempre, aunque el PID
     * no se haya refrescado en este frame. Se silencia con el menu
     * abierto para que este se pueda leer y navegar sin tirones. */
    int menu_open = (g_settings_open || g_edit_open || g_tel_edit_open);
    int want_chime = (!menu_open && g_cfg.theme == 8 &&
                      g_cfg.chime_kmh > 0 &&
                      spd >= (float)g_cfg.chime_kmh);
    if (want_chime != g_chime_on) {
        g_chime_on = want_chime;
        audio_chime(want_chime);
        log_file_printf("AVISO velocidad %s a %.0f km/h\n",
                        want_chime ? "ON" : "OFF", spd);
    }

    if (sv->ts_ms == g_fx_speed_ts) {
        return;
    }

    float dt = (g_fx_speed_ts == 0) ? 0.5f
              : (float)(sv->ts_ms - g_fx_speed_ts) / 1000.0f;
    if (dt < 0.08f) dt = 0.08f;
    if (dt > 1.5f) dt = 1.5f;

    float acc = (spd - g_fx_speed_prev) / dt;
    if (acc > 40.0f) acc = 40.0f;
    if (acc < -40.0f) acc = -40.0f;
    g_fx_accel = g_fx_accel * 0.55f + acc * 0.45f;
    g_fx_speed_prev = spd;
    g_fx_speed_ts = sv->ts_ms;

    float thr = 0.0f;
    int thr_ok = 0;
    if (sample_value(0x11, now, &thr)) {
        thr_ok = 1;
    } else if (sample_value(0x49, now, &thr)) {
        thr_ok = 1;
    } else if (g_thr_pid != 0x11 && sample_value(g_thr_pid, now, &thr)) {
        thr_ok = 1;
    }

    if (g_cfg.theme == 6) {
        /* NFS: el nitro sube al acelerar fuerte y baja al soltar. */
        int hard = (g_fx_accel > 4.0f) && (!thr_ok || thr >= 55.0f);
        if (hard) {
            g_fx_nitro += dt * 1.6f;
        } else {
            g_fx_nitro -= dt * 0.9f;
        }
        if (g_fx_nitro < 0.0f) g_fx_nitro = 0.0f;
        if (g_fx_nitro > 1.0f) g_fx_nitro = 1.0f;
    } else {
        if (g_fx_nitro > 0.02f) g_fx_nitro -= dt * 1.5f;
        else g_fx_nitro = 0.0f;
    }

    if (g_cfg.theme == 5) {
        /* Forza: puntos de skill mientras hay aceleracion fuerte. */
        if (g_fx_accel > 2.5f && spd > 15.0f) {
            g_fx_skill += dt * 42.0f;
            g_fx_skill_until = now + 1500u;
        } else if (g_fx_skill_until != 0 && now > g_fx_skill_until) {
            g_fx_skill = 0.0f;
            g_fx_skill_until = 0;
        }
    } else {
        g_fx_skill = 0.0f;
        g_fx_skill_until = 0;
    }
}

static void draw_theme_fx(const dash_data *d, uint32_t now)
{
    if (g_mode != MODE_QV || g_settings_open) {
        return;
    }

    if (g_cfg.theme == 6 && g_fx_nitro > 0.03f) {
        float n = g_fx_nitro;

        /* Tinte y estelas laterales tipo nitro. */
        gfx_rect(0, 0, (float)GFX_W, (float)GFX_H,
                 GFX_RGBA(0, 190, 255, (int)(6.0f + n * 16.0f)));

        for (int i = 0; i < 12; i++) {
            int y = 30 + (int)((i * 47 + (int)(now / 3)) % 214);
            float len = 12.0f + n * 40.0f + (float)(i % 4) * 7.0f;
            uint32_t c = gfx_shade(QV_CYAN,
                                   35 + (int)(n * 45.0f));
            gfx_rect(0.0f, (float)y, len, 2.0f, c);
            gfx_rect((float)GFX_W - len, (float)((y + 40) % 246),
                     len, 2.0f, c);
        }

        gfx_text(398, 40, "NITRO", QV_CYAN, 1);
        gfx_rect(398.0f, 50.0f, 68.0f, 5.0f, QV_TRACK);
        gfx_rect(398.0f, 50.0f, 68.0f * n, 5.0f, QV_GREEN);
        gfx_border(0, 0, (float)GFX_W, (float)GFX_H, 2.0f,
                   gfx_shade(QV_CYAN, 50 + (int)(n * 40.0f)));
    }

    if (g_cfg.theme == 5 && g_fx_skill >= 3.0f &&
        now < g_fx_skill_until) {
        char b[24];
        snprintf(b, sizeof(b), "SKILL +%d", (int)g_fx_skill);
        gfx_rect(300.0f, 34.0f, 172.0f, 26.0f, QV_PANEL2);
        gfx_border(300.0f, 34.0f, 172.0f, 26.0f, 2.0f,
                   theme_accent());
        gfx_text(312, 43, b, QV_TEXT, 2);
    }

    if (g_cfg.theme == 7) {
        /* GT: medidor de G derivado de la velocidad. */
        float g = fabsf(g_fx_accel) / 35.3f;
        if (g > 2.0f) g = 2.0f;
        char b[16];
        snprintf(b, sizeof(b), "G %.2f", g);
        gfx_text(402, 44, b, QV_CYAN, 1);

        float gw = 48.0f * (g / 2.0f);
        gfx_rect(402.0f, 54.0f, 48.0f, 3.0f, QV_TRACK);
        if (gw > 0.0f) {
            gfx_rect(402.0f, 54.0f, gw, 3.0f, QV_CYAN);
        }

        /* Con perfil, el aviso salta al acercarse al corte real; sin el,
         * 150 rpm antes de la zona roja como siempre. */
        float warn_rpm = g_profile_cut > 0 ? (float)g_profile_cut - 300.0f
                                          : redline_rpm() - 150.0f;
        if (d->rpm_valid && d->rpm_val >= warn_rpm && ((now / 140u) & 1u)) {
            gfx_border(0, 0, (float)GFX_W, (float)GFX_H, 2.0f, QV_RED);
        }
    }

    if (g_cfg.theme == 8 && g_chime_on && ((now / 260u) & 1u)) {
        gfx_text(400, 44, "AVISO", QV_RED, 1);
    }
}

/* Ayuda en pantalla: descripcion breve de cada parametro que se puede
 * pintar. Solo texto informativo; no altera ningun dato ni PID. */
typedef struct {
    int item;
    const char *name;
    const char *desc;
} info_entry;

#define INFO_VISIBLE 9

static const info_entry g_info[] = {
    { W_RPM, "RPM",
      "Revoluciones del motor (PID 0x0C)." },
    { W_SPEED, "VELOCIDAD",
      "Velocidad del vehiculo (PID 0x0D)." },
    { W_BOOST, "BOOST",
      "Presion de soplado: MAP menos barometrica." },
    { W_ECT, "ECT",
      "Temperatura del refrigerante del motor (0x05)." },
    { W_IAT, "IAT",
      "Temperatura del aire de admision (0x0F)." },
    { W_MAP, "MAP",
      "Presion absoluta del colector de admision (0x0B)." },
    { W_MAF, "MAF",
      "Flujo de aire medido por el caudalimetro (0x10)." },
    { W_THR, "ACELERADOR",
      "Posicion del pedal (0x49, 0x4A o 0x5A)." },
    { W_LOAD, "CARGA",
      "Carga calculada del motor en % (0x04)." },
    { W_PLATE, "MARIPOSA",
      "Posicion real de la mariposa (0x11)." },
    { W_VOLT, "VOLT",
      "Tension de alimentacion del modulo (0x42)." },
    { W_OIL, "ACEITE",
      "Temperatura del aceite del motor (0x5C)." },
    { W_FUEL, "COMBUSTIBLE",
      "Nivel de combustible en % (0x2F)." },
    { W_STFT, "STFT",
      "Correccion corta de mezcla, banco 1 (0x06)." },
    { W_LTFT, "LTFT",
      "Correccion larga de mezcla, banco 1 (0x07)." },
    { W_TIMING, "AVANCE",
      "Avance de encendido respecto al PMS (0x0E)." },
    { W_O2_1, "O2-1",
      "Tension de la sonda lambda delantera (0x14)." },
    { W_O2_2, "O2-2",
      "Tension de la sonda lambda trasera (0x15)." },
    { W_DIST_MIL, "MIL",
      "Distancia recorrida con el testigo de averia (0x21)." },
    { W_BARO, "BARO",
      "Presion barometrica ambiente (0x33/captura al arrancar)." },
    { W_OFF, "N/D",
      "Dato no disponible en esta ECU o sin respuesta." },
    { W_OFF, "NO SOP.",
      "PID no soportado por la centralita del coche." },
    { W_OFF, "EST / SIM",
      "Valor estimado (EST) o simulado en modo demo (SIM)." },
};

static void draw_info(void)
{
    int count = (int)(sizeof(g_info) / sizeof(g_info[0]));
    int max_scroll = count - INFO_VISIBLE;
    if (max_scroll < 0) {
        max_scroll = 0;
    }
    if (g_info_scroll > max_scroll) {
        g_info_scroll = max_scroll;
    }
    if (g_info_scroll < 0) {
        g_info_scroll = 0;
    }

    gfx_text(8, 32, "INFORMACION DE PARAMETROS", QV_TEXT, 1);
    if (count > INFO_VISIBLE) {
        gfx_text_right(GFX_W - 8, 32, "ARRIBA/ABAJO: DESPLAZAR", QV_DIM, 1);
    }

    gfx_rect(8.0f, 46.0f, GFX_W - 16.0f, 1.0f, QV_TRACK);

    for (int i = 0; i < INFO_VISIBLE; i++) {
        int idx = g_info_scroll + i;
        if (idx >= count) {
            break;
        }
        float y = 54.0f + (float)i * 24.0f;
        uint32_t name_color = (g_info[idx].item == W_OFF)
                              ? QV_DIM : tel_color(g_info[idx].item);
        gfx_text(10, (int)y, g_info[idx].name, name_color, 1);
        gfx_text(10, (int)y + 11, g_info[idx].desc, QV_DIM, 1);
    }

    if (max_scroll > 0) {
        float tx = GFX_W - 8.0f;
        float ty = 50.0f;
        float th = 208.0f;
        gfx_rect(tx, ty, 2.0f, th, QV_TRACK);
        float bh = th * (float)INFO_VISIBLE / (float)count;
        float by = ty + (th - bh) * (float)g_info_scroll / (float)max_scroll;
        gfx_rect(tx, by, 2.0f, bh, QV_DIM);
    }
}

/* ------------------------------------------------------------------ */
/* Rendimiento (FASE B2 + F): FPS reales por VBlank y desglose de      */
/* tiempos. Todo es solo lectura; el coste es nulo si DEBUG esta off.  */
/* ------------------------------------------------------------------ */

static uint32_t now_us(void)
{
    return (uint32_t)sceKernelGetSystemTimeWide();
}

static uint32_t g_perf_cpu_us = 0;       /* CPU de preparar el frame */
static uint32_t g_perf_wait_us = 0;      /* espera de VBlank */
static uint32_t g_perf_interval_us = 0;  /* intervalo real entre frames */
static int g_perf_vblanks = 0;           /* vblanks objetivo por frame */
static int g_perf_verts = 0;             /* vertices del ultimo frame */
static int g_perf_widgets = 0;           /* huecos visibles en la vista */
static uint32_t g_perf_last_vcount = 0;  /* vcount al presentar el frame */
static uint32_t g_perf_window_vcount = 0;
static int g_perf_window_frames = 0;

/* Objetivo de frames por VBlank. El dibujo va sincronizado al VBlank,
 * asi que los valores utiles son 60/30/20/15 fps (1/2/3/4 vblanks).
 * Se adapta al tiempo de CPU medido (sin contar la espera de VBlank). */
static int perf_frame_target_vblanks(void)
{
    if (g_settings_open || g_edit_open || g_tel_edit_open) {
        return 2;
    }
    if (g_perf_cpu_us > 20000u) {
        return 3;   /* 20 fps: la CPU va justa */
    }
    if (g_perf_cpu_us > 9000u) {
        return 2;   /* 30 fps */
    }
    /* Con el bus OBD activo se mantiene 30 fps para no robar tiempo al
     * polling de PIDs; en demo (sin polling) se permite 60 fps. */
    if (g_demo || !g_link_connected) {
        return 1;
    }
    return 2;
}

static int dash_widget_count(void)
{
    int n = 0;
    for (int i = 0; i < DASH_SLOTS; i++) {
        if (slot_item_eff(i) != W_OFF && slot_style_eff(i) != STYLE_OFF) {
            n++;
        }
    }
    return n;
}

static int tel_series_count(void)
{
    int n = 0;
    for (int i = 0; i < TEL_SLOTS; i++) {
        if (g_tel_item[i] != W_OFF) {
            n++;
        }
    }
    return n;
}

static void perf_frame_done(uint32_t cpu_us)
{
    uint32_t vc = (uint32_t)sceDisplayGetVcount();

    g_perf_verts = gfx_vertex_count();
    g_perf_wait_us = gfx_last_wait_us();
    g_perf_vblanks = perf_frame_target_vblanks();

    /* Media movil de CPU para que el objetivo no oscile con un frame
     * puntual. */
    if (g_perf_cpu_us == 0) {
        g_perf_cpu_us = cpu_us;
    } else {
        g_perf_cpu_us = (g_perf_cpu_us * 3u + cpu_us) / 4u;
    }

    if (g_perf_last_vcount != 0) {
        uint32_t dv = vc - g_perf_last_vcount;
        g_perf_interval_us = (uint32_t)(((uint64_t)dv * 1000000u) / 60u);
    }
    g_perf_last_vcount = vc;

    /* FPS reales: frames contados en una ventana de 60 vblanks (=1 s). */
    g_perf_window_frames++;
    if (g_perf_window_vcount == 0) {
        g_perf_window_vcount = vc;
    }
    uint32_t wv = vc - g_perf_window_vcount;
    if (wv >= 60u) {
        int f = (int)(((uint32_t)g_perf_window_frames * 60u + wv / 2u) / wv);
        if (f > 99) {
            f = 99;
        }
        g_fps = f;
        g_perf_window_frames = 0;
        g_perf_window_vcount = vc;
    }
}

/* Panel de diagnostico: se pinta solo con DEBUG activo. */
static void draw_debug_overlay(void)
{
    if (!g_cfg.debug) {
        return;
    }

    char l1[48];
    char l2[48];
    char l3[48];

    unsigned int freekb = (unsigned int)(sceKernelTotalFreeMemSize() / 1024u);
    unsigned int cpu = g_perf_cpu_us;
    unsigned int wait = g_perf_wait_us;
    unsigned int inter = g_perf_interval_us;

    snprintf(l1, sizeof(l1), "FPS %d  INT %u.%ums  VB %d",
             g_fps, inter / 1000u, (inter / 100u) % 10u, g_perf_vblanks);
    snprintf(l2, sizeof(l2), "CPU %u.%ums  WAIT %u.%ums  V %d",
             cpu / 1000u, (cpu / 100u) % 10u,
             wait / 1000u, (wait / 100u) % 10u, g_perf_verts);
    snprintf(l3, sizeof(l3), "RAM %uKB  WID %d  TEL %d",
             freekb, g_perf_widgets, tel_series_count());

    const float x = 6.0f;
    const float w = 348.0f;
    const float h = 40.0f;
    const float y = (float)GFX_H - h - 4.0f;

    gfx_rect(x, y, w, h, QV_PANEL);
    gfx_border(x, y, w, h, 1.0f, QV_TRACK);
    gfx_text((int)x + 6, (int)y + 3, l1, QV_TEXT, 1);
    gfx_text((int)x + 6, (int)y + 14, l2, QV_TEXT, 1);
    gfx_text((int)x + 6, (int)y + 25, l3, QV_DIM, 1);
}

static void draw_screen(uint32_t now, int requests, int data_count,
                        int no_data_count, int timeouts, int last_rtt)
{
    (void)requests;
    (void)data_count;
    (void)no_data_count;
    (void)timeouts;
    (void)last_rtt;

    uint32_t cpu_t0 = now_us();

    /* Mientras se negocia la conexion no se guarda historial: evita
     * dibujar graficas con muestras antiguas de la sesion anterior. */
    int menu_only = (g_settings_open || g_edit_open || g_tel_edit_open);

    if (!g_net_busy && !menu_only) {
        history_update(now);
    }

    dash_data d;
    memset(&d, 0, sizeof(d));
    snprintf(d.rpm, sizeof(d.rpm), "N/D");
    snprintf(d.speed, sizeof(d.speed), "N/D");
    snprintf(d.boost, sizeof(d.boost), "N/D");
    snprintf(d.ect, sizeof(d.ect), "N/D");
    snprintf(d.iat, sizeof(d.iat), "N/D");
    snprintf(d.map_kpa, sizeof(d.map_kpa), "N/D");
    snprintf(d.map_bar, sizeof(d.map_bar), "N/D");
    snprintf(d.maf, sizeof(d.maf), "N/D");
    snprintf(d.thr, sizeof(d.thr), "N/D");
    snprintf(d.plate, sizeof(d.plate), "N/D");
    snprintf(d.load, sizeof(d.load), "N/D");
    snprintf(d.volt, sizeof(d.volt), "N/D");
    snprintf(d.oil, sizeof(d.oil), "N/D");
    snprintf(d.fuel, sizeof(d.fuel), "N/D");
    snprintf(d.stft, sizeof(d.stft), "N/D");
    snprintf(d.ltft, sizeof(d.ltft), "N/D");
    snprintf(d.timing, sizeof(d.timing), "N/D");
    snprintf(d.o2_1, sizeof(d.o2_1), "N/D");
    snprintf(d.o2_2, sizeof(d.o2_2), "N/D");
    snprintf(d.dist_mil, sizeof(d.dist_mil), "N/D");
    snprintf(d.baro_hpa, sizeof(d.baro_hpa), "N/D");
    d.baro_src = "EST";

    float value;

    if (sample_value(0x0C, now, &value)) {
        fmt_0(d.rpm, sizeof(d.rpm), value);
        d.rpm_valid = 1;
        d.rpm_val = value;
    }
    if (sample_value(0x0B, now, &value)) {
        if (units_imperial()) {
            fmt_1(d.map_kpa, sizeof(d.map_kpa), value * 0.145038f);
        } else {
            fmt_0(d.map_kpa, sizeof(d.map_kpa), value);
        }
        fmt_2(d.map_bar, sizeof(d.map_bar), units_map_from_kpa(value));
        d.map_valid = 1;
        d.map_val = value;
    }
    if (sample_value(0x10, now, &value)) {
        fmt_2(d.maf, sizeof(d.maf), units_maf_from_gs(value));
        d.maf_valid = 1;
        d.maf_val = value;
    }
    if (sample_value(0x05, now, &value)) {
        fmt_0(d.ect, sizeof(d.ect), units_temp_from_c(value));
        d.ect_valid = 1;
        d.ect_val = value;
    }
    if (sample_value(0x0F, now, &value)) {
        fmt_0(d.iat, sizeof(d.iat), units_temp_from_c(value));
        d.iat_valid = 1;
        d.iat_val = value;
    }
    if (sample_value(0x0D, now, &value)) {
        fmt_0(d.speed, sizeof(d.speed), units_speed_from_kmh(value));
        d.speed_valid = 1;
        d.speed_val = units_speed_from_kmh(value);
    }
    if (sample_value(g_thr_pid, now, &value)) {
        fmt_0(d.thr, sizeof(d.thr), value);
        d.thr_valid = 1;
        d.thr_val = value;
    }
    if (sample_value(0x04, now, &value)) {
        fmt_0(d.load, sizeof(d.load), value);
        d.load_valid = 1;
        d.load_val = value;
    }
    /* 0x11 es siempre la mariposa; el pedal (0x49/0x4A/0x5A) va en
     * d.thr y puede no estar soportado. */
    if (sample_value(0x11, now, &value)) {
        fmt_0(d.plate, sizeof(d.plate), value);
        d.plate_valid = 1;
        d.plate_val = value;
    }
    if (sample_value(0x42, now, &value)) {
        fmt_1(d.volt, sizeof(d.volt), value);
        d.volt_valid = 1;
        d.volt_val = value;
    }
    if (sample_value(0x5C, now, &value)) {
        fmt_0(d.oil, sizeof(d.oil), units_temp_from_c(value));
        d.oil_valid = 1;
        d.oil_val = value;
    }
    if (sample_value(0x2F, now, &value)) {
        fmt_0(d.fuel, sizeof(d.fuel), value);
        d.fuel_valid = 1;
        d.fuel_val = value;
    }

    if (sample_value(0x06, now, &value)) {
        fmt_1(d.stft, sizeof(d.stft), value);
        d.stft_valid = 1;
        d.stft_val = value;
    }
    if (sample_value(0x07, now, &value)) {
        fmt_1(d.ltft, sizeof(d.ltft), value);
        d.ltft_valid = 1;
        d.ltft_val = value;
    }
    if (sample_value(0x0E, now, &value)) {
        fmt_1(d.timing, sizeof(d.timing), value);
        d.timing_valid = 1;
        d.timing_val = value;
    }
    if (sample_value(0x14, now, &value)) {
        fmt_2(d.o2_1, sizeof(d.o2_1), value);
        d.o2_1_valid = 1;
        d.o2_1_val = value;
    }
    if (sample_value(0x15, now, &value)) {
        fmt_2(d.o2_2, sizeof(d.o2_2), value);
        d.o2_2_valid = 1;
        d.o2_2_val = value;
    }
    if (sample_value(0x21, now, &value)) {
        fmt_0(d.dist_mil, sizeof(d.dist_mil), value);
        d.dist_mil_valid = 1;
        d.dist_mil_val = value;
    }

    if (g_boost.boost_valid) {
        char tmp[16];
        fmt_signed_2(tmp, sizeof(tmp),
                     units_press_from_bar(g_boost.boost_bar));
        snprintf(d.boost, sizeof(d.boost), "%s", tmp);
        d.boost_valid = 1;
        d.boost_est = g_boost.boost_is_estimate;
        d.boost_val = g_boost.boost_bar;
    }

    if (g_boost.baro_valid) {
        fmt_0(d.baro_hpa, sizeof(d.baro_hpa), g_boost.baro_kpa * 10.0f);
        d.baro_valid = 1;
        d.baro_kpa_val = g_boost.baro_kpa;
    }
    if (g_boost.baro_source == BARO_PID) {
        d.baro_src = "PID33";
    } else if (g_boost.baro_source == BARO_KEY_ON) {
        d.baro_src = "KEY";
    } else if (g_boost.baro_source == BARO_CONFIG) {
        d.baro_src = "EST";
    }

    int stale = data_is_stale(now);
    uint32_t age_ms = data_age_ms(now);

    /* FASE F2: si el dato envejece, el contenido se apaga (el marco se
     * queda). El panel deja de parecer congelado y empieza a explicar
     * que lo que hay dentro ya no es de ahora. */
    g_gfx_dim = (age_ms >= DATA_OLD_MS) ? 55 : 100;

    theme_effects_update(now);

    /* Menu abierto: se pinta solo el menu, sin dashboard ni calculos.
     * El menu nunca se atenua: es una pantalla de control, no datos. */
    if (menu_only) {
        g_gfx_dim = 100;
        gfx_frame_begin();
        if (g_settings_open && !g_edit_open) {
            draw_settings_overlay();
        }
        if (g_edit_open) {
            draw_edit_overlay();
        }
        if (g_tel_edit_open) {
            draw_tel_edit_overlay();
        }
        gfx_frame_end();
        perf_frame_done(now_us() - cpu_t0 - gfx_last_wait_us());
        return;
    }

    if (!g_net_busy) {
        history_update(now);
    }

    if (g_cfg.peak_hold) {
        if (d.rpm_valid && d.rpm_val > g_peak_rpm) {
            g_peak_rpm = d.rpm_val;
        }
        if (d.boost_valid && d.boost_val > g_peak_boost) {
            g_peak_boost = d.boost_val;
        }
        float sp;
        if (sample_value(0x0D, now, &sp) && sp > g_peak_speed) {
            g_peak_speed = sp;
        }
    }

    gfx_frame_begin();

    draw_top_bar(g_link_connected, stale, age_ms);

    if (g_mode == MODE_QV) {
        draw_qv(&d);
    } else if (g_mode == MODE_ENGINE) {
        draw_engine(&d);
    } else if (g_mode == MODE_MOTOR) {
        draw_motor(&d);
    } else if (g_mode == MODE_TELEMETRY) {
        draw_telemetry();
    } else if (g_mode == MODE_MODULES) {
        draw_modules();
    } else if (g_mode == MODE_SNIFFER) {
        draw_sniffer();
    } else if (g_mode == MODE_PROBE) {
        draw_probe();
    } else if (g_mode == MODE_FALLOS) {
        draw_fallos();
    } else {
        draw_info();
    }

    if (g_mode == MODE_QV) {
        draw_theme_fx(&d, now);
    }
    draw_net_banner();

    if (g_picker_open) {
        draw_picker();
    }

    if (g_settings_open && !g_edit_open) {
        draw_settings_overlay();
    }
    if (g_edit_open) {
        draw_edit_overlay();
    }
    if (g_tel_edit_open) {
        draw_tel_edit_overlay();
    }

    if (g_cfg.show_fps) {
        char fpsbuf[8];
        snprintf(fpsbuf, sizeof(fpsbuf), "%d", g_fps);
        gfx_text(GFX_W - 28, GFX_H - 12, fpsbuf, QV_DIM, 1);
    }

    g_perf_widgets = dash_widget_count();
    draw_debug_overlay();

    gfx_frame_end();
    perf_frame_done(now_us() - cpu_t0 - gfx_last_wait_us());
}

/* ------------------------------------------------------------------ */
/* Avisos graficos de conexion.                                       */
/* ------------------------------------------------------------------ */

static void net_ui_set(const char *msg)
{
    if (msg != NULL) {
        snprintf(g_net_msg, sizeof(g_net_msg), "%s", msg);
    }
    g_net_spin = 0;
}

/* Redibuja el dashboard con el cartel de conexion. Se llama desde las
 * esperas de red para que el usuario vea progreso sin pantallas negras. */
static void net_ui_tick(void)
{
    if (!g_gfx_active || !g_net_busy) {
        return;
    }

    uint32_t now = now_ms();
    g_net_spin++;
    if (g_net_ui_last != 0 && (uint32_t)(now - g_net_ui_last) < 100u) {
        return;
    }
    g_net_ui_last = now;
    draw_screen(now, 0, 0, 0, 0, -1);
}

/* Cede CPU, refresca la pantalla de conexion y atiende la cancelacion
 * con SELECT. Sustituye a net_ui_tick() dentro del proceso de conexion
 * para que una conexion larga (o una reconexion) sea interrumpible sin
 * congelar la respuesta al usuario (FASE D3). */
static void connect_yield(void)
{
    SceCtrlData pad;
    sceCtrlReadBufferPositive(&pad, 1);
    g_prev_buttons = pad.Buttons;
    if (pad.Buttons & PSP_CTRL_SELECT) {
        g_connect_cancel = 1;
    }
    net_ui_tick();
}

static int handle_mode_input(unsigned int pressed)
{
    int changed = 0;

    if (pressed & (PSP_CTRL_RTRIGGER | PSP_CTRL_RIGHT)) {
        g_mode = mode_step(g_mode, 1);
        changed = 1;
    }
    if (pressed & (PSP_CTRL_LTRIGGER | PSP_CTRL_LEFT)) {
        g_mode = mode_step(g_mode, -1);
        changed = 1;
    }

    /* FASE F3: al cambiar de pantalla se reparte el ancho de banda.
     * Los datos que acaban de aparecer pasan a pedirse de inmediato. */
    if (changed && !g_settings_open) {
        apply_screen_schedule(now_ms());
    }

    if (g_mode == MODE_TELEMETRY && !g_settings_open) {
        if (pressed & PSP_CTRL_CROSS) {
            g_tel_edit_open = 1;
            g_tel_edit_sel = 0;
            changed = 1;
        }
    }

    if (g_mode == MODE_FALLOS && !g_settings_open) {
        if ((pressed & PSP_CTRL_CROSS) && g_dtc_state != 1) {
            g_dtc_request = 1;
            changed = 1;
        }
        /* Limpia la PANTALLA, no la centralita: el modo 04 no se envia
         * nunca. Es solo volver a "sin leer" para forzar una lectura. */
        if (pressed & PSP_CTRL_CIRCLE) {
            g_dtc_state = 0;
            g_dtc_stored_n = 0;
            g_dtc_pending_n = 0;
            g_dtc_permanent_n = 0;
            g_dtc_mil = -1;
            g_dtc_count = -1;
            changed = 1;
        }
    }

    if (g_mode == MODE_MODULES && !g_settings_open) {
        if ((pressed & PSP_CTRL_CROSS) && screen_available(MODE_MODULES)) {
            g_scan_request = 1;
            changed = 1;
        }
        if (pressed & PSP_CTRL_UP) {
            if (g_scan_sel > 0) {
                g_scan_sel--;
            }
            changed = 1;
        }
        if (pressed & PSP_CTRL_DOWN) {
            if (g_scan_sel + 1 < g_scan_count) {
                g_scan_sel++;
            }
            changed = 1;
        }
    }

    if (g_mode == MODE_SNIFFER && !g_settings_open) {
        if (pressed & PSP_CTRL_CROSS) {
            if (!g_snif_running) {
                g_snif_request = 1;
            } else {
                /* Se para dentro de run_sniff por pulsacion. */
            }
            changed = 1;
        }
        if (pressed & PSP_CTRL_UP) {
            if (g_snif_view + 1 < g_snif_total) {
                g_snif_view++;
            }
            changed = 1;
        }
        if (pressed & PSP_CTRL_DOWN) {
            if (g_snif_view > 0) {
                g_snif_view--;
            }
            changed = 1;
        }
    }

    if (g_mode == MODE_PROBE && !g_settings_open) {
        if (pressed & PSP_CTRL_CROSS) {
            g_probe_request = 1;
            changed = 1;
        }
        if (pressed & PSP_CTRL_UP) {
            if (g_probe_scroll > 0) {
                g_probe_scroll--;
            }
            changed = 1;
        }
        if (pressed & PSP_CTRL_DOWN) {
            if (g_probe_scroll + 1 < g_probe_count) {
                g_probe_scroll++;
            }
            changed = 1;
        }
    }

    if (g_mode == MODE_INFO && !g_settings_open) {
        int count = (int)(sizeof(g_info) / sizeof(g_info[0]));
        int max_scroll = count - INFO_VISIBLE;
        if (max_scroll < 0) {
            max_scroll = 0;
        }
        if (pressed & PSP_CTRL_UP) {
            if (g_info_scroll > 0) {
                g_info_scroll--;
            }
            changed = 1;
        }
        if (pressed & PSP_CTRL_DOWN) {
            if (g_info_scroll < max_scroll) {
                g_info_scroll++;
            }
            changed = 1;
        }
    }

    return changed;
}

static int handle_settings_input(unsigned int pressed)
{
    int changed = 0;

    /* START cierra el menu: es lo que dice el pie de la pantalla y lo que
     * hacia antes de que SELECT se quedase el selector de pantallas. Sin
     * esto el mando parece muerto con el menu abierto. */
    if (pressed & (PSP_CTRL_START | PSP_CTRL_SELECT)) {
        g_settings_open = 0;
        settings_save();
        return 1;
    }

    if (pressed & PSP_CTRL_CROSS) {
        if (g_settings_sel == 6) {
            g_edit_open = 1;
            g_edit_sel = 0;
            return 1;
        }
        if (g_settings_sel == SET_EXIT) {
            g_exit_request = 1;
        } else {
            g_settings_open = 0;
            settings_save();
        }
        return 1;
    }

    /* Borra el maximo de turbo aprendido. */
    if (pressed & PSP_CTRL_CIRCLE) {
        g_cfg.boost_peak = 0;
        g_boost_learn_saved = now_ms();
        settings_save();
        log_file_printf("TURBO: maximo aprendido borrado\n");
        log_sync();
        return 1;
    }

    if (pressed & PSP_CTRL_UP) {
        g_settings_sel = (g_settings_sel + SET_COUNT - 1) % SET_COUNT;
        changed = 1;
    }
    if (pressed & PSP_CTRL_DOWN) {
        g_settings_sel = (g_settings_sel + 1) % SET_COUNT;
        changed = 1;
    }

    int dir = 0;
    if (pressed & PSP_CTRL_RIGHT) {
        dir = 1;
    }
    if (pressed & PSP_CTRL_LEFT) {
        dir = -1;
    }

    if (dir != 0) {
        if (g_settings_sel == 0) {
            /* Los temas 1-4 son solo colores del tema NORMAL: se saltan
             * al cambiar de tema. */
            do {
                g_cfg.theme = (g_cfg.theme + dir + GFX_THEME_COUNT) %
                              GFX_THEME_COUNT;
            } while (g_cfg.theme >= 1 && g_cfg.theme <= 4);
        } else if (g_settings_sel == 1) {
            if (g_cfg.theme == 0) {
                g_cfg.color = (g_cfg.color + dir + 5) % 5;
            }
        } else if (g_settings_sel == 2) {
            g_cfg.contrast += dir;
            if (g_cfg.contrast < 0) {
                g_cfg.contrast = 0;
            }
            if (g_cfg.contrast > 2) {
                g_cfg.contrast = 2;
            }
        } else if (g_settings_sel == 3) {
            g_cfg.anim = (g_cfg.anim + 1) % 3;
        } else if (g_settings_sel == 4) {
            g_cfg.fast = !g_cfg.fast;
        } else if (g_settings_sel == 5) {
            /* FASE F1: se recorre la lista de valores del hueco, no se
             * suma 1 ms libre: el umbral del ELM327 se mide en pasos. */
            int idx = 0;
            for (int k = 0; k < 7; k++) {
                if (g_elm_gap_values[k] == g_elm_min_gap_ms) {
                    idx = k;
                    break;
                }
            }
            idx += dir;
            if (idx < 0) idx = 0;
            if (idx > 6) idx = 6;
            g_elm_min_gap_ms = g_elm_gap_values[idx];
            log_file_printf("AJUSTE: hueco ELM = %d ms\n", g_elm_min_gap_ms);
            log_sync();
        } else if (g_settings_sel == 7) {
            g_cfg.peak_hold = !g_cfg.peak_hold;
        } else if (g_settings_sel == 8) {
            g_cfg.shift_rpm += dir * 100;
            if (g_cfg.shift_rpm < 2000) g_cfg.shift_rpm = 2000;
            if (g_cfg.shift_rpm > g_cfg.rpm_max) {
                g_cfg.shift_rpm = g_cfg.rpm_max;
            }
        } else if (g_settings_sel == 9) {
            /* Ajustar el tacometro a mano devuelve el mando al usuario
             * hasta la proxima conexion, cuando se vuelve a detectar. */
            g_cfg.rpm_max = (int)tach_max_rpm() + dir * 500;
            vehicle_release_tacho();
            if (g_cfg.rpm_max < 4000) g_cfg.rpm_max = 4000;
            if (g_cfg.rpm_max > 9000) g_cfg.rpm_max = 9000;
            if (g_cfg.shift_rpm > g_cfg.rpm_max) {
                g_cfg.shift_rpm = g_cfg.rpm_max;
            }
        } else if (g_settings_sel == 10) {
            g_cfg.units = (g_cfg.units + 1) % 2;
        } else if (g_settings_sel == 11) {
            g_cfg.boost_auto = !g_cfg.boost_auto;
        } else if (g_settings_sel == 12) {
            /* Ajustar a mano el fondo de escala pasa a modo manual. */
            if (g_cfg.boost_auto) {
                g_cfg.boost_auto = 0;
                log_file_printf("TURBO: escala manual (auto desactivado)\n");
            }
            g_cfg.boost_max += dir;
            if (g_cfg.boost_max < 5) g_cfg.boost_max = 5;
            if (g_cfg.boost_max > 30) g_cfg.boost_max = 30;
        } else if (g_settings_sel == 13) {
            g_cfg.lang += dir;
            if (g_cfg.lang < 0) g_cfg.lang = 0;
            if (g_cfg.lang > 2) g_cfg.lang = 2;
        } else if (g_settings_sel == 14) {
            g_cfg.show_fps = !g_cfg.show_fps;
        } else if (g_settings_sel == 15) {
            /* -1 = la ultima, y luego las pantallas en el orden del
             * selector. */
            g_start_mode++;
            if (g_start_mode >= MODE_COUNT) {
                g_start_mode = -1;
            }
        } else if (g_settings_sel == 16) {
            g_cfg.debug = !g_cfg.debug;
        }

        settings_apply();
        settings_save();
        changed = 1;
    }

    return changed;
}

static int handle_edit_input(unsigned int pressed)
{
    int changed = 0;

    if (pressed & (PSP_CTRL_START | PSP_CTRL_SELECT | PSP_CTRL_CROSS)) {
        g_edit_open = 0;
        settings_save();
        return 1;
    }

    if (pressed & PSP_CTRL_UP) {
        g_edit_sel = (g_edit_sel + DASH_SLOTS - 1) % DASH_SLOTS;
        changed = 1;
    }
    if (pressed & PSP_CTRL_DOWN) {
        g_edit_sel = (g_edit_sel + 1) % DASH_SLOTS;
        changed = 1;
    }
    if (pressed & PSP_CTRL_RIGHT) {
        g_slot_style[g_edit_sel] =
            (g_slot_style[g_edit_sel] + 1) % STYLE_COUNT;
        changed = 1;
    }
    if (pressed & PSP_CTRL_LEFT) {
        g_slot_style[g_edit_sel] =
            (g_slot_style[g_edit_sel] + STYLE_COUNT - 1) % STYLE_COUNT;
        changed = 1;
    }
    if (pressed & PSP_CTRL_TRIANGLE) {
        g_slot_item[g_edit_sel] = widget_cycle(g_slot_item[g_edit_sel], 1, 0);
        g_layout_user = 1;
        changed = 1;
    }
    if (pressed & PSP_CTRL_SQUARE) {
        g_slot_item[g_edit_sel] = widget_cycle(g_slot_item[g_edit_sel], -1, 0);
        g_layout_user = 1;
        changed = 1;
    }

    if (changed) {
        /* FASE F3: al cambiar un hueco, el PID que acaba de aparecer
         * tiene que pedirse de inmediato; si no, se queda con su
         * periodo lento de fuera-de-pantalla durante segundos. */
        apply_screen_schedule(now_ms());
        settings_save();
    }
    return changed;
}

static int handle_tel_edit_input(unsigned int pressed)
{
    int changed = 0;

    if (pressed & (PSP_CTRL_START | PSP_CTRL_SELECT | PSP_CTRL_CROSS)) {
        g_tel_edit_open = 0;
        settings_save();
        return 1;
    }

    if (pressed & PSP_CTRL_UP) {
        g_tel_edit_sel = (g_tel_edit_sel + TEL_SLOTS - 1) % TEL_SLOTS;
        changed = 1;
    }
    if (pressed & PSP_CTRL_DOWN) {
        g_tel_edit_sel = (g_tel_edit_sel + 1) % TEL_SLOTS;
        changed = 1;
    }
    if (pressed & PSP_CTRL_RIGHT) {
        g_tel_item[g_tel_edit_sel] =
            tel_cycle_step(g_tel_item[g_tel_edit_sel], 1);
        g_layout_user = 1;
        changed = 1;
    }
    if (pressed & PSP_CTRL_LEFT) {
        g_tel_item[g_tel_edit_sel] =
            tel_cycle_step(g_tel_item[g_tel_edit_sel], -1);
        g_layout_user = 1;
        changed = 1;
    }

    if (changed) {
        /* FASE F3: lo mismo para las series de telemetria. */
        apply_screen_schedule(now_ms());
        settings_save();
    }
    return changed;
}

/* START (o SELECT) abre/cierra el menu de ajustes. Con la ventana
 * abierta no se cambia de pantalla: las flechas mueven la seleccion y
 * la opcion SALIR cierra la aplicacion. TRIANGLE y SQUARE quedan
 * reservados para reintentar la conexion WiFi. */
static int handle_ui_input(unsigned int pressed)
{
    if (g_edit_open) {
        return handle_edit_input(pressed);
    }
    if (g_tel_edit_open) {
        return handle_tel_edit_input(pressed);
    }

    /* Con el menu de ajustes abierto el mando es del menu. */
    if (g_settings_open) {
        return handle_settings_input(pressed);
    }

    /* Selector de pantallas: SELECT. El menu de ajustes se queda en
     * START, que es lo que SELECT tambien hacia hasta ahora. */
    if (g_picker_open) {
        if (pressed & (PSP_CTRL_SELECT | PSP_CTRL_CIRCLE)) {
            g_picker_open = 0;
            return 1;
        }
        if (pressed & PSP_CTRL_UP) {
            g_picker_sel = picker_step(g_picker_sel, -1);
            return 1;
        }
        if (pressed & PSP_CTRL_DOWN) {
            g_picker_sel = picker_step(g_picker_sel, 1);
            return 1;
        }
        if (pressed & (PSP_CTRL_CROSS | PSP_CTRL_START)) {
            int m = g_picker_order[g_picker_sel];
            if (screen_offered(m)) {
                g_mode = m;
                g_picker_open = 0;
                apply_screen_schedule(now_ms());
                log_file_printf("Pantalla: %s\n", mode_name(m));
            }
            return 1;
        }
        return 1;
    }

    if (pressed & PSP_CTRL_SELECT) {
        g_picker_open = 1;
        g_picker_sel = picker_index_of(g_mode);
        return 1;
    }

    if (pressed & PSP_CTRL_START) {
        if (!g_settings_open) {
            g_settings_open = 1;
            g_settings_sel = 0;
        } else {
            g_settings_open = 0;
            settings_save();
        }
        return 1;
    }


    return handle_mode_input(pressed);
}

static void draw_splash_frame(unsigned int t)
{
    gfx_frame_begin();

    float prog = (t < 250u) ? 0.0f : (float)(t - 250u) / 750.0f;
    prog = clamp_float(prog, 0.0f, 1.0f);

    if (prog > 0.0f) {
        float cx = 240.0f;
        float cy = 116.0f;
        float dist = 6.0f + 24.0f * prog;
        float r = 6.0f + 20.0f * prog;
        const float k = 0.70710678f;

        float lx[4] = { cx + dist * k, cx - dist * k,
                        cx - dist * k, cx + dist * k };
        float ly[4] = { cy + dist * k, cy + dist * k,
                        cy - dist * k, cy - dist * k };
        uint32_t fill = QV_GREEN;

        for (int i = 0; i < 4; i++) {
            gfx_circle(lx[i], ly[i], r + 2.0f, QV_RED);
            gfx_circle(lx[i], ly[i], r, fill);
        }

        gfx_line(cx, cy + dist, cx, cy + dist + r * 1.4f,
                 4.0f, QV_GREEN);
        gfx_circle(cx, cy, 4.0f, QV_TEXT);
    }

    if (t >= 1000u) {
        gfx_text_center(GFX_W / 2, 190, "QV TELEMETRY", QV_TEXT, 3);
        gfx_text_center(GFX_W / 2, 222, "ALFA MITO QV 2010",
                        QV_DIM, 1);

        float linew = 100.0f + 140.0f * (float)(t - 1000u) / 600.0f;
        linew = clamp_float(linew, 0.0f, 240.0f);
        gfx_rect(240.0f - linew * 0.5f, 214.0f,
                 linew * 0.5f, 2.0f, QV_GREEN);
        gfx_rect(240.0f, 214.0f, linew * 0.5f, 2.0f, QV_RED);
    }

    gfx_frame_end();
}

static void startup_animation(void)
{
    /* Sin animacion se va directo al dashboard: ni fotograma ni espera. */
    if (g_cfg.anim == 0) {
        return;
    }

    unsigned int start = now_ms();
    /* Cada animacion dice cuanto dura; hoy las dos son 1600 ms, pero el
     * bucle no lo da por supuesto. */
    const uint32_t total = (g_cfg.anim == 2) ? splash_mx5_total_ms() : 1600u;
    SceCtrlData pad;

    for (;;) {
        unsigned int t = now_ms() - start;
        if (t >= total) {
            break;
        }

        sceCtrlReadBufferPositive(&pad, 1);
        g_prev_buttons = pad.Buttons;
        if (pad.Buttons & (PSP_CTRL_CROSS | PSP_CTRL_START)) {
            break;
        }

        if (g_cfg.anim == 2) {
            splash_mx5_draw(t);
        } else {
            draw_splash_frame(t);
        }
    }
}

/* Dashboard unico. Arranca siempre, con o sin adaptador, y permite
 * reintentar la conexion sin salir de la aplicacion:
 *   TRIANGLE = reintentar el perfil WiFi guardado
 *   SQUARE   = abrir la configuracion WiFi de la PSP y reconectar
 *   SELECT   = ajustes de tema
 *   START    = abrir/cerrar el menu (opcion SALIR dentro) */
static void run_dashboard(Target *targets, int target_count)
{
    SceCtrlData pad;
    uint32_t start = now_ms();

    g_hist_count = 0;
    g_hist_pos = 0;
    g_hist_last = 0;
    g_session_start_ms = start;
    g_exit_request = 0;

    int sock = connect_network_and_elm(targets, &target_count, 0);

    uint32_t last_status = now_ms();
    int requests = 0;
    int data_count = 0;
    int no_data_count = 0;
    int timeouts = 0;
    int last_rtt = -1;
    int consecutive_timeouts = 0;
    int consecutive_errors = 0;

    /* FASE D3: reconexion no bloqueante. Los contadores de sesion son
     * acumulativos: no se reinician al reconectar, para que el resumen
     * final refleje toda la sesion (antes se enmascaraba como "OK"). */
    int link_losses = 0;
    int reconnects = 0;
    int reconn_active = 0;
    int reconn_tries = 0;
    uint32_t reconn_next = 0;

    if (sock >= 0) {
        g_session_start_ms = now_ms();
        last_status = g_session_start_ms;
    }

    log_file_printf("Dashboard %s\n",
                    sock >= 0 ? "con adaptador" : "sin adaptador (N/D)");
    log_sync();

    /* L+R mantenidos 1,5 s: activa/desactiva el modo demo. */
    uint32_t chord_since = 0;
    int chord_done = 0;

    /* FASE D1: peticion OBD en vuelo (una sola). */
    elm_rx rx;
    int rx_idx = -1;
    uint8_t rx_pid = 0;
    uint32_t rx_t0 = 0;
    /* FASE E1: la peticion se acaba de completar; la siguiente espera
     * es fina (2 ms) para que el comando siguiente salga ya. */
    int rx_just_done = 0;
    /* FASE C: hueco real entre el fin de una respuesta y el envio de la
     * siguiente. Se registra en el log para ver si el ELM327 tolera que
     * el comando salga de inmediato. */
    uint32_t gap_acc = 0;
    uint32_t gap_n = 0;
    memset(&rx, 0, sizeof(rx));

    for (;;) {
        uint32_t now = now_ms();
        uint32_t session_ms = now - g_session_start_ms;

        sceCtrlReadBufferPositive(&pad, 1);
        unsigned int pressed = pad.Buttons & ~g_prev_buttons;
        g_prev_buttons = pad.Buttons;

        unsigned int chord = pad.Buttons &
            (PSP_CTRL_LTRIGGER | PSP_CTRL_RTRIGGER);
        if (chord == (PSP_CTRL_LTRIGGER | PSP_CTRL_RTRIGGER)) {
            if (chord_since == 0) {
                chord_since = now ? now : 1;
            } else if (!chord_done &&
                       (uint32_t)(now - chord_since) >= 1500u) {
                chord_done = 1;
                demo_toggle();
                /* Si hay una peticion en vuelo, se cancela para no dejar
                 * respuestas a medias al entrar/salir del modo demo. */
                if (rx.active) {
                    obd_sched_end(&g_sched, rx_idx, now_ms(), 0);
                    if (sock >= 0) {
                        elm_flush_input(sock);
                    }
                    memset(&rx, 0, sizeof(rx));
                }
                now = now_ms();
                draw_screen(now, requests, data_count,
                            no_data_count, timeouts, last_rtt);
            }
            pressed &= ~(PSP_CTRL_LTRIGGER | PSP_CTRL_RTRIGGER);
        } else {
            chord_since = 0;
            chord_done = 0;
        }

        /* Reintento de conexion; deshabilitado con los ajustes abiertos.
         * El selector de pantallas se cierra antes: si no, taparia el
         * aviso de CONECTANDO y SELECT pasaria a cancelar la conexion. */
        if (!g_settings_open &&
            (pressed & (PSP_CTRL_TRIANGLE | PSP_CTRL_SQUARE))) {
            int with_dialog = (pressed & PSP_CTRL_SQUARE) ? 1 : 0;
            g_picker_open = 0;

            if (g_demo) {
                demo_toggle();
            }
            if (sock >= 0) {
                sceNetInetClose(sock);
                sock = -1;
            }
            g_link_connected = 0;
            memset(&rx, 0, sizeof(rx));
            reconn_active = 0;
            reconn_tries = 0;

            sock = connect_network_and_elm(targets, &target_count,
                                           with_dialog);
            now = now_ms();

            if (sock >= 0) {
                g_session_start_ms = now;
                last_rtt = -1;
                consecutive_timeouts = 0;
                consecutive_errors = 0;
                g_elm_stuck = 0;
                g_hist_count = 0;
                g_hist_pos = 0;
                g_hist_last = 0;
            }

            last_status = now;
            continue;
        }

        if (handle_ui_input(pressed)) {
            uint32_t draw_now = now_ms();
            draw_screen(draw_now, requests, data_count,
                        no_data_count, timeouts, last_rtt);
        }

        if (g_exit_request) {
            g_exit_request = 0;
            log_file_printf("Salida solicitada desde el menu\n");
            /* Guardar al salir sella la pantalla donde se dejo. */
            settings_save();
            break;
        }

        if (g_dtc_request) {
            g_dtc_request = 0;
            if (rx.active) {
                obd_sched_end(&g_sched, rx_idx, now_ms(), 0);
                if (sock >= 0) {
                    elm_flush_input(sock);
                }
                memset(&rx, 0, sizeof(rx));
            }
            g_dtc_state = 0;
            run_fallos(sock);
            now = now_ms();
            last_status = now;
            continue;
        }

        if (g_scan_request) {
            g_scan_request = 0;
            /* El escaneo usa el socket de forma bloqueante: no debe
             * solaparse con una peticion en vuelo. */
            if (rx.active) {
                obd_sched_end(&g_sched, rx_idx, now_ms(), 0);
                if (sock >= 0) {
                    elm_flush_input(sock);
                }
                memset(&rx, 0, sizeof(rx));
            }
            run_scan(sock);
            now = now_ms();
            last_status = now;
            continue;
        }

        if (g_snif_request) {
            g_snif_request = 0;
            if (rx.active) {
                obd_sched_end(&g_sched, rx_idx, now_ms(), 0);
                if (sock >= 0) {
                    elm_flush_input(sock);
                }
                memset(&rx, 0, sizeof(rx));
            }
            run_sniff(sock);
            now = now_ms();
            last_status = now;
            continue;
        }

        if (g_probe_request) {
            g_probe_request = 0;
            /* La sonda usa el socket de forma bloqueante: no debe
             * solaparse con una peticion en vuelo. */
            if (rx.active) {
                obd_sched_end(&g_sched, rx_idx, now_ms(), 0);
                if (sock >= 0) {
                    elm_flush_input(sock);
                }
                memset(&rx, 0, sizeof(rx));
            }
            run_probe(sock);
            now = now_ms();
            last_status = now;
            continue;
        }

        if (g_demo) {
            demo_update(now);
        }
        boost_learn_update(now);

        /* FASE D1 + F1: lectura OBD no bloqueante con hueco minimo. */
        int rx_ready = 0;
        int gap_pending = 0;
        if (sock >= 0 && !g_demo) {
            if (!rx.active) {
                /* FASE F1: si el ELM327 aun no ha tenido su silencio,
                 * no se envia nada: el bucle sigue y la espera de la
                 * cadencia duerme 2 ms en vez de un VBlank. */
                if (g_last_resp_ms != 0u &&
                    (int32_t)(now_ms() - g_last_resp_ms) < g_elm_min_gap_ms) {
                    gap_pending = 1;
                } else {
                    int sent = obd_send_next(sock, &rx, &rx_idx,
                                             &rx_pid, &rx_t0);
                    if (sent > 0 && g_last_resp_ms != 0u) {
                        gap_acc += (uint32_t)(rx_t0 - g_last_resp_ms);
                        gap_n++;
                    } else if (sent < 0) {
                        rx_ready = 1;
                    }
                }
            } else if (elm_rx_step(&rx) != ELM_RX_PENDING) {
                rx_ready = 1;
            }
        }
        if (rx_ready) {
            elm_reply *reply = &rx.reply;
            uint8_t pid = rx_pid;
            uint32_t t1 = now_ms();

            /* FASE F1: se ancla el hueco en el instante en que la
             * respuesta se completo. */
            g_last_resp_ms = t1;

            last_rtt = (int)(t1 - rx_t0);
            requests++;

            int got_data = 0;
            if (reply->data && elm_frame_matches(&reply->frame, 0x01, pid)) {
                store_value(pid, &reply->frame, t1);
                got_data = 1;
                data_count++;
            } else if (reply->no_data) {
                no_data_count++;
            } else if (reply->unable) {
                no_data_count++;
            } else if (!reply->error) {
                timeouts++;
            }

            if (reply->error) {
                consecutive_errors++;
            } else {
                consecutive_errors = 0;
            }

            /* FASE F2: contador de STOPPED seguidos. El ELM327 se puede
             * quedar pegado en ese estado y no sale ni con reintentos. */
            if (reply->stopped) {
                g_elm_stuck++;
            } else {
                g_elm_stuck = 0;
            }

            obd_sched_end(&g_sched, rx_idx, t1,
                          (reply->no_data || reply->unable) ? 0 : 1);

            if (got_data &&
                (pid == 0x0B || pid == 0x0C || pid == 0x0D)) {
                maybe_capture_key_on(t1, t1 - g_session_start_ms);
            }

            int link_bad = 0;
            if (reply->bytes == 0) {
                consecutive_timeouts++;
                if (consecutive_timeouts >= 8) {
                    link_bad = 1;
                }
            } else {
                consecutive_timeouts = 0;
                if (consecutive_errors >= 40) {
                    link_bad = 1;
                }
            }

            /* FASE F2: el adaptador atascado en STOPPED no se
             * recupera solo. Se escala al camino de reconexion (que
             * rehace el handshake y deja el ELM327 como nuevo) en vez
             * de seguir mandando comandos a un adaptador muerto. */
            if (g_elm_stuck >= ELM_STUCK_RECOVER) {
                link_bad = 1;
            }

            if (link_bad) {
                    if (g_elm_stuck >= ELM_STUCK_RECOVER) {
                        log_file_printf("Enlace perdido: adaptador atascado "
                                        "en STOPPED (%d)\n", g_elm_stuck);
                    } else {
                        log_file_printf("Enlace perdido: %s\n",
                                        reply->bytes == 0
                                            ? "8 timeouts"
                                            : "40 errores de bus");
                    }
                    log_sync();
                    sceNetInetClose(sock);
                    sock = -1;
                    g_link_connected = 0;
                    memset(&rx, 0, sizeof(rx));
                    link_losses++;
                    consecutive_timeouts = 0;
                    consecutive_errors = 0;
                    g_elm_stuck = 0;

                    /* D3: en lugar de bloquear 2 s y reconectar en el
                     * sitio (congelaba la GUI y dejaba el socket en un
                     * estado que provocaba "TCP error -3"), se programa
                     * el reintento y el bucle principal lo ejecuta. */
                    reconn_active = 1;
                    reconn_tries = 0;
                    reconn_next = now_ms() + RECONN_DELAY_MS;
                    net_ui_set("REINTENTANDO");
                    g_net_busy = 1;
            }

            memset(&rx, 0, sizeof(rx));

            /* FASE F1: el comando siguiente no se envia aqui. El
             * planificador lo enviara en la siguiente iteracion, tras
             * comprobar que ha pasado el hueco minimo. Si el envio
             * fallase, la ruta normal cuenta el timeout y dispara la
             * reconexion al llegar a 8. */

            /* FASE E1: la peticion acaba de completarse. Se marca para
             * que la espera siguiente sea de 2 ms y no un VBlank entero:
             * asi el siguiente comando sale de inmediato en vez de
             * esperar 16,7 ms al frame siguiente. */
            rx_just_done = 1;
        }

        /* FASE D3: reconexion automatica no bloqueante. La espera entre
         * intentos transcurre en el bucle (la GUI sigue viva); el propio
         * connect_network_and_elm() refresca la pantalla y admite
         * cancelacion con SELECT. */
        if (reconn_active && !g_demo && !g_settings_open) {
            g_net_busy = 1;
            if ((int32_t)(now_ms() - reconn_next) >= 0) {
                reconn_tries++;
                log_file_printf("Auto-reintento %d/%d...\n",
                                reconn_tries, RECONN_MAX_TRIES);
                log_sync();
                /* FASE F2: el cartel dice en que intento estamos. */
                char msg[32];
                snprintf(msg, sizeof(msg), "REINTENTANDO %d/%d",
                         reconn_tries, RECONN_MAX_TRIES);
                net_ui_set(msg);
                sock = connect_network_and_elm(targets, &target_count, 0);
                now = now_ms();
                if (sock >= 0) {
                    reconn_active = 0;
                    reconnects++;
                    consecutive_timeouts = 0;
                    consecutive_errors = 0;
                    g_elm_stuck = 0;
                    last_rtt = -1;
                    g_hist_count = 0;
                    g_hist_pos = 0;
                    g_hist_last = 0;
                    last_status = now;
                    log_file_printf("Auto-reintento OK\n");
                    log_sync();
                } else if (reconn_tries >= RECONN_MAX_TRIES) {
                    reconn_active = 0;
                    net_ui_set("SIN ADAPTADOR");
                    log_file_printf("Auto-reintento agotado\n");
                    log_sync();
                } else {
                    reconn_next = now_ms() + RECONN_DELAY_MS;
                }
            }
        }

        now = now_ms();

        /* Cadencia de dibujo por VBlank (FASE B6): sin sleeps fijos ni
         * jitter. Solo se espera cuando no hay nada pendiente. */
        uint32_t vc = (uint32_t)sceDisplayGetVcount();
        if (g_perf_last_vcount == 0) {
            g_perf_last_vcount = vc;
        }
        int target_vb = perf_frame_target_vblanks();
        /* gfx_frame_end() ya consume 1 VBlank por frame (la espera que
         * evita el tearing), asi que para un periodo de target_vb VBlanks
         * hay que empezar a dibujar cuando han pasado target_vb - 1. */
        int draw_due = ((uint32_t)(vc - g_perf_last_vcount) >=
                        (uint32_t)(target_vb - 1));

        if (draw_due) {
            draw_screen(now, requests, data_count,
                        no_data_count, timeouts, last_rtt);
        } else if (rx.active || rx_just_done || gap_pending) {
            /* Con una peticion en vuelo, o esperando el hueco minimo del
             * adaptador, se duerme 2 ms en vez de un VBlank (16,7 ms).
             * El dibujo no se resiente: su cadencia sigue la cuenta de
             * VBlanks, no la del bucle. */
            sceKernelDelayThread(2000);
        } else {
            /* Nada que dibujar: un VBlank como referencia. */
            sceDisplayWaitVblankStart();
        }
        rx_just_done = 0;

        if ((uint32_t)(now - last_status) >= 2000u) {
            float f;
            char rpm[16] = "N/D";
            char map[16] = "N/D";
            char boost[16] = "N/D";
            char ect[16] = "N/D";
            char iat[16] = "N/D";
            char spd[16] = "N/D";

            if (sample_value(0x0C, now, &f)) fmt_0(rpm, sizeof(rpm), f);
            if (sample_value(0x0B, now, &f)) fmt_0(map, sizeof(map), f);
            if (g_boost.boost_valid) fmt_1(boost, sizeof(boost), g_boost.boost_bar);
            if (sample_value(0x05, now, &f)) fmt_0(ect, sizeof(ect), f);
            if (sample_value(0x0F, now, &f)) fmt_0(iat, sizeof(iat), f);
            if (sample_value(0x0D, now, &f)) fmt_0(spd, sizeof(spd), f);

            log_file_printf("t=%u rpm=%s map=%s boost=%s ect=%s iat=%s "
                            "spd=%s rtt=%d gap=%d gapcfg=%d ok=%d nd=%d "
                            "to=%d fps=%d "
                            "cpu=%u wait=%u int=%u vb=%d v=%d%s\n",
                            (unsigned)(session_ms / 1000), rpm, map,
                            boost, ect, iat, spd, last_rtt,
                            (int)(gap_n ? (gap_acc / gap_n) : 0),
                            g_elm_min_gap_ms,
                            data_count, no_data_count, timeouts,
                            g_fps,
                            (unsigned)(g_perf_cpu_us / 1000u),
                            (unsigned)(g_perf_wait_us / 1000u),
                            (unsigned)(g_perf_interval_us / 1000u),
                            g_perf_vblanks, g_perf_verts,
                            g_demo ? " (SIM)" :
                            (sock >= 0 ? "" : " (offline)"));
            last_status = now;
            /* La linea de estado sale una vez por segundo: escritura a
             * la tarjeta tambien cada segundo. Si la app muere sin
             * cerrar, se pierde como mucho el ultimo segundo de log. */
            log_sync();
            /* Un cambio de pantalla se escribe una sola vez, aqui: asi
             * "arrancar donde lo deje" tambien vale con L/R. */
            if (g_last_saved_mode != g_mode) {
                g_last_saved_mode = g_mode;
                settings_save();
            }

            /* FASE K2: cada ventana de 10 s se compara lo pedido con lo
             * entregado. Solo en K-line: en CAN la tabla ya cabe en el
             * presupuesto y ese coche no puede depender de una medida. */
            if (obd_vehicle_proto_class(g_proto) != OBD_PROTO_CAN) {
                if (last_rtt > 0) {
                    g_link_service_ms = (uint16_t)last_rtt;
                }
                if (g_cap_window_ms == 0) {
                    g_cap_window_ms = now;
                    g_cap_window_reqs = requests;
                } else if ((uint32_t)(now - g_cap_window_ms) >= 10000u) {
                    uint32_t elapsed = (uint32_t)(now - g_cap_window_ms);
                    float rate = (float)(requests - g_cap_window_reqs) *
                                 1000.0f / (float)elapsed;
                    float ceiling = obd_profile_link_class(OBD_PROTO_KLINE);
                    float base = (g_link_measured > 0.0f) ? g_link_measured
                                                          : ceiling;
                    float next = obd_capacity_next(base, rate,
                                                   g_plan_demand, ceiling);
                    if (next != g_link_measured) {
                        g_link_measured = next;
                        log_file_printf(
                            "Enlace: %.2f req/s entregadas, pedidas %.2f; "
                            "plan %.2f req/s (%s)\n",
                            rate, g_plan_demand, g_link_measured,
                            g_link_plan.compressed ? "comprimido" : "holgado");
                        log_sync();
                        apply_screen_schedule(now);
                    }
                    g_cap_window_ms = now;
                    g_cap_window_reqs = requests;
                }
            }

            gap_acc = 0;
            gap_n = 0;
        }
    }

    if (sock >= 0) {
        sceNetInetClose(sock);
    }

    g_summary.requests = requests;
    g_summary.data_count = data_count;
    g_summary.no_data_count = no_data_count;
    g_summary.timeouts = timeouts;
    g_summary.link_losses = link_losses;
    g_summary.reconnects = reconnects;

    log_file_printf("Fin de sesion: peticiones=%d datos=%d nd=%d to=%d\n",
                    requests, data_count, no_data_count, timeouts);
    log_sync();
}

/* ------------------------------------------------------------------ */
/* Principal.                                                         */
/* ------------------------------------------------------------------ */

int main(void)
{
    setup_exit_callback();

    /* El EBOOT puede estar instalado en cualquier carpeta, pero los
     * datos (log, ajustes, host.txt) van siempre a LOG_DIR. Si esa
     * carpeta no existe no se escribia nada y nadie avisaba: se crea. */
    sceIoMkdir("ms0:/PSP/SAVEDATA", 0777);   /* el firmware la crea */
    sceIoMkdir(LOG_DIR, 0777);

    /* Solo archivo: la pantalla grafica es lo primero que se ve. */
    log_open();
    log_file_printf("PSP Torque - fase 2\n");
    log_file_printf("build: %s\n", BUILD_TAG);
    log_file_printf("log: %s\n\n", g_log_path);

    /* Paleta y estado inicial, aun sin GU. */
    settings_load();
    proto_load();
    /* Se arranca donde se quedo, pero solo si era una pantalla de
     * conduccion: abrir siempre en SNIFFER no sirve al conducir. */
    /* start= manda; si no hay, la ultima pantalla de conduccion. */
    if (g_start_mode >= 0 && g_start_mode < MODE_COUNT) {
        g_mode = g_start_mode;
    } else {
        g_mode = mode_is_driving(g_last_mode) ? g_last_mode : MODE_QV;
    }
    /* Ya esta en disco: no hace falta reescribirlo en el primer tick. */
    g_last_saved_mode = g_mode;
    log_file_printf("Pantalla inicial: %s\n", mode_name(g_mode));
    obd_store_init(&g_store);
    boost_init(&g_boost, PROFILE_BARO_FALLBACK_KPA);

    Target targets[MAX_TARGETS];
    int target_count = load_targets(targets, MAX_TARGETS);

    /* GUI inmediata: animacion del trebol y dashboard. */
    g_gfx_active = 1;
    gfx_init();
    int audio_ok = audio_init();
    log_file_printf("gfx_init OK (audio %s)\n", audio_ok ? "OK" : "NO");
    log_sync();
    startup_animation();
    log_file_printf("animacion OK\n");
    log_sync();

    sceCtrlSetSamplingCycle(0);
    sceCtrlSetSamplingMode(PSP_CTRL_MODE_ANALOG);

    /* Los modulos de red se cargan detras de la GUI, con aviso grafico. */
    net_ui_set("INICIANDO RED");
    g_net_busy = 1;
    g_net_ui_last = 0;
    net_ui_tick();

    int ret;
    ret = sceUtilityLoadNetModule(PSP_NET_MODULE_COMMON);
    log_file_printf("net common: 0x%08X\n", ret);
    ret = sceUtilityLoadNetModule(PSP_NET_MODULE_INET);
    log_file_printf("net inet  : 0x%08X\n", ret);
    ret = sceNetInit(0x20000, 0x20, 0x1000, 0x20, 0x1000);
    log_file_printf("sceNetInit: 0x%08X\n", ret);
    ret = sceNetInetInit();
    log_file_printf("inet init : 0x%08X\n", ret);
    ret = sceNetApctlInit(0x8000, 0x30);
    log_file_printf("apctl init: 0x%08X\n", ret);
    ret = sceNetResolverInit();
    log_file_printf("resolver  : 0x%08X\n", ret);
    log_sync();
    g_net_busy = 0;

    run_dashboard(targets, target_count);

    /* Resumen final en texto, ya sin GU activa. */
    log_sync();
    audio_shutdown();
    gfx_shutdown();
    g_gfx_active = 0;

    g_screen_ready = 1;
    pspDebugScreenInit();
    pspDebugScreenSetXY(0, 0);

    int supported = 0;
    for (int i = 0; i < g_sched.count; i++) {
        if (g_sched.items[i].supported != 2) {
            supported++;
        }
    }

    if (g_summary.link_losses > 0) {
        log_printf("Enlace perdido %d vez/veces; reconexiones OK: %d\n\n",
                   g_summary.link_losses, g_summary.reconnects);
    }

    log_printf("PSP Torque - fin de sesion\n");
    log_printf("PIDs activos: %d/%d\n", supported, g_sched.count);
    log_printf("Peticiones: %d  datos: %d  NO DATA: %d  timeouts: %d\n",
               g_summary.requests, g_summary.data_count,
               g_summary.no_data_count, g_summary.timeouts);
    log_printf("Mascara 0100: %08X  0120: %08X\n",
               (unsigned)g_mask1, (unsigned)g_mask2);
    log_printf("BARO: %s\n",
               g_boost.baro_source == BARO_PID ? "PID 0x33" :
               g_boost.baro_source == BARO_KEY_ON ? "captura KEY-ON" :
               "fallback EST");

    char result[48];
    if (g_summary.data_count > 10 && g_summary.link_losses == 0) {
        snprintf(result, sizeof(result), "OK");
    } else if (g_summary.data_count > 10) {
        snprintf(result, sizeof(result), "OK (%d cortes)",
                 g_summary.link_losses);
    } else {
        snprintf(result, sizeof(result), "INCOMPLETO");
    }
    log_printf("RESULTADO: %s\n", result);
    log_printf("Log: %s\n", g_log_path);
    log_printf("\nSTART: salir\n");
    log_sync();

    SceCtrlData pad;
    for (;;) {
        sceCtrlReadBufferPositive(&pad, 1);
        if (pad.Buttons & PSP_CTRL_START) {
            break;
        }
        sceDisplayWaitVblankStart();
    }

    log_close();
    sceKernelExitGame();
    return 0;
}
