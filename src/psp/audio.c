/*
 * audio.c - campaneo de aviso de velocidad (tema INITIAL D).
 *
 * El sonido se prepara UNA sola vez al arrancar:
 *   - Si existe chime.wav junto al EBOOT, se usa ese archivo.
 *   - Si no, se sintetiza el aviso "tin tun" (dos campanas).
 *
 * El hilo de audio solo trabaja mientras hay que sonar: reproduce el
 * buffer una vez, duerme la pausa en trozos cortos y vuelve a esperar.
 * No mantiene el canal ocupado con silencio ni hace sinf/cosf en vuelo.
 */

#include <pspkernel.h>
#include <pspaudio.h>
#include <pspiofilemgr.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "audio.h"

#define AUDIO_RATE     44100
#define AUDIO_BLOCK    1024
#define AUDIO_VOLUME   0x4800

#define CHIME_MAX_SEC  4
#define CHIME_MAX_LEN  (AUDIO_RATE * CHIME_MAX_SEC)

/* Pausa entre repeticiones (ms). Fija: 0,40 s de campaneo + 0,90 s de
 * pausa = un aviso cada 1,30 s. */
#define CHIME_GAP_DEFAULT_MS 900
#define CHIME_GAP_MIN_MS     300
#define CHIME_GAP_MAX_MS     5000

#define SYNTH_SEC      0.40f
#define SYNTH_LEN      (AUDIO_RATE * 4 / 10)

static int g_ch = -1;
static int g_thread = -1;
static volatile int g_run = 0;
static volatile int g_chime_on = 0;
static short g_buf[AUDIO_BLOCK];

static short g_synth[SYNTH_LEN];
static short *g_wav = NULL;
static short *g_play = NULL;
static int g_play_len = 0;
static int g_gap_len = AUDIO_RATE * CHIME_GAP_DEFAULT_MS / 1000;

/* ------------------------------------------------------------------ */
/* Campaneo sintetizado: tres golpes agudos y metalicos.              */
/* ------------------------------------------------------------------ */

static void build_synth(void)
{
    const float two_pi = 6.28318530718f;
    /* Dos campanas: "tin" aguda y "tun" mas grave, muy juntas. Luego la
     * pausa configurable (0,90 s por defecto) hasta el siguiente aviso. */
    const float strikes[2] = { 0.00f, 0.14f };
    const float freqs[2] = { 2093.0f, 1568.0f };

    for (int i = 0; i < SYNTH_LEN; i++) {
        float t = (float)i / (float)AUDIO_RATE;
        float s = 0.0f;

        for (int k = 0; k < 2; k++) {
            float tau = t - strikes[k];
            if (tau < 0.0f || tau > 0.26f) {
                continue;
            }

            float env = expf(-tau * 14.0f);
            if (tau < 0.003f) {
                env *= tau / 0.003f;
            }

            float f0 = freqs[k];
            float b = sinf(two_pi * f0 * tau) +
                      0.50f * sinf(two_pi * f0 * 2.01f * tau) +
                      0.25f * sinf(two_pi * f0 * 3.02f * tau);

            s += b * env * ((k == 0) ? 1.0f : 0.90f);
        }

        s *= 0.24f;
        if (s > 1.0f) s = 1.0f;
        if (s < -1.0f) s = -1.0f;
        g_synth[i] = (short)(s * 32760.0f);
    }

    /* Rampa final para que el bucle no haga clic. */
    int fade = AUDIO_RATE / 100;   /* 10 ms */
    if (fade > SYNTH_LEN) {
        fade = SYNTH_LEN;
    }
    for (int i = 0; i < fade; i++) {
        int idx = SYNTH_LEN - 1 - i;
        g_synth[idx] = (short)((int)g_synth[idx] * i / fade);
    }

    g_play = g_synth;
    g_play_len = SYNTH_LEN;
}

/* ------------------------------------------------------------------ */
/* WAV del usuario (PCM 16 bits, mono o estereo, cualquier frecuencia).*/
/* ------------------------------------------------------------------ */

static unsigned int rd_u16(const unsigned char *p)
{
    return (unsigned int)p[0] | ((unsigned int)p[1] << 8);
}

static unsigned int rd_u32(const unsigned char *p)
{
    return (unsigned int)p[0] | ((unsigned int)p[1] << 8) |
           ((unsigned int)p[2] << 16) | ((unsigned int)p[3] << 24);
}

static void fade_edges(short *buf, int len)
{
    int fade = AUDIO_RATE / 500;   /* 2 ms */
    if (fade < 1) {
        return;
    }
    if (fade * 2 > len) {
        fade = len / 2;
    }

    for (int i = 0; i < fade; i++) {
        int a = (i * 32768) / fade;
        buf[i] = (short)((int)buf[i] * a / 32768);
        buf[len - 1 - i] = (short)((int)buf[len - 1 - i] * a / 32768);
    }
}

static int load_wav(const char *path)
{
    SceUID fd = sceIoOpen(path, PSP_O_RDONLY, 0777);
    if (fd < 0) {
        return 0;
    }

    int size = (int)sceIoLseek(fd, 0, PSP_SEEK_END);
    if (size <= 44 || size > (6 * 1024 * 1024)) {
        sceIoClose(fd);
        return 0;
    }
    sceIoLseek(fd, 0, PSP_SEEK_SET);

    unsigned char *raw = (unsigned char *)malloc((size_t)size);
    if (raw == NULL) {
        sceIoClose(fd);
        return 0;
    }

    int got = 0;
    while (got < size) {
        int n = sceIoRead(fd, raw + got, size - got);
        if (n <= 0) {
            break;
        }
        got += n;
    }
    sceIoClose(fd);

    if (got < 44 || memcmp(raw, "RIFF", 4) != 0 ||
        memcmp(raw + 8, "WAVE", 4) != 0) {
        free(raw);
        return 0;
    }

    int fmt = 0, ch = 0, rate = 0, bits = 0;
    const unsigned char *data = NULL;
    int data_size = 0;

    int pos = 12;
    while (pos + 8 <= got) {
        const unsigned char *ck = raw + pos;
        unsigned int csz = rd_u32(ck + 4);
        int body = pos + 8;
        unsigned int avail = (unsigned int)(got - body);

        if (csz > avail) {
            if (memcmp(ck, "data", 4) != 0) {
                break;
            }
            csz = avail;
        }

        if (memcmp(ck, "fmt ", 4) == 0) {
            if (csz >= 16 && body + 16 <= got) {
                fmt = (int)rd_u16(raw + body);
                ch = (int)rd_u16(raw + body + 2);
                rate = (int)rd_u32(raw + body + 4);
                bits = (int)rd_u16(raw + body + 14);
            }
        } else if (memcmp(ck, "data", 4) == 0) {
            data = raw + body;
            data_size = (int)csz;
        }

        pos = body + (int)csz + ((int)csz & 1);
    }

    if (fmt != 1 || bits != 16 || (ch != 1 && ch != 2) || rate <= 0 ||
        data == NULL || data_size < 2) {
        free(raw);
        return 0;
    }

    int frames = data_size / (ch * 2);
    if (frames < 1) {
        free(raw);
        return 0;
    }

    long long calc = (long long)frames * AUDIO_RATE / rate;
    int out_len = (int)calc;
    if (out_len < 1) {
        out_len = 1;
    }
    if (out_len > CHIME_MAX_LEN) {
        out_len = CHIME_MAX_LEN;
    }

    short *out = (short *)malloc((size_t)out_len * sizeof(short));
    if (out == NULL) {
        free(raw);
        return 0;
    }

    for (int i = 0; i < out_len; i++) {
        double src = (double)i * (double)rate / (double)AUDIO_RATE;
        int i0 = (int)src;
        if (i0 >= frames) {
            i0 = frames - 1;
        }
        int i1 = (i0 + 1 < frames) ? (i0 + 1) : i0;
        double frac = src - (double)i0;

        const unsigned char *pa = data + (size_t)i0 * ch * 2;
        const unsigned char *pb = data + (size_t)i1 * ch * 2;

        int a = (short)(pa[0] | (pa[1] << 8));
        int b = (short)(pb[0] | (pb[1] << 8));

        if (ch == 2) {
            int a2 = (short)(pa[2] | (pa[3] << 8));
            int b2 = (short)(pb[2] | (pb[3] << 8));
            a = (a + a2) / 2;
            b = (b + b2) / 2;
        }

        int v = (int)((double)a + ((double)b - (double)a) * frac);
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        out[i] = (short)v;
    }

    free(raw);
    fade_edges(out, out_len);

    g_wav = out;
    g_play = g_wav;
    g_play_len = out_len;
    return 1;
}

static void build_play_buffer(void)
{
    static const char *paths[] = {
        "chime.wav",
        "ms0:/PSP/SAVEDATA/PSPTORQUE/chime.wav",
        "ms0:/PSP/SAVEDATA/PSPTORQUE/CHIME.WAV",
        "ms0:/PSP/GAME/PSPTorqueOBD/chime.wav",
        "ms0:/PSP/GAME/PSPTorqueOBD/CHIME.WAV"
    };

    for (int i = 0; i < (int)(sizeof(paths) / sizeof(paths[0])); i++) {
        if (load_wav(paths[i])) {
            return;
        }
    }

    build_synth();
}

/* ------------------------------------------------------------------ */
/* Reproduccion.                                                      */
/* ------------------------------------------------------------------ */

/* Reproduce el buffer una sola vez. El ultimo bloque se completa con
 * silencio para no arrastrar muestras viejas. */
static void play_once(void)
{
    int pos = 0;

    while (pos < g_play_len && g_chime_on && g_run) {
        int n = g_play_len - pos;
        if (n > AUDIO_BLOCK) {
            n = AUDIO_BLOCK;
        }

        memcpy(g_buf, &g_play[pos], (size_t)n * sizeof(short));
        if (n < AUDIO_BLOCK) {
            memset(&g_buf[n], 0,
                   (size_t)(AUDIO_BLOCK - n) * sizeof(short));
        }

        if (sceAudioOutputBlocking(g_ch, AUDIO_VOLUME, g_buf) < 0) {
            break;
        }
        pos += n;
    }
}

/* Espera entre avisos en trozos de 10 ms: asi el hilo reacciona rapido
 * cuando se apaga el aviso o se cierra la aplicacion. */
static void wait_gap(void)
{
    int remain = g_gap_len;
    const int chunk = AUDIO_RATE / 100;

    while (remain > 0 && g_chime_on && g_run) {
        int n = (remain > chunk) ? chunk : remain;
        sceKernelDelayThread((unsigned int)(n * 1000000 / AUDIO_RATE));
        remain -= n;
    }
}

static int audio_thread(SceSize args, void *argp)
{
    (void)args;
    (void)argp;

    while (g_run) {
        if (!g_chime_on || g_ch < 0 || g_play_len <= 0) {
            sceKernelDelayThread(20000);
            continue;
        }

        play_once();
        wait_gap();
    }

    sceKernelExitDeleteThread(0);
    return 0;
}

int audio_init(void)
{
    if (g_ch >= 0) {
        return 1;
    }

    build_play_buffer();

    g_ch = sceAudioChReserve(PSP_AUDIO_NEXT_CHANNEL, AUDIO_BLOCK,
                             PSP_AUDIO_FORMAT_MONO);
    if (g_ch < 0) {
        return 0;
    }

    g_run = 1;
    g_thread = sceKernelCreateThread("obd_audio", audio_thread,
                                     0x38, 0x2000, 0, NULL);
    if (g_thread < 0) {
        g_run = 0;
        sceAudioChRelease(g_ch);
        g_ch = -1;
        return 0;
    }

    if (sceKernelStartThread(g_thread, 0, NULL) < 0) {
        g_run = 0;
        sceKernelDeleteThread(g_thread);
        g_thread = -1;
        sceAudioChRelease(g_ch);
        g_ch = -1;
        return 0;
    }

    return 1;
}

void audio_chime(int on)
{
    g_chime_on = on ? 1 : 0;
}

void audio_set_gap_ms(int ms)
{
    if (ms < CHIME_GAP_MIN_MS) {
        ms = CHIME_GAP_MIN_MS;
    }
    if (ms > CHIME_GAP_MAX_MS) {
        ms = CHIME_GAP_MAX_MS;
    }

    g_gap_len = AUDIO_RATE * ms / 1000;
}

void audio_shutdown(void)
{
    g_chime_on = 0;
    g_run = 0;

    if (g_thread >= 0) {
        sceKernelWaitThreadEnd(g_thread, NULL);
        g_thread = -1;
    }

    if (g_ch >= 0) {
        sceAudioChRelease(g_ch);
        g_ch = -1;
    }

    if (g_wav != NULL) {
        free(g_wav);
        g_wav = NULL;
    }
}
