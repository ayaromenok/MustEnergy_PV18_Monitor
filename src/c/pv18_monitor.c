/*
 * pv18_monitor.c — MustPower PV1800 (SP1800, "18-series") status monitor.
 *
 * C port of src/py/pv18_monitor.py.
 *
 * Reads the inverter over RS-485 Modbus RTU (FC 0x03) and prints a decoded
 * status to the terminal. The Modbus RTU layer (frames, CRC) is implemented
 * here; the serial port is handled with plain termios.
 *
 * MQTT (optional, --mqtt-host): each successfully read snapshot is published
 * with libpaho-mqtt (MQTTAsync) under the "PV18" prefix (configurable via
 * --mqtt-prefix):
 *   PV18/status            full decoded snapshot (JSON)
 *   PV18/online            true/false (retained, last will set on disconnect)
 *   PV18/inverter/...      individual values (JSON, retained), e.g.
 *   PV18/battery/voltage       PV18/pv/power      PV18/errors
 *
 * The display carries a status bar at the bottom: application version (HEAD
 * commit date as YYTMMDDHHMM) and short commit hash ("*" if the working tree
 * is dirty), plus the active settings (port, slave, baud, interval, MQTT).
 *
 * Hardware quirks handled (see PV1800 docstring in the Python version):
 *   * the inverter answers only the first request after the port is opened,
 *     so every request uses a freshly opened port;
 *   * the inverter answers only sporadically (roughly 3 of 4 requests,
 *     worse after bursts); failed blocks are retried after a pause and the
 *     remaining blocks are still read;
 *   * the device drops the last register of each block (byte count field
 *     still says the full count); the parser accepts the shorter frame.
 *
 * Protocol reference: doc/PV1800_protocol.md
 *   * RS-485, 19200 baud, 8 data bits, no parity, 1 stop bit
 *   * Modbus slave address 4
 *   * Blocks polled: 10001(8) charger id, 10103(10) charger settings,
 *     15201(21) charger status, 20000(17) inverter id, 20101(43) inverter
 *     settings, 25201(79) inverter status
 *
 * Usage:
 *   pv18_monitor                     # single snapshot
 *   pv18_monitor --watch             # refresh every 2 s
 *   pv18_monitor --port /dev/ttyUSB1 --slave 4
 *   pv18_monitor --watch --mqtt-host 192.168.1.10
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "MQTTAsync.h"

/* ---------------------------------------------------------------------------
 * Protocol constants (see doc/PV1800_protocol.md)
 * --------------------------------------------------------------------------- */

#define DEFAULT_PORT    "/dev/ttyUSB0"
#define DEFAULT_BAUD    19200
#define DEFAULT_SLAVE   4
#define NREG            65536
#define MAX_BLOCK_REGS  128
#define max_block_regs MAX_BLOCK_REGS

typedef struct {
    const char *name;
    int start;
    int count;
} Block;

/* The six blocks SolarPowerMonitor polls */
static const Block BLOCKS[] = {
    {"charger_id", 10001, 8},
    {"charger_set", 10103, 10},
    {"charger_st", 15201, 21},
    {"inv_id", 20000, 17},
    {"inv_set", 20101, 43},
    {"inv_st", 25201, 79},
};
#define N_BLOCKS ((int)(sizeof(BLOCKS) / sizeof(BLOCKS[0])))

/* Blocks refreshed on every watch cycle (SolarPowerMonitor 'alwaysRead'). */
static int is_status_block(int i) {
    return strcmp(BLOCKS[i].name, "charger_st") == 0 ||
           strcmp(BLOCKS[i].name, "inv_st") == 0;
}

/* Register addresses used for display (doc §6) */
typedef struct { const char *key; int addr; } RegAddr;
__attribute__((unused))
static const RegAddr A[] = {
    /* inverter identification */
    {"model_h", 20000}, {"model_l", 20001},
    {"hw", 20004}, {"sw", 20005}, {"proto", 20006},
    /* inverter status */
    {"work", 25201}, {"vac_class", 25202}, {"rated_va", 25203},
    {"batt_v", 25205}, {"inv_v", 25206}, {"grid_v", 25207}, {"bus_v", 25208},
    {"inv_i", 25210}, {"grid_i", 25211}, {"load_i", 25212},
    {"p_inv", 25213}, {"p_grid", 25214}, {"p_load", 25215}, {"load_pct", 25216},
    {"s_inv", 25217}, {"s_grid", 25218}, {"s_load", 25219},
    {"f_inv", 25225}, {"f_grid", 25226},
    {"t_ac", 25233}, {"t_tr", 25234}, {"t_dc", 25235},
    {"r_inv", 25237}, {"r_grid", 25238}, {"r_load", 25239},
    {"r_n", 25240}, {"r_dc", 25241}, {"r_earth", 25242},
    {"err1", 25261}, {"err2", 25262}, {"err3", 25263},
    {"warn1", 25265}, {"warn2", 25266},
    {"batt_p", 25273}, {"batt_i", 25274}, {"batt_class", 25275}, {"rated_w", 25277},
    /* charger status */
    {"c_work", 15201}, {"c_mppt", 15202}, {"c_charge", 15203},
    {"pv_v", 15205}, {"c_batt_v", 15206}, {"c_i", 15207}, {"c_p", 15208},
    {"c_t", 15209}, {"c_r_batt", 15211}, {"c_r_pv", 15212},
    {"c_err", 15213}, {"c_warn", 15214}, {"c_batt_class", 15215},
    {"c_rated_i", 15216},
    /* charger settings */
    {"s_float", 10103}, {"s_absorb", 10104}, {"s_max_i", 10108},
    {"s_batt_type", 10110}, {"s_batt_ah", 10111},
    /* inverter settings */
    {"s_offgrid", 20101}, {"s_v_set", 20102}, {"s_f_set", 20103},
    {"s_mode", 20109}, {"s_grid_prot", 20111}, {"s_max_disch_i", 20113},
    {"s_stop_disch_v", 20118}, {"s_stop_ch_v", 20119}, {"s_grid_ch_i", 20125},
    {"s_batt_low", 20127}, {"s_batt_high", 20128}, {"s_max_comb_i", 20132},
    {"s_priority", 20143},
};

/* Lifetime energy counters: label, high word, low word; kWh = (H*1000+L)*0.1 */
typedef struct { const char *label; int hi; int lo; } EnergyPair;
static const EnergyPair ENERGY_PAIRS[] = {
    {"PV", 15217, 15218},
    {"grid charge", 25245, 25246},
    {"battery discharge", 25247, 25248},
    {"grid buy", 25249, 25250},
    {"grid sell", 25251, 25252},
    {"load", 25253, 25254},
    {"self-use", 25255, 25256},
    {"PV sell", 25257, 25258},
};
#define N_ENERGY ((int)(sizeof(ENERGY_PAIRS) / sizeof(ENERGY_PAIRS[0])))

static const RegAddr RELAYS[] = {
    {"inverter", 25237}, {"grid", 25238}, {"load", 25239},
    {"n-line", 25240}, {"dc", 25241}, {"earth", 25242},
};
#define N_RELAYS ((int)(sizeof(RELAYS) / sizeof(RELAYS[0])))

/* ---------------------------------------------------------------------------
 * Decoding tables
 * --------------------------------------------------------------------------- */

static const char *WORK_STATE[] = {"PowerOn", "SelfTest", "OffGrid", "GridTie",
                                   "ByPass", "Stop", "GridCharging"};
static const char *MPPT_STATE[] = {"Stop", "MPPT", "CurrentLimit"};
static const char *CHARGE_STATE[] = {"Stop", "Absorb", "Float"};
static const char *CHR_WORK[] = {"Initialization", "Selftest", "Work", "Stop"};
/* index 0 unused */
static const char *BATTERY_TYPE[] = {NULL, "Defined", "Lithium", "SEALED_LEAD",
                                     "AGM", "GEL", "FLOODED"};
static const char *ENERGY_MODE[] = {NULL, "SBU", "SUB", "UTI", "SOL"};
static const char *GRID_PROTECT[] = {"VDE4105", "UPS", "Home", "GEN"};
/* index 1 unused */
static const char *CHARGE_PRIORITY[] = {"Solar first", NULL, "Solar+Utility",
                                        "Only solar"};

/* Error / warning bit tables (doc §7). NULL = bit exists but no text defined. */
static const char *INV_ERR1[] = {
    "Fan locked (inverter off)", "Transformer over temperature",
    "Battery voltage too high", "Battery voltage too low",
    "Output short circuit", "Inverter output voltage high",
    "Overload time out", "Bus voltage too high",
    "Bus soft start failed", "Main relay failed",
    "Output voltage sensor error", "Grid voltage sensor error",
    "Output current sensor error", "Grid current sensor error",
    "Load current sensor error", "Grid over current error",
};
static const char *INV_ERR2[] = {
    "Inverter radiator over temperature",
    "Solar charger battery voltage class error",
    "Solar charger current sensor error",
    "Solar charger current uncontrollable",
    "Inverter grid voltage low", "Inverter grid voltage high",
    "Inverter grid under frequency", "Inverter grid over frequency",
    "Inverter over current protection", "Bus voltage too low",
    "Inverter soft start failed", "Over DC voltage in AC output",
    "Battery connection open", "Control current sensor error",
    "Output voltage too low", NULL,
};
static const char *INV_WARN1[] = {
    "Fan locked (inverter on)", "Fan2 locked (inverter on)",
    "Battery over-charged", "Low battery", "Overload",
    "Output power derating", "Solar charger stop: low battery",
    "Solar charger stop: high PV voltage",
    "Solar charger stop: overload", "Solar charger over temperature",
    "PV charger communication error",
};
static const char *CHR_ERR1[] = {
    "Hardware protection", "Over current", "Current sensor error",
    "Over temperature", "PV voltage too high", NULL,
    "Battery voltage too high", "Battery voltage too low",
    "Current uncontrollable", "Parameter error",
};
static const char *CHR_WARN1[] = {"Fan error"};

#define TBL(t) (const char *const *)(t), (int)(sizeof(t) / sizeof((t)[0]))

/* ---------------------------------------------------------------------------
 * Register store (last correctly received values, kept across cycles)
 * --------------------------------------------------------------------------- */

static int reg_valid[NREG];
static int reg_val[NREG];

static int reg_get(int addr, int *val)
{
    if (addr < 0 || addr >= NREG || !reg_valid[addr])
        return 0;
    *val = reg_val[addr];
    return 1;
}

static void reg_set(int addr, int v)
{
    if (addr < 0 || addr >= NREG)
        return;
    reg_valid[addr] = 1;
    reg_val[addr] = v;
}

/* ---------------------------------------------------------------------------
 * String buffer
 * --------------------------------------------------------------------------- */

typedef struct {
    char *s;
    size_t len, cap;
} SB;

static void sb_init(SB *sb)
{
    sb->cap = 4096;
    sb->s = malloc(sb->cap);
    sb->s[0] = '\0';
    sb->len = 0;
}

static void sb_reserve(SB *sb, size_t extra)
{
    if (sb->len + extra + 1 > sb->cap) {
        while (sb->len + extra + 1 > sb->cap)
            sb->cap *= 2;
        sb->s = realloc(sb->s, sb->cap);
    }
}

static void sb_put(SB *sb, const char *s)
{
    size_t n = strlen(s);
    sb_reserve(sb, n);
    memcpy(sb->s + sb->len, s, n);
    sb->len += n;
    sb->s[sb->len] = '\0';
}

static void sb_printf(SB *sb, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    for (;;) {
        va_list ap2;
        int avail, n;
        sb_reserve(sb, 256);
        avail = (int)(sb->cap - sb->len);
        va_copy(ap2, ap);
        n = vsnprintf(sb->s + sb->len, (size_t)avail, fmt, ap2);
        va_end(ap2);
        if (n < 0) {
            va_end(ap);
            return;
        }
        if ((size_t)n < (size_t)avail) {
            sb->len += (size_t)n;
            break;
        }
        sb_reserve(sb, (size_t)n + 1); /* need the full length: retry */
    }
    va_end(ap);
    sb->s[sb->len] = '\0';
}

/* ---------------------------------------------------------------------------
 * Modbus RTU (FC 0x03) — minimal implementation
 * --------------------------------------------------------------------------- */

static uint16_t crc16(const unsigned char *data, size_t len)
{
    /* Modbus CRC-16 (poly 0xA001, init 0xFFFF) */
    uint16_t crc = 0xFFFF;
    size_t i;
    int b;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (b = 0; b < 8; b++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001)
                            : (uint16_t)(crc >> 1);
    }
    return crc;
}

typedef struct {
    const char *device;
    int baud;
    int slave;
    double settle;        /* pause after open before the first request */
    double timeout;       /* max wait for a response */
    int retries;
    double retry_pause;   /* pause between retry attempts */
    int fd;
} PV1800;

static double now_mono(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void sleep_s(double s)
{
    struct timespec ts;
    ts.tv_sec = (time_t)s;
    ts.tv_nsec = (long)((s - (double)ts.tv_sec) * 1e9);
    nanosleep(&ts, NULL);
}

/* Interruptible sleep: returns early on Ctrl-C. */
static volatile sig_atomic_t g_interrupt;

static void sleep_s_int(double s)
{
    int steps = (int)(s * 20.0 + 0.5);
    if (steps < 1)
        steps = 1;
    int i;
    for (i = 0; i < steps && !g_interrupt; i++)
        usleep(50000);
}

static int pv_open(PV1800 *pv)
{
    int fd = open(pv->device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
        return -1;
    struct termios t;
    memset(&t, 0, sizeof(t));
    if (tcgetattr(fd, &t) == 0) {
        cfmakeraw(&t);
        cfsetispeed(&t, (speed_t)pv->baud);
        cfsetospeed(&t, (speed_t)pv->baud);
        t.c_cflag |= CLOCAL | CREAD;
        t.c_cflag &= ~CSIZE;
        t.c_cflag |= CS8;
        t.c_cflag &= ~PARENB;
        t.c_cflag &= ~CSTOPB;
        t.c_cflag &= ~CRTSCTS;
        t.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        tcsetattr(fd, TCSANOW, &t);
    }
    tcflush(fd, TCIOFLUSH);
    fcntl(fd, F_SETFL, 0); /* back to blocking; select() manages waits */
    pv->fd = fd;
    sleep_s(pv->settle);
    return 0;
}

static void pv_close(PV1800 *pv)
{
    if (pv->fd >= 0) {
        close(pv->fd);
        pv->fd = -1;
    }
}

/* Collect one response frame: stop after a quiet gap or timeout. */
static size_t pv_read_response(PV1800 *pv, unsigned char *buf, size_t cap)
{
    size_t len = 0;
    double deadline = now_mono() + pv->timeout;
    double last = now_mono();
    while (now_mono() < deadline) {
        fd_set rset;
        struct timeval tv;
        FD_ZERO(&rset);
        FD_SET(pv->fd, &rset);
        tv.tv_sec = 0;
        tv.tv_usec = 10000;
        int rc = select(pv->fd + 1, &rset, NULL, NULL, &tv);
        if (rc > 0) {
            ssize_t n = read(pv->fd, buf + len, cap - len);
            if (n > 0) {
                len += (size_t)n;
                last = now_mono();
            }
        }
        if (len && now_mono() - last > 0.05) /* frame done */
            break;
    }
    return len;
}

/*
 * Parse a response frame.
 * Returns number of registers on success, 0 if the frame is not usable,
 * or a negative value (-code) if the slave sent a Modbus exception.
 * Values (offset -> value) go into out_val[].
 */
static int pv_parse(const unsigned char *buf, size_t len, int slave,
                    int *out_val, int max_regs)
{
    uint16_t crc;
    int claimed, actual, n, i;
    if (len < 5)
        return 0;
    if (buf[0] != (unsigned char)slave)
        return 0; /* bus noise / not our frame */
    if (buf[1] == 0x83)
        return -(int)buf[2];
    if (buf[1] != 0x03)
        return 0;
    if (len < 7)
        return 0;
    crc = (uint16_t)((uint16_t)buf[len - 1] << 8 | buf[len - 2]);
    if (crc16(buf, len - 2) != crc)
        return 0;
    claimed = buf[2] / 2;                 /* registers per header */
    actual = (int)(len - 5) / 2;          /* registers really sent */
    n = claimed < actual ? claimed : actual; /* device may drop the last one */
    if (n <= 0)
        return 0;
    if (n > max_regs)
        n = max_regs;
    for (i = 0; i < n; i++)
        out_val[i] = (buf[3 + 2 * i] << 8) | buf[4 + 2 * i];
    return n;
}

static const char *modbus_exception_text(int code)
{
    switch (code) {
    case 1: return "IllegalFunction";
    case 2: return "IllegalDataAddress";
    case 3: return "IllegalDataValue";
    case 4: return "SlaveDeviceFailure";
    default: return NULL;
    }
}

/*
 * Read one register block on a freshly opened port.
 * Returns 0 on success (n_out = number of registers, values in out_val),
 * -1 on failure with a message in err (errsz).
 */
static int pv_read_block(PV1800 *pv, int start, int count,
                         int *n_out, int *out_val, char *err, size_t errsz)
{
    unsigned char body[6], frame[8];
    int attempt;
    const char *last_err = NULL;
    char last_errbuf[128];
    int last_errno = 0;

    body[0] = (unsigned char)pv->slave;
    body[1] = 0x03;
    body[2] = (unsigned char)((start >> 8) & 0xFF);
    body[3] = (unsigned char)(start & 0xFF);
    body[4] = (unsigned char)((count >> 8) & 0xFF);
    body[5] = (unsigned char)(count & 0xFF);
    uint16_t c = crc16(body, 6);
    frame[0] = body[0]; frame[1] = body[1]; frame[2] = body[2];
    frame[3] = body[3]; frame[4] = body[4]; frame[5] = body[5];
    frame[6] = (unsigned char)(c & 0xFF);      /* CRC low byte first */
    frame[7] = (unsigned char)((c >> 8) & 0xFF);

    for (attempt = 0; attempt < pv->retries; attempt++) {
        unsigned char buf[MAX_BLOCK_REGS * 2 + 5];
        size_t len;
        int n;
        if (attempt)
            sleep_s(pv->retry_pause);
        if (pv_open(pv) != 0) {
            last_errno = errno;
            last_err = "port open failed";
            pv_close(pv);
            continue;
        }
        if (write(pv->fd, frame, sizeof(frame)) < 0) {
            last_errno = errno;
            last_err = "port write failed";
        } else {
            len = pv_read_response(pv, buf, sizeof(buf));
            n = pv_parse(buf, len, pv->slave, out_val, max_block_regs);
            pv_close(pv);
            if (n > 0) {
                *n_out = n;
                return 0;
            }
            if (n < 0) {
                const char *t = modbus_exception_text(-n);
                snprintf(last_errbuf, sizeof(last_errbuf), "%s",
                         t ? t : "unknown exception");
                last_err = last_errbuf;
            } else {
                snprintf(last_errbuf, sizeof(last_errbuf),
                         "no valid response from slave %d (%zu raw bytes)",
                         pv->slave, len);
                last_err = last_errbuf;
            }
        }
        pv_close(pv);
    }
    if (last_errno && last_err && strncmp(last_err, "port", 4) == 0)
        snprintf(err, errsz, "%s: %s", last_err, strerror(last_errno));
    else
        snprintf(err, errsz, "%s", last_err ? last_err
                    : "no response from slave");
    return -1;
}

/* ---------------------------------------------------------------------------
 * Decoding helpers
 * --------------------------------------------------------------------------- */

static void fmt_edition(int n, char *out, size_t sz)
{
    /* 10414 -> '1.04.14' (group digits in pairs from the right) */
    char digits[16];
    char parts[8][3];
    int d = 0, i, np = 0;
    if (n <= 0)
        n = 0;
    do {
        digits[d++] = '0' + (n % 10);
        n /= 10;
    } while (n && d < (int)sizeof(digits) - 1);
    for (i = 0; i < d; i += 2) {
        if (i + 1 < d) {
            parts[np][0] = digits[i + 1];
            parts[np][1] = digits[i];
            parts[np][2] = '\0';
        } else {
            parts[np][0] = digits[i];
            parts[np][1] = '\0';
        }
        np++;
    }
    out[0] = '\0';
    for (i = np - 1; i >= 0; i--) {
        if (out[0])
            strcat(out, ".");
        strcat(out, parts[i]);
    }
    (void)sz;
}

static void model_name(char *out, size_t sz)
{
    int mh, ml, ed;
    if (!reg_get(20000, &mh) || !reg_get(20001, &ml)) {
        snprintf(out, sz, "PV1800 (unidentified)");
        return;
    }
    if (reg_get(20006, &ed) && ed >= 10413) {
        snprintf(out, sz, "%c%c%d",
                 (char)((mh >> 8) & 0xFF), (char)(mh & 0xFF), ml);
        return;
    }
    snprintf(out, sz, "PV %d", ml);
}

static double energy_kwh(int hi, int lo)
{
    int h = 0, l = 0;
    reg_get(hi, &h);
    reg_get(lo, &l);
    return ((double)h * 1000.0 + (double)l) * 0.1;
}

/* ---------------------------------------------------------------------------
 * Rendering
 * --------------------------------------------------------------------------- */

static void fval(SB *sb, int addr, double scale, int nd, const char *suffix)
{
    int v;
    if (!reg_get(addr, &v)) {
        sb_put(sb, "  --");
        return;
    }
    sb_printf(sb, "%.*f%s", nd, v * scale, suffix);
}

static void fsigned(SB *sb, int addr, double scale, int nd, const char *suffix)
{
    int v;
    if (!reg_get(addr, &v)) {
        sb_put(sb, "  --");
        return;
    }
    if (v >= 32768)
        v -= 65536;
    sb_printf(sb, "%+.*f%s", nd, v * scale, suffix);
}

static void fstate(SB *sb, int addr, const char *const *tbl, int n)
{
    int v;
    if (!reg_get(addr, &v)) {
        sb_put(sb, "--");
        return;
    }
    if (v >= 0 && v < n && tbl[v])
        sb_put(sb, tbl[v]);
    else
        sb_printf(sb, "unknown(%d)", v);
}

static void fonoff(SB *sb, int addr)
{
    int v;
    if (!reg_get(addr, &v))
        sb_put(sb, "?");
    else
        sb_put(sb, v ? "ON" : "OFF");
}

static void fint(SB *sb, int addr)
{
    int v;
    if (!reg_get(addr, &v))
        sb_put(sb, "--");
    else
        sb_printf(sb, "%d", v);
}

/* Append error/warning bits of one register (terminal style). */
static void bits_terminal(SB *sb, const char *label, int addr,
                          const char *const *tbl, int n)
{
    int v, bit;
    if (!reg_get(addr, &v))
        return;
    for (bit = 0; bit < 16; bit++) {
        if (v & (1 << bit)) {
            const char *name = (bit < n && tbl[bit]) ? tbl[bit] : NULL;
            sb_printf(sb, "  %s bit %d: %s\n", label, bit,
                      name ? name : "undefined bit");
        }
    }
}

static void render(SB *out)
{
    int i, v;

    /* --- header ----------------------------------------------------------- */
    {
        char model[64];
        model_name(model, sizeof(model));
        sb_printf(out, "MustPower %s  \342\200\224  status\n", model);
        for (i = 0; i < 60; i++)
            sb_put(out, "=");
        sb_put(out, "\n");
        time_t t = time(NULL);
        char ts[64];
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", localtime(&t));
        sb_printf(out, "Time: %s\n", ts);
        {
            char hwe[16], swe[16], pe[16];
            int hw = 0, sw = 0;
            reg_get(20004, &hw);
            reg_get(20005, &sw);
            fmt_edition(hw ? hw : 0, hwe, sizeof(hwe));
            fmt_edition(sw ? sw : 0, swe, sizeof(swe));
            sb_printf(out, "Firmware: HW %s, SW %s, protocol ", hwe, swe);
            if (reg_get(20006, &v)) {
                fmt_edition(v, pe, sizeof(pe));
                sb_printf(out, "%s\n", pe);
            } else {
                sb_put(out, "--\n");
            }
        }
        sb_put(out, "\n");
    }

    /* --- inverter ----------------------------------------------------------- */
    sb_put(out, "[Inverter]\n");
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Work state:   ");
        fstate(&l, 25201, WORK_STATE, 7);
        sb_printf(out, "%s\n", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Output:       ");
        fval(&l, 25206, 0.1, 1, "");
        sb_put(&l, " V / ");
        fval(&l, 25225, 0.01, 2, "");
        sb_put(&l, " Hz   P=");
        fval(&l, 25213, 1, 0, "");
        sb_put(&l, " W  S=");
        fval(&l, 25217, 1, 0, "");
        sb_put(&l, " VA  load ");
        fint(&l, 25216);
        sb_put(&l, "%\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Grid:         ");
        fval(&l, 25207, 0.1, 1, "");
        sb_put(&l, " V / ");
        fval(&l, 25226, 0.01, 2, "");
        sb_put(&l, " Hz   P=");
        fsigned(&l, 25214, 1, 0, "");
        sb_put(&l, " W  S=");
        fsigned(&l, 25218, 1, 0, "");
        sb_put(&l, " VA\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  DC bus:       ");
        fval(&l, 25208, 0.1, 1, "");
        sb_put(&l, " V\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Battery:      ");
        fval(&l, 25205, 0.1, 1, "");
        sb_put(&l, " V  ");
        fsigned(&l, 25274, 1, 0, "");
        sb_put(&l, " A (");
        if (reg_get(25274, &v)) {
            int sv = (v >= 32768) ? v - 65536 : v;
            sb_put(&l, sv > 0 ? "discharge" : (sv < 0 ? "charge" : "idle"));
        } else {
            sb_put(&l, "--");
        }
        sb_put(&l, ")  ");
        fval(&l, 25273, 1, 0, "");
        sb_put(&l, " W   (");
        fint(&l, 25275);
        sb_put(&l, " V class)\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Temperatures: AC heatsink ");
        fint(&l, 25233);
        sb_put(&l, " C, DC heatsink ");
        fint(&l, 25235);
        sb_put(&l, " C, transformer ");
        fint(&l, 25234);
        sb_put(&l, " C\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Relays:       ");
        for (i = 0; i < N_RELAYS; i++) {
            if (i)
                sb_put(&l, "  ");
            sb_printf(&l, "%s ", RELAYS[i].key);
            fonoff(&l, RELAYS[i].addr);
        }
        sb_put(&l, "\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    sb_put(out, "\n");

    /* --- charger ------------------------------------------------------------- */
    sb_put(out, "[Charger / MPPT]\n");
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Work state:   ");
        fstate(&l, 15201, CHR_WORK, 4);
        sb_put(&l, "   MPPT: ");
        fstate(&l, 15202, MPPT_STATE, 3);
        sb_put(&l, "   charging: ");
        fstate(&l, 15203, CHARGE_STATE, 3);
        sb_put(&l, "\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  PV input:     ");
        fval(&l, 15205, 0.1, 1, "");
        sb_put(&l, " V   ");
        fval(&l, 15207, 0.1, 1, "");
        sb_put(&l, " A   ");
        fval(&l, 15208, 1, 0, "");
        sb_put(&l, " W\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Battery:      ");
        fval(&l, 15206, 0.1, 1, "");
        sb_put(&l, " V   rated current ");
        fval(&l, 15216, 0.1, 1, "");
        sb_put(&l, " A\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Heatsink:     ");
        fint(&l, 15209);
        sb_put(&l, " C   relays: batt ");
        fonoff(&l, 15211);
        sb_put(&l, ", pv ");
        fonoff(&l, 15212);
        sb_put(&l, "\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    sb_put(out, "\n");

    /* --- settings ------------------------------------------------------------- */
    sb_put(out, "[Settings]\n");
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Output setpoint: ");
        fval(&l, 20102, 0.1, 1, "");
        sb_put(&l, " V / ");
        fval(&l, 20103, 0.01, 2, "");
        sb_put(&l, " Hz   off-grid enable: ");
        fint(&l, 20101);
        sb_put(&l, "\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Energy mode:     ");
        fstate(&l, 20109, ENERGY_MODE, 5);
        sb_put(&l, "   grid protection: ");
        fstate(&l, 20111, GRID_PROTECT, 4);
        sb_put(&l, "   priority: ");
        fstate(&l, 20143, CHARGE_PRIORITY, 5);
        sb_put(&l, "\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Battery:         ");
        fstate(&l, 10110, BATTERY_TYPE, 7);
        sb_put(&l, " ");
        fint(&l, 10111);
        sb_put(&l, " Ah, float ");
        fval(&l, 10103, 0.1, 1, "");
        sb_put(&l, " V, absorb ");
        fval(&l, 10104, 0.1, 1, "");
        sb_put(&l, " V, max charge ");
        fval(&l, 10108, 0.1, 1, "");
        sb_put(&l, " A\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Charge limits:   stop discharge ");
        fval(&l, 20118, 0.1, 1, "");
        sb_put(&l, " V, stop charge ");
        fval(&l, 20119, 0.1, 1, "");
        sb_put(&l, " V, low ");
        fval(&l, 20127, 0.1, 1, "");
        sb_put(&l, " V, high ");
        fval(&l, 20128, 0.1, 1, "");
        sb_put(&l, " V\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    {
        SB l;
        sb_init(&l);
        sb_printf(&l, "  Max currents:    discharge ");
        fval(&l, 20113, 0.1, 1, "");
        sb_put(&l, " A, grid charge ");
        fval(&l, 20125, 0.1, 1, "");
        sb_put(&l, " A, combined ");
        fval(&l, 20132, 0.1, 1, "");
        sb_put(&l, " A\n");
        sb_printf(out, "%s", l.s);
        free(l.s);
    }
    sb_put(out, "\n");

    /* --- energy --------------------------------------------------------------- */
    sb_put(out, "[Energy (lifetime)]\n");
    {
        char part[N_ENERGY][64];
        int p, i2;
        for (p = 0; p < N_ENERGY; p++)
            snprintf(part[p], sizeof(part[p]), "%s %.1f kWh",
                     ENERGY_PAIRS[p].label,
                     energy_kwh(ENERGY_PAIRS[p].hi, ENERGY_PAIRS[p].lo));
        for (i2 = 0; i2 < N_ENERGY; i2 += 3) {
            int k, cnt = (N_ENERGY - i2 < 3) ? N_ENERGY - i2 : 3;
            sb_put(out, "  ");
            for (k = 0; k < cnt; k++) {
                if (k)
                    sb_put(out, "  ");
                sb_put(out, part[i2 + k]);
            }
            sb_put(out, "\n");
        }
    }
    sb_put(out, "\n");

    /* --- errors / warnings ------------------------------------------------------ */
    sb_put(out, "[Errors / Warnings]\n");
    {
        SB problems;
        sb_init(&problems);
        bits_terminal(&problems, "Inverter error", 25261, TBL(INV_ERR1));
        bits_terminal(&problems, "Inverter error", 25262, TBL(INV_ERR2));
        bits_terminal(&problems, "Charger error", 15213, TBL(CHR_ERR1));
        if (reg_get(25263, &v) && v)
            sb_printf(&problems, "  Inverter error3: raw 0x%04X (no text table)\n",
                      v & 0xFFFF);
        bits_terminal(&problems, "Inverter warning", 25265, TBL(INV_WARN1));
        bits_terminal(&problems, "Charger warning", 15214, TBL(CHR_WARN1));
        if (reg_get(25266, &v) && v)
            sb_printf(&problems, "  Inverter warning2: raw 0x%04X (no text table)\n",
                      v & 0xFFFF);
        if (problems.len)
            sb_printf(out, "%s", problems.s);
        else
            sb_put(out, "  none active");
        free(problems.s);
    }
}

/* ---------------------------------------------------------------------------
 * JSON status (MQTT)
 * --------------------------------------------------------------------------- */

static void jstr(SB *sb, const char *s)
{
    const unsigned char *p;
    sb_put(sb, "\"");
    for (p = (const unsigned char *)s; *p; p++) {
        switch (*p) {
        case '"': sb_put(sb, "\\\""); break;
        case '\\': sb_put(sb, "\\\\"); break;
        case '\n': sb_put(sb, "\\n"); break;
        case '\r': sb_put(sb, "\\r"); break;
        case '\t': sb_put(sb, "\\t"); break;
        default:
            if (*p < 0x20)
                sb_printf(sb, "\\u%04x", *p);
            else
                sb_put(sb, (char[]){*p, 0});
        }
    }
    sb_put(sb, "\"");
}

static double sval(int addr, double scale)
{
    int v;
    if (!reg_get(addr, &v))
        return NAN;
    return v * scale;
}

static double ssval(int addr, double scale)
{
    int v;
    if (!reg_get(addr, &v))
        return NAN;
    if (v >= 32768)
        v -= 65536;
    return v * scale;
}

/* Append a JSON number or null (trailing zeros trimmed, like Python's
 * json.dumps(round(v, nd))). */
static void jnum(SB *sb, double v, int nd)
{
    char buf[64];
    if (isnan(v)) {
        sb_put(sb, "null");
        return;
    }
    snprintf(buf, sizeof(buf), "%.*f", nd, v);
    if (strchr(buf, '.')) {
        size_t len = strlen(buf);
        while (len > 2 && buf[len - 1] == '0')
            buf[--len] = '\0';
        if (buf[len - 1] == '.')
            buf[len++] = '0';
        buf[len] = '\0';
    }
    sb_put(sb, buf);
}

static void jnumi(SB *sb, int addr)
{
    int v;
    if (!reg_get(addr, &v))
        sb_put(sb, "null");
    else
        sb_printf(sb, "%d", v);
}

static void jbool(SB *sb, int addr)
{
    int v;
    if (!reg_get(addr, &v))
        sb_put(sb, "null");
    else
        sb_put(sb, v ? "true" : "false");
}

static void jstate(SB *sb, int addr, const char *const *tbl, int n)
{
    int v;
    if (!reg_get(addr, &v)) {
        sb_put(sb, "null");
        return;
    }
    if (v >= 0 && v < n && tbl[v])
        jstr(sb, tbl[v]);
    else {
        char b[32];
        snprintf(b, sizeof(b), "unknown(%d)", v);
        jstr(sb, b);
    }
}

typedef struct {
    const char *label;
    int addr;
    const char *const *tbl;
    int n;
} BitPair;

/* Append a JSON array of active error/warning bits. */
static void jbits(SB *sb, const BitPair *pairs, int npairs)
{
    int p, bit, v;
    int first = 1;
    sb_put(sb, "[");
    for (p = 0; p < npairs; p++) {
        if (!reg_get(pairs[p].addr, &v))
            continue;
        for (bit = 0; bit < 16; bit++) {
            if (!(v & (1 << bit)))
                continue;
            const char *name = (bit < pairs[p].n && pairs[p].tbl[bit])
                                   ? pairs[p].tbl[bit] : NULL;
            char item[128];
            if (name)
                snprintf(item, sizeof(item), "%s: %s", pairs[p].label, name);
            else
                snprintf(item, sizeof(item), "%s: bit %d (undefined)",
                         pairs[p].label, bit);
            if (!first)
                sb_put(sb, ", ");
            jstr(sb, item);
            first = 0;
        }
    }
    sb_put(sb, "]");
}

/*
 * Build the full status JSON (mirrors Python build_status + json.dumps).
 * Returns a malloc'd string.
 */
static char *build_status_json(void)
{
    SB sb;
    char model[64], ts[32], ed[16];
    int v;
    double batt_i;
    time_t t;

    sb_init(&sb);
    model_name(model, sizeof(model));
    t = time(NULL);
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", localtime(&t));

    sb_put(&sb, "{\"model\": ");
    jstr(&sb, model);
    sb_printf(&sb, ", \"timestamp\": \"%s\", ", ts);

    sb_put(&sb, "\"firmware\": {");
    sb_put(&sb, "\"hardware\": ");
    if (reg_get(20004, &v)) { fmt_edition(v, ed, sizeof(ed)); jstr(&sb, ed); }
    else sb_put(&sb, "null");
    sb_put(&sb, ", \"software\": ");
    if (reg_get(20005, &v)) { fmt_edition(v, ed, sizeof(ed)); jstr(&sb, ed); }
    else sb_put(&sb, "null");
    sb_put(&sb, ", \"protocol\": ");
    if (reg_get(20006, &v)) { fmt_edition(v, ed, sizeof(ed)); jstr(&sb, ed); }
    else sb_put(&sb, "null");
    sb_put(&sb, "}, ");

    sb_put(&sb, "\"inverter\": {");
    sb_put(&sb, "\"work_state\": "); jstate(&sb, 25201, WORK_STATE, 7);
    sb_put(&sb, ", \"voltage\": "); jnum(&sb, sval(25206, 0.1), 1);
    sb_put(&sb, ", \"frequency\": "); jnum(&sb, sval(25225, 0.01), 2);
    sb_put(&sb, ", \"power\": "); jnum(&sb, sval(25213, 1), 1);
    sb_put(&sb, ", \"apparent_power\": "); jnum(&sb, sval(25217, 1), 1);
    sb_put(&sb, ", \"load_percent\": "); jnumi(&sb, 25216);
    sb_put(&sb, ", \"bus_voltage\": "); jnum(&sb, sval(25208, 0.1), 1);
    sb_put(&sb, ", \"temperature_ac\": "); jnumi(&sb, 25233);
    sb_put(&sb, ", \"temperature_dc\": "); jnumi(&sb, 25235);
    sb_put(&sb, ", \"relay_inverter\": "); jbool(&sb, 25237);
    sb_put(&sb, ", \"relay_grid\": "); jbool(&sb, 25238);
    sb_put(&sb, ", \"relay_load\": "); jbool(&sb, 25239);
    sb_put(&sb, ", \"relay_n_line\": "); jbool(&sb, 25240);
    sb_put(&sb, ", \"relay_dc\": "); jbool(&sb, 25241);
    sb_put(&sb, ", \"relay_earth\": "); jbool(&sb, 25242);
    sb_put(&sb, "}, ");

    sb_put(&sb, "\"grid\": {");
    sb_put(&sb, "\"voltage\": "); jnum(&sb, sval(25207, 0.1), 1);
    sb_put(&sb, ", \"frequency\": "); jnum(&sb, sval(25226, 0.01), 2);
    sb_put(&sb, ", \"power\": "); jnum(&sb, ssval(25214, 1), 1);
    sb_put(&sb, ", \"apparent_power\": "); jnum(&sb, ssval(25218, 1), 1);
    sb_put(&sb, "}, ");

    batt_i = ssval(25274, 1);
    sb_put(&sb, "\"battery\": {");
    sb_put(&sb, "\"voltage\": "); jnum(&sb, sval(25205, 0.1), 1);
    sb_put(&sb, ", \"current\": "); jnum(&sb, batt_i, 1);
    sb_put(&sb, ", \"direction\": ");
    if (isnan(batt_i))
        sb_put(&sb, "null");
    else if (batt_i > 0)
        jstr(&sb, "discharge");
    else if (batt_i < 0)
        jstr(&sb, "charge");
    else
        jstr(&sb, "idle");
    sb_put(&sb, ", \"power\": "); jnum(&sb, sval(25273, 1), 1);
    sb_put(&sb, ", \"class\": "); jnumi(&sb, 25275);
    sb_put(&sb, "}, ");

    sb_put(&sb, "\"charger\": {");
    sb_put(&sb, "\"work_state\": "); jstate(&sb, 15201, CHR_WORK, 4);
    sb_put(&sb, ", \"mppt_state\": "); jstate(&sb, 15202, MPPT_STATE, 3);
    sb_put(&sb, ", \"charging_state\": "); jstate(&sb, 15203, CHARGE_STATE, 3);
    sb_put(&sb, ", \"battery_voltage\": "); jnum(&sb, sval(15206, 0.1), 1);
    sb_put(&sb, ", \"current\": "); jnum(&sb, sval(15207, 0.1), 1);
    sb_put(&sb, ", \"temperature\": "); jnumi(&sb, 15209);
    sb_put(&sb, ", \"relay_battery\": "); jbool(&sb, 15211);
    sb_put(&sb, ", \"relay_pv\": "); jbool(&sb, 15212);
    sb_put(&sb, "}, ");

    sb_put(&sb, "\"pv\": {");
    sb_put(&sb, "\"voltage\": "); jnum(&sb, sval(15205, 0.1), 1);
    sb_put(&sb, ", \"current\": "); jnum(&sb, sval(15207, 0.1), 1);
    sb_put(&sb, ", \"power\": "); jnum(&sb, sval(15208, 1), 1);
    sb_put(&sb, "}, ");

    sb_put(&sb, "\"energy_kwh\": {");
    sb_put(&sb, "\"pv\": "); jnum(&sb, energy_kwh(15217, 15218), 2);
    sb_put(&sb, ", \"grid_charge\": "); jnum(&sb, energy_kwh(25245, 25246), 2);
    sb_put(&sb, ", \"battery_discharge\": "); jnum(&sb, energy_kwh(25247, 25248), 2);
    sb_put(&sb, ", \"grid_buy\": "); jnum(&sb, energy_kwh(25249, 25250), 2);
    sb_put(&sb, ", \"grid_sell\": "); jnum(&sb, energy_kwh(25251, 25252), 2);
    sb_put(&sb, ", \"load\": "); jnum(&sb, energy_kwh(25253, 25254), 2);
    sb_put(&sb, ", \"self_use\": "); jnum(&sb, energy_kwh(25255, 25256), 2);
    sb_put(&sb, ", \"pv_sell\": "); jnum(&sb, energy_kwh(25257, 25258), 2);
    sb_put(&sb, "}, ");

    {
        BitPair errpairs[] = {
            {"inverter", 25261, TBL(INV_ERR1)},
            {"inverter", 25262, TBL(INV_ERR2)},
            {"charger", 15213, TBL(CHR_ERR1)},
        };
        BitPair warnpairs[] = {
            {"inverter", 25265, TBL(INV_WARN1)},
            {"charger", 15214, TBL(CHR_WARN1)},
        };
        sb_put(&sb, "\"errors\": ");
        jbits(&sb, errpairs, 3);
        sb_put(&sb, ", \"warnings\": ");
        jbits(&sb, warnpairs, 2);
    }

    sb_put(&sb, "}");
    return sb.s;
}

/* ---------------------------------------------------------------------------
 * MQTT publishing (optional, libpaho-mqtt)
 * --------------------------------------------------------------------------- */

typedef struct {
    MQTTAsync handle;
    char prefix[128];
    char online_topic[256];
} Mqtt;

static void mqtt_on_success(void *context, MQTTAsync_successData *response);
static void mqtt_on_failure(void *context, MQTTAsync_failureData *response);
static void mqtt_on_conn_lost(void *context, char *cause);

static int mqtt_init(Mqtt *m, const char *host, int port, const char *prefix,
                     const char *user, const char *pass)
{
    char addr[256], cid[64];
    MQTTAsync_connectOptions co = MQTTAsync_connectOptions_initializer;
    MQTTAsync_willOptions will = MQTTAsync_willOptions_initializer;
    int rc;

    snprintf(addr, sizeof(addr), "tcp://%s:%d", host, port);
    snprintf(cid, sizeof(cid), "pv18-monitor-%d", (int)getpid());
    snprintf(m->prefix, sizeof(m->prefix), "%s", prefix);
    snprintf(m->online_topic, sizeof(m->online_topic), "%s/online", prefix);

    rc = MQTTAsync_create(&m->handle, addr, cid, MQTTCLIENT_PERSISTENCE_NONE,
                          NULL);
    if (rc != MQTTASYNC_SUCCESS) {
        fprintf(stderr, "mqtt: create failed (rc=%d)\n", rc);
        return -1;
    }

    /* last will: retained "false" on the online topic */
    will.topicName = m->online_topic;
    will.message = "false";
    will.retained = 1;
    will.qos = 1;
    co.will = &will;

    co.username = user;
    co.password = pass;
    co.keepAliveInterval = 60;
    co.cleansession = 1;
    co.automaticReconnect = 1;
    co.minRetryInterval = 1;
    co.maxRetryInterval = 60;
    co.connectTimeout = 30;
    co.context = m;
    co.onSuccess = mqtt_on_success;
    co.onFailure = mqtt_on_failure;
    MQTTAsync_setConnectionLostCallback(m->handle, m, mqtt_on_conn_lost);

    rc = MQTTAsync_connect(m->handle, &co);
    if (rc != MQTTASYNC_SUCCESS) {
        fprintf(stderr, "mqtt: connect call failed (rc=%d)\n", rc);
        MQTTAsync_destroy(&m->handle);
        m->handle = NULL;
        return -1;
    }
    return 0;
}

static void mqtt_on_success(void *context, MQTTAsync_successData *response)
{
    Mqtt *m = (Mqtt *)context;
    (void)response;
    /* fires on every successful (re)connect: announce online */
    MQTTAsync_send(m->handle, m->online_topic, 5, "true", 1, 1, NULL);
}

static void mqtt_on_failure(void *context, MQTTAsync_failureData *response)
{
    (void)context;
    fprintf(stderr, "mqtt: broker connection failed (rc=%d), "
                    "will keep retrying\n", response ? response->code : -1);
}

static void mqtt_on_conn_lost(void *context, char *cause)
{
    (void)context;
    fprintf(stderr, "mqtt: lost broker connection (%s), "
                    "will keep retrying\n", cause ? cause : "unknown");
}

static void mqtt_send(Mqtt *m, const char *topic, const char *payload)
{
    MQTTAsync_send(m->handle, topic, (int)strlen(payload), payload, 1, 1, NULL);
}

static void mqtt_publish_snapshot(Mqtt *m)
{
    char *status = build_status_json();
    {
        char topic[256];
        snprintf(topic, sizeof(topic), "%s/status", m->prefix);
        mqtt_send(m, topic, status);
    }

    /* Individual topics (relative to the prefix). */
    {
        struct {
            const char *topic;
            /* value source */
            int kind;
            int addr;
            double scale;
            int signed_;
            int nd;
        } topics[] = {
            {"inverter/voltage", 0, 25206, 0.1, 0, 1},
            {"inverter/frequency", 0, 25225, 0.01, 0, 2},
            {"inverter/power", 0, 25213, 1, 0, 1},
            {"bus/voltage", 0, 25208, 0.1, 0, 1},
            {"grid/voltage", 0, 25207, 0.1, 0, 1},
            {"grid/frequency", 0, 25226, 0.01, 0, 2},
            {"grid/power", 0, 25214, 1, 1, 1},
            {"battery/voltage", 0, 25205, 0.1, 0, 1},
            {"battery/current", 0, 25274, 1, 1, 1},
            {"battery/power", 0, 25273, 1, 0, 1},
            {"battery/energy_discharged", 1, 25247, 0, 0, 2},
            {"pv/voltage", 0, 15205, 0.1, 0, 1},
            {"pv/current", 0, 15207, 0.1, 0, 1},
            {"pv/power", 0, 15208, 1, 0, 1},
            {"pv/energy", 1, 15217, 0, 0, 2},
            {"charger/temperature", 2, 15209, 0, 0, 0},
            {"relay/inverter", 3, 25237, 0, 0, 0},
            {"relay/grid", 3, 25238, 0, 0, 0},
            {"relay/load", 3, 25239, 0, 0, 0},
        };
        size_t i;
        for (i = 0; i < sizeof(topics) / sizeof(topics[0]); i++) {
            char topic[256];
            SB val;
            snprintf(topic, sizeof(topic), "%s/%s", m->prefix, topics[i].topic);
            sb_init(&val);
            switch (topics[i].kind) {
            case 0:
                if (topics[i].signed_)
                    jnum(&val, ssval(topics[i].addr, topics[i].scale),
                         topics[i].nd);
                else
                    jnum(&val, sval(topics[i].addr, topics[i].scale),
                         topics[i].nd);
                break;
            case 1:
                jnum(&val, energy_kwh(topics[i].addr, topics[i].addr + 1), 2);
                break;
            case 2:
                jnumi(&val, topics[i].addr);
                break;
            case 3:
                jbool(&val, topics[i].addr);
                break;
            }
            mqtt_send(m, topic, val.s);
            free(val.s);
        }
    }

    /* string-state topics */
    {
        struct { const char *topic; int addr; const char *const *tbl; int n; }
        states[] = {
            {"inverter/work_state", 25201, WORK_STATE, 7},
            {"charger/state", 15201, CHR_WORK, 4},
            {"charger/mppt", 15202, MPPT_STATE, 3},
        };
        size_t i;
        for (i = 0; i < sizeof(states) / sizeof(states[0]); i++) {
            char topic[256];
            SB v;
            snprintf(topic, sizeof(topic), "%s/%s", m->prefix, states[i].topic);
            sb_init(&v);
            jstate(&v, states[i].addr, states[i].tbl, states[i].n);
            mqtt_send(m, topic, v.s);
            free(v.s);
        }
    }

    /* errors / warnings */
    {
        BitPair errpairs[] = {
            {"inverter", 25261, TBL(INV_ERR1)},
            {"inverter", 25262, TBL(INV_ERR2)},
            {"charger", 15213, TBL(CHR_ERR1)},
        };
        BitPair warnpairs[] = {
            {"inverter", 25265, TBL(INV_WARN1)},
            {"charger", 15214, TBL(CHR_WARN1)},
        };
        char topic[256];
        SB v;
        snprintf(topic, sizeof(topic), "%s/errors", m->prefix);
        sb_init(&v);
        jbits(&v, errpairs, 3);
        mqtt_send(m, topic, v.s);
        free(v.s);
        snprintf(topic, sizeof(topic), "%s/warnings", m->prefix);
        sb_init(&v);
        jbits(&v, warnpairs, 2);
        mqtt_send(m, topic, v.s);
        free(v.s);
    }

    free(status);
}

static void mqtt_close(Mqtt *m)
{
    if (m->handle) {
        /* wait for queued messages to be delivered (bounded, like paho's
         * loop_stop) so single-shot mode does not lose the last snapshot */
        int i;
        for (i = 0; i < 50; i++) { /* up to 5 s */
            MQTTAsync_token *tokens = NULL;
            if (MQTTAsync_getPendingTokens(m->handle, &tokens) ==
                MQTTASYNC_SUCCESS) {
                if (tokens == NULL)
                    break;
                MQTTAsync_free(tokens);
            }
            usleep(100000);
        }
        MQTTAsync_disconnectOptions dco = MQTTAsync_disconnectOptions_initializer;
        MQTTAsync_disconnect(m->handle, &dco);
        MQTTAsync_destroy(&m->handle);
        m->handle = NULL;
    }
}

/* ---------------------------------------------------------------------------
 * Status bar (bottom of the TUI)
 * --------------------------------------------------------------------------- */

/* Display width of a UTF-8 string (each character counts as 1). */
static int disp_width(const char *s)
{
    int w = 0;
    const unsigned char *p = (const unsigned char *)s;
    while (*p) {
        if (*p < 0x80)
            p += 1;
        else if ((*p & 0xE0) == 0xC0)
            p += 2;
        else if ((*p & 0xF0) == 0xE0)
            p += 3;
        else if ((*p & 0xF8) == 0xF0)
            p += 4;
        else
            p += 1;
        w++;
    }
    return w;
}

/* Truncate *s in place so its display width is at most w. */
static void cut_width(char *s, int w)
{
    int cw = disp_width(s);
    if (cw <= w)
        return;
    const unsigned char *p = (const unsigned char *)s;
    int n = 0;
    while (*p && n < w) {
        p += (*p < 0x80) ? 1 : ((*p & 0xE0) == 0xC0) ? 2
             : ((*p & 0xF0) == 0xE0) ? 3 : 4;
        n++;
    }
    s[p - (const unsigned char *)s] = '\0';
}

/* Version string for the git checkout containing *dir*:
 * "<commit date as YYTMMDDHHMM> (<short hash>)" — e.g. "26T08280725 (c159076)".
 * A trailing "*" on the hash marks uncommitted changes.  Returns "unknown"
 * when the file is not inside a git repository.
 */
static void git_version(const char *dir, char *out, size_t sz)
{
    char cmd[8192], buf[32];
    FILE *f;
    snprintf(out, sz, "unknown");
    snprintf(cmd, sizeof(cmd),
             "git -C '%s' log -1 --date=format:%%yT%%m%%d%%H%%M "
             "--format='%%cd|%%h' 2>/dev/null", dir);
    f = popen(cmd, "r");
    if (!f)
        return;
    if (!fgets(buf, sizeof(buf), f)) {
        pclose(f);
        return;
    }
    pclose(f);
    {
        char *nl = strchr(buf, '\n');
        char *bar;
        char date[32], hash[32];
        if (nl)
            *nl = '\0';
        bar = strchr(buf, '|');
        if (!bar)
            return;
        *bar = '\0';
        snprintf(date, sizeof(date), "%s", buf);
        snprintf(hash, sizeof(hash), "%s", bar + 1);
        snprintf(cmd, sizeof(cmd),
                 "git -C '%s' status --porcelain 2>/dev/null", dir);
        f = popen(cmd, "r");
        if (f) {
            if (fgets(buf, sizeof(buf), f)) {
                size_t hl = strlen(hash);
                if (hl + 2 < sizeof(hash)) {
                    hash[hl] = '*';
                    hash[hl + 1] = '\0';
                }
            }
            pclose(f);
        }
        snprintf(out, sz, "%s (%s)", date, hash);
    }
}

typedef struct {
    const char *port;
    int baud;
    int slave;
    double timeout;
    double interval;
    int retries;
    int snapshot_passes;
    int watch;
    const char *mqtt_host;
    int mqtt_port;
    const char *mqtt_user;
    const char *mqtt_password;
    const char *mqtt_prefix;
} Args;

static void status_bar(char *out, size_t sz, const char *version, const Args *a)
{
    SB sb;
    int width, room;
    char part[256];

    sb_init(&sb);
    sb_printf(&sb, "PV18 %s \342\224\202 %s slave %d %d",
              version, a->port, a->slave, a->baud);
    if (a->watch)
        sb_printf(&sb, " \342\224\202 interval %.1fs", a->interval);
    else
        sb_put(&sb, " \342\224\202 snapshot");
    if (a->mqtt_host) {
        snprintf(part, sizeof(part), "mqtt %s:%d", a->mqtt_host, a->mqtt_port);
        sb_printf(&sb, " \342\224\202 %s", part);
        if (strcmp(a->mqtt_prefix, "PV18") != 0)
            sb_printf(&sb, " [%s]", a->mqtt_prefix);
    } else {
        sb_put(&sb, " \342\224\202 mqtt off");
    }
    /* bar = "── " + body, padded with '─' to the terminal width */
    {
        char body[1024];
        snprintf(body, sizeof(body), "%s", sb.s);
        snprintf(out, sz, "\342\200\224\342\200\224 %s", body);
    }
    free(sb.s);

    {
        struct winsize ws;
        if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0)
            width = ws.ws_col;
        else
            width = 80;
    }
    room = width - disp_width(out);
    if (room < 0) {
        cut_width(out, width);
        return;
    }
    if (room <= 2)
        return;
    {
        int i;
        SB pad;
        sb_init(&pad);
        for (i = 0; i < room - 2; i++)
            sb_put(&pad, "\342\200\224");
        sb_printf(&pad, "\342\200\224\342\200\224");
        strncat(out, pad.s, sz - strlen(out) - 1);
        free(pad.s);
    }
}

/* Pad *text* with blank lines so *bar* lands on the last terminal row. */
static void frame_with_bar(SB *out, const char *text, const char *bar,
                           int height)
{
    int nlines = 0, blank, i;
    size_t len;

    /* rstrip trailing newlines */
    len = strlen(text);
    while (len && text[len - 1] == '\n')
        len--;
    for (i = 0; i < (int)len; i++)
        if (text[i] == '\n')
            nlines++;
    if (len)
        nlines++; /* last non-empty line */

    sb_init(out);
    sb_put(out, text); /* trailing newlines will be replaced below */
    {
        size_t l2 = strlen(out->s);
        while (l2 && out->s[l2 - 1] == '\n')
            out->s[--l2] = '\0';
        out->len = l2;
    }
    blank = height - nlines - 1;
    if (blank < 0)
        blank = 0;
    for (i = 0; i < blank; i++)
        sb_put(out, "\n");
    sb_put(out, "\n");
    sb_put(out, bar);
}

static int term_height(void)
{
    struct winsize ws;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_row > 0)
        return ws.ws_row;
    return 24;
}

/* Print a report frame; in TTY watch mode the status bar (version, commit
 * hash, settings) is pinned to the bottom row. */
static void show_frame(SB *text, int is_tty, const char *version, const Args *a)
{
    char bar[2048];
    status_bar(bar, sizeof(bar), version, a);
    if (is_tty && a->watch) {
        SB frame;
        frame_with_bar(&frame, text->s, bar, term_height());
        printf("\x1b[2J\x1b[H%s\n", frame.s);
        free(frame.s);
    } else {
        printf("%s\n%s\n", text->s, bar);
    }
}

/* ---------------------------------------------------------------------------
 * Argument parsing
 * --------------------------------------------------------------------------- */

static void usage(FILE *f, const char *prog)
{
    fprintf(f,
        "MustPower PV1800 (18-series) RS-485 Modbus RTU status monitor\n"
        "\n"
        "Usage: %s [options]\n"
        "\n"
        "Options:\n"
        "  --port PATH          serial device (default %s)\n"
        "  --baud N             baud rate (default %d)\n"
        "  --slave N            Modbus slave address (default %d)\n"
        "  --timeout S          response timeout in seconds (default 0.6)\n"
        "  --interval S         refresh interval in seconds for --watch "
        "(default 2.0)\n"
        "  --retries N          attempts per register block (default 3)\n"
        "  --snapshot-passes N  single-shot: max read passes until all blocks "
        "have been read (default 3)\n"
        "  --mqtt-host HOST     MQTT broker host; enables MQTT publishing\n"
        "  --mqtt-port N        MQTT broker port (default 1883)\n"
        "  --mqtt-user USER     MQTT username (optional)\n"
        "  --mqtt-password PW   MQTT password (optional)\n"
        "  --mqtt-prefix STR    MQTT topic prefix (default PV18)\n"
        "  --watch              refresh continuously instead of a single "
        "snapshot\n"
        "  --help               show this help\n",
        prog, DEFAULT_PORT, DEFAULT_BAUD, DEFAULT_SLAVE);
}

static int parse_args(int argc, char **argv, Args *a, PV1800 *pv)
{
    int i;
    memset(a, 0, sizeof(*a));
    a->port = DEFAULT_PORT;
    a->baud = DEFAULT_BAUD;
    a->slave = DEFAULT_SLAVE;
    a->timeout = 0.6;
    a->interval = 2.0;
    a->retries = 3;
    a->snapshot_passes = 3;
    a->mqtt_port = 1883;
    a->mqtt_prefix = "PV18";
    a->watch = 0;

    for (i = 1; i < argc; i++) {
        const char *arg = argv[i];
        if (!strcmp(arg, "--help") || !strcmp(arg, "-h")) {
            usage(stdout, argv[0]);
            exit(0);
        } else if (!strcmp(arg, "--port") && i + 1 < argc) {
            a->port = argv[++i];
        } else if (!strcmp(arg, "--baud") && i + 1 < argc) {
            a->baud = atoi(argv[++i]);
        } else if (!strcmp(arg, "--slave") && i + 1 < argc) {
            a->slave = atoi(argv[++i]);
        } else if (!strcmp(arg, "--timeout") && i + 1 < argc) {
            a->timeout = atof(argv[++i]);
        } else if (!strcmp(arg, "--interval") && i + 1 < argc) {
            a->interval = atof(argv[++i]);
        } else if (!strcmp(arg, "--retries") && i + 1 < argc) {
            a->retries = atoi(argv[++i]);
        } else if (!strcmp(arg, "--snapshot-passes") && i + 1 < argc) {
            a->snapshot_passes = atoi(argv[++i]);
        } else if (!strcmp(arg, "--mqtt-host") && i + 1 < argc) {
            a->mqtt_host = argv[++i];
        } else if (!strcmp(arg, "--mqtt-port") && i + 1 < argc) {
            a->mqtt_port = atoi(argv[++i]);
        } else if (!strcmp(arg, "--mqtt-user") && i + 1 < argc) {
            a->mqtt_user = argv[++i];
        } else if (!strcmp(arg, "--mqtt-password") && i + 1 < argc) {
            a->mqtt_password = argv[++i];
        } else if (!strcmp(arg, "--mqtt-prefix") && i + 1 < argc) {
            a->mqtt_prefix = argv[++i];
        } else if (!strcmp(arg, "--watch")) {
            a->watch = 1;
        } else {
            fprintf(stderr, "error: unknown argument: %s\n", arg);
            usage(stderr, argv[0]);
            exit(2);
        }
    }

    pv->device = a->port;
    pv->baud = a->baud;
    pv->slave = a->slave;
    pv->settle = 0.1;
    pv->timeout = a->timeout;
    pv->retries = a->retries;
    pv->retry_pause = 1.5;
    pv->fd = -1;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Block reading (multi-block pass)
 * --------------------------------------------------------------------------- */

/* Read the given blocks.
 * Returns the number of newly read registers; per-block errors go into
 * errstr[] (keyed by block index). A block that fails after all retries is
 * reported in the error dict; the other blocks are still read.
 */
static int read_blocks(PV1800 *pv, const int *idx, int nidx,
                       char errstr[][256], int *nregs_out)
{
    int nnew = 0;
    int i;
    for (i = 0; i < nidx; i++) {
        int bi = idx[i];
        int n, v[MAX_BLOCK_REGS];
        if (i)
            sleep_s(0.5); /* let the bus settle between opens */
        if (pv_read_block(pv, BLOCKS[bi].start, BLOCKS[bi].count, &n, v,
                          errstr[bi], 256) == 0) {
            int k;
            for (k = 0; k < n; k++)
                reg_set(BLOCKS[bi].start + k, v[k]);
            nnew += n;
            errstr[bi][0] = '\0';
        }
    }
    *nregs_out = nnew;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Main
 * --------------------------------------------------------------------------- */

static void on_sigint(int sig)
{
    (void)sig;
    g_interrupt = 1;
}

int main(int argc, char **argv)
{
    Args args;
    PV1800 pv;
    Mqtt mqtt;
    int mqtt_on = 0;
    int is_tty;
    char version[128];
    char progdir[4096];
    int pending = (1 << N_BLOCKS) - 1;
    int passes = 0;
    char *last_good = NULL;
    char errstr[N_BLOCKS][256];
    int exit_code = 0;

    signal(SIGINT, on_sigint);
    parse_args(argc, argv, &args, &pv);

    if (args.mqtt_host) {
        if (mqtt_init(&mqtt, args.mqtt_host, args.mqtt_port,
                      args.mqtt_prefix, args.mqtt_user, args.mqtt_password)
            == 0)
            mqtt_on = 1;
        else
            fprintf(stderr,
                    "warning: mqtt: init failed (continuing without MQTT)\n");
    }

    is_tty = isatty(STDOUT_FILENO);
    {
        const char *slash = strrchr(argv[0], '/');
        if (slash)
            snprintf(progdir, sizeof(progdir), "%.*s",
                     (int)(slash - argv[0]), argv[0]);
        else
            snprintf(progdir, sizeof(progdir), ".");
    }
    git_version(progdir, version, sizeof(version));

    while (!g_interrupt) {
        int idx[N_BLOCKS], nidx = 0;
        int nnew = 0, i;

        for (i = 0; i < N_BLOCKS; i++) {
            if (args.watch) {
                if (is_status_block(i) || (pending & (1 << i)))
                    idx[nidx++] = i;
            } else if (pending & (1 << i)) {
                idx[nidx++] = i;
            }
        }
        memset(errstr, 0, sizeof(errstr));
        read_blocks(&pv, idx, nidx, errstr, &nnew);
        for (i = 0; i < nidx; i++)
            if (errstr[idx[i]][0] == '\0')
                pending &= ~(1 << idx[i]);

        if (nnew == 0) {
            /* all requested blocks failed — communication error */
            SB msg;
            sb_init(&msg);
            {
                int first = 1;
                for (i = 0; i < nidx; i++) {
                    if (errstr[idx[i]][0]) {
                        if (!first)
                            sb_put(&msg, "; ");
                        sb_printf(&msg, "%s: %s", BLOCKS[idx[i]].name,
                                  errstr[idx[i]]);
                        first = 0;
                    }
                }
                if (first)
                    sb_printf(&msg, "no response from slave %d", pv.slave);
            }
            if (args.watch) {
                SB out;
                sb_init(&out);
                if (last_good) {
                    sb_printf(&out, "%s\n  !! communication error: %s "
                                   "(keeping last snapshot)",
                              last_good, msg.s);
                    show_frame(&out, is_tty, version, &args);
                } else {
                    fprintf(stderr, "  !! communication error: %s\n", msg.s);
                }
                free(out.s);
            } else if (passes + 1 >= args.snapshot_passes) {
                fprintf(stderr, "error: %s\n", msg.s);
                if (mqtt_on)
                    mqtt_close(&mqtt);
                free(msg.s);
                free(last_good);
                return 1;
            }
            free(msg.s);
        } else {
            SB out;
            sb_init(&out);
            render(&out);
            for (i = 0; i < nidx; i++) {
                if (errstr[idx[i]][0])
                    sb_printf(&out, "\n  !! %s read failed (stale value "
                                       "kept): %s",
                              BLOCKS[idx[i]].name, errstr[idx[i]]);
            }
            if (mqtt_on)
                mqtt_publish_snapshot(&mqtt);
            show_frame(&out, is_tty, version, &args);
            free(last_good);
            last_good = strdup(out.s);
            free(out.s);
        }

        fflush(stdout);
        if (!args.watch) {
            passes++;
            if (!pending || passes >= args.snapshot_passes)
                break;
            sleep_s_int(1.0); /* let the inverter recover between passes */
        } else {
            sleep_s_int(args.interval);
        }
    }

    if (mqtt_on)
        mqtt_close(&mqtt);
    free(last_good);
    return exit_code;
}
