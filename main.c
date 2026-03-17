/*
 * PlutoSDR IQ Retranslator
 * Receives IQ samples on RX and retransmits them on TX.
 *
 * Usage:
 *   ./retranslator [options]
 *   -u  URI             Device URI         (default: ip:192.168.2.1)
 *   -f  RX_FREQ_HZ      RX LO freq Hz      (default: 433000000)
 *   -x  TX_FREQ_HZ      TX LO freq Hz      (default: 868000000)
 *   -r  SAMP_RATE_HZ    Sample rate Hz     (default: 2500000)
 *   -b  BANDWIDTH_HZ    RF bandwidth Hz    (default: 2000000)
 *   -t  TX_ATTEN_DB     TX attenuation dB  (default: -10.0, range: 0.0 to -89.75)
 *   -s  BUF_SIZE        Buffer size        (default: 4096)
 *   -g  GAIN_MODE       RX gain mode       (default: slow_attack)
 *   -h                  Show this help
 *
 * Build:
 *   gcc -Wall -Wextra -O2 retranslator.c -o retranslator -liio
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <syslog.h>
#include <iio.h>

/* ── defaults ──────────────────────────────────────────────────────────── */
#define DEFAULT_URI         "ip:192.168.2.1"
#define DEFAULT_RX_FREQ     433000000LL      /* Hz  */
#define DEFAULT_TX_FREQ     868000000LL      /* Hz  */
#define DEFAULT_SAMP_RATE   2500000LL        /* SPS */
#define DEFAULT_BANDWIDTH   2000000LL        /* Hz  */
#define DEFAULT_TX_ATTEN_DB  -10.0           /* dB, 0.0 = max power, -89.75 = min power */
#define DEFAULT_BUF_SIZE    4096
#define DEFAULT_GAIN_MODE   "slow_attack"
#define TIMEOUT_MS          5000
#define RECONNECT_DELAY_S   5
#define STATS_INTERVAL_S    60

/* ── runtime config ────────────────────────────────────────────────────── */
typedef struct {
    char        uri[256];
    long long   rx_freq;
    long long   tx_freq;
    long long   samp_rate;
    long long   bandwidth;
    double      tx_atten_db;
    size_t      buf_size;
    char        gain_mode[32];
} config_t;

/* ── statistics ────────────────────────────────────────────────────────── */
typedef struct {
    uint64_t rx_buffers;
    uint64_t tx_errors;
    uint64_t rx_errors;
    time_t   start_time;
    time_t   last_stat_time;
} stats_t;

/* ── graceful shutdown ─────────────────────────────────────────────────── */
static volatile sig_atomic_t running = 1;

static void handle_signal(int sig)
{
    (void)sig;
    running = 0;
}

/* ── logging ───────────────────────────────────────────────────────────── */
#define LOG_I(fmt, ...) do { \
    syslog(LOG_INFO,    fmt, ##__VA_ARGS__); \
    printf ("[INFO]  " fmt "\n", ##__VA_ARGS__); \
} while (0)

#define LOG_W(fmt, ...) do { \
    syslog(LOG_WARNING, fmt, ##__VA_ARGS__); \
    fprintf(stderr, "[WARN]  " fmt "\n", ##__VA_ARGS__); \
} while (0)

#define LOG_E(fmt, ...) do { \
    syslog(LOG_ERR,     fmt, ##__VA_ARGS__); \
    fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__); \
} while (0)

/* ── helpers ───────────────────────────────────────────────────────────── */
static struct iio_channel *require_channel(struct iio_device *dev,
                                           const char *name, bool output)
{
    struct iio_channel *ch = iio_device_find_channel(dev, name, output);
    if (!ch)
        LOG_E("Channel '%s' (output=%d) not found", name, (int)output);
    return ch;
}

static int phy_write_ll(struct iio_device *phy,
                        const char *ch_name, bool output,
                        const char *attr, long long value)
{
    struct iio_channel *ch = iio_device_find_channel(phy, ch_name, output);
    if (!ch) {
        LOG_E("PHY channel '%s' not found", ch_name);
        return -1;
    }
    int ret = iio_channel_attr_write_longlong(ch, attr, value);
    if (ret < 0)
        LOG_E("Failed to write %s/%s = %lld: %s", ch_name, attr, value, strerror(-ret));
    return ret;
}

/* Write a long long as a string — more compatible across firmware versions */
static int phy_write_ll_str(struct iio_device *phy,
                             const char *ch_name, bool output,
                             const char *attr, long long value)
{
    struct iio_channel *ch = iio_device_find_channel(phy, ch_name, output);
    if (!ch) {
        LOG_E("PHY channel '%s' not found", ch_name);
        return -1;
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "%lld", value);
    int ret = iio_channel_attr_write(ch, attr, buf);
    if (ret < 0)
        LOG_E("Failed to write %s/%s = %s: %s", ch_name, attr, buf, strerror(-ret));
    return ret;
}

static int phy_write_str(struct iio_device *phy,
                         const char *ch_name, bool output,
                         const char *attr, const char *value)
{
    struct iio_channel *ch = iio_device_find_channel(phy, ch_name, output);
    if (!ch) {
        LOG_E("PHY channel '%s' not found", ch_name);
        return -1;
    }
    int ret = iio_channel_attr_write(ch, attr, value);
    if (ret < 0)
        LOG_E("Failed to write %s/%s = %s: %s", ch_name, attr, value, strerror(-ret));
    return ret;
}

/* ── PHY configuration ─────────────────────────────────────────────────── */
static int configure_phy(struct iio_context *ctx, const config_t *cfg)
{
    struct iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    if (!phy) {
        LOG_E("ad9361-phy device not found");
        return -1;
    }

    LOG_I("Configuring AD9361 PHY...");
    LOG_I("  RX frequency: %lld Hz", cfg->rx_freq);
    LOG_I("  TX frequency: %lld Hz", cfg->tx_freq);
    LOG_I("  Sample rate : %lld SPS", cfg->samp_rate);
    LOG_I("  Bandwidth   : %lld Hz", cfg->bandwidth);
    LOG_I("  TX atten    : %.2f dB", cfg->tx_atten_db);
    LOG_I("  RX mode     : %s", cfg->gain_mode);

    /* sample rate — affects both RX and TX */
    if (phy_write_ll_str(phy, "voltage0", false, "sampling_frequency", cfg->samp_rate) < 0)
        return -1;

    /* RX LO frequency */
    if (phy_write_ll_str(phy, "altvoltage0", true, "frequency", cfg->rx_freq) < 0)
        return -1;

    /* TX LO frequency */
    if (phy_write_ll_str(phy, "altvoltage1", true, "frequency", cfg->tx_freq) < 0)
        return -1;

    /* RX RF bandwidth */
    if (phy_write_ll_str(phy, "voltage0", false, "rf_bandwidth", cfg->bandwidth) < 0)
        return -1;

    /* TX RF bandwidth */
    if (phy_write_ll_str(phy, "voltage0", true, "rf_bandwidth", cfg->bandwidth) < 0)
        return -1;

    /* RX gain control mode */
    if (phy_write_str(phy, "voltage0", false, "gain_control_mode", cfg->gain_mode) < 0)
        return -1;

    /* TX attenuation: 0.0 = max TX power, -89.75 = min TX power (dB, written as double) */
    {
        struct iio_channel *ch = iio_device_find_channel(phy, "voltage0", true);
        if (!ch) {
            LOG_E("PHY TX voltage0 channel not found");
            return -1;
        }
        int ret = iio_channel_attr_write_double(ch, "hardwaregain", cfg->tx_atten_db);
        if (ret < 0) {
            LOG_E("Failed to write TX hardwaregain = %.2f dB: %s",
                  cfg->tx_atten_db, strerror(-ret));
            return -1;
        }
    }

    LOG_I("PHY configuration done");
    return 0;
}

/* ── statistics reporter ───────────────────────────────────────────────── */
static void maybe_report_stats(stats_t *st)
{
    time_t now = time(NULL);
    if ((now - st->last_stat_time) < STATS_INTERVAL_S)
        return;

    double uptime = difftime(now, st->start_time);
    LOG_I("Stats | uptime=%.0fs rx_buffers=%lu rx_errors=%lu tx_errors=%lu",
          uptime, (unsigned long)st->rx_buffers,
          (unsigned long)st->rx_errors,
          (unsigned long)st->tx_errors);

    st->last_stat_time = now;
}

/* ── retranslator core ─────────────────────────────────────────────────── */
static int run_retranslator(struct iio_context *ctx,
                             const config_t *cfg,
                             stats_t *st)
{
    int ret = -1;

    struct iio_device  *rxdev = NULL, *txdev = NULL;
    struct iio_channel *rx_i  = NULL, *rx_q  = NULL;
    struct iio_channel *tx_i  = NULL, *tx_q  = NULL;
    struct iio_buffer  *rxbuf = NULL, *txbuf = NULL;

    /* devices */
    rxdev = iio_context_find_device(ctx, "cf-ad9361-lpc");
    txdev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    if (!rxdev || !txdev) {
        LOG_E("RX or TX streaming device not found");
        goto done;
    }

    /* channels */
    rx_i = require_channel(rxdev, "voltage0", false);
    rx_q = require_channel(rxdev, "voltage1", false);
    tx_i = require_channel(txdev, "voltage0", true);
    tx_q = require_channel(txdev, "voltage1", true);
    if (!rx_i || !rx_q || !tx_i || !tx_q)
        goto done;

    iio_channel_enable(rx_i);
    iio_channel_enable(rx_q);
    iio_channel_enable(tx_i);
    iio_channel_enable(tx_q);

    /* buffers */
    rxbuf = iio_device_create_buffer(rxdev, cfg->buf_size, false);
    txbuf = iio_device_create_buffer(txdev, cfg->buf_size, false);
    if (!rxbuf || !txbuf) {
        LOG_E("Buffer creation failed: %s", strerror(errno));
        goto done;
    }

    LOG_I("Retranslator running (buf_size=%zu)", cfg->buf_size);

    while (running) {

        /* refill RX */
        ssize_t rx_bytes = iio_buffer_refill(rxbuf);
        if (rx_bytes < 0) {
            LOG_E("RX refill error: %s", strerror(-(int)rx_bytes));
            st->rx_errors++;
            break;
        }
        st->rx_buffers++;

        /* copy samples */
        char     *rx_ptr  = (char *)iio_buffer_first(rxbuf, rx_i);
        char     *rx_end  = (char *)iio_buffer_end(rxbuf);
        ptrdiff_t rx_step = iio_buffer_step(rxbuf);

        char     *tx_ptr  = (char *)iio_buffer_first(txbuf, tx_i);
        char     *tx_end  = (char *)iio_buffer_end(txbuf);
        ptrdiff_t tx_step = iio_buffer_step(txbuf);

        while (rx_ptr < rx_end && tx_ptr < tx_end) {
            ((int16_t *)tx_ptr)[0] = ((int16_t *)rx_ptr)[0]; /* I */
            ((int16_t *)tx_ptr)[1] = ((int16_t *)rx_ptr)[1]; /* Q */
            rx_ptr += rx_step;
            tx_ptr += tx_step;
        }

        /* push TX */
        ssize_t tx_bytes = iio_buffer_push(txbuf);
        if (tx_bytes < 0) {
            LOG_E("TX push error: %s", strerror(-(int)tx_bytes));
            st->tx_errors++;
            break;
        }

        maybe_report_stats(st);
    }

    ret = 0;

done:
    if (rxbuf) iio_buffer_destroy(rxbuf);
    if (txbuf) iio_buffer_destroy(txbuf);
    return ret;
}

/* ── usage ─────────────────────────────────────────────────────────────── */
static void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n"
           "  -u URI           Device URI         (default: %s)\n"
           "  -f RX_FREQ_HZ    RX LO freq Hz      (default: %lld)\n"
           "  -x TX_FREQ_HZ    TX LO freq Hz      (default: %lld)\n"
           "  -r SAMP_RATE_HZ  Sample rate Hz     (default: %lld)\n"
           "  -b BANDWIDTH_HZ  RF bandwidth Hz    (default: %lld)\n"
           "  -t TX_ATTEN_DB   TX attenuation dB  (default: %.1f, range: 0.0 to -89.75)\n"
           "  -s BUF_SIZE      Buffer size        (default: %d)\n"
           "  -g GAIN_MODE     RX gain mode       (default: %s)\n"
           "  -h               Show this help\n",
           prog,
           DEFAULT_URI, DEFAULT_RX_FREQ, DEFAULT_TX_FREQ, DEFAULT_SAMP_RATE,
           DEFAULT_BANDWIDTH, DEFAULT_TX_ATTEN_DB,
           DEFAULT_BUF_SIZE, DEFAULT_GAIN_MODE);
}

/* ── main ──────────────────────────────────────────────────────────────── */
int main(int argc, char *argv[])
{
    /* default config */
    config_t cfg = {
        .uri       = DEFAULT_URI,
        .rx_freq   = DEFAULT_RX_FREQ,
        .tx_freq   = DEFAULT_TX_FREQ,
        .samp_rate = DEFAULT_SAMP_RATE,
        .bandwidth = DEFAULT_BANDWIDTH,
        .tx_atten_db = DEFAULT_TX_ATTEN_DB,
        .buf_size  = DEFAULT_BUF_SIZE,
        .gain_mode = DEFAULT_GAIN_MODE,
    };

    /* parse CLI args */
    int opt;
    while ((opt = getopt(argc, argv, "u:f:x:r:b:t:s:g:h")) != -1) {
        switch (opt) {
        case 'u': strncpy(cfg.uri,       optarg, sizeof(cfg.uri) - 1);       break;
        case 'f': cfg.rx_freq   = atoll(optarg);                              break;
        case 'x': cfg.tx_freq   = atoll(optarg);                              break;
        case 'r': cfg.samp_rate = atoll(optarg);                              break;
        case 'b': cfg.bandwidth = atoll(optarg);                              break;
        case 't': cfg.tx_atten_db = atof(optarg);                              break;
        case 's': cfg.buf_size  = (size_t)atoi(optarg);                       break;
        case 'g': strncpy(cfg.gain_mode, optarg, sizeof(cfg.gain_mode) - 1); break;
        case 'h': print_usage(argv[0]); return 0;
        default:  print_usage(argv[0]); return 1;
        }
    }

    /* signal handlers */
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    /* syslog */
    openlog("retranslator", LOG_PID | LOG_CONS, LOG_USER);
    LOG_I("PlutoSDR retranslator starting");
    LOG_I("URI: %s", cfg.uri);

    /* stats */
    stats_t st = {
        .start_time     = time(NULL),
        .last_stat_time = time(NULL),
    };

    /* reconnection loop */
    while (running) {

        LOG_I("Connecting to %s ...", cfg.uri);
        struct iio_context *ctx = iio_create_context_from_uri(cfg.uri);
        if (!ctx) {
            LOG_W("Connection failed, retrying in %ds...", RECONNECT_DELAY_S);
            sleep(RECONNECT_DELAY_S);
            continue;
        }

        iio_context_set_timeout(ctx, TIMEOUT_MS);
        LOG_I("Connected");

        /* configure PHY */
        if (configure_phy(ctx, &cfg) < 0) {
            LOG_E("PHY configuration failed");
            iio_context_destroy(ctx);
            sleep(RECONNECT_DELAY_S);
            continue;
        }

        /* run until error or signal */
        run_retranslator(ctx, &cfg, &st);
        iio_context_destroy(ctx);

        if (running) {
            LOG_W("Disconnected. Reconnecting in %ds...", RECONNECT_DELAY_S);
            sleep(RECONNECT_DELAY_S);
        }
    }

    LOG_I("Shutdown complete. rx_buffers=%lu rx_errors=%lu tx_errors=%lu",
          (unsigned long)st.rx_buffers,
          (unsigned long)st.rx_errors,
          (unsigned long)st.tx_errors);

    closelog();
    return 0;
}
