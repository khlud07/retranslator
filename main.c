#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <iio.h>

#define URI      "ip:192.168.2.1"
#define BUF_SIZE 4096
#define TIMEOUT_MS 5000

/* ── graceful shutdown ─────────────────────────────────────────────────── */
static volatile sig_atomic_t running = 1;

static void handle_signal(int sig)
{
    (void)sig;
    running = 0;
}

/* ── helpers ───────────────────────────────────────────────────────────── */
static struct iio_channel *get_channel(struct iio_device *dev,
                                       const char *name, bool output)
{
    struct iio_channel *ch = iio_device_find_channel(dev, name, output);
    if (!ch)
        fprintf(stderr, "Channel '%s' (output=%d) not found\n", name, output);
    return ch;
}

/* ── main ──────────────────────────────────────────────────────────────── */
int main(void)
{
    int ret = -1;

    struct iio_context *ctx   = NULL;
    struct iio_device  *rxdev = NULL;
    struct iio_device  *txdev = NULL;
    struct iio_channel *rx_i  = NULL, *rx_q = NULL;
    struct iio_channel *tx_i  = NULL, *tx_q = NULL;
    struct iio_buffer  *rxbuf = NULL, *txbuf = NULL;

    /* ── signal handling ── */
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    /* ── context ── */
    printf("Connecting to PlutoSDR at %s ...\n", URI);
    ctx = iio_create_context_from_uri(URI);
    if (!ctx) {
        perror("Context creation failed");
        goto cleanup;
    }
    iio_context_set_timeout(ctx, TIMEOUT_MS);

    /* ── devices ── */
    rxdev = iio_context_find_device(ctx, "cf-ad9361-lpc");
    txdev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    if (!rxdev || !txdev) {
        fprintf(stderr, "RX or TX device not found\n");
        goto cleanup;
    }

    /* ── channels ── */
    rx_i = get_channel(rxdev, "voltage0", false);
    rx_q = get_channel(rxdev, "voltage1", false);
    tx_i = get_channel(txdev, "voltage0", true);
    tx_q = get_channel(txdev, "voltage1", true);
    if (!rx_i || !rx_q || !tx_i || !tx_q)
        goto cleanup;

    iio_channel_enable(rx_i);
    iio_channel_enable(rx_q);
    iio_channel_enable(tx_i);
    iio_channel_enable(tx_q);

    /* ── buffers ── */
    rxbuf = iio_device_create_buffer(rxdev, BUF_SIZE, false);
    txbuf = iio_device_create_buffer(txdev, BUF_SIZE, false);
    if (!rxbuf || !txbuf) {
        fprintf(stderr, "Buffer creation failed\n");
        goto cleanup;
    }

    /* ── retranslator loop ── */
    printf("Running retranslator (Ctrl-C to stop)...\n");

    while (running) {

        /* fill RX buffer from radio */
        ssize_t rx_bytes = iio_buffer_refill(rxbuf);
        if (rx_bytes < 0) {
            fprintf(stderr, "RX refill error: %s\n", strerror(-(int)rx_bytes));
            break;
        }

        /* derive per-buffer geometry */
        char     *rx_ptr  = (char *)iio_buffer_first(rxbuf, rx_i);
        char     *rx_end  = (char *)iio_buffer_end(rxbuf);
        ptrdiff_t rx_step = iio_buffer_step(rxbuf);

        char     *tx_ptr  = (char *)iio_buffer_first(txbuf, tx_i);
        char     *tx_end  = (char *)iio_buffer_end(txbuf);
        ptrdiff_t tx_step = iio_buffer_step(txbuf);

        /* copy samples — respect BOTH buffer bounds */
        while (rx_ptr < rx_end && tx_ptr < tx_end) {
            ((int16_t *)tx_ptr)[0] = ((int16_t *)rx_ptr)[0]; /* I */
            ((int16_t *)tx_ptr)[1] = ((int16_t *)rx_ptr)[1]; /* Q */
            rx_ptr += rx_step;
            tx_ptr += tx_step;
        }

        /* push TX buffer to radio */
        ssize_t tx_bytes = iio_buffer_push(txbuf);
        if (tx_bytes < 0) {
            fprintf(stderr, "TX push error: %s\n", strerror(-(int)tx_bytes));
            break;
        }
    }

    printf("\nShutting down...\n");
    ret = 0;

cleanup:
    if (rxbuf) iio_buffer_destroy(rxbuf);
    if (txbuf) iio_buffer_destroy(txbuf);
    if (ctx)   iio_context_destroy(ctx);
    return ret;
}
