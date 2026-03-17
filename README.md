# PlutoSDR IQ Retranslator

Receives IQ samples on the AD9361 RX path and immediately retransmits them on the TX path using libiio. Intended to run continuously as a server-side process on Linux.

---

## How It Works

```
RX antenna
     │
 AD9361 ADC
     │
 DMA (cf-ad9361-lpc)
     │
 rxbuf  ──[copy]──►  txbuf
     │
 DMA (cf-ad9361-dds-core-lpc)
     │
 AD9361 DAC
     │
TX antenna
```

On startup the program connects to the PlutoSDR and opens RX and TX streaming channels. It then enters a loop:

1. `iio_buffer_refill()` blocks until 4096 IQ samples arrive from the ADC
2. Each I and Q value is copied from the RX buffer to the TX buffer
3. `iio_buffer_push()` sends the TX buffer to the DAC for transmission

A signal handler catches `SIGINT` and `SIGTERM` so buffers and the IIO context are always destroyed cleanly on exit.

---

## Future Work

- [ ] Runtime PHY configuration — RX/TX frequency, sample rate, bandwidth via CLI flags
- [ ] Automatic reconnection — retry loop if PlutoSDR disconnects
- [ ] syslog integration for server-side logging
- [ ] Buffer statistics — throughput, error rate, uptime reporting
- [ ] FPGA-side FIR filter via Vitis HLS inserted in the AXI-Stream data path
- [ ] Squelch — detect signal presence and only key TX when a signal is actually received