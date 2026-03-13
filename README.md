# PlutoSDR IQ Retranslator

A lightweight C daemon that captures IQ samples from an **ADALM-PlutoSDR** over the network (libiio) and immediately retransmits them — turning the device into a real-time RF retranslator. Designed to run as a hardened `systemd` service with FPGA-side SDR integration planned.

---

## Features

- Real-time RX → TX IQ loopback via [libiio](https://github.com/analogdevicesinc/libiio)
- Connects to PlutoSDR over Ethernet (`ip:` URI)
- Graceful shutdown on `SIGINT` / `SIGTERM`
- Minimal dependencies — pure C, no C++ runtime

---

## Planned Improvements

The current codebase is a functional prototype. The following enhancements are in progress, in recommended order:

| # | Area | Status |
|---|------|--------|
| 1 | **Reconnection loop** — auto-recover if PlutoSDR disconnects | 🔲 TODO |
| 2 | **PHY configuration** — explicitly set frequency, bandwidth, gain at startup | 🔲 TODO |
| 3 | **syslog integration** — replace `printf` with proper daemon logging | 🔲 TODO |
| 4 | **systemd unit** — deploy and supervise as a system service | 🔲 TODO |
| 5 | **Loopback & endurance tests** — validate stability under continuous load | 🔲 TODO |
| 6 | **CLI / config file** — replace hardcoded `#define` with runtime configuration | 🔲 TODO |
| 7 | **FPGA integration** — utilize the FPGA fabric side of the SDR | 🔲 TODO |
| 8 | **Statistics / watchdog** — dropped buffer counters, periodic throughput logging | 🔲 TODO |

---

## Requirements

### Hardware
- [ADALM-PlutoSDR](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/adalm-pluto.html) reachable at `192.168.2.1` (default USB/Ethernet address)

### Software
- Linux host (tested on x86-64; ARM cross-compilation supported)
- [libiio](https://github.com/analogdevicesinc/libiio) ≥ 0.21
- GCC or Clang
- `make`

Install libiio on Debian/Ubuntu:

```bash
sudo apt install libiio-dev
```

---

## Building

```bash
git clone https://github.com/khlud07/retranslator.git
cd retranslator
make
```

Or manually:

```bash
gcc -O2 -Wall -Wextra -o retranslator main.c -liio
```

---

## Usage

```bash
./retranslator
```

The program will:
1. Connect to PlutoSDR at `ip:192.168.2.1`
2. Enable RX and TX channels (`voltage0` / `voltage1`)
3. Continuously refill RX buffers and push them to TX
4. Print status to stdout until `Ctrl-C` or `SIGTERM`

> **Note:** PHY parameters (center frequency, sample rate, bandwidth, gain) are not yet configured by the program — ensure the radio is pre-configured or wait for improvement #2.

---

## Project Structure

```
plutosdr-retranslator/
├── main.c          # Core retranslator loop
├── Makefile        # Build rules (TODO)
├── retranslator.service  # systemd unit (TODO)
└── README.md
```

---

## Roadmap Detail

### 1. Reconnection Loop
Wrap the context creation and main loop in a `while(1)` retry block. On any fatal IIO error, destroy all resources, wait a backoff interval, and reconnect.

### 2. PHY Configuration
Use `iio_device_find_channel` on `phy` (`ad9361-phy`) to set:
- `out_altvoltage0_RX_LO_frequency`
- `in_voltage_rf_bandwidth`
- `in_voltage_sampling_frequency`
- `in_voltage0_gain_control_mode` / `hardwaregain`

### 3. syslog
Replace all `printf` / `fprintf(stderr, ...)` calls with `openlog` + `syslog(LOG_INFO, ...)`. Messages will then appear in `journalctl` when running under systemd.

### 4. systemd Deployment
```ini
[Unit]
Description=PlutoSDR IQ Retranslator
After=network.target

[Service]
ExecStart=/usr/local/bin/retranslator
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### 5. CLI / Config File
Replace hardcoded `#define URI`, `BUF_SIZE`, etc. with `getopt`-parsed CLI flags and/or a simple INI-style config file.

### 6. FPGA Integration
Leverage the FPGA fabric via custom IP cores or the PlutoSDR HDL reference design to offload filtering, decimation, or custom waveform processing before the IQ data reaches the CPU retranslator loop.

---

## License

MIT — see `LICENSE`.

---

## References

- [libiio documentation](https://analogdevicesinc.github.io/libiio/)
- [PlutoSDR HDL Reference Design](https://github.com/analogdevicesinc/hdl)
- [AD9361 Linux Driver](https://wiki.analog.com/resources/tools-software/linux-drivers/iio-transceiver/ad9361)
