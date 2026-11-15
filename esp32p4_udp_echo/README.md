# Waveshare ESP32-P4-WIFI6-DEV-KIT Rev1.2: Ethernet + UDP echo in C

This is a **board-configured update** of the minimal UDP echo project. It uses ESP-IDF for the ESP32-P4 and a POSIX C client for a Mac/Linux PC or NVIDIA Jetson. No extra Ethernet module is needed; use the board's built-in RJ45 jack and IP101GRI PHY.

**Status:** The PC UDP client was previously compiled and simulated with a local UDP echo server. The P4 project has **not** been compiled with ESP-IDF or tested on the physical Waveshare board in this environment. Check all build and boot logs.

## What was customized for Rev 1.2

`p4/sdkconfig.defaults` selects IP101 RMII PHY with **external RMII clock input** on GPIO50 (PHY produces the 50 MHz clock), 1 PHY address, and these board pins:

| Signal | GPIO |
| --- | ---: |
| RMII clock input | 50 |
| TX_EN | 49 |
| TXD0 | 34 |
| TXD1 | 35 |
| CRS_DV | 28 |
| RXD0 | 29 |
| RXD1 | 30 |
| MDC | 31 |
| MDIO | 52 |
| PHY reset | 51 |

These are board-level pins, **not external connections** you need to wire manually. Do not use the ESP32-P4-Function-EV-Board's example pin assignments for this board.

Waveshare reports PCB Rev1.2 uses ESP32-P4 silicon v3.1. That is **not** the same as PCB Rev1.2; use compatible firmware for actual silicon revision. Waveshare recommends ESP-IDF 5.5.1–5.5.4.

## Files

- `p4/main/main.c` initializes Ethernet and static IPv4 address, binds UDP port 3333, receives any payload <=512 bytes, and echoes identical bytes back.
- `p4/sdkconfig.defaults` is the Waveshare-specific Ethernet configuration to be consumed by ESP-IDF. It is only applied when creating a new sdkconfig; if an old `sdkconfig` exists, rename/remove it and reconfigure.
- `p4/main/idf_component.yml` declares espressif/ethernet_init 1.4.1 (downloaded during first build).
- `p4/main/CMakeLists.txt`, `p4/CMakeLists.txt` identify the source and main IDF project.
- `jetson/udp_client.c` is a desktop/Jetson C UDP client; uses POSIX sockets (works on macOS and Linux, not directly on native Windows).

## Direct Ethernet cable wiring

1. Connect a USB-C data cable from Mac/Linux PC to P4 **Type-C UART** for flashing and monitoring; it also supplies power. Do not rely on PoE unless you installed the optional PoE module.
2. Connect Ethernet cable between Mac/PC Ethernet port (or USB-Ethernet adapter) and P4 RJ45 port. No router needed.
3. Give the computer's Ethernet interface static IP **192.168.7.1/24** (netmask 255.255.255.0). Leave gateway blank.
4. The P4 firmware assigns itself **192.168.7.2/24** and listens UDP **3333**.

## Build & flash (ESP-IDF v5.5.x)

Open an ESP-IDF terminal:

```sh
cd p4
idf.py set-target esp32p4
idf.py menuconfig
# Verify: Ethernet Configuration > Internal EMAC, RMII, clock input GPIO50,
# IP101 PHY address 1, MDC 31, MDIO 52, reset 51, all data pin assignments.
idf.py build
idf.py -p PORT flash monitor
```

Replace `PORT` with your board's UART port, e.g. `/dev/cu.usbserial-...` on macOS or `/dev/ttyUSB0` on Linux. Use `idf.py monitor` to view logs. If an old `sdkconfig` was generated for a different board, remove it and `build/` before running `idf.py set-target`/`menuconfig` again so defaults apply.

Expected P4 logs **approximately**:

```
P4_UDP_ECHO: P4 IPv4: 192.168.7.2, UDP port: 3333
P4_UDP_ECHO: Listening for UDP on port 3333
P4_UDP_ECHO: Ethernet link UP
```

## Mac client

Configure USB Ethernet adapter: System Settings > Network > [Ethernet] > TCP/IP > Configure IPv4: Manually; IP `192.168.7.1`, mask `255.255.255.0`, blank router.

```sh
cd jetson
clang -std=c11 -O2 -Wall -Wextra udp_client.c -o udp_client
ping -c 3 192.168.7.2
./udp_client 192.168.7.2 'Hello P4!'
```

Linux or Jetson:

```sh
ip -br link # identify the actual wired interface
sudo ip link set eth0 up
sudo ip addr add 192.168.7.1/24 dev eth0 # replace eth0 if needed; skip if already configured
cd jetson
gcc -std=c11 -O2 -Wall -Wextra udp_client.c -o udp_client
./udp_client 192.168.7.2 'Hello P4!'
```

Expected client output includes received identical bytes and `PASS: UDP echo matched exactly!`. Results depend on hardware and local network; UDP can occasionally drop packets.

## Debugging

- If the monitor reports PHY init failure: check that the build generated `sdkconfig` correctly. Verify IP101, external clock **input** on GPIO50, MDC 31, MDIO 52, reset 51; some boards allow PHY address autodetection by changing `CONFIG_ETHERNET_PHY_ADDR=-1` in menuconfig if address 1 fails.
- If `Ethernet link DOWN` is continuous: verify RJ45, powered board, PC interface/adapter is enabled, and PHY initialization.
- If `ping` fails but link UP: verify computer static IP and subnet; test UDP directly in case ICMP is filtered.
- If UDP times out: verify firewall, IP address, port 3333 and P4 logs.
- If `idf.py` complains about chip revision: follow Waveshare's P4 rev-v3.1 guidance; never bypass revision safety checks.

## Documentation references

- https://docs.waveshare.com/ESP32-P4-WIFI6-DEV-KIT
- https://docs.waveshare.com/ESP32-P4-WIFI6-DEV-KIT/FAQ
- https://esp-cpp.github.io/espp/dev_boards/waveshare/esp32_p4_wifi6_dev_kit.html
- https://components.espressif.com/components/espressif/ethernet_init/versions/1.4.1/readme
