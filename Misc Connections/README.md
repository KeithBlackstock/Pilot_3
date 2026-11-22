# ESP-01S AT setup bridge

Temporary side-firmware that turns the Pilot_3 FC into an ESP-01S programming bridge. The PC talks to the FC through ST-Link/OpenOCD; the FC forwards bytes between an SRAM mailbox and USART2. No USB-TTL adapter is needed.

**This is not flight firmware.** Flashing it temporarily replaces Pilot_3. It does not initialize the IMU, GPS, motor, or servos. Remove the propeller before flashing, and restore the normal firmware when finished.

## Wiring and power

- FC PA2 / USART2 TX -> ESP-01S RX
- FC PA3 / USART2 RX <- ESP-01S TX
- GND -> GND
- 3.3 V -> VCC; EN/CH_PD must remain high

The ESP8266 is a 3.3 V device with WiFi current peaks of several hundred mA. Verify the supply and local decoupling are adequate.

## Build, flash, and open the console

From this folder:

```bat
build.bat
flash.bat
console.bat
```

`flash.bat` asks for confirmation before programming. `console.bat` starts OpenOCD if necessary and connects to its telnet port. The console defaults to 115200 baud, the usual ESP-AT default.

Type an AT command and press Enter; CR LF is appended automatically:

```text
AT
AT+GMR
```

A working link returns `OK` and firmware information. If there is no response, check wiring and try another baud:

```text
:baud 9600
```

Use `:quit` to exit. Other local commands are sent to the ESP unless they begin with one of those two commands.

## Current WiFi setup

The configured ESP advertises the open `ESP-704796` access point at `192.168.4.1`. Its UART is set to 115200 8N1 to match the flight firmware, and its saved transparent TCP-client connection targets the PC at `192.168.4.2`, port 8082. The commands used for those two settings are:

```text
AT+UART_DEF=115200,8,1,0,0
```

After the UART command, change the console to the same speed and verify that AT still responds:

```text
:baud 115200
AT
```

Connect the PC to `ESP-704796` and verify its address with `ipconfig`; it should be `192.168.4.2`. Then save the transparent connection:

```text
AT+SAVETRANSLINK=1,"192.168.4.2",8082,"TCP"
AT+RST
```

The PC must run a TCP server on that address/port; the ESP connects to it. Passthrough prevents AT framing such as `+IPD` from entering the FC's existing binary protocol. This ESP-AT 1.7.5.0 firmware rejects `AT+CIPRECONNINTV`, so no reconnect-interval command is used.

## Re-entering AT mode (changing settings later)

With saved passthrough, the ESP boots straight into transparent mode and ignores AT commands. The console's `:esc` sends the bare, guard-timed `+++` that drops it back to command mode (it waits ~1 s on each side). Set the console to the ESP's *current* UART speed first. For example, moving the UART from 9600 to 115200:

```text
:baud 9600
:esc
AT
AT+UART_DEF=115200,8,1,0,0
:baud 115200
AT
AT+RST
```

`AT+RST` reboots into the still-saved passthrough link. To disable passthrough entirely instead, use `AT+SAVETRANSLINK=0` after `:esc`.

## Dashboard and restore flight firmware

After configuring the ESP, reflash the normal Pilot_3 firmware using its usual build/flash procedure or the dashboard's **Compile & Flash** with ST-Link attached. Keep the ESP UART baud matched to `firmware/src/bt.cpp` (currently 115200).

With the PC joined to `ESP-704796`, run `pilot3.bat` and choose option 2 (WiFi). The PC listener is implemented in `tools/pilot3_tcp_link.py`; the dashboard shows live telemetry and supports the usual wireless config, waypoint, and blackbox operations. Compile & Flash still requires ST-Link.
