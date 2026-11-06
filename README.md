# PT USB to CAN DL
The PT Motorsport USB to CAN Data Logger is a three in one device for use in CANbus systems to view CAN messages with SLCAN, Log CAN messages to SD Card and provide RTC information for use in logs and also make avalibale on the CANbus.

Thank you for your interest in another PT Motorsport Open Source Project! We're so glad you're here and looking forward to seeing what you can do! Make sure you tag us @ptmotorsportau to show off your creations!

Hardware Kits are avaliable from PT Motorsport AU website https://www.ptmotorsport.com.au/

## Thank you!
Projects like these are made possible by your support through purchases on the PT Motorsport AU website.

www.ptmotorsport.com.au

Thank you for your support! We greatly appreciate you spreading the word about our products and services to your fellow motorsport enthusiasts.

Thank you to Tessa who started this project in both hardware and firmware while she was interning at PT Motorsport Au


## Status LEDs

LED numbering follows the physical chain, starting with LED 1 at pixel index 0.

| LED | Function | Indication |
| --- | --- | --- |
| 1 | SD card | Green when available, red when unavailable or failed |
| 2 | Recording | Green while messages are queued or recently written; white when ready and idle; red for SD/CAN errors or buffer overflow; off in SavvyCAN mode |
| 3 | CAN termination | White when the configured sense input is active, off otherwise |
| 4 | CAN communications | Red on initialization or reported CAN error; green for recent received frames; blue when initialized without recent received frames |
| 5 | Power | Green when measured VBATT is greater than 6 V, blue otherwise; off if the ADC read fails |

Settings are in [include/status_config.h](include/status_config.h). `CANTIMEOUT`
defaults to 1000 ms. LEDs refresh every 500 ms, subject to SD operation delays.
CAN errors remain indicated until CAN is reinitialized, for example with `CANSPEED`.
Receive activity refers to frames delivered by the controller, so hardware filters
can exclude traffic from this indication.

VBATT is read directly from P003 / AN003 using the Renesas FSP ADC driver because
P003 is not in the Uno R4 Minima Arduino pin table. The divider maps 30 V battery
voltage to 5 V ADC voltage (6:1 ratio). Conversion uses a 12-bit ADC and nominal
5 V AVCC reference: `VBATT = ADC_count * 30 / 4095`. Adjust
`VBATT_ADC_REFERENCE_VOLTS` to the actual AVCC voltage for calibration. The direct
driver owns ADC unit 0; do not add Arduino `analogRead` calls on that unit without
coordinating ADC ownership. `STATUS` reports voltage, raw ADC count and read failure.
VBATT above 6 V takes priority even with USB connected; blue is inferred from
VBATT at or below 6 V, not an independent USB supply measurement.

Termination is driven on D3 (HIGH = on) and stored in EEPROM byte 64 (default off).
Termination sensing remains unconfigured (`TERMINATION_SENSE_PIN = -1`), so LED 3
shows the commanded state (white = on). Parameter-triggered recording is not yet defined: the
existing behavior still records every CAN frame that passes the logging filters.
White currently means ready and waiting for those frames, not a configured trigger.

## Button Menu

Hold Button 2 for one second to enter or leave the menu. Press Button 1 to
scroll through CAN speed, termination, then logging. Non-selected LEDs are amber;
the selected item's LED displays its setting. Press Button 2 briefly to change it.

On the logging item, LED 2 is **green for enabled** and **red for disabled**.
Disabling stops accepting new log frames, drains the queue, syncs and closes the
CSV using the same graceful-stop path as `LOG STOP`. Wait for the menu LED to
change to red before removing the card. Enabling resumes recording in a new
segment. This setting is not saved to EEPROM: logging defaults to enabled after
a reboot. Outside the menu, the normal recording status colours still apply.

The red menu setting reports disabled logging, not successful SD recovery. If
the card has faulted, draining/closing may have failed; check the SD status rather
than treating red alone as confirmation. The menu is inactive in SLCAN mode.

## SD Card Diagnostics

Open the serial CLI at 115200 baud and enter `SD`. The read-only report shows
the saved startup result, initialization start time and duration, requested SPI
clock and pins, saved and current SdFat error codes with symbolic names, and
filesystem mount status. It does not reinitialize the card or write a test file.
Startup tries SD initialization up to five times, waiting 500 ms between failed
attempts and stopping immediately on success. Normal logging starts after these
attempts finish. The report includes the attempt count, first failure when retried,
final initialization error, and total elapsed initialization time including waits.
After startup, logger mode retries an unavailable SD card every two seconds until
it first becomes ready. A ready card is checked once per second, even without
CAN traffic. Once ready, a failed check, card swap, or file open/write/sync/close
failure latches SD access off for the rest of that boot and turns the SD LED red.
There are no automatic remounts or new file segments after a runtime fault, and
the fault handler does not attempt another close/sync on a suspect file. This
avoids repeated metadata writes to potentially damaged media. Inspect the card
before rebooting; a reboot does not repair filesystem corruption. There is no configured
card-detect pin, so removal detection can take the check interval plus the SD
library timeout. Individual SD operations are synchronous and can briefly delay
CAN processing. Automatic remounts are paused in SavvyCAN mode.

The logger stores up to 512 compact raw frames, formats them only when writing,
and releases each queue slot after SdFat accepts its row. CAN is drained in bursts
of up to 32 frames before and after CLI work and between written rows. The CSV
file stays open, with sync approximately once per second and close on rotation,
`LOG STOP`, or entry to adapter mode. `LOG STOP` stops accepting new log frames,
drains queued frames and closes the file; `LOG START` resumes in a new segment.
After a latched runtime fault, queued messages remain in RAM; attempted rows
not confirmed by sync are counted separately and are not blindly replayed.
Power loss can lose queued frames and recent unsynced writes. This is a
throughput/durability tradeoff, not a guarantee of a one-second loss limit.

## Sudden Vehicle Power Loss

Logs are ordinary **`.csv` files from the moment they are created**, ready to
open directly in the analyser. There is no preallocation, journal, rename step,
or recovery-tool requirement. Previously synced data does not require the file
to be gracefully closed to be readable, provided the filesystem remains intact.

The header is synced when the file is created. During recording, sync runs
approximately every 1000 ms, flushing SdFat's buffers and updating file metadata.
Segments close and rotate before a row would exceed **4 MiB**, or on the first
written row after **five minutes**. Idle segments need not rotate at exactly
five minutes. `LOG STOP` and adapter entry drain queued rows, sync and close.
After reboot, exclusive filenames prevent overwriting or appending to earlier
logs. Existing logs are not repaired or rewritten automatically.

### Expected Tail Loss

**Healthy-operation target: roughly the latest 1 second of writes, plus frames
still queued in RAM.** At an observed 800 frames/second, a full 512-frame queue
represents another 0.64 seconds. With the earlier measured queue high-water of
34, that example backlog is about 43 ms. These are planning estimates, not
measured power-cut guarantees or proof that the CAN driver lost no frames.

This is not necessarily just the last line: power loss can discard many recent
rows. Sync duration, blocking allocation/SD calls, rotation, stalled processing
and card behaviour can extend the window. The interval runs from the previous
completed sync and is checked after a write step. `SD` reports the last synced
byte position for the current or most recently opened segment and sync duration.
An interrupted file can end with an incomplete final row; analysers that tolerate
that row can load the preceding records directly.

There is **no hard upper bound under arbitrary power failure or hot removal**.
FAT allocation and directory updates are not atomic power-loss transactions.
A card can corrupt older data or filesystem metadata; a later partial-sector
write can damage previously synced bytes. SPI CRC detects transfer errors but
does not guarantee storage integrity after power loss. Power-loss detection,
hold-up energy and a power-loss-resilient card are needed for stronger guarantees.

The intended everyday workflow is power off, remove the card, and open its CSVs
on the laptop without conversion. Hot removal is an expected user action, but
**cannot be guaranteed safe while writes are active**. `LOG STOP` remains an
optional safer maintenance procedure, not a required normal shutdown step.
Removal triggers the existing latched SD fault; reinserting a card after that
fault requires a reboot. This does not provide a live hot-swap/eject protocol.

### Power-Cut Test Procedure

1. Start with a backed-up, known-good card and record a reference capture on the
	analyser. Run longer than five minutes to exercise rotation, or until 4 MiB.
2. Cut vehicle power while traffic continues, without `LOG STOP`. Vary timing
	across repeated tests, including soon after logging starts and near rotation.
3. Remove the card and open the `.csv` files directly in the analyser. Test
   powered hot removal separately with expendable, backed-up media. Preserve
   original files before any repair if an error occurs.
4. Compare the last readable frame with the reference and measure tail loss.
	Check malformed rows, older segments and filesystem integrity as well as the
	final timestamp. Do not run filesystem repair on the only original copy.
5. Reinsert and cold-boot, confirming it creates a new segment and leaves old
	interrupted segments untouched. Also test `LOG STOP` and adapter handover.

Repeated physical power-cut and hot-removal tests have not yet been performed
on this version. Earlier experimental `.part`/`.idx` files are left untouched;
this version neither creates nor automatically converts them.

The `SD` report separates queue overflow, unconfirmed writes and pending sync,
and includes queue high-water, last/max sync duration, and CLI output overflow.
The Arduino_CAN library's 32-frame receive queue can still overflow during a
long synchronous SD operation; those losses are not counted by the application.
Receive dequeue is briefly interrupt-protected to avoid competing queue-count
updates. Timestamps still represent foreground dequeue, not interrupt arrival.

The requested SPI maximum is 12 MHz, reduced from 50 MHz as a conservative
signal-integrity precaution, not a proven fix for a faulty card or power supply.
The actual clock depends on the SPI implementation. Runtime read-token errors
can indicate card, power, or SPI communication problems; a successful mount does
not establish that FAT metadata is healthy.

SPI command and data CRC is enabled globally with `-DUSE_SD_CRC=1` in
[platformio.ini](platformio.ini), overriding SdFat's disabled default. The `SD`
report shows whether checking is enabled. CRC checks received blocks and enables
card-side checking of write transfers; detected failures use the existing latched
SD fault policy. This helps detect transfer corruption, but does not validate
application RAM, guarantee card health, or repair existing damaged logs. Keep
the clock at 12 MHz for the next short test and capture the `SD` report if it
fails, before resetting the device.

SdFat block transfers currently use `-DUSE_SPI_ARRAY_TRANSFER=0`, bypassing the
Renesas core's bulk SPI routine in favor of byte transfers. This is a controlled
workaround test for runtime timeouts/corruption under CAN load, not a confirmed
driver diagnosis. CRC and the 12 MHz limit remain enabled. The `SD` report should
show `SPI transfer mode: BYTE (bulk path bypassed)`. Byte transfers may reduce
throughput; check the dropped/unconfirmed count during testing. If timeouts
persist, retain the fault report and investigate SD power, grounding and SPI
signals rather than disabling CRC or repeatedly remounting suspect media.

If Windows reports illegal filenames or filesystem errors, stop using the card
for logging. Preserve a sector image before attempting recovery. Do not run
`chkdsk /F` or format the original before preserving important data; repairs can
rename, truncate, or discard damaged entries. Use a separate freshly formatted,
known-good card for validation and check its filesystem after a short test before
attempting another overnight run.

All new segments are direct `.csv` files in SavvyCAN's native CSV format.
The `Time Stamp`
field is microseconds since device boot, represented at millisecond capture
resolution; the RTC date and time are included in the filename when available.
Rows record received frames on bus 0, with hexadecimal IDs and data bytes and
explicit `true`/`false` extended-ID flags. Open the file in SavvyCAN as a GVRET
log if automatic format detection does not select it.

CSV timestamp formatting uses 32-bit millisecond capture values with three zeroes
appended to express microseconds, avoiding `%llu` support requirements in the
embedded C library. The capture value wraps after approximately 49.7 days.
SD service writes at most eight rows per step. CLI output is queued in a fixed
4096-byte FIFO and drained in bounded chunks only when USB has space. Excess
output is counted rather than blocking capture. CLI commands accept CR, LF,
or CRLF line endings. Individual SD operations remain synchronous.

## USB to CAN (SLCAN)

Connect SavvyCAN using its LAWICEL/SLCAN driver: its `C`, `S0`-`S8`, `O`
initialization sequence automatically drains/closes the SD segment and hands
the serial port exclusively to the adapter. No CLI command is required.
`SAVVYCAN` remains an optional manual service command. Idle CLI banner, prompt
and character echo are suppressed to keep connection detection quiet; human
commands such as `HELP` and `SD` still return responses. There is no CLI
diagnostic output in adapter mode. Then connect the analyser using its SLCAN
driver (not GVRET). Adapter mode starts with its CAN channel closed and defaults
to 1 Mbps. Standard `S0` through `S8` bitrate commands, `O` open, `C` close,
`t`/`T` standard/extended data transmission, `V`, `N`, `F` and `Z0`/`Z1` are
supported. Commands terminate with CR; LF is ignored. For example `S6` then `O`
selects 500 kbps and opens the channel. Success replies are CR; transmission
acceptance replies are `z`/`Z` plus CR. Failed commands return BEL. Transmission
acknowledgment means the CAN driver accepted the request, not proven bus delivery.

Close with `C`, then send `EXIT` to return to the CLI. The previous logging
enabled/stopped state is restored. Disconnecting USB does not exit adapter mode;
reconnect and close/exit explicitly, or reset. No SD health checks/writes run in
adapter mode. A fixed 1024-byte output queue drops frames rather than blocking
on a slow host, reserves room for command replies, and reports sticky receive
overflow via `F08`. The CLI reports adapter drops after exit. `F` currently
reports adapter output overflow only, not a complete hardware error register.

Listen-only `L`, remote frames `r`/`R`, custom timing and protocol filter commands
are explicitly rejected: the current Arduino_CAN wrapper does not expose all
required functionality. This is a supported SLCAN subset, not GVRET or CAN FD.
Use an isolated test bus to verify transmission before connecting live equipment.

Close the VS Code serial monitor before connecting SavvyCAN: Windows serial-port
ownership is exclusive, and firmware cannot release a port held by another PC
application. Automatic handover only changes the firmware mode. USB disconnect
still does not automatically resume SD logging; close/exit explicitly or reset.
Native USB reception does not require host DTR assertion. The Renesas core's
`dtr()` opt-out enables reads/writes for hosts such as SavvyCAN's Lawicel driver;
TinyUSB mount state, rather than DTR, gates connection handling. The serial
connection check still services USB events. A mounted USB device does not prove
that a PC application currently has its COM port open.

## Logging Filters

Logging filters are numeric software rules; hardware reception stays open.
`FILTERS ON/OFF` and `WHITELIST ON/OFF` take effect immediately without restarting
CAN. Whitelist mode accepts matching rules; blacklist mode excludes them. Empty
whitelists accept nothing. Adapter mode is not restricted by logging filters.

Use `WL SET 1 STD 700` or `BL SET 1 EXT 18DAF110`, slots 1 through 5.
`WL CLEAR 1`/`BL CLEAR 1` deactivate a slot, `WL READ`/`BL READ` display rules,
and `WL SAVE`/`BL SAVE` persist them. Legacy lists are imported if no new-format
list exists. New lists occupy EEPROM bytes 96-121 and 160-185, separate from the
legacy data and termination setting. Interrupted saves invalidate that list's
new-format marker; EEPROM storage is not a transactional database.

## Regression Tests

Native C++11 tests in `test/` cover CSV formatting, ring ordering/overflow,
numeric filter matching, bounded console output, SLCAN parsing/formatting and
mocked USB/CAN bridge lifecycle. Compile each `*_test.cpp` independently with
`g++ -std=c++11 -Wall -Wextra -Werror -Iinclude`; the bridge test additionally links
`src/slcan_bridge.cpp` with `-Itest/support -Iinclude -Isrc`. These tests do not
establish hardware throughput or loss-free capture.

Removing a card during writes can corrupt its filesystem or lose data. Automatic
recovery is not a safe-eject mechanism; stop logging or power down before removal.

For power-related failures, compare reports from separate USB-only and vehicle-power
cold boots using the same card. Connect the CLI after startup without resetting
the board to retrieve the saved failure. Adding USB may change the power conditions,
so note how it was connected. The firmware does not measure the power source or
SD supply voltage. Check the SD socket's 3.3V rail during startup for slow rise,
dips or ripple, along with common ground, SPI signal levels and decoupling.

## Licence
Copyright (c) 2026 - PT Motorsport AU Pty Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


