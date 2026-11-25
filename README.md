# PT USB to CAN DL
The PT Motorsport USB to CAN Data Logger is a three in one device for use in CANbus systems to view CAN messages with SLCAN, Log CAN messages to SD Card and provide RTC information for use in logs and also make avalibale on the CANbus.

Thank you for your interest in another PT Motorsport Open Source Project! We're so glad you're here and looking forward to seeing what you can do! Make sure you tag us @ptmotorsportau to show off your creations!

Hardware Kits are avaliable from PT Motorsport AU website https://www.ptmotorsport.com.au/

## Thank you!
Projects like these are made possible by your support through purchases on the PT Motorsport AU website.

www.ptmotorsport.com.au

Thank you for your support! We greatly appreciate you spreading the word about our products and services to your fellow motorsport enthusiasts.

Thank you to Tessa who started this project in both hardware and firmware while she was interning at PT Motorsport Au

## ADC and RPM logging test firmware

The default PlatformIO environment, `uno_r4_minima_adc`, is a dedicated ADC/tach
test. It replaces the normal CAN logger in the flashed firmware: CAN logging,
SLCAN, the normal CLI, RTC and VBATT status are not active. To build the
original CAN firmware instead, select `uno_r4_minima` explicitly.

Insert an SD card before powering on. The test samples **P002 / AN002 / A3** at
a nominal **10,000 samples per second**, using the RA4M1 ADC's native 14-bit
resolution and AVCC reference. The raw counts range from 0 to 16383; no voltage
scaling is applied. Observe the board's input voltage limits.

The conditioned tach square wave is connected to **P106 / D6 / GTIOC0B**.
GPT0 captures **falling edges** in hardware with a 3 MHz counter (PCLKD / 16)
and the PCLKD / 64 input noise filter, following the input-capture approach in
[PT-IO-Mini2](https://github.com/ptmotorsport/PT-IO-Mini2).
The pin is an input, not a PWM output, and has no internal pull-up. It replaces
Button 1 in this test firmware; do not operate Button 1 during tach capture.
Button 2 on P107 remains available for stopping capture.

RPM uses **one pulse per revolution** and the period between successive falling
edges. The latest period is included alongside each 10 kHz ADC reading; RPM
is updated per tach pulse, not measured afresh at 10 kHz. A **300 ms no-pulse
timeout** sets RPM to zero. Two edges are required after startup or timeout
before RPM becomes valid. This timeout means signals at or below approximately
200 RPM cannot be tracked continuously. Settings are in
[include/tach_period.h](include/tach_period.h).
GPT0 is reserved before selecting the independent ADC sample timer. Capture
and ADC interrupts share a priority so their period state is read consistently.

Each power-up creates the first available `ADC0000.BIN` through `ADC9999.BIN`
without overwriting an existing file. Hold **button 2** for one second to stop,
drain, sync and close the file before removing the card. Restart the device for
a new capture.

Convert a capture using Python 3 (no additional packages required):

```powershell
python scripts\adc_binary_to_csv.py ADC0000.BIN ADC0000.csv
```

New captures convert to `Time,MAP,RPM`, extending the supplied UltraLog example.
`Time`
is elapsed microseconds starting at zero, increasing by 100 for each sample;
`MAP` is the unscaled ADC count and `RPM` is engine speed, formatted to three
decimal places. Times are reconstructed from the nominal rate,
not individually measured timestamps. The converter refuses to overwrite an
existing CSV and rejects unsupported headers, incomplete samples and values
outside the 14-bit range.

New files use binary version 2: a 20-byte header (`PTADC`, version 2, resolution
14, reserved zero, then little-endian uint32 values for sample rate 10,000,
tach counter frequency 3,000,000 and pulses per revolution 1). Each six-byte
record contains a little-endian uint16 ADC count and uint32 tach period in
counter ticks (zero means no valid RPM). The converter calculates
`RPM = 60 * 3000000 / period_ticks`. It also continues to read version 1
ADC-only files, outputting their original `Time,MAP` columns.
There is no end marker, so a truncated file ending on a sample boundary cannot
be detected by the converter.

The test buffers up to 2047 combined samples while SD writes and periodic
one-second syncs run (about 204.7 ms of samples, with 60 kB/s record data).
An ADC error, full buffer or detected
timer interval outside 50–150 microseconds
stops capture instead of silently continuing across a gap. SD errors or card
removal also stop capture; the final rows may be incomplete. The SD and
recording LEDs turn red on an error, and USB serial at 115200 baud reports the
reason and number of rows written. After a normal stop the recording LED is
white. Power loss or removing the card before stopping can lose unsynced data.

Timing and sustained recording depend on the hardware and SD card; timer
interrupts, SD stalls and actual sample quality should be verified on the
target device. Use a known square-wave frequency to verify RPM (100 Hz should
read 6000 RPM), and check the 300 ms zero-RPM timeout. Multiple tach edges while
capture interrupts are blocked can overwrite the hardware capture register.
The interval check cannot prove that every missed interrupt is
detected if interrupts are blocked long enough to affect the system clock.

The documentation below describes the original CAN firmware unless stated
otherwise.


## Digital Pin Assignments

| Arduino pin | MCU pin | Function |
| --- | --- | --- |
| D0 | P301 | UART RX |
| D1 | P302 | UART TX |
| D2 | P105 | Addressable LED output |
| D3 | P104 | CAN termination control |
| D6 | P106 | Button 1 |
| D7 | P107 | Button 2 |
| D8 | P304 | SD card detect (LOW = inserted, internal pull-up) |
| D10 | P112 | SD card chip select |

## Status LEDs

LED numbering follows the physical chain, starting with LED 1 at pixel index 0.
The five WS2812-compatible LEDs use FastLED on pin D2 with GRB color order.

| LED | Function | Indication |
| --- | --- | --- |
| 1 | SD card | Off when no card is present; green when available, red when a present card is unavailable or failed |
| 2 | Recording | Off when logging is disabled, no card is present, or in SavvyCAN mode; otherwise green while messages are queued or recently written, white when ready and idle, red for SD/CAN errors or buffer overflow |
| 3 | CAN termination | White when the configured sense input is active, off otherwise |
| 4 | CAN communications | Red on initialization or reported CAN error; green for recent received frames; blue when initialized without recent received frames |
| 5 | Power | Green when measured VBATT is greater than 6 V, blue otherwise; off if the ADC read fails |

Settings are in [include/status_config.h](include/status_config.h). `CANTIMEOUT`
defaults to 1000 ms. LEDs refresh every 500 ms, subject to SD operation delays.
CAN errors remain indicated until CAN is reinitialized, for example with `CANSPEED`.
Receive activity refers to frames delivered by the controller, so hardware filters
can exclude traffic from this indication.

The analog pin assignments are:

| Arduino pin | MCU pin | Function |
| --- | --- | --- |
| A0 | P014 | VBATT sense |
| A1 | P000 | Analog AV1 |
| A2 | P001 | Analog AV2 |
| A3 | P002 | Analog AV2 |

Only VBATT is currently sampled by the firmware. It is read from A0 / P014 /
AN009 using the Renesas FSP ADC driver. The divider maps 30 V battery voltage
to 5 V ADC voltage (6:1 ratio). Conversion uses a 12-bit ADC and nominal
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
scroll through CAN speed, termination, logging, then SD log format. Non-selected LEDs are amber;
the selected item's LED displays its setting. Press Button 2 briefly to change it.

The menu indications take priority over card presence: non-selected LEDs remain
amber even without a card. On the logging item, LED 2 is **green for enabled**
and **red for disabled**, regardless of card presence. Outside the menu, the
recording LED is off when logging is disabled, even with a card inserted.
Disabling stops accepting new log frames, drains the queue, syncs and closes the
file using the same graceful-stop path as `LOG STOP`. Wait for the menu LED to
change to red before removing the card. Enabling resumes recording in a new
segment. This setting is not saved to EEPROM: logging defaults to enabled after
a reboot. Outside the menu, the normal recording status colours still apply.

The red menu setting reports disabled logging, not successful SD recovery. If
the card has faulted, draining/closing may have failed; check the SD status rather
than treating red alone as confirmation. The menu is inactive in SLCAN mode.

On the SD format item, the SD LED is **green for CSV** or **blue for binary**.
Press Button 2 briefly to toggle. If switching fails, it turns red until the
selection is changed or retried; use `SD` for diagnostics. The format is saved
to EEPROM byte 65 and survives reboot; unconfigured devices default to CSV.
While recording, a format change drains and closes the old segment and opens
a new one automatically. Capture pauses during this transition and frames can
be lost; stop traffic first if loss matters. If the old segment cannot be safely
closed, the format stays unchanged. If opening the new segment fails, the new
format remains saved and the existing SD recovery policy applies.

## SD Card Diagnostics

Card detect is wired to **D8 / P304** (not D9 / P303). The socket switch pulls
the input LOW when a card is inserted; firmware enables the internal pull-up.
Physical presence does not imply the card has mounted successfully.

Open the serial CLI at 115200 baud and enter `SD`. The read-only report shows
the saved startup result, initialization start time and duration, requested SPI
clock and pins, saved and current SdFat error codes with symbolic names, and
filesystem mount status. It does not reinitialize the card or write a test file.
Startup skips SD initialization when no card is detected. With a card present,
it tries SD initialization up to five times, waiting 500 ms between failed
attempts and stopping immediately on success. Normal logging starts after these
attempts finish. The report includes the attempt count, first failure when retried,
final initialization error, and total elapsed initialization time including waits.
After startup, logger mode retries an unavailable, physically present card every
two seconds while no fault is latched. A ready card is checked once per second, even without
CAN traffic. Once ready, a failed check, card swap, or file open/write/sync/close
failure latches SD access off and turns the SD LED red if a card is present
(SD and recording LEDs are off when absent). The fault handler does
not attempt another close/sync on a suspect file. Recovery requires an observed
removal followed by reinsertion; a fault with the card still inserted does not
cause repeated mount attempts. Inspect or replace suspect media before reinserting
it; reinsertion does not repair filesystem corruption.
Card detect is polled each loop (including SavvyCAN mode), before log flushes,
between written batches, and before sync/close operations. Removal of a mounted
card latches SD access off without attempting to sync or close it. Reinsertion
clears the latch after the detect signal stays present for 100 ms. The stale
file handle is discarded without syncing, the card is remounted, and queued frames
are written to a new, exclusively created file in the selected format. Already attempted rows not
confirmed by sync remain counted as unconfirmed/dropped and are not replayed.
Logging remains disabled if the user previously stopped it. Mounting is deferred
until logger mode if reinsertion happens in SavvyCAN mode.
Any observed absent level while ready is treated as removal immediately for
safety; insertion bounce restarts the 100 ms settling period.
Individual SD operations are synchronous and can briefly
delay CAN processing and removal detection; an operation already in progress
cannot be interrupted by this polling. Automatic mounts and SPI health checks
are paused in SavvyCAN mode, but card detection remains active.

The logger stores up to 288 compact raw frames and formats up to eight CSV rows
or 24 binary records into a 512-byte buffer per synchronous SD write. Attempted rows leave the queue
even on a short or failed write; unattempted rows remain queued. CAN is drained
in bursts of up to 32 frames before and after CLI work and between batches. The
file stays open, with sync approximately once per second and close on rotation,
`LOG STOP`, or entry to adapter mode. `LOG STOP` stops accepting new log frames,
drains queued frames and closes the file; `LOG START` resumes in a new segment.
The next file is created and its header synced as soon as logging and the card
are ready, rather than waiting for the first frame; an empty file may therefore
remain if no frames arrive. `LOG START` acknowledges success after the file is
prepared, or reports an SD error while leaving logging enabled for recovery.
After a latched runtime fault, queued messages remain in RAM; attempted rows
not confirmed by sync are counted separately and are not blindly replayed.
Power loss can lose queued frames and recent unsynced writes. This is a
throughput/durability tradeoff, not a guarantee of a one-second loss limit.

### Selecting the log format

Use `LOG FORMAT` to display the current format, `LOG FORMAT CSV` or
`LOG FORMAT BINARY` to change it, or use the button menu above. The setting is
saved across reboots. Changing it while recording automatically closes the
current segment and resumes in the selected format, with a potential capture
gap. `STATUS`, `CONFIG`, and `SD` report the current format. `SD LIST` and
`SD GET` export both formats after logging stops. Convert a downloaded binary
file without overwriting an existing destination:

```
python scripts/binary_log_to_csv.py 123_1.bin 123_1.csv
```

Binary version 1 starts with the eight bytes `PTCAN 01 11 00` (the first five
are ASCII). Each 17-byte record contains little-endian 32-bit foreground
timestamp in milliseconds, little-endian 32-bit identifier with bit 31 set
for an extended ID, one DLC byte and eight payload bytes (unused bytes are
zero). The timestamp wraps as in CSV. Converter validation rejects invalid
headers, records and incomplete tails rather than silently accepting them.
Power loss or card removal can leave an incomplete tail or other corruption;
this format does not add record checksums or stronger durability guarantees.
In bench testing, binary captured an independently verified 30-second 70% load
without loss, but a longer rotation test lost frames. No lossless bus-load
guarantee is implied.

## Downloading Logs over USB

Leave the card in the device and use the logger's USB serial port, not a
SavvyCAN connection. Close any serial monitor using the same port first. Stop
the CAN test sender before using `--stop`, which sends `LOG STOP` to drain and
close the active file. This leaves logging disabled until you send `LOG START`
or re-enable it with the buttons.

With Python 3.8+ and `pyserial` installed (`python -m pip install pyserial`):

```powershell
python scripts\download_sd_logs.py --port COM22 --stop list
python scripts\download_sd_logs.py --port COM22 pull --output .\bench-logs 96_0.csv 96_1.csv
```

Replace `COM22` with the logger's port. Omit the filenames to download all
matching root-level log CSV and BIN files. The script verifies the byte count and CRC-32
before keeping each download and refuses to overwrite an existing file.
For automated tests, compare pre-run and post-run file listings to select the
new segments. Keep export outside the timed CAN capture.

The firmware commands are `SD LIST` and `SD GET <filename>`. Both require
logging to be stopped, with no pending rows and no open logging file. The
read-only serial protocol starts replies with `SDX LIST` or `SDX GET <size>`;
list entries use `SDX FILE <name>` followed by `SDX END`. File data uses
colon-prefixed hex lines followed by `SDX CRC <crc32-hex>`. Errors use
`SDX ERR <message>`. Transfers are not available in SavvyCAN mode.

## Sudden Vehicle Power Loss

Default-mode logs are ordinary **`.csv` files from the moment they are created**, ready to
open directly in the analyser. There is no preallocation, journal, rename step,
or recovery-tool requirement. Previously synced data does not require the file
to be gracefully closed to be readable, provided the filesystem remains intact.

The header is synced when the file is created. During recording, sync runs
approximately every 1000 ms, flushing SdFat's buffers and updating file metadata.
Segments close and rotate before a row would exceed **4 MiB**, or on the first
written row after **five minutes**. Idle segments need not rotate at exactly
five minutes. `LOG STOP` and adapter entry drain queued rows, sync and close.
After reboot, exclusive filenames prevent overwriting or appending to earlier
logs. A new log checks for existing CSV and binary names and adds a suffix when
needed. Segment rollover skips those preliminary checks and creates the next
file exclusively. If that exact filename already exists, or creation fails for
another reason, logging reports an SD fault rather than overwriting or silently
retrying; use a different name or card before restarting. Earlier `.part` and
`.idx` files do not reserve names. Existing logs are not repaired or rewritten
automatically.

### Expected Tail Loss

**Healthy-operation target: roughly the latest 1 second of writes, plus frames
still queued in RAM.** At an observed 800 frames/second, the 288-frame logging
queue represents up to 0.36 seconds, in addition to frames still in the
256-frame receive ring. These are planning estimates, not measured power-cut
guarantees or proof that the CAN driver lost no frames.

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
fault clears the latch after 100 ms of stable presence and resumes in a new file
without rebooting, if logging is enabled. This recovery is not a safe-eject
protocol and cannot recover unsynced data from the removed card.

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
It also reports the last and maximum synchronous batch-write duration and counts
batch writes taking at least 1 ms and at least 10 ms. The foreground wall-time
profile reports call counts, cumulative milliseconds and maximum microseconds
for CAN drain, batch preparation (including CSV formatting), SD batch write,
sync, file preparation, filename checks, header writes, close and periodic SD health checks. Timings include
interrupt preemption and measurement overhead, not just CPU work. File
preparation includes any filename checks for new logs, but excludes the header write and sync;
filename-check timing is therefore nested within file preparation. Startup and recovery checks are
not included in periodic health timings. They are not a complete breakdown of
elapsed time. Counts saturate at 32 bits; timing totals and maxima persist until
reboot, and the 1 ms and 10 ms thresholds overlap.

During segment creation and closing, the foreground services the CAN receive
ring between individual file operations when queue space is available. Each SD
operation is still synchronous; a single long operation or a full queue can
still cause receive-ring overflow. This reduces neither the durability risks
nor the potential for losses at a rotation boundary.

The project-local Arduino_CAN library is based on the Uno R4 core 1.5.1 library.
Its interrupt-driven receive ring holds 256 frames and reports its high-water
mark, frames rejected when full, and mailbox-loss events. A mailbox event does
not necessarily identify the number of frames lost. These counters saturate at
their maximum and persist until reboot, including across CAN reinitialization.
RAM was rebalanced from the downstream 512-frame queue to the interrupt-fed
ring; the downstream queue now holds 288 frames. This gives the receive
interrupt more headroom during blocking SD calls, at the cost of less buffering
after filtering.
While the card is ready, a full logging queue pauses transfer from the CAN
ring until a slot is freed. The receive ring may then fill and reject frames;
its drop counter includes frames before software filtering. When logging is
stopped, the logger drains the CAN frames already queued at the stop boundary,
along with the downstream queue, before closing the file. Frames arriving later
are not included. Buffered timestamps still reflect foreground dequeue time.
The library ring can still overflow during a long synchronous SD operation.
Receive dequeue is briefly interrupt-protected. Timestamps still represent
foreground dequeue, not interrupt arrival. The larger receive ring improves
short-burst headroom but does not prevent loss if storage cannot sustain the
incoming rate.

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

The experimental `uno_r4_minima_spi24` build requests 24 MHz with byte
transfers and CRC enabled; the default remains 12 MHz. A requested frequency
is not a measured SCK frequency. Higher-speed operation is not qualified for
production: back up the card, check signal integrity and CRC/errors, and test
representative cards before adopting it.

The default `uno_r4_minima` environment uses byte transfers
(`-DUSE_SPI_ARRAY_TRANSFER=0`). The optional `uno_r4_minima_bulk` environment
uses the Renesas core's bulk SPI routine (`-DUSE_SPI_ARRAY_TRANSFER=1`) for
testing. Earlier hardware had a 3.3 V rail fault and encountered
timeouts/corruption; bulk transfer on corrected hardware remains experimental,
not a reliability guarantee. Both environments retain CRC and the 12 MHz
limit. Check `SPI transfer mode` in the `SD` report, back up data before
testing bulk mode, and compare exported CSVs with an independent reference.
If SD faults occur, retain diagnostics and investigate before further writes.

If Windows reports illegal filenames or filesystem errors, stop using the card
for logging. Preserve a sector image before attempting recovery. Do not run
`chkdsk /F` or format the original before preserving important data; repairs can
rename, truncate, or discard damaged entries. Use a separate freshly formatted,
known-good card for validation and check its filesystem after a short test before
attempting another overnight run.

Default-mode segments are direct `.csv` files in SavvyCAN's native CSV format.
The `Time Stamp`
field is microseconds since device boot, represented at millisecond capture
resolution; the RTC date and time are included in the filename when available.
Rows record received frames on bus 0, with hexadecimal IDs and data bytes and
explicit `true`/`false` extended-ID flags. Open the file in SavvyCAN as a GVRET
log if automatic format detection does not select it.

CSV timestamp formatting uses 32-bit millisecond capture values with three zeroes
appended to express microseconds, avoiding `%llu` support requirements in the
embedded C library. The capture value wraps after approximately 49.7 days.
SD service writes at most eight CSV rows or 24 binary records per step. CLI output is queued in a fixed
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

Native C++11 tests in `test/` cover CSV and binary batching, filename creation,
format persistence, receive flow, ring ordering/overflow, numeric filter
matching, bounded console output, SLCAN parsing/formatting, mocked USB/CAN
bridge lifecycle and SD card reinsertion settling.
Compile each `*_test.cpp` independently with
`g++ -std=c++11 -Wall -Wextra -Werror -Iinclude`; the USB connection test also
requires `-Itest/support`, and the bridge test additionally links
`src/slcan_bridge.cpp` with `-Itest/support -Iinclude -Isrc`. Run the binary
converter tests with `python -m unittest discover -s test -p binary_log_converter_test.py`.
These tests do not establish hardware throughput or loss-free capture.

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
