# arduino-pico patches (Pico 2 W build)

The Pico 2 W build runs arduino-pico with FreeRTOS SMP, so DonutShop's `DDloop` and `GIDloop` tasks can
use the network in parallel, like on the Nano ESP32. arduino-pico 6.1.1 has three FreeRTOS networking bugs
that crash or hang the board. `6.1.1/0001-freertos-networking-races.patch` fixes them:

1. **`netif_set_default()` is not handled by the LWIP task.** Starting a soft AP from a user task panics
   with `Unimplemented LWIP thread action`. The patch adds the missing case to `freertos-lwip.cpp`.
2. **`ClientContext` (TCP) state races with its lwIP callbacks.** Under FreeRTOS the callbacks run in the
   LWIP task, possibly on the other core, while the application uses the same object. Each wrapped lwIP call
   is atomic, but the check-then-act sequences around them are not. For example, `connect()` times out and
   calls `abort()` on a pcb that `_error()` has just freed, or `_recv()` appends to `_rx_buf` while `read()`
   consumes it. The result is corrupted lwIP state: lockups, and panics such as
   `tcp_receive: valid queue length`.
3. **The same race in `UdpContext` (UDP receive chain) and `WiFiServer` (accept queue).**

For 2 and 3, the patch adds `lwip_run()` (`libraries/WiFi/src/include/lwip_run.h`). It runs a block of code
synchronously inside the LWIP task through `lwip_callback()`, so that block cannot interleave with any lwIP
callback. The patch wraps every method that touches the pcb or the shared buffers with it. Waits (connect,
send, flush) stay in the calling task. Without FreeRTOS, `lwip_run()` just calls the block, so bare-metal
builds behave as before.

## Using it

CI applies the patch automatically (`.github/workflows/build.yml`, `release-pico2w.yml`). For a local
arduino-cli build, compile once so the core gets installed, then run:

```
extras/arduino-pico-patches/apply.sh
arduino-cli compile --profile pico2w .
```

The script finds every installed copy of the matching core version and patches it. Running it again is
harmless. A build with an unpatched core still compiles, but it is not reliable.

Delete a version's folder once upstream ships the fixes.
