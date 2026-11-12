# UM32 USB VCP host regression test

This harness compiles the Betaflight VCP driver and CherryUSB CDC integration
with mock USB endpoints and a simulated clock. Its nine checks cover
unbuffered TX, the batch completion wait, the shared queue-full timeout,
ordered recovery, suppression of a failed batch's suffix, endpoint submission
retry, zero-length packets, TX wrap and session resets, and clearing wrapped
RX and deferred OUT data before rearming after `CONFIGURED`.

The proof branch is based on Betaflight commit
`ad29e34ea61004564d657d4e368a0d2ae4c57a86` with UM324xF SDK submodule commit
`21178179de6a4b5e3011b3fa635cf9604f595e16`.

Run these commands from the root of the proof branch checkout. Bash and a
host C compiler with GNU C11 support are required. Set `CC` to select a
compiler executable. The default is `cc`.

```bash
git submodule update --init --checkout -- lib/modules/UM324xF
bash tools/um32-vcp-host-test/test.sh
```

To run the harness from a separate directory, pass the checkout root to
`test.sh` as its first argument. On success, it prints `UM32 VCP checks passed`
followed by a coverage summary.
