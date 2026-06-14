# iMac19,1 Warm-Reboot Trace

This branch is based on the cleaned working iMac19,1 5K kernel line. It traces
the display state Linux leaves to EFI during a warm restart.

The trace is armed only for `Apple Inc.` / `iMac19,1`, only on `SYS_RESTART`.
Behavior-changing modes additionally require both real Apple routes and an
active pair of 2560x2880 streams.

All messages begin with:

```text
IMAC5K-REBOOT:
```

## Modes

Select one mode on the kernel command line:

```text
amdgpu.imac5k_reboot_handoff=0
```

- `0`: log only; preserve the current working-kernel behavior.
- `1`: preserve receiver D0 on both tiles during normal teardown.
- `2`: write and verify root DPCD `0x4F1=0`, wait 10 ms, then run normal
  teardown. This is a CoreEG2/firmware-shaped experiment, not Windows parity.

Run mode `0` first. Run modes `1` and `2` as separate A/B tests only after the
mode-0 result is captured.

## What It Records

- exact root/slave route and active-pair gate;
- DC stream count before and after atomic suspend;
- pipe, TG, stream-encoder, and DIG enabled state;
- the exact per-tile blank, stream-disable, link-disable, and receiver-D3
  order;
- DPCD `0x100..0x10A`, `0x111`, `0x202..0x207`, `0x300..0x317`,
  `0x4F0..0x4F2`, and `0x600`;
- requested DPCD `0x600` writes, status, and readback;
- completion of `amdgpu_device_suspend()`.

Full DPCD snapshots are taken only before atomic teardown. After teardown the
trace records source/link software state without issuing extra AUX reads that
could perturb the firmware handoff.

## Preserve The Previous Kernel Log

The important messages happen after userspace is shutting down. On this kernel
configuration, enable the EFI pstore backend and shutdown kmsg dumping:

```text
efi_pstore.pstore_disable=0 printk.always_kmsg_dump=1
```

Do not increase `pstore.kmsg_bytes` initially; EFI variable storage is limited.

After the next boot, collect both locations because `systemd-pstore` may move
records out of sysfs:

```sh
sudo grep -h 'IMAC5K-REBOOT' /sys/fs/pstore/* 2>/dev/null
sudo grep -R -h 'IMAC5K-REBOOT' /var/lib/systemd/pstore 2>/dev/null
sudo journalctl -b -1 -k | grep 'IMAC5K-REBOOT'
```

Archive and remove old pstore records between runs so EFI NVRAM does not fill.

## Visual Result

Record the Apple/EFI boot logo before systemd-boot switches the display to its
4K mode:

- correct full-width image;
- left tile stretched across the panel;
- duplicated image;
- right tile missing;
- no visible logo.

The useful comparison is the mode-0 shutdown trace plus the visual result,
followed by one mode-1 run and one mode-2 run.
