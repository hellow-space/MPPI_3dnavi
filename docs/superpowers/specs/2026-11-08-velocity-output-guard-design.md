# Single velocity output with a downstream zero guard

Approved in conversation on 2026-11-08. All existing navigation scripts keep
launching `src/keep_zero.py`. Both navigation launch files route the velocity
smoother's `cmd_vel_smoothed` output to `/command/cmd_twist_smoothed`. The guard
consumes that topic and is the sole publisher of `/cmd_vel`.

Normal finite Twist messages are forwarded immediately. After 0.3 seconds
without valid input, zeros are published at 20 Hz, including before the first
command (after the existing startup grace period). Input and output queue depth
is one. Timeout measurement and the watchdog timer use steady time. NaN/Inf
messages neither forward nor renew the timeout.

Direct smoother output was considered, but loses continuous idle zeros and
independent protection against smoother failure. An upstream guard cannot
observe smoother failure. The downstream guard preserves both properties.

Existing smoother parameters remain unchanged: 20 Hz, 0.3-second raw input
timeout, configured acceleration/deceleration limits. Raw input loss initiates
smooth deceleration; it does not guarantee a physical stop within 0.3 seconds.
Chassis-side communication timeout remains necessary if the guard itself exits.
Do not restart or change the currently running robot during verification.
