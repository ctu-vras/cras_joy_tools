<!--
SPDX-License-Identifier: BSD-3-Clause
SPDX-FileCopyrightText: Czech Technical University in Prague
-->

# Standard Linux gamepad mappings

This directory contains mappings for translating gamepads to the following structure:

```yaml
buttons:
  0: A, green
  1: B, red
  2: X, blue
  3: Y, yellow
  4: LB (top left on back side)
  5: RB (top right on back side)
  6: Apps/Back button (left small)
  7: Options/Start button (right small)
  8: Vendor button (power-on)
  9: Left joy press
  10: Right joy press
axes:
  0: Left joy left-right (left = 1, right = -1)
  1: Left joy top-down (top = 1, down = -1)
  2: Axis for LT (fully released = 1, fully pressed = -1)
  3: Right joy left-right (left = 1, right = -1)
  4: Right joy top-down (top = 1, down = -1)
  5: Axis for RT (fully released = 1, fully pressed = -1)
  6: D-Pad/hat/arrows left-right (left = 1, right = -1)
  7: D-Pad/hat/arrows top-down (top = 1, down = -1)
```