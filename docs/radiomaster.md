# RadioMaster Pocket as USB Gamepad for ground_control

How to configure a RadioMaster Pocket (EdgeTX) to act as a drop-in USB HID gamepad for `ground_control`, so the stick layout matches a regular Xbox-style controller with no code changes.

## Why this isn't plug and play

`ground_control` reads input through [gilrs](https://gitlab.com/gilrs-project/gilrs), which recognizes controllers by matching an SDL-style mapping database (Xbox controllers, PlayStation controllers, etc. are all in there). EdgeTX radios in USB joystick mode present a generic, fully configurable HID device under EdgeTX's own vendor ID, which is in nobody's mapping database.

Without a database entry, gilrs's Linux backend falls back to a fixed positional mapping based on the raw evdev axis/button codes (`native_ev_codes`):

| evdev code           | gilrs axis/button   |
| -------------------- | ------------------- |
| `ABS_X`              | `Axis::LeftStickX`  |
| `ABS_Y`              | `Axis::LeftStickY`  |
| `ABS_RX`             | `Axis::RightStickX` |
| `ABS_RY`             | `Axis::RightStickY` |
| `BTN_START` (0x13b)  | `Button::Start`     |
| `BTN_SELECT` (0x13a) | `Button::Select`    |

`ground_control` (`ground_control/src/main.rs`) maps those to controls:

- `LeftStickY` -> throttle
- `LeftStickX` -> yaw
- `RightStickX` -> roll
- `RightStickY` -> pitch
- `Button::Start` -> toggles armed (momentary press, matches a real Xbox controller's Start button)
- `Button::Select` -> armed follows the switch position directly (pressed = armed, released = disarmed), but only from a gamepad whose USB vendor/product id matches the Pocket's (`1209:4f54`) - so an Xbox controller's Back/View button, which also reports as `Button::Select`, can never trigger it

So the axis layout is just: configure the Pocket's USB joystick output to
match this table exactly, and gilrs/`ground_control` will treat it
identically to any other gamepad. The arm/disarm switch below relies on the
device-id check already built into `ground_control`, not on more config here.

## Use a dedicated Model

Do this setup in its own EdgeTX Model (Model Select screen), separate from any model you bind to a real receiver over ELRS later. Mixer, Outputs, and USB Joystick settings are all per-model in EdgeTX - nothing here can leak into a flight model, but it's one less thing to think about if the USB test config and the real flight config never share a save file.

## Advanced mode may not be available on your firmware

The instructions below assume EdgeTX's "Advanced" USB Joystick mode
(free-form per-channel axis/button picker, per-axis invert, Gamepad/Joystick
toggle). The Pocket uses an STM32F407xE chip with only 512KB flash, and since
EdgeTX 2.11 the build unconditionally disables this feature (`USBJ_EX`) for
that flash size because it doesn't fit
(`radio/src/targets/taranis/CMakeLists.txt`, confirmed in
[EdgeTX/edgetx#6434](https://github.com/EdgeTX/edgetx/issues/6434)). If the
USB Joystick page only shows Classic mode with no Advanced option, you're on
2.11+ and this is why - it's not a menu you're missing.

Three ways to proceed, roughly in order of effort:

1. **Flash EdgeTX 2.10.6** - the last release with Advanced mode intact on the
   Pocket. A normal firmware flash via EdgeTX Buddy/the bootloader, no
   building required. Since this is a new model anyway there's no old-model
   storage format to migrate. Back up the SD card first as a matter of
   course. Once on 2.10.6, follow the Advanced-mode steps below as written.
1. **Stay on stock 2.11+ firmware and use Classic mode instead** - no
   firmware changes at all, but more limited (fixed channel-to-axis order, no
   per-axis invert). See "USB Joystick setup - Classic mode" further down.

## USB Joystick setup - Advanced mode

1. Press MDL, page to SETUP
1. Set USB mode to **Advanced**
1. Set **Interface Mode** to **Gamepad**, not Joystick. This matters more than
   it sounds: Linux's HID driver maps HID button usages to a totally different
   evdev range depending on this setting (`BTN_TRIGGER`-family for Joystick,
   `BTN_SOUTH..BTN_THUMBR`-family for Gamepad). gilrs's fallback mapping only
   recognizes the Gamepad range, under Joystick mode, no button will ever
   register as `Button::Start`, no matter which number you pick.
1. In Channel Settings, assign axes by **which physical stick drives the
   channel**, not by channel number (channel order depends on your model's
   template/mode and won't match this table 1:1):

   | Control                              | Axis |
   | ------------------------------------ | ---- |
   | Rudder / yaw (left stick, horiz)     | X    |
   | Throttle (left stick, vert)          | Y    |
   | Aileron / roll (right stick, horiz)  | rotX |
   | Elevator / pitch (right stick, vert) | rotY |

   Leave every other channel `None` for now.

1. Arm/disarm: `ground_control` supports two independent controls here, wire
   up either or both:
   - **SE** (momentary button): a good fit precisely because it's momentary,
     not latching - each press cleanly toggles arm state without a mismatch
     between switch position and armed state. Pick a spare channel, set its
     source to SE, channel type **Btn**, button number **12** (HID button
     usage 12 -> evdev `BTN_START` -> gilrs `Button::Start`).
   - **SA** (2-position switch): a real latching switch, so `ground_control`
     treats it as a live level rather than a toggle - armed always matches
     SA's current position, up or down, even across a USB reconnect. Pick a
     different spare channel, set its source to SA, channel type **Btn**,
     button number **11** (HID button usage 11 -> evdev `BTN_SELECT` -> gilrs
     `Button::Select`).
1. If an axis reads backwards (e.g. throttle low at stick-up), use the invert
   control inside this same USB Joystick screen (not the Mixer's channel
   weight) - it only affects how the value is packaged into the HID report,
   not the underlying channel output other things (like a real RF link) would
   read.

After any change here, disconnect and reconnect USB, you may need to power cycle the controller too.

## USB Joystick setup - Classic mode

If you're staying on stock 2.11+ firmware (option 3 above), Classic mode has
no per-channel axis picker - it maps a fixed channel order onto axes and
buttons: **channel 1->X, 2->Y, 3->Z, 4->rotX, 5->rotY, 6->rotZ, 9-32->buttons
1-24** (button N = channel N+8). There's also no per-axis invert control - use
Mixer Weight/Offset on the channel instead. Interface Mode (Gamepad vs
Joystick) isn't selectable either, but that's fine: without `USBJ_EX` it's
hardcoded to Gamepad, which is what we want anyway (confirmed in the same
issue thread - someone without `USBJ_EX` had to hand-patch firmware source to
get Joystick type instead, meaning Gamepad is the compiled-in default).

Since Classic mode just reads whatever value ends up on a given channel
number, the trick is to **use the Mixer to put the right control on the right
channel number**, rather than assigning an axis to an existing channel:

1. In Model Setup (or Outputs page), set **Number of Channels** to at least
   **20** - channel 20 is needed for the arm button below, and the default
   model channel count usually doesn't go that high.
2. Turn off the internal/external RF module for this model (recommended by
   the EdgeTX manual for USB joystick use, and irrelevant anyway since this
   model is USB-only).
3. In the Mixer, set up these channels (delete/reassign whatever the default
   template put on CH1/2/4/5 first):

   | Channel | Source                               | -> Axis (Classic order)   |
   | ------- | ------------------------------------ | ------------------------- |
   | CH1     | Rudder / yaw (left stick, horiz)     | X                         |
   | CH2     | Throttle (left stick, vert)          | Y                         |
   | CH4     | Aileron / roll (right stick, horiz)  | rotX                      |
   | CH5     | Elevator / pitch (right stick, vert) | rotY                      |
   | CH19    | **SA** switch (2-position)           | button 11 -> `BTN_SELECT` |
   | CH20    | **SE** button (momentary)            | button 12 -> `BTN_START`  |

   Leave CH3, CH6-18 unassigned - Z/rotZ (CH3/CH6) aren't read by
   `ground_control`, and gilrs doesn't care about gaps in the button range.
   CH19/SA and CH20/SE are independent arm/disarm controls - wire up either
   or both. SE toggles arm on each press (momentary, so no
   switch-position/armed-state mismatch); SA's position is read live, so
   armed always matches wherever the switch currently sits.

4. If an axis reads backwards (e.g. throttle low at stick-up), fix it on the
   same Mixer line with **Weight -100%** instead of a dedicated invert
   control. Since this model is USB-only, inverting the real channel value
   here is fine - it doesn't feed a real RF link.
5. Optional: since a gamepad thumbstick springs back to center but the
   Pocket's throttle stick doesn't, you can use the _entire_ physical throttle
   throw for 0-100% instead of just the top half (which is what
   `ground_control` expects from a self-centering stick) by setting the CH2 mix to **Weight 50%,
   Offset +50** (adjust sign to taste). That compresses the full stick travel
   into the positive half of the reported axis, giving full resolution with
   no code changes. This trick works the same way regardless of Classic vs
   Advanced mode, since it's purely a Mixer-level rescale.

After any change here, disconnect and reconnect USB - you may need to power
cycle the controller too (see the USB wedge gotcha below).

## Verifying

Check the device enumerated correctly and has axes/buttons mapped right:

```sh
journalctl -k -n 100 | grep -i "radiomaster\|1209:4f54"
udevadm info -q property -n /dev/input/eventNN | grep ID_INPUT
```

You want to see `ID_INPUT_JOYSTICK=1`. If it's missing, the device has no axes/buttons configured yet.

Then run `ground_control` and watch its existing log lines while moving each stick and the arm button - `throttle:`, `roll:`, `pitch:`, `yaw:`, and `armed:` to confirm the mapping and polarity match a normal controller before flying anything.
