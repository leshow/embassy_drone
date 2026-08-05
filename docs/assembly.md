# Drone Assembly

## Parts

- ESP32-S3 board
- ICM-20948 IMU breakout
- 4× 100N03A N-channel MOSFET (one per motor)
- 4× 10kΩ resistor (gate pull-down, one per MOSFET)
- 4× 8520 brushed DC motor
- 5V boost converter
- 2× 470μF electrolytic capacitor
- 1S LiPo battery (3.7V)
- 55mm or 65mm propellers

---

## Connection Tables

### Power

| From                   | To                            | Notes                                 |
| ---------------------- | ----------------------------- | ------------------------------------- |
| Battery +              | Boost converter IN+           |                                       |
| Battery −              | Boost converter IN−           |                                       |
| Boost converter 5V out | ESP32 VIN                     | 5V regulated                          |
| Boost converter GND    | ESP32 GND                     | Common ground for all logic           |
| 470μF cap (boost) +    | Boost converter 5V out        | Positive leg to 5V out                |
| 470μF cap (boost) −    | Boost converter GND           | Prevents brownout during motor inrush |
| 470μF cap (battery) +  | Battery +                     | Second cap, across the battery rail   |
| 470μF cap (battery) −  | Battery −                     |                                       |
| Battery +              | motor+                        | Raw battery rail to motor switches    |
| Battery − / GND        | All GND lines + mosfet source | Shared ground for motors and logic    |

### ICM-20948 IMU (SPI)

Uses the ESP32-S3's actual dedicated IO_MUX FSPI pins (GPIO10-13), not
GPIO-matrix-routed ones - avoids the extra input-delay ceiling on reliable
SPI clock speed that GPIO-matrix routing would otherwise impose.

| ICM-20948  | ESP32-S3 | Notes                                                                                                       |
| ---------- | -------- | ----------------------------------------------------------------------------------------------------------- |
| VIN        | 3.3V     | Regulated output from ESP32                                                                                 |
| GND        | GND      |                                                                                                             |
| SDI (SDA)  | GPIO11   | SPI MOSI                                                                                                    |
| SCLK (SCL) | GPIO12   | SPI clock                                                                                                   |
| SDO (AD0)  | GPIO13   | SPI MISO - if AD0 was tied to a fixed level for I2C addressing, remove that tie, it's the same physical pin |
| nCS        | GPIO10   | Actively driven by the ESP32, not tied to a fixed level - see Assembly Notes                                |
| INT        | GPIO6    | Data-ready interrupt                                                                                        |

### Motors (via 100N03A MOSFET)

| Motor       | Gate (ESP32) | Gate-Source   | Drain   | Source |
| ----------- | ------------ | ------------- | ------- | ------ |
| Front Left  | GPIO3        | 10k pull-down | Motor − | GND    |
| Front Right | GPIO4        | 10k pull-down | Motor − | GND    |
| Rear Left   | GPIO1        | 10k pull-down | Motor − | GND    |
| Rear Right  | GPIO2        | 10k pull-down | Motor − | GND    |

Motor + on all four motors connects directly to Battery +. Motor − connects to MOSFET Drain.

---

## Wiring Diagrams

### ESP32-S3 Connections

```text
          ┌───────────────────────┐
   3.3V ──┤                       ├──► ICM-20948 VIN
    GND ──┤                       ├──► ICM-20948 GND
          │       ESP32-S3        │
 GPIO11 ──┤ MOSI                  ├──► ICM-20948 SDI
 GPIO12 ──┤ SCLK                  ├──► ICM-20948 SCLK
 GPIO13 ──┤ MISO                  ├──► ICM-20948 SDO
 GPIO10 ──┤ CS                    ├──► ICM-20948 nCS
  GPIO6 ──┤ INT                   ├──► ICM-20948 INT
          │                       │
  GPIO3 ──┤ FL PWM                ├──► MOSFET FL Gate
  GPIO4 ──┤ FR PWM                ├──► MOSFET FR Gate
  GPIO1 ──┤ RL PWM                ├──► MOSFET RL Gate
  GPIO2 ──┤ RR PWM                ├──► MOSFET RR Gate
          └───────────────────────┘
```

### Motor Layout (top-down view)

```text
                    FRONT
                      ▲
                      │
      FL (GPIO3)      │       FR (GPIO4)
         ◎────────────┼────────────◎
         │            │            │
         │       ┌────┴────┐       │
         │       │ICM20948 │       │
         │       └────┬────┘       │
         │            │            │
         ◎────────────┼────────────◎
     RL (GPIO1)       │       RR (GPIO2)
                      │
                      ▼
                     BACK
```

### MOSFET Wiring (per motor, repeated ×4)

```text
  Battery + ──────────────────────── Motor +
                                       │
                                    [Motor]
                                       │
                                    Motor − ──── Drain
                                              [100N03A]
  ESP32 GPIO ──────────── Gate         Source ──── GND
                        ┌──┘
                     [10kΩ]        pull-down holds gate low during boot/reset
                        └──┐
                          GND
```

---

## Assembly Notes

- ICM-20948 nCS must be actively driven by the ESP32 (GPIO10), not tied to a fixed level — SPI requires CS to toggle low to select the chip and high between transactions; a permanently-high CS means the sensor never responds. If your board previously had CS wired to 3.3V for I2C mode, that connection needs to change to the GPIO10 signal wire instead. A weak pull-up (~10kΩ) on the CS line is fine to leave in place as a power-up safety net — a proper push-pull GPIO output easily overrides it — but a hard direct wire to 3.3V must be removed, or the GPIO driving CS low would short against it
- ICM-20948 AD0/SDO is a shared pin — if AD0 was previously tied to a fixed level for I2C address selection, remove that tie before using this pin as an active SPI MISO output; the two functions share the same physical pin
- FSYNC (unused): tie to GND rather than leaving it floating — it's an edge-sensitive digital input, and a floating input near motor/ESC electrical noise can pick up spurious transitions even though nothing reads it currently
- The 10kΩ gate–source resistor on each MOSFET holds the gate low when the ESP32 GPIO is floating (boot/reset), preventing unintended motor spin-up
- MOSFETs are wired low-side: Drain to Motor −, Source to GND, Motor + connects directly to Battery +. N-channel MOSFETs cannot be used as high-side switches with a 3.3V gate signal
- Solder one 470μF electrolytic capacitor across the boost converter 5V output and GND, as close to those pads as possible (positive leg to 5V). This prevents the ESP32 from browning out during motor inrush current spikes
- (if using boost converter with bat +/- out) Solder the second 470μF electrolytic capacitor across Battery + and Battery −, as close to the battery leads as possible (positive leg to Battery +). This buffers the raw battery rail against motor inrush current
- All four MOSFET sources share GND; run a single wire to a bus point and branch from there
- Keep IMU signal wires (MOSI/SCLK/MISO/CS/INT) routed away from motor wires to reduce noise coupling — this matters more for SPI than I2C was, since GPIO-matrix routing and breadboard/jumper signal integrity both degrade at the multi-MHz clock rates SPI runs at
- The ESP32 3.3V output powers the IMU; do not connect IMU VIN to the 5V boost output
