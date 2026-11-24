# Bill of materials: SO-101 arm with a camera (desk → backpack → voice)

In phases: each one already works on its own. Approximate prices in euros (October 2026); they vary
a lot depending on the shop, shipping and taxes: confirm before buying.

You already have: an **Orange Pi 5 Pro** (the brain), a 3D printer (assumed) and a PC with an NVIDIA
GPU (for training later on).

## Phase 1: the arm on the desk (~250 €)

| Qty | What | Approx. price | Notes |
|---|---|---|---|
| 6 | **Feetech STS3215-C018** servo (12 V, 30 kg·cm, 1/345 gearing) | ~28 € each | the SO-101 "follower" arm. The 7.4 V version (C001, ~25 €) is enough to look at and track faces; the 12 V one gives margin for holding things |
| 1 | **Waveshare Bus Servo Adapter (A)** servo board | ~5–12 € | USB ↔ servo bus (the one the SO-101 uses) |
| 1 | **12 V / 5 A** power supply (5.5 x 2.1 barrel connector) | ~12 € | for use on the desk |
| 1 | **32 x 32 mm USB UVC camera** (wide angle) | ~15–25 € | goes on the wrist (the SO-101 model already includes the mount) |
| 1 | **PETG filament, 1 kg** | ~20 € | SO-101 parts (~0.4 kg) + cradle, receiver and desk base (~0.3 kg) |
| 2 | C-clamps | ~10 € | for the desk base |
| — | Heatsink with fan for the Orange Pi 5 Pro, microSD card or eMMC, USB cable | ~15 € | if you do not have them |

Alternative: the "SO-ARM101" kits from Seeed, WowRobo and others bundle the servos, the board and
the power supply (some with printed parts and a camera). Compare the price with the list above.

## Phase 2: the backpack (~170 €)

| Qty | What | Approx. price | Notes |
|---|---|---|---|
| 3 | **Molicel P30B 18650** cells (or similar high-drain) | ~5 € each | 3S battery = 11.1 V (12.6 V charged) for the 12 V servos |
| 1 | 3 x 18650 battery holder + **3S 20 A BMS** | ~8 € | protection and balancing |
| 1 | 3S balance charger | ~25 € | |
| 1 | **12 V → 5 V / 5 A** converter (USB-C or terminals) | ~8 € | the Orange Pi 5 Pro needs 5 V / 4 A |
| 1 | **BNO085 IMU** (or BNO055) | ~20–30 € | on the cradle, under the arm base (compensates for the sway) |
| 1 | **Emergency stop button** (mushroom, latching) | ~8 € | on the shoulder strap |
| 1 | **15 A MOSFET** module (or 12 V / 10 A relay) | ~4 € | the button cuts the servo power |
| 1 | **A3144 Hall** sensor | ~1 € | on the hinge: detects whether the arm has folded |
| 1 | Fuse holder + 7.5 A fuse, XT60 connectors | ~6 € | |
| 1 | Fireproof battery bag | ~8 € | the battery goes inside the box |
| 1 | 20 x 20 x 1.5 mm square aluminium tube (cut to 142 mm) | ~6 € | hardware store |
| 1 | 3 mm HDPE or PP sheet, 25 x 38 cm | ~10 € | or a plastic chopping board |
| 1 | **PETG filament, 1 kg** (another spool) | ~20 € | backpack mount parts (~470 g) and the box |
| 8 | N52 neodymium magnets, 10 x 5 mm | ~6 € | hinge (glued in with epoxy) |
| 1 | M8 x 70 bolt + locknut + 2 nylon washers | ~3 € | hinge pin |
| — | Assorted M5 screws (12–30 mm), nuts and washers; 1 M4 x 10 screw | ~10 € | |
| — | 1 mm rubber or TPU, epoxy, cable ties, 18 AWG silicone wire | ~10 € | |

Plus the backpack: any one with a **laptop pocket, chest strap and hip belt**.

## Phase 3: voice (~40 €)

| Qty | What | Approx. price | Notes |
|---|---|---|---|
| 1 | Small USB microphone (or a ReSpeaker-style microphone array) | ~10–30 € | the array hears better with street noise |
| 1 | Small speaker + MAX98357 I2S amplifier (or a USB speaker) | ~10–15 € | |

The answers come from a vision-language model in the cloud (over the phone's hotspot) or a small
one on the Orange Pi 5 Pro: to be decided in that phase.

## Optional: "leader" arm to teach it tasks (~120 €)

For LeRobot imitation learning (you move the leader by hand and the follower copies it):
6 STS3215 7.4 V servos with different gearings depending on the joint (see the SO-101 guide) +
another Waveshare board. This can be left for later.

## Summary

| Phase | Approx. |
|---|---|
| 1. Desk | ~250 € |
| 2. Backpack | ~170 € (+ the backpack) |
| 3. Voice | ~40 € |
| Optional: leader | ~120 € |
| **Total without the leader** | **~460 €** |

## Before buying

- Decide on **7.4 V or 12 V** for the servos (they do not mix on the same bus): 12 V is what I
  recommend for the margin, and with that the backpack battery is 3S.
- Print the **rail coupons** (`cad/stl/rail_coupon.stl`, `cad/stl/receiver_coupon.stl`) and
  measure the SO-101 base.
