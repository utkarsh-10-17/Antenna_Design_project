# 📡 Wide Band Dual Beam U-Slot Microstrip Antenna

This repository contains design files, simulation results, and documentation for a **Wide Band Dual Beam U-slot microstrip antenna**, created as part of the **Antenna Design** project.

The antenna operates across the **5.7–6.0 GHz** and **7.7–8.4 GHz** bands (C and X bands), with a peak gain of **5.052 dBi**, supporting applications like Wi-Fi, radar, wireless backhaul, and satellite communication.

---

## 📌 About the Project

The design uses:
- A **rectangular patch** with a U-shaped slot to enable dual-band operation.
- A **microstrip line feed** for simple integration.
- An **FR4 substrate** (εr = 4.4, tanδ ≈ 0.02) with dimensions 67 mm × 50 mm × 3.175 mm.

By introducing the U-slot, the antenna achieves:
- Multiple resonances → **broader bandwidth**
- **Dual beam radiation** → better spatial coverage

---

## 🛠️ Tools & Software Used

- **CST Studio Suite** (for simulation and modeling)
- (Optional) **AutoCAD** or similar CAD tool for drawing mechanical layouts
- Standard text/PDF viewers

---

## ⚙️ Design Highlights

- **Operating bands:** 5.7–6.0 GHz (C band) and 7.7–8.4 GHz (X band)
- **Return loss (S11):** Better than -10 dB across both bands; simulated ~ -25 dB at resonance
- **Peak gain:** ~5.052 dBi
- **Dual beam:** ±30° off broadside, ~40° beamwidth per lobe
- **Substrate:** FR4 (εr=4.4)
- **Compact footprint:** 67 mm × 50 mm

---

## 📊 Results (From CST Studio Suite)

- **S-parameters:** Strong impedance matching at both resonant frequencies
- **3D farfield radiation pattern:** Shows dual beams
- **Polar plot and Smith chart:** Validate directional radiation and impedance behavior
- **Efficiency:** >85%

---

Open u_slot_antenna.cst in CST Studio Suite.

Run simulations to view S-parameters, farfield radiation, and Smith chart.

