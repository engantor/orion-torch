# ORION — Current State

Snapshot for the design collaborator. **This reflects the code as it actually
builds and runs on the target machine**, which differs in several places from the
original design chat's assumptions. Read the "Changes during build" section —
those are the deltas you have not seen.

Last updated: 2026-07-14.

---

## Files in this repo

| File | Purpose |
|---|---|
| `ShipCore.h` / `ShipCore.cpp` | **Engine-agnostic** ship systems model: reactor state machine, electrical bus solve, coolant/thermal loop, torch drive + mass-augmentation mixing. Zero Orbiter dependencies. Unit-tested headless. |
| `Avionics.h` / `Avionics.cpp` | **Engine-agnostic** IMU (gimbal lock, P52 star alignment), GDC backup reference, pressure-fed RCS with helium makeup from reactor ash. Zero Orbiter dependencies. |
| `Orion.h` | Declaration of the `Orion` `VESSEL4` class. Extracted from `Orion.cpp` so the separate DPS MFD DLL can cast the focus vessel and read the systems model via inline `Core()`. |
| `Orion.cpp` | Thin Orbiter `VESSEL4` glue. Owns one `ShipCore`, steps it at fixed 0.1 s, maps its output to Orbiter (thrust, Isp), save/load, temporary keyboard controls, debug string. No physics. |
| `OrionDPS.cpp` | **Data Processing System** MFD (Shuttle-GPC/MEDS style), separate plugin DLL. Phase 1+2 built: MDU render, keyboard + scratchpad, SPEC/DISP page switching, and DISP 78 SM SYS SUMM 1 reading live `ShipCore`. |
| `OrionNavMFD.cpp` | Nav MFD plugin (separate DLL). Has a **real Lambert solver** (universal variables) + a brachistochrone estimate. **Not yet built.** Note: its `MsgProc` returns `int` and casts the instance pointer to `int` — that truncates on x64 and would crash on open; `OrionDPS.cpp` shows the corrected pattern. |
| `test_core.cpp` | 9 headless tests for `ShipCore` (cold start, dry-start abort, coil quench, stranding, drive gating, mixing ratio, design point, avionics fold-in, persistence). |
| `test_avionics.cpp` | 8 headless tests for `Avionics` (P52 alignment, maneuver rejection, gimbal lock, GDC survival, realign cadence, RCS helium gating, frozen quad, helium makeup). |
| `build_tests.bat` | Compiles and runs both headless suites with MSVC. No Orbiter needed. |
| `Orion.cfg` | Vessel class config. `MeshName = ShuttlePB` (stock stand-in mesh). |
| `Orion Cold Start.scn` | Scenario: cold-and-dark Orion in a 185 km parking orbit. |
| `BUILD_WINDOWS.md` | Original build/setup walkthrough from the design chat. **Partly stale** — see below. |
| `orion.slnx`, `orion/orion.vcxproj`, `orion/orion.vcxproj.filters` | Visual Studio project for the vessel DLL. |
| `claude/panel_gen.py` | 2D panel generator — single source of truth for artwork + hit-boxes. |
| `claude/PanelAreas.h` | Auto-generated enum + `PANEL_AREAS[72]` rects (from `panel_gen.py`). |
| `claude/orion_panel.png`, `claude/ship_systems.pdf`, `claude/ship_systems.tex` | Panel artwork + physics design writeup. |

---

## ShipCore — public API (signatures only)

```cpp
// free function
const char* StageName(RctStage s);

class ShipCore {
    ShipCore();

    void Step(double dt);                                  // call with fixed dt <= 0.25 s

    // commands
    void ArmBank(int i, bool on);
    void SetBreaker(const std::string& name, bool closed); // names: GEN_A GEN_B TIE ESS
                                                           //   INV1 INV2 AVI LSS PUMP1
                                                           //   PUMP2 DRV RCS SCI
    bool ResetTrip(const std::string& name);
    void StartReactor(int i);
    void ShutdownReactor(int i);
    void SetThrottle(double g);                            // commanded accel [g], clamped 0..1
    void SetMixture(double R);                             // dilution ratio, MIX_MIN..MIX_MAX

    // avionics access
    Avionics&       Avio();
    const Avionics& Avio() const;
    void SetAttitude(double roll, double pitch, double yaw, double bodyRate);

    // queries
    const ElecState& Elec() const;
    const Reactor&   Rct(int i) const;
    const Breakers&  Bkr() const;
    const Bank&      BankAt(int i) const;
    bool  Tripped(const std::string& n) const;

    double CoolantT()  const;
    double RadiatorT() const;
    double TorchT()    const;
    double HeatIn()    const;
    double HeatOut()   const;

    double ThrottleLevel() const;                          // 0..1 delivered to Orbiter
    bool   DriveFailed()   const;
    double ThrottleCmd()   const;

    // mass-augmentation readouts
    double Mixture()   const;
    double ExhaustV()  const;                              // [m/s]
    double Isp()       const;                              // [s]
    double JetPower()  const;                              // [W]
    double Thrust()    const;                              // [N]
    double MdotFuel()  const;                              // [kg/s]
    double MdotInert() const;                              // [kg/s]
    double FuelKg()    const;
    double InertKg()   const;
    double MaxAccelG() const;

    void SetVesselMass(double kg);                         // Orbiter owns mass; feed it in

    const std::deque<std::string>& Log() const;
    void  Say(const std::string& msg);

    // persistence (scenario file)
    std::string Serialize() const;
    void        Deserialize(const std::string& line);
};
```

## Avionics — public API (signatures only)

```cpp
// free functions
const char* AlignOptName(AlignOpt o);
const char* AlignPhaseName(AlignPhase p);

class Avionics {
    Avionics();

    void Step(double dt, double met, double bodyRate,
              bool aviPower, bool rcsPower, double fusionPower);

    // IMU
    void   SetAlignOption(AlignOpt o);
    void   SetAlignInterval(double sec);
    void   StartAlign(double met, double bodyRate);        // "P52"
    void   AbortAlign();
    bool   AlignDue(double met) const;
    double TimeToAlign(double met) const;

    // GDC
    void AlignGdcToImu();

    // attitude feed
    void SetTrueAttitude(double roll, double pitch, double yaw);

    // RCS
    void   SetQuadEnabled(int i, bool on);
    void   SetHeTank(int i, int tank, bool open);
    void   SetQuadHeater(int i, bool on);
    double FireQuad(int i, double demandImpulse, double dt); // returns impulse delivered [N.s]

    // queries
    const Imu&     GetImu() const;
    const Gdc&     GetGdc() const;
    const RcsQuad& Quad(int i) const;
    double HeReserve() const;
    double TotalRcsProp() const;
    const std::deque<std::string>& Log() const;
    void Say(const std::string& m);
};
```

> These signatures are **unchanged** from the original design. `ShipCore.{h,cpp}`
> and `Avionics.{h,cpp}` were compiled as-is — the physics core needed no edits.

---

## Changes during the build (deltas from the original design)

These are the things the design chat has **not** seen:

1. **Built 64-bit (x64), not Win32/x86.** The installed Orbiter is a 64-bit build;
   `Orbiter.lib` / `Orbitersdk.lib` are x64 (verified with `dumpbin`). Building
   Win32 as the design assumed would fail to link with "unresolved external
   symbol" errors. `BUILD_WINDOWS.md`'s "Orbiter is 32-bit" guidance is stale for
   this install.
2. **Orbiter lives at `F:\Games\Orbiter-x64`, not `C:\Orbiter2024`.** SDK at
   `F:\Games\Orbiter-x64\Orbitersdk\{include,lib}`; DLL output goes to that
   install's `Modules\`.
3. **`Orion.cpp`: added `SetThrusterIsp(th_main_, core_.ExhaustV())` every step**
   in `clbkPreStep`. The mixing valve changes exhaust velocity at runtime, so a
   thruster's Isp is not fixed once created — this call was missing.
4. **`Orion.cpp`: `AddMesh("Orion\\orion")` → `AddMesh("ShuttlePB")`.** No Orion
   mesh exists yet; this matches `Orion.cfg`'s `MeshName = ShuttlePB` so the ship
   is visible immediately.
5. **`Orion.cpp`: added an `oapiDebugString` telemetry line** in `clbkPreStep`
   (reactor stage, Q, B, battery %, coolant T, throttle) so startup is observable
   before the panel exists.
6. **`orion/orion.vcxproj` rewritten** for this machine: x64-only configs, PCH off,
   `/MD` runtime, Multi-Byte character set, SDK include/lib dirs, `Orbiter.lib;
   Orbitersdk.lib`, output dir = Orbiter `Modules\`, target name `Orion`. The
   auto-generated `dllmain.cpp` / `pch.*` / `framework.h` stubs were removed.
7. **`Orion Cold Start.scn`: corrected a debug hint.** `Ctrl+F1` opens the camera
   dialog in this install's keymap, not a debug toggle; the debug string renders
   automatically once populated.
8. **`ShipCore.{h,cpp}` and `Avionics.{h,cpp}`: NOT changed.** The core compiled
   and passed all tests unmodified.

Also worth flagging for the design chat: `BUILD_WINDOWS.md` Step 1 points at
`https://cdn.openorbiter.space/...` for the Orbiter download — that is not a
recognized official Orbiter distribution source and should be corrected in the
doc.

---

## What works / stubbed / broken

**Works**
- Physics core: 9 core + 8 avionics headless tests all PASS (`build_tests.bat`).
- Vessel DLL builds clean (x64) and loads in Orbiter as `Orion`.
- Cold-start reactor sequence runs and is visible on the debug string:
  PUMPDOWN → FIELD → HEAT → IGNITE → ASCENT → ONLINE.
- Temporary keyboard controls (`1`–`6`, `+`/`-`).
- Thrust + runtime Isp (mass augmentation) fed to Orbiter; scenario save/load.
- **DPS MFD (`OrionDPS.dll`, Phase 1+2):** builds clean, renders a GPC/MEDS-style
  MDU with a working keypad + scratchpad, SPEC/DISP page switching, and a live
  DISP 78 SM SYS SUMM 1 reading `ShipCore` (EPS/reactor/thermal/prop/GNC/RCS,
  colour-coded to the real thresholds). Two DISP 78 fields have no backing model
  and are proxied, not faked: **CNTL** (no control-bus model → shows AVI power)
  and **FLOW kg/s** (no mass-flow value → shows `OK`/`NONE` from pump count).

**Stubbed / not done**
- **DPS beyond Phase 2:** item-entry actuation, fault system, OPS/major modes,
  GPC voting, and the SPEC pages other than DISP 78 are placeholders. Per the
  DPS spec, several later pages need `ShipCore` to grow (control buses, richer
  EPS, environment) — DISP 78 itself needed no core changes.
- **2D instrument panel: not implemented.** `panel_gen.py`, `PanelAreas.h`, and
  `orion_panel.png` exist as design assets, but no panel code
  (`clbkLoadPanel2D` / `clbkPanelMouseEvent` / `clbkPanelRedrawEvent`) is in
  `Orion.cpp` yet, and no DDS texture has been generated. (Superseded in priority
  by the DPS; the DPS spec discards the panel-era DSKY draft.)
- **`OrionNavMFD` (nav MFD plugin DLL): not built** (and needs the x64 `MsgProc`
  fix noted above before it will run).
- **Custom mesh: none** — uses the stock `ShuttlePB` stand-in.
- **RCS thruster groups: only pitch up/down defined** in `clbkSetClassCaps`;
  yaw / roll / translation groups are marked as a TODO in the code.
- **IMU attitude is not fed from Orbiter** — `clbkPreStep` does not call
  `core_.SetAttitude(...)`, so gimbal-angle and drift readouts will read zero
  until that is wired (needs a chosen reference frame — a design decision).

**Broken**
- None known.
