# ORION — Windows Setup, Start to Finish

Nothing here costs money. Total download ≈ 6 GB (most of it Visual Studio).

Work through it in order. **Steps 1–3 get you a working, tested physics core
before Orbiter is even involved** — that's deliberate. If the tests pass and
something later breaks, you know it's an integration bug, not a physics bug.
That halves your debugging forever.

---

## STEP 1 — Install Orbiter 2024

1. Download: **https://cdn.openorbiter.space/orbiter-releases/Orbiter-2024.zip**
   (Official release, 31 Dec 2024. Mirror: https://github.com/orbitersim/orbiter/releases)

2. **It is a ZIP, not an installer.** Extract it to a path with **no spaces**:

       C:\Orbiter2024

   Not `C:\Program Files\...` — Orbiter writes to its own folder, and
   Program Files is permission-protected. This causes hours of confusion.

3. Run `Orbiter_ng.exe` (the launcher). Pick any scenario, e.g.
   *Checklists → Delta-glider → Orbital Ops*. Fly for 30 seconds.

   **If Orbiter doesn't run, stop and fix that before anything else.**

4. In the launcher, go to the **Modules** tab and tick:
   - `ScnEditor` — the in-game scenario editor. You will want this to
     place and re-orbit your ship without hand-writing scenario files.
   - `D3D9Client` if present (nicer graphics; optional).

You now have the SDK too — look in `C:\Orbiter2024\Orbitersdk\`. That folder
has `include\` (the headers) and `lib\` (the link libraries). You'll point
Visual Studio at both.

---

## STEP 2 — Install Visual Studio 2022 Community (free)

1. Download: **https://visualstudio.microsoft.com/vs/community/**

2. In the installer, tick exactly one workload:

       [x] Desktop development with C++

   That's it. You don't need the game/Azure/Python workloads. ~5 GB.

3. Finish and reboot if it asks.

> **Not Visual Studio *Code*.** Different product. You want Visual Studio 2022,
> the full IDE, because it gives you the MSVC compiler and the DLL project type.

---

## STEP 3 — Prove the physics works (no Orbiter yet)

Put all the project files in one folder, e.g. `C:\dev\orion\`:

    ShipCore.h      ShipCore.cpp
    Avionics.h      Avionics.cpp
    Orion.cpp       OrionNavMFD.cpp
    test_core.cpp   test_avionics.cpp
    build_tests.bat Orion.cfg

Open **Start → "x86 Native Tools Command Prompt for VS 2022"** (search for it).
Then:

    cd C:\dev\orion
    build_tests.bat

You should see every line end in **PASS** — reactor cold start, dry-start abort,
coil quench, stranding, the mixing-ratio table, gimbal lock, RCS helium.

**This is the whole point of the architecture.** `ShipCore` has zero Orbiter
dependencies, so it compiles and tests in two seconds on any machine. Orbiter
can't break it and doesn't need to be involved.

---

## STEP 4 — Create the addon DLL project

1. Visual Studio → **Create a new project**
2. Choose **Dynamic-Link Library (DLL)** — C++, Windows.
   Name it `Orion`. Location: `C:\dev\`.
3. Delete the auto-generated `dllmain.cpp`, `framework.h`, `pch.h`, `pch.cpp`.
4. **Project → Add Existing Item** and add:
   `ShipCore.h`, `ShipCore.cpp`, `Avionics.h`, `Avionics.cpp`, `Orion.cpp`

---

## STEP 5 — Configure the project (this is where people get stuck)

Open **Project → Orion Properties**. Set **Configuration: Release**.

### 5a. Platform — check this first

Look in `C:\Orbiter2024\Orbitersdk\lib\`. Orbiter has historically been a
**32-bit** application, so you almost certainly want:

    Solution Platforms dropdown (top toolbar):  x86    (a.k.a. Win32)

**Whatever platform Orbiter's `Orbiter.lib` was built for, match it.** A
32/64-bit mismatch produces "unresolved external symbol" errors that look
like missing code but aren't. If your DLL won't link and the symbols clearly
exist, this is the first thing to suspect.

### 5b. Settings

| Where | Setting | Value |
|---|---|---|
| C/C++ → General → Additional Include Directories | | `C:\Orbiter2024\Orbitersdk\include` |
| C/C++ → Language → C++ Language Standard | | ISO C++17 |
| C/C++ → Precompiled Headers | Precompiled Header | **Not Using Precompiled Headers** |
| C/C++ → Code Generation → Runtime Library | | Multi-threaded DLL (`/MD`) |
| Linker → General → Additional Library Directories | | `C:\Orbiter2024\Orbitersdk\lib` |
| Linker → Input → Additional Dependencies | | `Orbiter.lib;Orbitersdk.lib;%(AdditionalDependencies)` |
| General → Character Set | | **Use Multi-Byte Character Set** |
| General → Output Directory | | `C:\Orbiter2024\Modules\` |
| General → Target Name | | `Orion` |

Setting the **Output Directory to Orbiter's `Modules\` folder** means every
build drops `Orion.dll` straight where Orbiter looks for it. No copying.

### 5c. Build

**Build → Build Solution** (Ctrl+Shift+B).

Result: `C:\Orbiter2024\Modules\Orion.dll`

---

## STEP 6 — Install the vessel config and scenario

Copy these two files:

    Orion.cfg              ->  C:\Orbiter2024\Config\Vessels\Orion.cfg
    Orion Cold Start.scn   ->  C:\Orbiter2024\Scenarios\Orion Cold Start.scn

`Orion.cfg` currently uses `MeshName = ShuttlePB` — a **stock Orbiter mesh**.
Your ship will look like a little pushbutton shuttle. That's intentional: it
makes the vessel *visible immediately* so you can test systems now and model
the real hull later. Change one line when you have a mesh.

---

## STEP 7 — Fly it

1. Run `Orbiter_ng.exe`
2. Scenario list → **Orion Cold Start**
3. Launch.

The ship is **cold and dark**. Run the startup:

| Key | Action |
|---|---|
| `1` | Arm all three battery banks |
| `2` | Close INV 1 + AVIONICS + LIFE SUPPORT |
| `3` | **Coolant PUMP 1** — before the reactor, or it scrams in 30 s |
| `4` | **START REACTOR A** — watch it climb: PUMPDOWN → FIELD → HEAT → IGNITE → ASCENT → ONLINE (~5 min) |
| `5` | Close GEN A + ESS FEED — the reactor now carries the ship, banks recharge |
| `6` | Close DRIVE CTL — the torch will not fire without this |
| `+` / `-` | Throttle |

Speed it up with `T` (time acceleration) while the reactor spins up.

To see what's happening, log to Orbiter's debug line — add this to the end of
`clbkPreStep` in `Orion.cpp`:

```cpp
sprintf(oapiDebugString(),
    "RCT A: %s  Q=%.1f  B=%.1fT | BATT %.0f%% | Tcool %.0fK | thr %.2f",
    orion::StageName(core_.Rct(0).stage), core_.Rct(0).Q, core_.Rct(0).B,
    100.0 * core_.Elec().soc, core_.CoolantT(), core_.ThrottleLevel());
```

That one line turns Orbiter into a debugger for your reactor.

---

## STEP 8 — The nav MFD (optional, do it after the vessel works)

`OrionNavMFD.cpp` is a **separate DLL** — MFDs are plugins, not vessels.

1. New DLL project `OrionMFD`, same settings as Step 5.
2. Add only `OrionNavMFD.cpp`.
3. Output directory: `C:\Orbiter2024\Modules\Plugin\`
4. In the Orbiter launcher → **Modules** tab → tick **OrionMFD**.
5. In flight, press an MFD's **SEL** button and pick *Orion Nav*.

---

## THINGS THAT WILL COST YOU AN HOUR EACH

1. **Time acceleration destroys ODE integrators.**
   Orbiter's `simdt` reaches hundreds of seconds at 10000×. Feeding that to
   the reactor model produces garbage. `Orion.cpp` already handles this:
   it accumulates `simdt` and steps `ShipCore` at a fixed 0.1 s. **Never**
   pass `simdt` straight into an integrator.

2. **The DLL is not unloaded between scenarios.**
   Global/static mutable state survives into your next flight and you'll chase
   a ghost. Keep all state inside the vessel object. (The code already does.)

3. **`Isp` in `CreateThruster` is exhaust velocity in m/s, not seconds.**
   The torch passes `9.81e6` directly. And because the mixing valve changes
   exhaust velocity at runtime, `Orion.cpp` must call `SetThrusterIsp()` every
   step — a thruster's Isp is *not* fixed once created.

4. **Thruster handles must be created in `clbkSetClassCaps`.**
   Touching them earlier is a crash to desktop, not an error message.

5. **Install path with spaces / Program Files.** Just don't.

---

## WHAT TO READ WHEN YOU'RE STUCK

- **Orbiter Forum** — https://www.orbiter-forum.com — the addon developers live here.
- **`Orbitersdk\samples\`** — working source for stock vessels. `ShuttlePB` is
  the minimal vessel example; `DeltaGlider` is the full-featured one with 2D
  panels and a virtual cockpit. When you build the panel, read DeltaGlider.
- **NASSP (Project Apollo)** — an Orbiter addon simulating Apollo down to
  individual circuit breakers. Proof the fidelity you want is achievable, and
  the best code to learn panel/systems architecture from.
- **Space Shuttle Vessel + Shuttle FDO MFD** — https://github.com/GLS-SSV/SSV —
  the model for your nav MFD.

---

## ORDER OF WORK FROM HERE

1. ✅ Physics core, tested headless
2. ⬜ Vessel flies in Orbiter with debug string (Steps 4–7)
3. ⬜ Nav MFD (Step 8)
4. ⬜ 2D panel — clickable breakers, reactor gauges, FDAI
5. ⬜ Custom mesh
6. ⬜ Virtual cockpit

Do **not** skip to 5. A ship you can fly with an ugly mesh beats a beautiful
mesh you can't power up.
