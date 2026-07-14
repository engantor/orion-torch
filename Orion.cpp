// ============================================================================
//  Orion.cpp -- Orbiter VESSEL4 addon.
//
//  This file is THIN ON PURPOSE. It contains no physics. Its only jobs are:
//    1. tell Orbiter what the vessel is (mass, PMI, thrusters, mesh)
//    2. pump ShipCore forward each frame with a SAFE timestep
//    3. translate ShipCore's output into Orbiter calls (SetThrusterLevel)
//    4. save/load state into the scenario file
//
//  Everything Orbiter already does well -- n-body gravity, VSOP87/ELP2000
//  ephemerides, atmospheric flight, rendering, time acceleration -- we do NOT
//  reimplement. That was the entire reason for choosing Orbiter.
//
//  BUILD: 64-bit DLL, links against Orbitersdk.lib, drops into Orbiter/Modules.
//  NOTE: not compiled here (needs the Windows SDK); compile in VS.
// ============================================================================
#define STRICT
#define ORBITER_MODULE
#include "orbitersdk.h"
#include "ShipCore.h"

using namespace orion;

// ---- vessel design constants ----
static const double ORION_SIZE      = 60.0;        // mean radius [m]
static const double ORION_DRYMASS   = 500000.0;    // 500 t dry
static const double ORION_PROPMASS  = 150000.0;    // 150 t propellant
static const double ORION_RCSMASS   = 5000.0;
static const VECTOR3 ORION_PMI      = {180.0, 180.0, 22.0};
static const VECTOR3 ORION_CS       = {320.0, 320.0, 90.0};

// Orbiter's "Isp" argument to CreateThruster is the EFFECTIVE EXHAUST VELOCITY
// in m/s -- not seconds. So our torch's 1e6 s Isp goes in directly as VEX.
static const double ORION_VEX       = orion::VEX;              // 9.81e6 m/s
// Size the main thruster for 1 g at full wet mass; ShipCore then commands
// a level in [0,1] which maps to 0..1 g.
static const double ORION_MAXTHRUST =
    (ORION_DRYMASS + ORION_PROPMASS + ORION_RCSMASS) * orion::G0;
static const double ORION_RCS_THRUST = 6.0e4;

// ============================================================================
class Orion : public VESSEL4 {
public:
    Orion(OBJHANDLE hVessel, int flightmodel);

    void clbkSetClassCaps(FILEHANDLE cfg) override;
    void clbkPostCreation() override;
    void clbkPreStep(double simt, double simdt, double mjd) override;
    void clbkSaveState(FILEHANDLE scn) override;
    void clbkLoadStateEx(FILEHANDLE scn, void* status) override;
    int  clbkConsumeBufferedKey(DWORD key, bool down, char* kstate) override;

    // MFDs and panels reach the systems model through this.
    ShipCore& Core() { return core_; }

private:
    ShipCore          core_;
    PROPELLANT_HANDLE ph_main_ = nullptr;
    PROPELLANT_HANDLE ph_rcs_  = nullptr;
    THRUSTER_HANDLE   th_main_ = nullptr;
    THRUSTER_HANDLE   th_rcs_[12] = {};
    double            carry_ = 0.0;   // leftover sim time for fixed-step integration
};

Orion::Orion(OBJHANDLE hVessel, int flightmodel)
    : VESSEL4(hVessel, flightmodel) {}

// ---------------------------------------------------------------------------
// Called ONCE by Orbiter at vessel creation. Never call it yourself.
void Orion::clbkSetClassCaps(FILEHANDLE cfg) {
    SetSize(ORION_SIZE);
    SetEmptyMass(ORION_DRYMASS);
    SetPMI(ORION_PMI);
    SetCrossSections(ORION_CS);
    SetRotDrag(_V(0.10, 0.10, 0.05));
    SetCameraOffset(_V(0, 2.0, 20.0));

    // Propellant. Orbiter tracks the mass; ShipCore reads it back.
    ph_main_ = CreatePropellantResource(ORION_PROPMASS);
    ph_rcs_  = CreatePropellantResource(ORION_RCSMASS);

    // The torch. Enormous exhaust velocity, thrust sized for 1 g.
    // We deliberately do NOT put it in THGROUP_MAIN, because we don't want
    // the player's throttle keys driving it directly -- ShipCore decides
    // whether the drive is allowed to fire at all.
    th_main_ = CreateThruster(_V(0, 0, -55.0), _V(0, 0, 1),
                              ORION_MAXTHRUST, ph_main_, ORION_VEX);
    AddExhaust(th_main_, 60.0, 4.0);

    // RCS -- these DO go into thruster groups so Orbiter's normal attitude
    // controls and autopilots (KILLROT, PROGRADE, etc.) work for free.
    const double x = 6.0, y = 6.0, z = 25.0;
    th_rcs_[0] = CreateThruster(_V( x, 0, z), _V(0, 1, 0), ORION_RCS_THRUST, ph_rcs_, 2943);
    th_rcs_[1] = CreateThruster(_V(-x, 0, z), _V(0,-1, 0), ORION_RCS_THRUST, ph_rcs_, 2943);
    th_rcs_[2] = CreateThruster(_V( x, 0,-z), _V(0,-1, 0), ORION_RCS_THRUST, ph_rcs_, 2943);
    th_rcs_[3] = CreateThruster(_V(-x, 0,-z), _V(0, 1, 0), ORION_RCS_THRUST, ph_rcs_, 2943);
    THRUSTER_HANDLE grp[2];
    grp[0] = th_rcs_[0]; grp[1] = th_rcs_[2];
    CreateThrusterGroup(grp, 2, THGROUP_ATT_PITCHDOWN);
    grp[0] = th_rcs_[1]; grp[1] = th_rcs_[3];
    CreateThrusterGroup(grp, 2, THGROUP_ATT_PITCHUP);
    // ... yaw / roll / translation groups follow the same pattern.

    AddMesh("ShuttlePB");      // stock mesh stand-in; swap for your own later
}

void Orion::clbkPostCreation() {
    core_.SetVesselMass(GetMass());
}

// ---------------------------------------------------------------------------
//  THE MOST IMPORTANT FUNCTION IN THE FILE.
//
//  Orbiter calls clbkPreStep once per frame, and simdt is the length of the
//  step it is ABOUT to take. Under time acceleration (Orbiter goes to 100000x)
//  simdt can be tens or hundreds of seconds. Feeding that straight into
//  ShipCore would blow up the reactor and thermal ODEs -- a 200-second Euler
//  step on a loop with a 200 s time constant is nonsense.
//
//  So we accumulate simdt and run the core at a FIXED small dt, capping the
//  number of substeps so a huge warp can't stall the frame. This is the single
//  detail that most often breaks Orbiter addons with real internal dynamics.
// ---------------------------------------------------------------------------
void Orion::clbkPreStep(double simt, double simdt, double mjd) {
    core_.SetVesselMass(GetMass());

    const double FIXED_DT  = 0.1;    // ShipCore integrates at 100 ms
    const int    MAX_STEPS = 200;    // frame-time guard

    carry_ += simdt;
    int steps = 0;
    while (carry_ >= FIXED_DT && steps < MAX_STEPS) {
        core_.Step(FIXED_DT);
        carry_ -= FIXED_DT;
        steps++;
    }
    if (steps >= MAX_STEPS) {
        // We're warping faster than we can simulate. Drain the backlog with
        // one coarse step rather than silently falling behind wall-clock.
        core_.Step(carry_);
        carry_ = 0.0;
    }

    // ShipCore decides whether the torch may fire, and how hard. This is where
    // "no main bus -> no thrust" becomes physically true in Orbiter.
    SetThrusterLevel(th_main_, core_.ThrottleLevel());

    // The mixing valve changes exhaust velocity at runtime, so Isp is not
    // fixed once created -- it must be pushed to Orbiter every step.
    SetThrusterIsp(th_main_, core_.ExhaustV());

    sprintf(oapiDebugString(),
        "RCT A: %s  Q=%.1f  B=%.1fT | BATT %.0f%% | Tcool %.0fK | thr %.2f",
        orion::StageName(core_.Rct(0).stage), core_.Rct(0).Q, core_.Rct(0).B,
        100.0 * core_.Elec().soc, core_.CoolantT(), core_.ThrottleLevel());
}

// ---------------------------------------------------------------------------
void Orion::clbkSaveState(FILEHANDLE scn) {
    SaveDefaultState(scn);                       // Orbiter's own state first
    std::string s = core_.Serialize();
    oapiWriteScenario_string(scn, (char*)"SYSTEMS", (char*)s.c_str());
}

void Orion::clbkLoadStateEx(FILEHANDLE scn, void* status) {
    char* line;
    while (oapiReadScenario_nextline(scn, line)) {
        if (!_strnicmp(line, "SYSTEMS", 7)) {
            core_.Deserialize(std::string(line + 8));
        } else {
            ParseScenarioLineEx(line, status);   // hand the rest back to Orbiter
        }
    }
}

// ---------------------------------------------------------------------------
// Temporary keyboard bindings so you can fly before the 2D panel exists.
int Orion::clbkConsumeBufferedKey(DWORD key, bool down, char* kstate) {
    if (!down) return 0;
    if (KEYMOD_CONTROL(kstate)) return 0;

    switch (key) {
    case OAPI_KEY_1: core_.ArmBank(0, true); core_.ArmBank(1, true); core_.ArmBank(2, true); return 1;
    case OAPI_KEY_2: core_.SetBreaker("INV1", true); core_.SetBreaker("AVI", true);
                     core_.SetBreaker("LSS", true);  return 1;
    case OAPI_KEY_3: core_.SetBreaker("PUMP1", true); return 1;
    case OAPI_KEY_4: core_.StartReactor(0); return 1;
    case OAPI_KEY_5: core_.SetBreaker("GEN_A", true); core_.SetBreaker("ESS", true); return 1;
    case OAPI_KEY_6: core_.SetBreaker("DRV", true); return 1;
    case OAPI_KEY_ADD:      core_.SetThrottle(core_.ThrottleCmd() + 0.05); return 1;
    case OAPI_KEY_SUBTRACT: core_.SetThrottle(core_.ThrottleCmd() - 0.05); return 1;
    }
    return 0;
}

// ============================================================================
//  Module entry points. NOTE: Orbiter does NOT unload the DLL between
//  scenarios, so global/static mutable state persists and WILL bite you.
//  Keep all state inside the vessel object.
// ============================================================================
DLLCLBK void InitModule(HINSTANCE hModule) {}
DLLCLBK void ExitModule(HINSTANCE hModule) {}

DLLCLBK VESSEL* ovcInit(OBJHANDLE hvessel, int flightmodel) {
    return new Orion(hvessel, flightmodel);
}
DLLCLBK void ovcExit(VESSEL* vessel) {
    if (vessel) delete (Orion*)vessel;
}
