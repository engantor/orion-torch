// ============================================================================
//  ShipCore.h  --  ENGINE-AGNOSTIC ship systems model.
//
//  DELIBERATELY has zero Orbiter dependencies. No orbitersdk.h, no VESSEL,
//  no VECTOR3. This is the part of the project that is *yours* -- the reactor,
//  the electrical system, the coolant loop. It compiles and unit-tests on any
//  machine with a C++ compiler, and it would drop unchanged into Godot, a
//  headless test harness, or a different sim entirely.
//
//  Orbiter never sees this file. Orion.cpp (the VESSEL4 subclass) owns one
//  ShipCore and calls Step() each frame. That boundary is the whole
//  architecture: Orbiter supplies orbital mechanics + rendering; ShipCore
//  supplies everything Orbiter doesn't know about.
// ============================================================================
#ifndef ORION_SHIPCORE_H
#define ORION_SHIPCORE_H

#include <string>
#include <vector>
#include <deque>
#include <cmath>
#include "Avionics.h"

namespace orion {

// ---- physical constants ----
constexpr double SIGMA   = 5.670374419e-8;   // Stefan-Boltzmann [W/m^2/K^4]
constexpr double G0      = 9.80665;

// ---- plant design point (tune here; physics below never changes) ----
constexpr double PE_RCT   = 20.0e6;   // electrical output per reactor  [W]
constexpr double QW_RCT   = 30.0e6;   // waste heat per reactor         [W]
constexpr double PTH_RCT  = 50.0e6;   // thermal power per reactor      [W]
constexpr double P_VAC    = 1.2e6;    // vacuum pump draw (startup)     [W]
constexpr double P_FIELD  = 4.0e6;    // coil charging draw             [W]
constexpr double P_RF     = 6.0e6;    // RF heating draw                [W]

constexpr double L_AVI    = 1.5e6;    // avionics / GN&C                [W]
constexpr double L_LSS    = 2.0e6;    // life support                   [W]
constexpr double L_PUMP   = 1.0e6;    // one coolant pump (AC)          [W]
constexpr double L_DRV    = 3.0e6;    // drive control electronics      [W]
constexpr double L_RCS    = 0.3e6;
constexpr double L_SCI    = 5.0e6;

constexpr double BANK_J   = 600e3 * 3600.0;  // 600 kWh per bank        [J]
constexpr double INV_EFF  = 0.92;

constexpr double A_HK     = 5000.0;   // housekeeping radiator area     [m^2]
constexpr double EPS_RAD  = 0.90;
constexpr double C_COOL   = 2.0e8;    // coolant loop heat capacity     [J/K]
constexpr double C_RAD    = 6.0e7;    // radiator heat capacity         [J/K]
constexpr double UA_PUMP  = 5.0e5;    // conductance per pump           [W/K]
constexpr double T_SHED   = 900.0;    // auto load-shed threshold       [K]
constexpr double T_SCRAM  = 950.0;    // reactor scram threshold        [K]

// ---- torch drive: MASS AUGMENTATION ----------------------------------------
// The plant burns D + 3He -> 4He + p, releasing 18.3 MeV per reaction. If the
// products were exhausted neat, they would leave at V_PROD -- 8.8% of light
// speed. That is a superb Isp and almost useless thrust, because at fixed jet
// power F = 2P/vex: thrust is INVERSELY proportional to exhaust velocity.
//
// So the drive injects inert hydrogen into the plasma stream. The added mass
// shares the same energy, so the exhaust slows and the thrust multiplies.
// Energy conservation fixes the mixing ratio exactly:
//
//     (1/2) mdot_tot * vex^2  =  (1/2) mdot_fuel * V_PROD^2
//         =>  R = mdot_tot/mdot_fuel = (V_PROD/vex)^2
//         =>  vex = V_PROD / sqrt(R)
//
// R is a REAL-TIME PILOT CONTROL. It trades Isp against thrust at constant
// reactor power -- and because waste heat depends only on fusion power, a
// richer mixture buys thrust for FREE thermally. You pay in propellant.
constexpr double V_PROD   = 2.650e7;  // neat fusion-product exhaust    [m/s]
constexpr double SPEC_E   = 3.51e14;  // D-3He specific energy          [J/kg]
constexpr double TORCH_PMAX = 15.0e12;// max fusion jet power           [W]
constexpr double MIX_MIN  = 1.0;      // neat products (max Isp)
constexpr double MIX_MAX  = 30.0;     // heavily diluted (max thrust)
constexpr double VEX      = 9.81e6;   // DESIGN vex at R = 7.3          [m/s]
constexpr double WASTE_FR = 0.005;    // waste heat / jet power
constexpr double A_TORCH  = 180000.0; // drive radiator area            [m^2]
constexpr double T_TORCH_LIM = 1800.0;// drive radiator limit           [K]
constexpr double FUEL0    = 16000.0;  // D/3He flask                    [kg]
constexpr double INERT0   = 100000.0; // inert H2 propellant            [kg]

constexpr double V_NOM    = 1000.0;   // nominal DC bus voltage         [V]
constexpr double ESS_RATING = 12000.0;// essential-bus breaker rating   [A]

// ---------------------------------------------------------------- reactor
enum class RctStage { Standby, Pumpdown, Field, Heat, Ignite, Ascent, Online, Scram };
const char* StageName(RctStage s);

struct Reactor {
    char        id      = 'A';
    RctStage    stage   = RctStage::Standby;
    double      press   = 1.0e5;   // chamber pressure   [Pa]
    double      coilI   = 0.0;     // coil current       [kA]
    double      B       = 0.0;     // magnetic field     [T]
    double      coilT   = 20.0;    // coil temperature   [K]
    double      Tion    = 0.0;     // ion temperature    [keV]
    double      dens    = 0.0;     // plasma density     [1e20 m^-3]
    double      Q       = 0.0;     // fusion gain
    double      Pth     = 0.0;     // thermal power      [W]
    double      Pe      = 0.0;     // electrical output  [W]
    double      rf      = 0.0;     // RF heating power   [W]
    double      noCool  = 0.0;     // seconds without cooling
    std::string fault;

    void Reset();
    double StartupDraw() const;    // power this reactor is *consuming* [W]
    bool   IsLive() const;
};

// ---------------------------------------------------------------- breakers
struct Breakers {
    // distribution
    bool GEN_A=false, GEN_B=false, TIE=false, ESS=false, INV1=false, INV2=false;
    // loads
    bool AVI=false, LSS=false, PUMP1=false, PUMP2=false, DRV=false, RCS=false, SCI=false;
};

struct Bank { char id; double E = BANK_J; bool armed = false; };

// snapshot of the electrical solution for one tick
struct ElecState {
    bool   mainA=false, mainB=false, mainBus=false;
    bool   essBus=false, acBus=false;
    int    pumps=0;
    double gen=0, load=0, V=0, I=0, soc=0;
    bool   aviPwr=false, drvPwr=false, rcsPwr=false;
};

// ---------------------------------------------------------------- core
class ShipCore {
public:
    ShipCore();

    // Advance all systems by dt seconds. Call with a SMALL fixed dt
    // (<= 0.25 s). The caller is responsible for substepping -- see
    // Orion.cpp, because Orbiter's simdt explodes under time acceleration.
    void Step(double dt);

    // --- commands (wire these to panel switches / MFD buttons) ---
    void ArmBank(int i, bool on);
    void SetBreaker(const std::string& name, bool closed);
    bool ResetTrip(const std::string& name);   // true if a trip was cleared
    void StartReactor(int i);
    void ShutdownReactor(int i);
    void SetThrottle(double g);                // commanded accel [g]
    void SetMixture(double R);                 // dilution ratio, MIX_MIN..MIX_MAX

    // --- avionics (IMU / GDC / RCS). Owned here so one Step() drives it all.
    Avionics&       Avio()       { return avio_; }
    const Avionics& Avio() const { return avio_; }
    void SetAttitude(double roll, double pitch, double yaw, double bodyRate);

    // --- queries (for MFDs / panels / the VESSEL4 glue) ---
    const ElecState& Elec()    const { return e_; }
    const Reactor&   Rct(int i) const { return rct_[i]; }
    const Breakers&  Bkr()     const { return bk_; }
    const Bank&      BankAt(int i) const { return bank_[i]; }
    bool  Tripped(const std::string& n) const;

    double CoolantT()  const { return Tcool_; }
    double RadiatorT() const { return Trad_;  }
    double TorchT()    const { return Ttorch_; }
    double HeatIn()    const { return qIn_;   }
    double HeatOut()   const { return qRad_;  }

    // Thrust the drive is actually allowed to produce, 0..1 of max.
    // Orbiter calls this and feeds it to SetThrusterLevel().
    double ThrottleLevel() const { return thrLevel_; }
    bool   DriveFailed()   const { return driveFail_; }
    double ThrottleCmd()   const { return thrCmd_; }

    // --- mass augmentation readouts ---
    double Mixture()   const { return mix_; }
    double ExhaustV()  const { return V_PROD / std::sqrt(mix_); }
    double Isp()       const { return ExhaustV() / G0; }
    double JetPower()  const { return jet_; }        // [W]
    double Thrust()    const { return thrust_; }     // [N]
    double MdotFuel()  const { return mdotFuel_; }   // [kg/s]
    double MdotInert() const { return mdotInert_; }  // [kg/s]
    double FuelKg()    const { return fuel_; }
    double InertKg()   const { return inert_; }
    // Highest acceleration this mixture can reach on full reactor power.
    double MaxAccelG() const {
        return (2.0 * TORCH_PMAX / ExhaustV()) / mass_ / G0;
    }

    // vessel mass is owned by Orbiter; we need it to size thrust/heat
    void SetVesselMass(double kg) { mass_ = kg; }

    const std::deque<std::string>& Log() const { return log_; }
    void  Say(const std::string& msg);

    // --- persistence (scenario file) ---
    std::string Serialize() const;
    void        Deserialize(const std::string& line);

private:
    void SolveElec();
    void StepReactor(Reactor& r, double dt);
    void StepThermal(double dt);
    void StepBreakers();
    void StepDrive(double dt);

    Reactor  rct_[2];
    Bank     bank_[3];
    Breakers bk_;
    ElecState e_;
    std::vector<std::string> trips_;

    double Tcool_   = 290.0;
    double Trad_    = 290.0;
    double Ttorch_  = 0.0;
    double qIn_     = 0.0;
    double qRad_    = 0.0;

    double mass_    = 650000.0;
    double thrCmd_  = 0.0;     // commanded [g]
    double thrLevel_= 0.0;     // 0..1 delivered to Orbiter
    double dmg_     = 0.0;
    bool   driveFail_ = false;

    // mass augmentation
    double mix_       = 7.30;      // dilution ratio (design point)
    double fuel_      = FUEL0;     // D/3He
    double inert_     = INERT0;    // H2
    double jet_       = 0.0;
    double thrust_    = 0.0;
    double mdotFuel_  = 0.0;
    double mdotInert_ = 0.0;
    bool   fuelOut_   = false;
    bool   inertOut_  = false;

    // avionics
    Avionics avio_;
    double bodyRate_ = 0.0;
    double met_ = 0.0;

    std::deque<std::string> log_;
};

} // namespace orion
#endif
