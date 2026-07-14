// ============================================================================
//  Avionics.h -- IMU / GDC / RCS. Engine-agnostic, like ShipCore.
//
//  Modelled directly on the Apollo CSM, because that vehicle solved these
//  problems and its failure modes are the interesting ones:
//
//   * IMU on a gimballed stable platform -> GIMBAL LOCK is a real hazard.
//     Apollo lit the GIMBAL LOCK light when the MIDDLE gimbal passed +/-70 deg;
//     past ~85 deg the platform tumbles and the reference is LOST.
//   * The platform drifts, so it must be realigned against stars (program P52),
//     on a schedule, with several possible orientation OPTIONS.
//   * A backup attitude reference (GDC) integrates rate gyros. It drifts much
//     faster, but it can be re-aligned to the IMU at the push of a button --
//     so a dead IMU doesn't mean a dead ship.
//   * RCS is hypergolic and PRESSURE-FED. Helium squeezes a bladder in each
//     propellant tank. No helium -> no thrust, no matter how much fuel is left.
//     Two helium tanks per quad, because a single regulator failure would
//     otherwise kill a whole axis.
//
//  ORION twist: the fusion plant burns D + 3He -> 4He + p, so its ASH IS
//  HELIUM-4 -- the ideal pressurant. The ship refills its own RCS helium from
//  the reactor divertor. See HeliumMakeup().
// ============================================================================
#ifndef ORION_AVIONICS_H
#define ORION_AVIONICS_H

#include <string>
#include <deque>

namespace orion {

// ---------------------------------------------------------------- IMU
// Apollo P52 orientation options. Same numbering as the real program.
enum class AlignOpt {
    Preferred = 1,   // orient to the attitude required by the NEXT burn
    Nominal   = 2,   // local vertical / local horizontal at a given time
    Refsmmat  = 3,   // keep current stored reference; just null the drift
    Target    = 4    // point the platform at the selected target
};

enum class AlignPhase {
    Idle,
    CoarseAlign,   // torque the platform roughly into place
    Sight1,        // first star sighting
    Sight2,        // second star sighting
    Torquing,      // apply computed torquing angles
    Done
};

const char* AlignOptName(AlignOpt o);
const char* AlignPhaseName(AlignPhase p);

struct Imu {
    bool   powered   = false;
    bool   aligned   = false;

    // gimbal angles [deg]. The MIDDLE gimbal is the dangerous one.
    double outer     = 0.0;   // roll
    double middle    = 0.0;   // yaw   <-- gimbal lock axis
    double inner     = 0.0;   // pitch

    double drift     = 0.0;   // accumulated platform error [arcmin]
    double driftRate = 0.9;   // [arcmin/hour]

    double lastAlign    = 0.0;      // MET of last good alignment [s]
    double alignInterval= 6*3600.0; // scheduled P52 period [s]

    AlignOpt   option = AlignOpt::Refsmmat;
    AlignPhase phase  = AlignPhase::Idle;
    double     phaseT = 0.0;        // seconds remaining in current phase

    double starAngleDiff = 0.0;     // sighting quality check [arcmin]
    int    star1 = 0, star2 = 0;    // catalog indices

    bool GimbalLockWarn() const { return fabs_(middle) > 70.0; }
    bool GimbalLocked()   const { return fabs_(middle) > 85.0; }
private:
    static double fabs_(double x) { return x < 0 ? -x : x; }
};

// ---------------------------------------------------------------- GDC
// Backup attitude reference: integrates the rate gyros. Cheap, no gimbals
// (so it CANNOT gimbal-lock), but it drifts ~20x faster than the IMU.
struct Gdc {
    bool   powered   = false;
    bool   aligned   = false;
    double att[3]    = {0,0,0};     // roll, pitch, yaw [deg]
    double drift     = 0.0;         // [arcmin]
    double driftRate = 18.0;        // [arcmin/hour] -- deliberately poor
};

// ---------------------------------------------------------------- RCS
// One quad = 4 thrusters + its own propellant + its own helium.
struct RcsQuad {
    char   id       = 'A';
    bool   enabled  = true;
    bool   heTank1  = false;   // helium isolation valves (two, for redundancy)
    bool   heTank2  = false;
    double hePress  = 200.0;   // regulated helium pressure [bar]
    double fuel     = 400.0;   // MMH [kg]
    double ox       = 640.0;   // N2O4 [kg]  (O/F = 1.6)
    double temp     = 290.0;   // quad temperature [K]
    bool   heaterOn = false;
    bool   leaking  = false;

    // Hypergolic propellant FREEZES. N2O4 freezes at 262 K; below that the
    // quad is dead even though the tanks are full. Apollo heated its quads
    // continuously for exactly this reason.
    bool Frozen()    const { return temp < 262.0; }
    bool HeAvail()   const { return (heTank1 || heTank2) && hePress > 20.0; }
    bool Operable()  const { return enabled && HeAvail() && !Frozen()
                                    && fuel > 0.1 && ox > 0.1; }
};

// ---------------------------------------------------------------- Avionics
class Avionics {
public:
    Avionics();

    // dt in seconds. met = mission elapsed time. bodyRate = |omega| [deg/s].
    // aviPower / rcsPower come from the electrical system.
    // fusionPower [W] drives the helium makeup from reactor ash.
    void Step(double dt, double met, double bodyRate,
              bool aviPower, bool rcsPower, double fusionPower);

    // --- IMU commands ---
    void SetAlignOption(AlignOpt o);
    void SetAlignInterval(double sec);
    void StartAlign(double met, double bodyRate);   // "P52"
    void AbortAlign();
    bool AlignDue(double met) const;                // scheduled realign nag
    double TimeToAlign(double met) const;

    // --- GDC ---
    void AlignGdcToImu();     // the classic "GDC ALIGN" pushbutton

    // --- attitude feed (the sim's true attitude drives the gimbals) ---
    void SetTrueAttitude(double roll, double pitch, double yaw);

    // --- RCS ---
    void SetQuadEnabled(int i, bool on);
    void SetHeTank(int i, int tank, bool open);
    void SetQuadHeater(int i, bool on);
    // Consume propellant for an impulse. Returns actual impulse delivered [N.s]
    // -- ZERO if the quad has no helium or is frozen, however full the tank is.
    double FireQuad(int i, double demandImpulse, double dt);

    // --- queries ---
    const Imu&     GetImu()  const { return imu_; }
    const Gdc&     GetGdc()  const { return gdc_; }
    const RcsQuad& Quad(int i) const { return quad_[i]; }
    double HeReserve() const { return heReserve_; }   // makeup gas from ash [kg]
    double TotalRcsProp() const;
    const std::deque<std::string>& Log() const { return log_; }
    void Say(const std::string& m);

private:
    void StepImu(double dt, double met, double bodyRate, bool pwr);
    void StepGdc(double dt, bool pwr);
    void StepRcs(double dt, bool pwr, double fusionPower);

    Imu     imu_;
    Gdc     gdc_;
    RcsQuad quad_[4];
    double  heReserve_ = 0.0;    // helium-4 recovered from the reactor [kg]
    double  trueAtt_[3] = {0,0,0};
    double  rngState_ = 12345.0;
    std::deque<std::string> log_;

    double Rand01();
};

// D + 3He -> 4He + p.  Of the 5 amu of reactants, 4 amu leaves as an alpha.
// Specific energy of the fuel is 3.51e14 J/kg, so per watt of fusion power the
// plant produces (4/5)/3.51e14 kg/s of helium-4 ash. We scavenge a fraction of
// it from the divertor.
constexpr double HE4_PER_JOULE  = 0.8 / 3.51e14;   // [kg per J of fusion energy]
constexpr double HE_SCAVENGE    = 0.01;            // 1% of the alpha flux captured
constexpr double HE_RESERVE_MAX = 120.0;           // storage bottle [kg]

} // namespace orion
#endif
