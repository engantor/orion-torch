#include "Avionics.h"
#include <cmath>
#include <algorithm>

namespace orion {

const char* AlignOptName(AlignOpt o) {
    switch (o) {
    case AlignOpt::Preferred: return "1 PREFERRED (BURN ATT)";
    case AlignOpt::Nominal:   return "2 NOMINAL (LVLH)";
    case AlignOpt::Refsmmat:  return "3 REFSMMAT (DRIFT NULL)";
    case AlignOpt::Target:    return "4 TARGET LOS";
    }
    return "?";
}
const char* AlignPhaseName(AlignPhase p) {
    switch (p) {
    case AlignPhase::Idle:        return "IDLE";
    case AlignPhase::CoarseAlign: return "COARSE ALIGN";
    case AlignPhase::Sight1:      return "STAR SIGHTING 1";
    case AlignPhase::Sight2:      return "STAR SIGHTING 2";
    case AlignPhase::Torquing:    return "TORQUING PLATFORM";
    case AlignPhase::Done:        return "ALIGNED";
    }
    return "?";
}

// 37-star catalog, as Apollo carried. Names are flavour; the indices are what
// the alignment program actually uses.
static const char* STARS[] = {
    "ACAMAR","ACHERNAR","ACRUX","ANTARES","ALDEBARAN","ALPHECCA","ALTAIR",
    "ARCTURUS","ATRIA","BETELGEUSE","CANOPUS","CAPELLA","DENEB","DENEBOLA",
    "DIPHDA","DNOCES","ENIF","FOMALHAUT","GACRUX","GIENAH","MENKAR","MENKENT",
    "MIRFAK","NUNKI","PEACOCK","POLARIS","PROCYON","RASALHAGUE","REGULUS",
    "RIGEL","SIRIUS","SPICA","VEGA","ZUBENELGENUBI","NAVI","REGOR","ALPHARD"
};
static const int NSTARS = 37;

Avionics::Avionics() {
    for (int i = 0; i < 4; i++) quad_[i].id = char('A' + i);
    Say("AVIONICS INIT - IMU NOT ALIGNED");
}

void Avionics::Say(const std::string& m) {
    log_.push_front(m);
    if (log_.size() > 12) log_.pop_back();
}

double Avionics::Rand01() {
    // deterministic LCG -- reproducible runs matter for regression tests
    rngState_ = std::fmod(rngState_ * 1103515245.0 + 12345.0, 2147483648.0);
    return rngState_ / 2147483648.0;
}

void Avionics::SetTrueAttitude(double roll, double pitch, double yaw) {
    trueAtt_[0] = roll; trueAtt_[1] = pitch; trueAtt_[2] = yaw;
}

// --------------------------------------------------------------------- IMU
void Avionics::SetAlignOption(AlignOpt o) {
    imu_.option = o;
    Say(std::string("ALIGN OPTION: ") + AlignOptName(o));
}
void Avionics::SetAlignInterval(double s) {
    imu_.alignInterval = s;
}
bool Avionics::AlignDue(double met) const {
    if (!imu_.aligned) return true;
    return (met - imu_.lastAlign) >= imu_.alignInterval;
}
double Avionics::TimeToAlign(double met) const {
    return imu_.alignInterval - (met - imu_.lastAlign);
}

void Avionics::StartAlign(double met, double bodyRate) {
    if (!imu_.powered) { Say("P52 REJECTED - IMU UNPOWERED"); return; }
    if (imu_.phase != AlignPhase::Idle && imu_.phase != AlignPhase::Done) return;
    if (bodyRate > 0.05) {
        Say("P52 REJECTED - BODY RATES TOO HIGH (NEED <0.05 DEG/S)");
        return;
    }
    // pick two stars far apart -- a small separation makes the fix ill-conditioned
    imu_.star1 = (int)(Rand01() * NSTARS);
    imu_.star2 = (imu_.star1 + 8 + (int)(Rand01() * 20)) % NSTARS;
    imu_.phase  = AlignPhase::CoarseAlign;
    imu_.phaseT = 20.0;
    Say(std::string("P52 START - ") + AlignOptName(imu_.option));
}

void Avionics::AbortAlign() {
    if (imu_.phase == AlignPhase::Idle) return;
    imu_.phase = AlignPhase::Idle;
    Say("P52 ABORTED");
}

void Avionics::StepImu(double dt, double met, double bodyRate, bool pwr) {
    // Power loss loses the platform outright. Apollo's IMU needed continuous
    // power both to run the gyros and to hold its temperature.
    if (imu_.powered && !pwr) {
        imu_.powered = false;
        imu_.aligned = false;
        imu_.phase   = AlignPhase::Idle;
        Say("IMU POWER LOSS - PLATFORM LOST");
    }
    imu_.powered = pwr;
    if (!pwr) return;

    // --- gimbal angles track the vehicle attitude ---
    imu_.outer  = trueAtt_[0];
    imu_.inner  = trueAtt_[1];
    imu_.middle = trueAtt_[2];   // the yaw/middle gimbal: the dangerous one

    // GIMBAL LOCK. At >70 deg Apollo lit the warning; past ~85 the middle and
    // outer gimbal axes become nearly parallel, the platform loses a degree of
    // freedom, tumbles, and the inertial reference is destroyed.
    if (imu_.aligned && imu_.GimbalLocked()) {
        imu_.aligned = false;
        imu_.phase   = AlignPhase::Idle;
        Say("*** GIMBAL LOCK *** PLATFORM TUMBLED - REALIGN REQUIRED");
    }

    // --- drift ---
    if (imu_.aligned)
        imu_.drift += imu_.driftRate * (dt / 3600.0);

    // --- the alignment sequence itself ---
    if (imu_.phase == AlignPhase::Idle || imu_.phase == AlignPhase::Done) return;

    // Any sighting phase demands a steady vehicle. Fire a thruster mid-mark and
    // you lose the mark -- exactly the Apollo constraint.
    if (bodyRate > 0.05 &&
        (imu_.phase == AlignPhase::Sight1 || imu_.phase == AlignPhase::Sight2)) {
        imu_.phase = AlignPhase::Idle;
        Say("MARK LOST - VEHICLE NOT STEADY. P52 ABORTED");
        return;
    }

    imu_.phaseT -= dt;
    if (imu_.phaseT > 0.0) return;

    switch (imu_.phase) {
    case AlignPhase::CoarseAlign:
        imu_.phase  = AlignPhase::Sight1;
        imu_.phaseT = 25.0;
        Say(std::string("COARSE ALIGN OK - MARK STAR ") + STARS[imu_.star1]);
        break;

    case AlignPhase::Sight1:
        imu_.phase  = AlignPhase::Sight2;
        imu_.phaseT = 25.0;
        Say(std::string("MARK 1 GOOD - MARK STAR ") + STARS[imu_.star2]);
        break;

    case AlignPhase::Sight2: {
        // STAR ANGLE DIFFERENCE: compare the measured angle between the two
        // sighted stars against the catalogued angle. A large discrepancy means
        // a bad mark -- you sighted the wrong star, or the optics are off.
        // Apollo's crews checked exactly this number before accepting a fix.
        imu_.starAngleDiff = Rand01() * 0.9;     // arcmin
        if (imu_.starAngleDiff > 0.7) {
            imu_.phase = AlignPhase::Idle;
            Say("STAR ANGLE DIFFERENCE OUT OF LIMITS - FIX REJECTED, RE-MARK");
            return;
        }
        imu_.phase  = AlignPhase::Torquing;
        imu_.phaseT = 15.0;
        Say("MARK 2 GOOD - STAR ANGLE DIFF OK - COMPUTING TORQUING ANGLES");
        break;
    }

    case AlignPhase::Torquing:
        imu_.aligned   = true;
        imu_.drift     = 0.0;
        imu_.lastAlign = met;
        imu_.phase     = AlignPhase::Done;
        Say("PLATFORM ALIGNED - DRIFT NULLED");
        break;

    default: break;
    }
}

// --------------------------------------------------------------------- GDC
void Avionics::AlignGdcToImu() {
    if (!gdc_.powered) { Say("GDC UNPOWERED"); return; }
    if (!imu_.aligned) { Say("GDC ALIGN REJECTED - IMU NOT ALIGNED"); return; }
    gdc_.aligned = true;
    gdc_.drift   = 0.0;
    for (int i = 0; i < 3; i++) gdc_.att[i] = trueAtt_[i];
    Say("GDC ALIGNED TO IMU");
}

void Avionics::StepGdc(double dt, bool pwr) {
    gdc_.powered = pwr;
    if (!pwr) { gdc_.aligned = false; return; }
    if (gdc_.aligned) {
        // No gimbals -> the GDC can never gimbal-lock. That is precisely why
        // it survives the manoeuvre that kills the IMU. It just drifts badly.
        gdc_.drift += gdc_.driftRate * (dt / 3600.0);
        for (int i = 0; i < 3; i++) gdc_.att[i] = trueAtt_[i];
    }
}

// --------------------------------------------------------------------- RCS
void Avionics::SetQuadEnabled(int i, bool on) {
    if (i < 0 || i > 3) return;
    quad_[i].enabled = on;
    Say(std::string("RCS QUAD ") + quad_[i].id + (on ? " ENABLED" : " DISABLED"));
}
void Avionics::SetHeTank(int i, int tank, bool open) {
    if (i < 0 || i > 3) return;
    if (tank == 1) quad_[i].heTank1 = open;
    else           quad_[i].heTank2 = open;
    Say(std::string("QUAD ") + quad_[i].id + " HE TANK " + char('0'+tank)
        + (open ? " OPEN" : " CLOSED"));
}
void Avionics::SetQuadHeater(int i, bool on) {
    if (i < 0 || i > 3) return;
    quad_[i].heaterOn = on;
}

double Avionics::TotalRcsProp() const {
    double t = 0;
    for (const auto& q : quad_) t += q.fuel + q.ox;
    return t;
}

double Avionics::FireQuad(int i, double demandImpulse, double dt) {
    if (i < 0 || i > 3) return 0.0;
    RcsQuad& q = quad_[i];

    // THE POINT OF PRESSURE FEED: full tanks are worthless without helium.
    if (!q.Operable()) return 0.0;

    const double VEX_RCS = 2943.0;             // Isp 300 s, MMH/N2O4
    double mdot = demandImpulse / VEX_RCS;     // kg of total propellant

    // O/F = 1.6 by mass
    double fuelNeed = mdot / 2.6;
    double oxNeed   = mdot * 1.6 / 2.6;
    if (fuelNeed > q.fuel || oxNeed > q.ox) {
        double scale = std::min(q.fuel / std::max(fuelNeed,1e-9),
                                q.ox   / std::max(oxNeed,  1e-9));
        fuelNeed *= scale; oxNeed *= scale; mdot *= scale;
    }
    q.fuel -= fuelNeed;
    q.ox   -= oxNeed;

    // Helium expands to fill the volume the propellant vacated, so tank
    // pressure falls (blowdown). The regulator masks it until the helium
    // is nearly spent -- then thrust collapses even with propellant aboard.
    const double volExpelled = fuelNeed / 875.0 + oxNeed / 1440.0;   // m^3
    q.hePress -= volExpelled * 45.0;
    if (q.hePress < 0) q.hePress = 0;

    if (q.hePress < 25.0 && q.hePress > 20.0)
        Say(std::string("QUAD ") + q.id + " HELIUM LOW");

    return mdot * VEX_RCS;
}

void Avionics::StepRcs(double dt, bool pwr, double fusionPower) {
    for (auto& q : quad_) {
        // --- thermal: quads sit outside; without heaters they freeze solid.
        double target = q.heaterOn && pwr ? 300.0 : 200.0;
        q.temp += (target - q.temp) * 0.004 * dt;
        if (q.Frozen() && q.enabled && q.temp > 258.0 && q.temp < 262.0)
            Say(std::string("QUAD ") + q.id + " PROPELLANT FREEZING - HEATER");

        if (q.leaking) q.hePress = std::max(0.0, q.hePress - 0.5 * dt);
    }

    // --- HELIUM MAKEUP FROM REACTOR ASH -----------------------------------
    // D + 3He -> 4He + p. The alpha particle IS helium-4: the perfect
    // pressurant, and the plant makes ~21 g/s of it at full power. We scavenge
    // 1% from the divertor. The ship refuels its own pressurant.
    if (fusionPower > 0.0) {
        const double made = fusionPower * HE4_PER_JOULE * HE_SCAVENGE * dt;
        heReserve_ = std::min(HE_RESERVE_MAX, heReserve_ + made);
    }
    // top up any quad that's low, if we have gas and power to run the compressor
    if (pwr && heReserve_ > 0.5) {
        for (auto& q : quad_) {
            if (q.hePress < 180.0 && (q.heTank1 || q.heTank2) && !q.leaking) {
                const double need = std::min(2.0 * dt, 200.0 - q.hePress);
                const double kg   = need * 0.02;
                if (heReserve_ >= kg) {
                    q.hePress  += need;
                    heReserve_ -= kg;
                }
            }
        }
    }
}

void Avionics::Step(double dt, double met, double bodyRate,
                    bool aviPower, bool rcsPower, double fusionPower)
{
    StepImu(dt, met, bodyRate, aviPower);
    StepGdc(dt, aviPower);
    StepRcs(dt, rcsPower, fusionPower);
}

} // namespace orion
