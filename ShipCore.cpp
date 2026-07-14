#include "ShipCore.h"
#include <cmath>
#include <algorithm>
#include <sstream>

namespace orion {

const char* StageName(RctStage s) {
    switch (s) {
    case RctStage::Standby:  return "STANDBY";
    case RctStage::Pumpdown: return "PUMPDOWN";
    case RctStage::Field:    return "FIELD";
    case RctStage::Heat:     return "HEAT";
    case RctStage::Ignite:   return "IGNITE";
    case RctStage::Ascent:   return "ASCENT";
    case RctStage::Online:   return "ONLINE";
    case RctStage::Scram:    return "SCRAM";
    }
    return "?";
}

static double clampd(double x, double a, double b) { return std::max(a, std::min(b, x)); }

// ------------------------------------------------------------------ Reactor
void Reactor::Reset() {
    stage = RctStage::Standby;
    press = 1.0e5; coilI = 0; B = 0; coilT = 20.0;
    Tion = 0; dens = 0; Q = 0; Pth = 0; Pe = 0; rf = 0; noCool = 0;
    fault.clear();
}
double Reactor::StartupDraw() const {
    switch (stage) {
    case RctStage::Pumpdown: return P_VAC;
    case RctStage::Field:    return P_FIELD;
    case RctStage::Heat:     return P_RF;
    case RctStage::Ignite:   return rf;
    default:                 return 0.0;
    }
}
bool Reactor::IsLive() const {
    return stage != RctStage::Standby && stage != RctStage::Scram;
}

// ------------------------------------------------------------------ ShipCore
ShipCore::ShipCore() {
    rct_[0].id = 'A';
    rct_[1].id = 'B';
    bank_[0].id = 'A';
    bank_[1].id = 'B';
    bank_[2].id = 'C';
    Say("COLD AND DARK - ARM BANKS, CLOSE INV, START PUMP, IGNITE REACTOR");
}

void ShipCore::Say(const std::string& m) {
    log_.push_front(m);
    if (log_.size() > 12) log_.pop_back();
}

bool ShipCore::Tripped(const std::string& n) const {
    return std::find(trips_.begin(), trips_.end(), n) != trips_.end();
}

// A breaker conducts only if closed AND not tripped.
static bool conduct(bool closed, bool tripped) { return closed && !tripped; }

void ShipCore::SolveElec() {
    ElecState e;
    const bool genA = rct_[0].stage == RctStage::Online && conduct(bk_.GEN_A, Tripped("GEN_A"));
    const bool genB = rct_[1].stage == RctStage::Online && conduct(bk_.GEN_B, Tripped("GEN_B"));
    const bool tie  = conduct(bk_.TIE, Tripped("TIE"));

    e.mainA = genA || (tie && genB);
    e.mainB = genB || (tie && genA);
    e.mainBus = e.mainA || e.mainB;

    bool battLive = false;
    for (const auto& b : bank_) if (b.armed && b.E > 1.0) battLive = true;

    e.essBus = (conduct(bk_.ESS, Tripped("ESS")) && e.mainBus) || battLive;
    e.acBus  = (conduct(bk_.INV1, Tripped("INV1")) || conduct(bk_.INV2, Tripped("INV2"))) && e.essBus;

    e.pumps = 0;
    if (e.acBus) {
        if (conduct(bk_.PUMP1, Tripped("PUMP1"))) e.pumps++;
        if (conduct(bk_.PUMP2, Tripped("PUMP2"))) e.pumps++;
    }

    e.gen = (genA ? PE_RCT : 0.0) + (genB ? PE_RCT : 0.0);

    double load = 0.0;
    if (e.essBus) {
        if (conduct(bk_.AVI, Tripped("AVI"))) load += L_AVI;
        if (conduct(bk_.LSS, Tripped("LSS"))) load += L_LSS;
        if (conduct(bk_.RCS, Tripped("RCS"))) load += L_RCS;
    }
    if (e.acBus) load += e.pumps * L_PUMP / INV_EFF;   // inverter losses are real heat
    if (e.mainBus) {
        if (conduct(bk_.SCI, Tripped("SCI"))) load += L_SCI;
        if (conduct(bk_.DRV, Tripped("DRV"))) load += L_DRV;
    }
    for (const auto& r : rct_) load += r.StartupDraw();
    e.load = load;

    double etot = 0.0;
    for (const auto& b : bank_) etot += b.E;
    e.soc = etot / (3.0 * BANK_J);

    // Bus voltage sags as the battery drains -> current rises for the same
    // load -> breakers can trip. This coupling is what makes load management
    // during a cold start actually matter.
    e.V = (e.gen > 0.0) ? V_NOM : (e.essBus ? 700.0 + 300.0 * e.soc : 0.0);
    e.I = (e.V > 0.0) ? e.load / e.V : 0.0;

    e.aviPwr = e.essBus  && conduct(bk_.AVI, Tripped("AVI"));
    e.drvPwr = e.mainBus && conduct(bk_.DRV, Tripped("DRV"));
    e.rcsPwr = e.essBus  && conduct(bk_.RCS, Tripped("RCS"));

    e_ = e;
}

void ShipCore::StepBreakers() {
    if (!e_.essBus || e_.V <= 0.0) return;
    if (e_.I > ESS_RATING && !Tripped("ESS")) {
        trips_.push_back("ESS");
        Say("ESS FEED BREAKER TRIPPED - OVERCURRENT");
    }
}

void ShipCore::StepReactor(Reactor& r, double dt) {
    const bool cooled = (e_.pumps >= 1) && e_.essBus;

    auto abort = [&](const char* why) {
        r.stage = RctStage::Scram;
        r.fault = why;
        r.Pe = r.Pth = r.Q = r.coilI = r.B = r.dens = r.Tion = r.rf = 0.0;
        Say(std::string("RCT ") + r.id + " ABORT: " + why);
    };

    if (r.IsLive()) {
        if (!e_.essBus) { abort("BUS LOSS"); return; }
        if (!cooled) {
            r.noCool += dt;
            if (r.noCool > 30.0) { abort("NO COOLANT FLOW"); return; }
        } else {
            r.noCool = std::max(0.0, r.noCool - dt);
        }
    }

    switch (r.stage) {
    case RctStage::Pumpdown:
        // exponential pump-down of the chamber
        r.press *= std::exp(-dt / 12.0);
        if (r.press < 1.0e-3) {
            r.stage = RctStage::Field;
            Say(std::string("RCT ") + r.id + ": VACUUM ACHIEVED, COILS RAMPING");
        }
        break;

    case RctStage::Field:
        // superconducting coils charge to 50 kA -> 8 T.
        r.coilI = std::min(50.0, r.coilI + (50.0 / 75.0) * dt);
        r.B = 8.0 * r.coilI / 50.0;
        // without cryo flow the coils warm and QUENCH -- the signature
        // failure of a superconducting magnet.
        if (!cooled) r.coilT += 1.4 * dt;
        else         r.coilT += (20.0 - r.coilT) * 0.06 * dt;
        if (r.coilT > 40.0) { abort("COIL QUENCH"); return; }
        if (r.coilI >= 50.0) {
            r.stage = RctStage::Heat;
            Say(std::string("RCT ") + r.id + ": 8 T FIELD, FUEL INJECTION");
        }
        break;

    case RctStage::Heat:
        r.dens = std::min(1.2, r.dens + (1.2 / 60.0) * dt);
        r.Tion = std::min(15.0, r.Tion + (15.0 / 60.0) * dt);
        if (r.dens > 1.5) { abort("DENSITY DISRUPTION"); return; }
        if (r.Tion >= 10.0 && r.dens >= 1.0) {
            r.stage = RctStage::Ignite;
            Say(std::string("RCT ") + r.id + ": PLASMA AT 10 KEV - IGNITING");
        }
        break;

    case RctStage::Ignite:
        if (r.press > 1.0e-3) { abort("VACUUM LOSS"); return; }
        if (r.B < 7.0)        { abort("FIELD COLLAPSE"); return; }
        r.Q  = std::min(12.0, r.Q + (12.0 / 35.0) * dt);
        r.rf = P_RF * std::max(0.0, 1.0 - r.Q / 12.0);   // RF backs off as burn takes over
        if (r.Q >= 5.0) {
            r.stage = RctStage::Ascent;
            Say(std::string("RCT ") + r.id + ": BURN SELF-SUSTAINING (Q>5)");
        }
        break;

    case RctStage::Ascent:
        r.Q   = std::min(12.0, r.Q + (12.0 / 35.0) * dt);
        r.rf  = 0.0;
        r.Pth = std::min(PTH_RCT, r.Pth + (PTH_RCT / 45.0) * dt);
        r.Pe  = 0.4 * r.Pth;
        if (r.Pth >= PTH_RCT) {
            r.stage = RctStage::Online;
            Say(std::string("RCT ") + r.id + " ONLINE - 20 MWe");
        }
        break;

    case RctStage::Online:
        r.Pth = PTH_RCT; r.Pe = PE_RCT; r.Q = 12.0;
        if (Tcool_ > T_SCRAM) { abort("COOLANT OVERTEMP"); return; }
        break;

    default: break;
    }
}

void ShipCore::StepThermal(double dt) {
    double waste = 0.0;
    for (const auto& r : rct_) {
        if (r.stage == RctStage::Online)      waste += QW_RCT;
        else if (r.stage == RctStage::Ascent) waste += QW_RCT * r.Pth / PTH_RCT;
    }
    // Conservation: every watt the electrical system consumes ends up as heat.
    qIn_ = waste + e_.load;

    const double UA = e_.pumps * UA_PUMP;
    const double qPump = UA * std::max(0.0, Tcool_ - Trad_);
    qRad_ = EPS_RAD * SIGMA * A_HK * (std::pow(Trad_, 4) - std::pow(3.0, 4));

    Tcool_ += (qIn_ - qPump) / C_COOL * dt;
    Trad_  += (qPump - qRad_) / C_RAD * dt;
}

void ShipCore::StepDrive(double dt) {
    // The drive fires only if the main bus is hot and DRIVE CTL is closed.
    if (!e_.drvPwr || driveFail_ || fuel_ <= 0.0 || inert_ <= 0.0) {
        thrLevel_ = 0.0; jet_ = 0.0; thrust_ = 0.0;
        mdotFuel_ = mdotInert_ = 0.0;
        Ttorch_ *= std::exp(-dt / 300.0);
        return;
    }

    // ---- MASS AUGMENTATION -------------------------------------------------
    // vex is set by the MIXING VALVE, not by the fuel:
    //     vex = V_PROD / sqrt(R)
    // A richer mixture (higher R) slows the exhaust and multiplies the thrust.
    const double vex = V_PROD / std::sqrt(mix_);

    // Pilot commands an acceleration; we work out what the plant must deliver.
    double F   = mass_ * thrCmd_ * G0;
    double P   = 0.5 * F * vex;              // required jet power

    // The reactor has a ceiling. If the commanded accel needs more power than
    // the plant can make, we clip -- and the pilot's remedy is to RICHEN THE
    // MIXTURE, which lowers vex and buys thrust at the same power.
    if (P > TORCH_PMAX) {
        P = TORCH_PMAX;
        F = 2.0 * P / vex;
    }

    // Fuel burn follows the POWER; inert flow follows the MIXTURE.
    mdotFuel_  = P / SPEC_E;
    const double mdotTot = (vex > 0.0) ? F / vex : 0.0;
    mdotInert_ = std::max(0.0, mdotTot - mdotFuel_);

    fuel_  = std::max(0.0, fuel_  - mdotFuel_  * dt);
    inert_ = std::max(0.0, inert_ - mdotInert_ * dt);
    if (fuel_ <= 0.0 && !fuelOut_)  { fuelOut_  = true; Say("D/3He FLASK EMPTY - TORCH DEAD"); }
    if (inert_ <= 0.0 && !inertOut_){ inertOut_ = true; Say("INERT PROPELLANT EMPTY - RUN LEAN OR COAST"); }

    jet_      = P;
    thrust_   = F;
    thrLevel_ = std::min(1.0, F / (mass_ * G0));   // fraction of a 1 g thruster

    // ---- WASTE HEAT DEPENDS ON POWER ONLY, NOT ON MIXTURE ------------------
    // This is the elegant part: diluting the exhaust buys thrust for FREE
    // thermally. The radiator never knows you changed the mixture. You pay for
    // the extra thrust in PROPELLANT, not in heat.
    const double wst = WASTE_FR * P;
    Ttorch_ = (wst > 0.0) ? std::pow(wst / (0.9 * SIGMA * A_TORCH), 0.25)
                          : Ttorch_ * std::exp(-dt / 300.0);

    if (Ttorch_ > T_TORCH_LIM) {
        dmg_ += (Ttorch_ - T_TORCH_LIM) * dt;
        if (dmg_ > 1.2e5 && !driveFail_) {
            driveFail_ = true; thrLevel_ = 0.0;
            Say("DRIVE OVERHEAT FAILURE - TORCH OFFLINE");
        }
    } else {
        dmg_ = std::max(0.0, dmg_ - 200.0 * dt);
    }
}

void ShipCore::SetMixture(double R) {
    mix_ = clampd(R, MIX_MIN, MIX_MAX);
}

void ShipCore::SetAttitude(double roll, double pitch, double yaw, double rate) {
    avio_.SetTrueAttitude(roll, pitch, yaw);
    bodyRate_ = rate;
}

void ShipCore::Step(double dt) {
    SolveElec();
    StepBreakers();
    SolveElec();

    for (auto& r : rct_) StepReactor(r, dt);
    SolveElec();

    // battery: net power splits across armed banks
    const double net = e_.gen - e_.load;
    int armed = 0;
    for (const auto& b : bank_) if (b.armed) armed++;
    if (armed > 0 && e_.essBus) {
        const double per = net / armed;
        for (auto& b : bank_)
            if (b.armed) b.E = clampd(b.E + per * dt, 0.0, BANK_J);
    }

    StepThermal(dt);
    StepDrive(dt);

    // Avionics runs off the same tick. It needs to know: how much power the
    // electrical system is giving it, and how much FUSION power is running --
    // because the RCS helium pressurant is scavenged from the reactor's own
    // helium-4 ash (D + 3He -> 4He + p).
    double fusionPower = jet_;
    for (const auto& r : rct_) fusionPower += r.Pth;
    met_ += dt;
    avio_.Step(dt, met_, bodyRate_, e_.aviPwr, e_.rcsPwr, fusionPower);
}

// ------------------------------------------------------------------ commands
void ShipCore::ArmBank(int i, bool on) {
    if (i < 0 || i > 2) return;
    bank_[i].armed = on;
    Say(std::string("BANK ") + bank_[i].id + (on ? " ARMED" : " ISOLATED"));
}

void ShipCore::SetBreaker(const std::string& n, bool c) {
    if      (n=="GEN_A") bk_.GEN_A=c; else if (n=="GEN_B") bk_.GEN_B=c;
    else if (n=="TIE")   bk_.TIE=c;   else if (n=="ESS")   bk_.ESS=c;
    else if (n=="INV1")  bk_.INV1=c;  else if (n=="INV2")  bk_.INV2=c;
    else if (n=="AVI")   bk_.AVI=c;   else if (n=="LSS")   bk_.LSS=c;
    else if (n=="PUMP1") bk_.PUMP1=c; else if (n=="PUMP2") bk_.PUMP2=c;
    else if (n=="DRV")   bk_.DRV=c;   else if (n=="RCS")   bk_.RCS=c;
    else if (n=="SCI")   bk_.SCI=c;   else return;
    Say(n + (c ? " CLOSED" : " OPEN"));
}

bool ShipCore::ResetTrip(const std::string& n) {
    auto it = std::find(trips_.begin(), trips_.end(), n);
    if (it == trips_.end()) return false;
    trips_.erase(it);
    Say(n + " BREAKER RESET");
    return true;
}

void ShipCore::StartReactor(int i) {
    if (i < 0 || i > 1) return;
    Reactor& r = rct_[i];
    if (r.stage != RctStage::Standby && r.stage != RctStage::Scram) return;
    SolveElec();
    if (!e_.essBus) { Say("CANNOT START - NO ESSENTIAL BUS POWER"); return; }
    r.Reset();
    r.stage = RctStage::Pumpdown;
    Say(std::string("RCT ") + r.id + " STARTUP INITIATED");
    if (e_.pumps < 1) Say("WARNING: NO COOLANT FLOW - STARTUP WILL ABORT");
}

void ShipCore::ShutdownReactor(int i) {
    if (i < 0 || i > 1) return;
    rct_[i].Reset();
    Say(std::string("RCT ") + rct_[i].id + " SHUTDOWN");
}

void ShipCore::SetThrottle(double g) { thrCmd_ = clampd(g, 0.0, 1.0); }

// ------------------------------------------------------------------ persistence
// Orbiter saves this one line into the scenario file so a reactor that was
// online stays online across a save/load.
std::string ShipCore::Serialize() const {
    std::ostringstream o;
    o << (int)rct_[0].stage << ' ' << (int)rct_[1].stage << ' '
      << bank_[0].E << ' ' << bank_[1].E << ' ' << bank_[2].E << ' '
      << Tcool_ << ' ' << Trad_ << ' ' << thrCmd_;
    return o.str();
}
void ShipCore::Deserialize(const std::string& s) {
    std::istringstream in(s);
    int s0, s1;
    in >> s0 >> s1 >> bank_[0].E >> bank_[1].E >> bank_[2].E
       >> Tcool_ >> Trad_ >> thrCmd_;
    rct_[0].stage = (RctStage)s0;
    rct_[1].stage = (RctStage)s1;
    if (rct_[0].stage == RctStage::Online) { rct_[0].Pe = PE_RCT; rct_[0].Pth = PTH_RCT; rct_[0].Q = 12; rct_[0].B = 8; rct_[0].coilI = 50; rct_[0].press = 1e-4; }
    if (rct_[1].stage == RctStage::Online) { rct_[1].Pe = PE_RCT; rct_[1].Pth = PTH_RCT; rct_[1].Q = 12; rct_[1].B = 8; rct_[1].coilI = 50; rct_[1].press = 1e-4; }
}

} // namespace orion
