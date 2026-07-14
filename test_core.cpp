// Headless test harness. Compiles and runs anywhere -- no Orbiter, no Windows.
// This is the payoff of keeping ShipCore engine-agnostic: you can regression-test
// the reactor and electrical logic in a terminal in half a second.
#include "ShipCore.h"
#include <cstdio>
#include <cmath>

using namespace orion;

static void run(ShipCore& s, double seconds, double dt = 0.1) {
    int n = (int)(seconds / dt);
    for (int i = 0; i < n; i++) s.Step(dt);
}
static double socPct(const ShipCore& s) { return 100.0 * s.Elec().soc; }

int main() {
    printf("=== T1: NOMINAL COLD START ===\n");
    {
        ShipCore s;
        s.ArmBank(0,true); s.ArmBank(1,true); s.ArmBank(2,true);
        s.SetBreaker("AVI",true); s.SetBreaker("LSS",true);
        s.SetBreaker("INV1",true); s.SetBreaker("PUMP1",true);
        run(s, 2);
        printf("  bus up: ess=%d ac=%d pumps=%d soc=%.0f%% V=%.0f\n",
            s.Elec().essBus, s.Elec().acBus, s.Elec().pumps, socPct(s), s.Elec().V);

        s.StartReactor(0);
        for (int i = 0; i < 6000 && s.Rct(0).stage != RctStage::Online
                              && s.Rct(0).stage != RctStage::Scram; i++) s.Step(0.1);
        const Reactor& r = s.Rct(0);
        printf("  reactor A -> %s | B=%.1fT Q=%.1f Pe=%.0f MW press=%.1e soc=%.0f%%\n",
            StageName(r.stage), r.B, r.Q, r.Pe/1e6, r.press, socPct(s));

        s.SetBreaker("GEN_A",true); s.SetBreaker("ESS",true);
        run(s, 600);
        printf("  on generator: net=%+.1f MW soc=%.0f%% (recharging)\n",
            (s.Elec().gen - s.Elec().load)/1e6, socPct(s));
        run(s, 6000);
        printf("  thermal equilibrium: Tcool=%.0f K Trad=%.0f K (shed %.0f scram %.0f)\n",
            s.CoolantT(), s.RadiatorT(), T_SHED, T_SCRAM);
        printf("  RESULT: %s\n", (s.Rct(0).stage==RctStage::Online && s.CoolantT()<T_SHED)
            ? "PASS" : "FAIL");
    }

    printf("\n=== T2: DRY START (no coolant) must abort ===\n");
    {
        ShipCore s;
        s.ArmBank(0,true); s.ArmBank(1,true); s.ArmBank(2,true);
        s.SetBreaker("AVI",true);
        s.StartReactor(0);
        run(s, 60);
        printf("  reactor A -> %s (%s)\n", StageName(s.Rct(0).stage), s.Rct(0).fault.c_str());
        printf("  RESULT: %s\n", s.Rct(0).stage==RctStage::Scram ? "PASS" : "FAIL");
    }

    printf("\n=== T3: COIL QUENCH (lose cooling during FIELD ramp) ===\n");
    {
        ShipCore s;
        s.ArmBank(0,true); s.ArmBank(1,true); s.ArmBank(2,true);
        s.SetBreaker("AVI",true); s.SetBreaker("INV1",true); s.SetBreaker("PUMP1",true);
        s.StartReactor(0);
        // pump-down takes ~221 s (12 * ln(1e5/1e-3)); run past it into FIELD
        run(s, 240);
        printf("  stage before pump loss: %s (coilT=%.1f K)\n",
            StageName(s.Rct(0).stage), s.Rct(0).coilT);
        s.SetBreaker("PUMP1", false);     // kill cooling mid coil-ramp
        run(s, 20);
        printf("  reactor A -> %s (%s) coilT=%.1f K\n",
            StageName(s.Rct(0).stage), s.Rct(0).fault.c_str(), s.Rct(0).coilT);
        printf("  RESULT: %s\n",
            (s.Rct(0).stage==RctStage::Scram && s.Rct(0).fault=="COIL QUENCH") ? "PASS" : "FAIL");
    }

    printf("\n=== T4: STRANDING (battery flat, no reactor) ===\n");
    {
        ShipCore s;
        s.ArmBank(0,true); s.ArmBank(1,true); s.ArmBank(2,true);
        s.SetBreaker("AVI",true); s.SetBreaker("LSS",true);
        s.SetBreaker("INV1",true); s.SetBreaker("PUMP1",true);
        double dead = -1;
        for (int i = 0; i < 200000; i++) {
            s.Step(0.1);
            if (s.Elec().soc <= 0.0005) { dead = i * 0.1; break; }
        }
        printf("  battery flat at t=%.0f min, ess bus=%d\n", dead/60.0, s.Elec().essBus);
        printf("  RESULT: %s\n", dead > 0 ? "PASS (stranding reachable)" : "FAIL");
    }

    printf("\n=== T5: DRIVE GATED BY POWER ===\n");
    {
        ShipCore s;
        s.SetVesselMass(650000);
        s.SetThrottle(0.30);
        run(s, 1);
        printf("  cold ship, throttle 0.30 g -> level=%.2f (must be 0)\n", s.ThrottleLevel());
        bool gate1 = s.ThrottleLevel() == 0.0;

        // full power-up
        s.ArmBank(0,true); s.ArmBank(1,true); s.ArmBank(2,true);
        s.SetBreaker("AVI",true); s.SetBreaker("INV1",true); s.SetBreaker("PUMP1",true);
        s.StartReactor(0);
        for (int i=0;i<6000 && s.Rct(0).stage!=RctStage::Online;i++) s.Step(0.1);
        s.SetBreaker("GEN_A",true); s.SetBreaker("ESS",true);
        run(s, 5);
        printf("  reactor online, DRV breaker OPEN -> level=%.2f (still 0)\n", s.ThrottleLevel());
        bool gate2 = s.ThrottleLevel() == 0.0;

        s.SetBreaker("DRV", true);
        run(s, 5);
        printf("  DRV closed -> level=%.2f, torch radiator=%.0f K\n",
            s.ThrottleLevel(), s.TorchT());
        bool gate3 = s.ThrottleLevel() > 0.0;
        printf("  RESULT: %s\n", (gate1 && gate2 && gate3) ? "PASS" : "FAIL");
    }

    printf("\n=== T7: MASS AUGMENTATION / MIXING RATIO ===\n");
    {
        ShipCore s;
        s.SetVesselMass(650000);
        // full power-up
        s.ArmBank(0,true); s.ArmBank(1,true); s.ArmBank(2,true);
        s.SetBreaker("AVI",true); s.SetBreaker("INV1",true); s.SetBreaker("PUMP1",true);
        s.StartReactor(0);
        for(int i=0;i<6000 && s.Rct(0).stage!=RctStage::Online;i++) s.Step(0.1);
        s.SetBreaker("GEN_A",true); s.SetBreaker("ESS",true); s.SetBreaker("DRV",true);
        s.SetBreaker("RCS",true);
        run(s, 2);

        printf("  R      vex[m/s]   Isp[Ms]  a_max[g]  mdot_fuel  mdot_inert  radiator\n");
        bool mono = true; double prevA = -1;
        for (double R : {1.0, 2.0, 4.0, 7.3, 15.0, 30.0}) {
            s.SetMixture(R);
            s.SetThrottle(s.MaxAccelG());     // command full available accel
            run(s, 2);
            printf("  %5.1f  %.3e  %6.3f  %7.3f  %9.4f  %10.4f  %6.0f K\n",
                s.Mixture(), s.ExhaustV(), s.Isp()/1e6, s.MaxAccelG(),
                s.MdotFuel(), s.MdotInert(), s.TorchT());
            if (s.MaxAccelG() < prevA) mono = false;
            prevA = s.MaxAccelG();
        }
        printf("  -> richer mixture = more thrust at the SAME power\n");
        printf("  RESULT: %s\n", mono ? "PASS (a_max rises with R)" : "FAIL");
    }

    printf("\n=== T8: DESIGN POINT + TANK SPLIT ===\n");
    {
        ShipCore s;
        s.SetVesselMass(650000);
        s.ArmBank(0,true); s.ArmBank(1,true); s.ArmBank(2,true);
        s.SetBreaker("AVI",true); s.SetBreaker("INV1",true); s.SetBreaker("PUMP1",true);
        s.StartReactor(0);
        for(int i=0;i<6000 && s.Rct(0).stage!=RctStage::Online;i++) s.Step(0.1);
        s.SetBreaker("GEN_A",true); s.SetBreaker("ESS",true); s.SetBreaker("DRV",true);
        s.SetMixture(7.30);
        s.SetThrottle(0.30);
        run(s, 2);
        printf("  0.30 g at R=7.3: vex=%.3e (Isp %.2f Ms), F=%.3e N, P=%.2f TW\n",
            s.ExhaustV(), s.Isp()/1e6, s.Thrust(), s.JetPower()/1e12);
        printf("  mdot fuel=%.4f kg/s  inert=%.4f kg/s  ratio=%.2f\n",
            s.MdotFuel(), s.MdotInert(), (s.MdotFuel()+s.MdotInert())/s.MdotFuel());
        double burn = 553001;
        printf("  Mars burn %.2f d ->  FUEL %.1f t   INERT %.1f t\n",
            burn/86400, s.MdotFuel()*burn/1000, s.MdotInert()*burn/1000);
        bool ok = std::fabs(s.JetPower()/1e12 - 9.38) < 0.2
               && std::fabs((s.MdotFuel()+s.MdotInert())/s.MdotFuel() - 7.30) < 0.05;
        printf("  RESULT: %s\n", ok ? "PASS (matches analysis)" : "FAIL");
    }

    printf("\n=== T9: AVIONICS FOLDED IN (P52 + gimbal lock + RCS helium) ===\n");
    {
        ShipCore s;
        s.ArmBank(0,true); s.ArmBank(1,true); s.ArmBank(2,true);
        s.SetBreaker("AVI",true); s.SetBreaker("RCS",true);
        s.SetBreaker("INV1",true); s.SetBreaker("PUMP1",true);
        s.SetAttitude(0,0,0, 0.0);
        run(s, 1);
        s.Avio().StartAlign(0.0, 0.0);
        for (int i=0;i<1500 && !s.Avio().GetImu().aligned; i++) s.Step(0.1);
        printf("  IMU via ShipCore: aligned=%d  drift=%.2f'\n",
            s.Avio().GetImu().aligned, s.Avio().GetImu().drift);
        s.SetAttitude(0,0,88, 0.0);
        run(s, 1);
        printf("  yaw 88 deg -> gimbal locked=%d, IMU aligned=%d\n",
            s.Avio().GetImu().GimbalLocked(), s.Avio().GetImu().aligned);
        s.Avio().SetHeTank(0,1,true); s.Avio().SetQuadHeater(0,true);
        run(s, 400);
        double imp = s.Avio().FireQuad(0, 5000.0, 0.1);
        printf("  RCS quad A (He open, heated): %.0f N.s @ %.0f bar\n",
            imp, s.Avio().Quad(0).hePress);
        printf("  RESULT: %s\n",
            (!s.Avio().GetImu().aligned && imp > 0) ? "PASS" : "FAIL");
    }

    printf("\n=== T6: PERSISTENCE ROUND-TRIP ===\n");
    {
        ShipCore a;
        a.ArmBank(0,true); a.ArmBank(1,true); a.ArmBank(2,true);
        a.SetBreaker("AVI",true); a.SetBreaker("INV1",true); a.SetBreaker("PUMP1",true);
        a.StartReactor(0);
        for (int i=0;i<6000 && a.Rct(0).stage!=RctStage::Online;i++) a.Step(0.1);
        std::string blob = a.Serialize();
        ShipCore b;
        b.Deserialize(blob);
        printf("  saved: [%s]\n", blob.c_str());
        printf("  restored reactor A: %s, Tcool=%.0f K\n",
            StageName(b.Rct(0).stage), b.CoolantT());
        printf("  RESULT: %s\n", b.Rct(0).stage==a.Rct(0).stage ? "PASS" : "FAIL");
    }
    return 0;
}
