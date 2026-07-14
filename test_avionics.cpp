#include "Avionics.h"
#include <cstdio>
#include <cmath>
using namespace orion;

static void run(Avionics& a, double sec, double met0, double rate,
                bool avi=true, bool rcs=true, double fus=50e6, double dt=0.1) {
    int n=(int)(sec/dt);
    for(int i=0;i<n;i++) a.Step(dt, met0 + i*dt, rate, avi, rcs, fus);
}
int main(){
    printf("=== T1: P52 STAR ALIGNMENT (steady vehicle) ===\n");
    {
        Avionics a; double met=0;
        a.Step(0.1, met, 0.0, true, true, 50e6);
        a.SetAlignOption(AlignOpt::Refsmmat);
        a.StartAlign(met, 0.0);
        for(int i=0;i<1200;i++){ met+=0.1; a.Step(0.1, met, 0.0, true, true, 50e6);
            if(a.GetImu().aligned) break; }
        printf("  phase=%s aligned=%d drift=%.2f' starAngleDiff=%.2f'\n",
            AlignPhaseName(a.GetImu().phase), a.GetImu().aligned,
            a.GetImu().drift, a.GetImu().starAngleDiff);
        printf("  RESULT: %s\n", a.GetImu().aligned?"PASS":"FAIL (retry -> bad mark, valid)");
    }

    printf("\n=== T2: ALIGN REJECTED WHEN MANEUVERING ===\n");
    {
        Avionics a; a.Step(0.1,0,0,true,true,50e6);
        a.StartAlign(0.0, 0.30);      // 0.30 deg/s -- way over the 0.05 limit
        printf("  phase after start attempt: %s\n", AlignPhaseName(a.GetImu().phase));
        printf("  RESULT: %s\n", a.GetImu().phase==AlignPhase::Idle?"PASS":"FAIL");
    }

    printf("\n=== T3: GIMBAL LOCK (middle gimbal past 85 deg) ===\n");
    {
        Avionics a; double met=0;
        a.Step(0.1,met,0,true,true,50e6);
        a.StartAlign(met,0.0);
        for(int i=0;i<1200 && !a.GetImu().aligned;i++){met+=0.1;a.Step(0.1,met,0,true,true,50e6);}
        printf("  aligned first: %d\n", a.GetImu().aligned);
        // now yaw the ship past the middle-gimbal limit
        a.SetTrueAttitude(0, 0, 60);  run(a,1,met,0.0);
        printf("  yaw 60 deg -> lockWarn=%d locked=%d aligned=%d\n",
            a.GetImu().GimbalLockWarn(), a.GetImu().GimbalLocked(), a.GetImu().aligned);
        a.SetTrueAttitude(0, 0, 75);  run(a,1,met,0.0);
        printf("  yaw 75 deg -> lockWarn=%d locked=%d aligned=%d  <- WARNING\n",
            a.GetImu().GimbalLockWarn(), a.GetImu().GimbalLocked(), a.GetImu().aligned);
        a.SetTrueAttitude(0, 0, 88);  run(a,1,met,0.0);
        printf("  yaw 88 deg -> lockWarn=%d locked=%d aligned=%d  <- PLATFORM LOST\n",
            a.GetImu().GimbalLockWarn(), a.GetImu().GimbalLocked(), a.GetImu().aligned);
        printf("  RESULT: %s\n", (!a.GetImu().aligned && a.GetImu().GimbalLocked())?"PASS":"FAIL");
    }

    printf("\n=== T4: GDC SURVIVES GIMBAL LOCK (backup reference) ===\n");
    {
        Avionics a; double met=0;
        a.Step(0.1,met,0,true,true,50e6);
        a.StartAlign(met,0.0);
        for(int i=0;i<1200 && !a.GetImu().aligned;i++){met+=0.1;a.Step(0.1,met,0,true,true,50e6);}
        a.AlignGdcToImu();
        bool gdcBefore = a.GetGdc().aligned;
        a.SetTrueAttitude(0,0,88); run(a,1,met,0.0);
        printf("  after gimbal lock: IMU aligned=%d | GDC aligned=%d\n",
            a.GetImu().aligned, a.GetGdc().aligned);
        printf("  RESULT: %s (GDC has no gimbals, so it cannot lock)\n",
            (gdcBefore && !a.GetImu().aligned && a.GetGdc().aligned)?"PASS":"FAIL");
    }

    printf("\n=== T5: SCHEDULED REALIGNMENT (6 hr cadence) ===\n");
    {
        Avionics a; double met=0;
        a.Step(0.1,met,0,true,true,50e6);
        a.SetAlignInterval(6*3600);
        a.StartAlign(met,0.0);
        for(int i=0;i<1200 && !a.GetImu().aligned;i++){met+=0.1;a.Step(0.1,met,0,true,true,50e6);}
        double t0=met;
        printf("  aligned at MET %.0f s. next P52 due in %.2f hr\n", t0, a.TimeToAlign(met)/3600);
        run(a, 5*3600, met, 0.0); met += 5*3600;
        printf("  +5 hr : drift=%.2f'  due=%d\n", a.GetImu().drift, a.AlignDue(met));
        run(a, 1.2*3600, met, 0.0); met += 1.2*3600;
        printf("  +6.2hr: drift=%.2f'  due=%d  <- P52 NAG\n", a.GetImu().drift, a.AlignDue(met));
        printf("  RESULT: %s\n", a.AlignDue(met)?"PASS":"FAIL");
    }

    printf("\n=== T6: RCS -- NO HELIUM MEANS NO THRUST (full tanks!) ===\n");
    {
        Avionics a;
        a.Step(0.1,0,0,true,true,0);
        printf("  quad A: fuel=%.0f kg ox=%.0f kg  heTanks CLOSED\n",
            a.Quad(0).fuel, a.Quad(0).ox);
        double imp = a.FireQuad(0, 5000.0, 0.1);
        printf("  fire 5000 N.s -> delivered %.0f N.s   (tanks are FULL)\n", imp);
        bool noHe = (imp == 0.0);
        a.SetHeTank(0,1,true);
        a.Step(0.1,0,0,true,true,0);
        imp = a.FireQuad(0, 5000.0, 0.1);
        printf("  open HE TANK 1 -> delivered %.0f N.s, He press %.0f bar\n",
            imp, a.Quad(0).hePress);
        printf("  RESULT: %s\n", (noHe && imp>0)?"PASS":"FAIL");
    }

    printf("\n=== T7: FROZEN QUAD (heater off) ===\n");
    {
        Avionics a;
        a.SetHeTank(1,1,true); a.SetQuadHeater(1,false);
        run(a, 3000, 0, 0.0);
        printf("  quad B temp=%.0f K frozen=%d (N2O4 freezes at 262 K)\n",
            a.Quad(1).temp, a.Quad(1).Frozen());
        double imp = a.FireQuad(1, 5000.0, 0.1);
        printf("  fire -> %.0f N.s\n", imp);
        a.SetQuadHeater(1,true);
        run(a, 3000, 0, 0.0);
        double imp2 = a.FireQuad(1, 5000.0, 0.1);
        printf("  heater ON, temp=%.0f K -> fire = %.0f N.s\n", a.Quad(1).temp, imp2);
        printf("  RESULT: %s\n", (imp==0 && imp2>0)?"PASS":"FAIL");
    }

    printf("\n=== T8: HELIUM MAKEUP FROM REACTOR ASH (D+3He -> 4He) ===\n");
    {
        Avionics a;
        for(int i=0;i<4;i++){ a.SetHeTank(i,1,true); a.SetQuadHeater(i,true); }
        // burn down quad A's helium
        for(int i=0;i<110;i++) a.FireQuad(0, 20000.0, 0.1);
        double pLow = a.Quad(0).hePress;
        printf("  after heavy firing: quad A He = %.0f bar, reserve = %.2f kg\n",
            pLow, a.HeReserve());
        // Helium ash scales with FUSION power. The housekeeping plant (100 MW)
        // makes only ~0.2 mg/s -- nothing. The TORCH runs at 9.37 TW and makes
        // ~21 g/s of He-4. So you replenish pressurant only while under thrust.
        run(a, 300, 0, 0.0, true, true, 100e6);   // housekeeping only
        printf("  housekeeping 100 MW, 5 min -> reserve = %.4f kg (negligible)\n", a.HeReserve());
        run(a, 20000, 0, 0.0, true, true, 9.37e12);  // TORCH BURNING
        printf("  TORCH 9.37 TW for 5.6 hr -> He reserve = %.2f kg, quad A = %.0f bar\n",
            a.HeReserve(), a.Quad(0).hePress);
        printf("  RESULT: %s (ship makes its own pressurant)\n",
            (a.Quad(0).hePress > pLow)?"PASS":"FAIL");
    }
    return 0;
}
