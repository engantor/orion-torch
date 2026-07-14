// ============================================================================
//  OrionDPS.cpp -- ORION Data Processing System.
//
//  Phase 1: MDU render + keyboard + scratchpad (zero logic).
//  Phase 2: SPEC/DISP PAGE SWITCHING + the first READ-ONLY page, DISP 78
//           SM SYS SUMM 1, which reads the live systems model from ShipCore.
//
//  Realised as an Orbiter MFD mode ("Orion DPS"), built as a plugin DLL into
//  Orbiter\Modules\Plugin\OrionDPS.dll and enabled in the Launchpad Modules tab.
//
//  How the MFD reaches the ship's systems: the focus VESSEL* is verified to be
//  an Orion by class name, then cast to Orion* (declared in the shared Orion.h)
//  and read through Orion::Core(). Only read-only, header-inline access is used
//  across the DLL boundary. ShipCore.cpp + Avionics.cpp are linked into this DLL
//  for the non-inline helpers (StageName, etc.); they carry no state -- the one
//  live ShipCore instance lives in the vessel.
// ============================================================================
#include "Orion.h"          // class Orion (+ orbitersdk.h + ShipCore.h)
#include <cstdio>
#include <cstring>
#include <string>

using namespace orion;

static int g_dpsMode = 0;

// page identifiers. 0 = OPS home display; others are the SPEC/DISP page numbers.
enum { PG_HOME = 0, PG_DISP78 = 78, PG_DISP99 = 99 };

// ---------------------------------------------------------------------------
class OrionDPS : public MFD2 {
public:
    OrionDPS(DWORD w, DWORD h, VESSEL* v);

    bool  Update(oapi::Sketchpad* skp) override;
    char* ButtonLabel(int bt) override;
    int   ButtonMenu(const MFDBUTTONMENU** menu) const override;
    bool  ConsumeKeyBuffered(DWORD key) override;
    bool  ConsumeButton(int bt, int event) override;

    static OAPI_MSGTYPE MsgProc(UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam);

private:
    bool    HandleKey(DWORD key);
    void    ExecEntry();                 // terminate the scratchpad entry (EXEC)
    Orion*  FocusOrion() const;          // focus vessel as Orion*, or nullptr
    static void FormatClock(char* out, double t);

    // page renderers
    void DrawHome(oapi::Sketchpad* skp, int x, int& y, int dy);
    void DrawSysSumm1(oapi::Sketchpad* skp, int x, int y, int dy, int w);
    void DrawPlaceholder(oapi::Sketchpad* skp, int x, int& y, int dy, const char* what);

    VESSEL*     v_;
    int         page_    = PG_HOME;
    std::string scratch_;
    std::string echo_;
    std::string annun_;
};

// DPS keypad mapped to the PC keyboard; command keys also appear as MFD buttons.
struct BtnMap { const char* label; char sel; DWORD key; };
static const BtnMap BTN[] = {
    { "OPS", 'O', OAPI_KEY_O }, { "SPC", 'S', OAPI_KEY_S },
    { "ITM", 'I', OAPI_KEY_I }, { "EXE", 'E', OAPI_KEY_E },
    { "RSM", 'R', OAPI_KEY_R }, { "CLR", 'C', OAPI_KEY_C },
    { "FLT", 'F', OAPI_KEY_F }, { "SYS", 'Y', OAPI_KEY_Y },
    { "MSG", 'M', OAPI_KEY_M }, { "ACK", 'A', OAPI_KEY_A },
    { "GPC", 'G', OAPI_KEY_G },
};
static const int NBTN = (int)(sizeof(BTN) / sizeof(BTN[0]));

// colours (0x00BBGGRR)
static const DWORD GRN = 0x0000FF00, CYA = 0x00FFFF00, WHT = 0x00FFFFFF,
                   AMB = 0x0000BEFF, RED = 0x004040FF, DIM = 0x00558855;

OrionDPS::OrionDPS(DWORD w, DWORD h, VESSEL* v) : MFD2(w, h, v), v_(v) {
    echo_  = "--";
    annun_ = "--";
}

OAPI_MSGTYPE OrionDPS::MsgProc(UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam) {
    // On x64, OAPI_MSGTYPE is LRESULT (pointer-width). Casting to int would
    // truncate the instance pointer and crash on open.
    if (msg == OAPI_MSG_MFD_OPENED)
        return (OAPI_MSGTYPE)(new OrionDPS(LOWORD(wparam), HIWORD(wparam), (VESSEL*)lparam));
    return 0;
}

Orion* OrionDPS::FocusOrion() const {
    if (!v_) return nullptr;
    const char* cn = v_->GetClassName();
    if (cn && !strcmp(cn, "Orion")) return (Orion*)v_;   // safe: verified class
    return nullptr;
}

void OrionDPS::FormatClock(char* out, double t) {
    if (t < 0) t = 0;
    int d = (int)(t / 86400.0);
    double r = t - d * 86400.0;
    int hh = (int)(r / 3600.0); r -= hh * 3600.0;
    int mm = (int)(r / 60.0);
    int ss = (int)(r - mm * 60.0);
    sprintf(out, "%03d/%02d:%02d:%02d", d, hh, mm, ss);
}

// ---------------------------------------------------------------------------
// EXEC: terminate a scratchpad entry. Phase 2 wires PAGE CALLS only
// (SPEC nn / OPS nn); ITEM entries still do nothing (Phase 3+).
void OrionDPS::ExecEntry() {
    if (scratch_.empty()) return;
    if (scratch_.rfind("SPEC ", 0) == 0) {
        int n = atoi(scratch_.c_str() + 5);
        page_ = n;
        echo_ = scratch_;
    } else if (scratch_.rfind("OPS ", 0) == 0) {
        page_ = PG_HOME;                 // OPS loads its home display
        echo_ = scratch_;
    } else {
        echo_ = scratch_ + "  (phase 2: no item logic)";
    }
    scratch_.clear();
}

bool OrionDPS::HandleKey(DWORD key) {
    switch (key) {
    case OAPI_KEY_0: case OAPI_KEY_NUMPAD0: scratch_ += '0'; return true;
    case OAPI_KEY_1: case OAPI_KEY_NUMPAD1: scratch_ += '1'; return true;
    case OAPI_KEY_2: case OAPI_KEY_NUMPAD2: scratch_ += '2'; return true;
    case OAPI_KEY_3: case OAPI_KEY_NUMPAD3: scratch_ += '3'; return true;
    case OAPI_KEY_4: case OAPI_KEY_NUMPAD4: scratch_ += '4'; return true;
    case OAPI_KEY_5: case OAPI_KEY_NUMPAD5: scratch_ += '5'; return true;
    case OAPI_KEY_6: case OAPI_KEY_NUMPAD6: scratch_ += '6'; return true;
    case OAPI_KEY_7: case OAPI_KEY_NUMPAD7: scratch_ += '7'; return true;
    case OAPI_KEY_8: case OAPI_KEY_NUMPAD8: scratch_ += '8'; return true;
    case OAPI_KEY_9: case OAPI_KEY_NUMPAD9: scratch_ += '9'; return true;
    case OAPI_KEY_EQUALS: case OAPI_KEY_ADD:      scratch_ += '+'; return true;
    case OAPI_KEY_MINUS:  case OAPI_KEY_SUBTRACT: scratch_ += '-'; return true;
    case OAPI_KEY_PERIOD: case OAPI_KEY_DECIMAL:  scratch_ += '.'; return true;
    case OAPI_KEY_BACK: if (!scratch_.empty()) scratch_.pop_back(); return true;
    case OAPI_KEY_C:    scratch_.clear(); return true;

    case OAPI_KEY_O: scratch_ = "OPS ";  return true;
    case OAPI_KEY_S: scratch_ = "SPEC "; return true;
    case OAPI_KEY_I: scratch_ = "ITEM "; return true;

    case OAPI_KEY_E: case OAPI_KEY_RETURN: case OAPI_KEY_NUMPADENTER:
        ExecEntry(); return true;

    case OAPI_KEY_R:                                  // RESUME -> back to OPS home
        scratch_.clear(); page_ = PG_HOME; annun_ = "RESUME"; return true;

    case OAPI_KEY_Y:                                  // SYS SUMM -> DISP 78
        page_ = PG_DISP78; annun_ = "SYS SUMM"; return true;
    case OAPI_KEY_F:                                  // FAULT SUMM -> DISP 99
        page_ = PG_DISP99; annun_ = "FAULT SUMM"; return true;

    case OAPI_KEY_G: annun_ = "GPC/CRT";    return true;
    case OAPI_KEY_M: annun_ = "MSG RESET";  return true;
    case OAPI_KEY_A: annun_ = "ACK";        return true;

    default: return false;
    }
}

bool OrionDPS::ConsumeKeyBuffered(DWORD key) {
    if (HandleKey(key)) { InvalidateDisplay(); return true; }
    return false;
}
bool OrionDPS::ConsumeButton(int bt, int event) {
    if (!(event & PANEL_MOUSE_LBDOWN)) return false;
    if (bt < 0 || bt >= NBTN) return false;
    if (HandleKey(BTN[bt].key)) { InvalidateDisplay(); return true; }
    return false;
}
char* OrionDPS::ButtonLabel(int bt) {
    return (bt >= 0 && bt < NBTN) ? (char*)BTN[bt].label : nullptr;
}
int OrionDPS::ButtonMenu(const MFDBUTTONMENU** menu) const {
    static MFDBUTTONMENU m[NBTN];
    static const char* desc[NBTN] = {
        "Major function OPS", "Call SPEC page", "Begin ITEM entry",
        "EXEC entry", "RESUME to OPS", "CLEAR scratchpad",
        "Fault summary", "Sys summary", "Msg reset", "Acknowledge", "GPC / CRT",
    };
    for (int i = 0; i < NBTN; i++) { m[i].line1 = desc[i]; m[i].line2 = 0; m[i].selchar = BTN[i].sel; }
    if (menu) *menu = m;
    return NBTN;
}

// ---------------------------------------------------------------------------
// helpers for the summary page: label + value at a column, colour-coded.
static void Cell(oapi::Sketchpad* skp, int x, int y,
                 const char* label, const char* val, DWORD col) {
    skp->SetTextColor(DIM);  skp->Text(x, y, label, (int)strlen(label));
    skp->SetTextColor(col);  skp->Text(x + 56, y, val, (int)strlen(val));
}
static const char* LiveDark(bool live) { return live ? "LIVE" : "DARK"; }

void OrionDPS::DrawHome(oapi::Sketchpad* skp, int x, int& y, int dy) {
    auto L = [&](const char* s, DWORD c){ skp->SetTextColor(c); skp->Text(x, y, s, (int)strlen(s)); y += dy; };
    char buf[96];
    L("DATA PROCESSING SYSTEM", GRN);
    L("OPS 000  STANDBY", GRN);
    y += dy / 2;
    L("SYS -> DISP 78 SM SYS SUMM", WHT);
    L("SPEC nn EXEC -> spec page", WHT);
    L("FLT -> DISP 99   RSM -> home", WHT);
    y += dy / 2;
    sprintf(buf, "KEY   %s", annun_.c_str()); L(buf, AMB);
    sprintf(buf, "LAST  %s", echo_.c_str());  L(buf, DIM);
}

void OrionDPS::DrawPlaceholder(oapi::Sketchpad* skp, int x, int& y, int dy, const char* what) {
    auto L = [&](const char* s, DWORD c){ skp->SetTextColor(c); skp->Text(x, y, s, (int)strlen(s)); y += dy; };
    L(what, AMB);
    L("NOT IMPLEMENTED YET", DIM);
    L("(RSM to return to OPS)", DIM);
}

// DISP 78 -- SM SYS SUMM 1. The page you leave up. Live from ShipCore.
void OrionDPS::DrawSysSumm1(oapi::Sketchpad* skp, int xL, int y0, int dy, int w) {
    Orion* o = FocusOrion();
    if (!o) {
        skp->SetTextColor(RED);
        skp->Text(xL, y0, "NO ORION VESSEL IN FOCUS", 24);
        return;
    }
    ShipCore&        c   = o->Core();
    const ElecState& e   = c.Elec();
    Avionics&        a   = c.Avio();
    const Imu&       imu = a.GetImu();

    const int xR = xL + w / 2;
    char b[48];
    int yL = y0, yR = y0;
    auto HL = [&](int x, int& y, const char* s){ skp->SetTextColor(CYA); skp->Text(x, y, s, (int)strlen(s)); y += dy; };

    // ----- left column: EPS / REACTOR / GNC -----
    HL(xL, yL, "EPS");
    Cell(skp, xL, yL, "MAIN A", LiveDark(e.mainA), e.mainA?GRN:AMB); yL += dy;
    Cell(skp, xL, yL, "MAIN B", LiveDark(e.mainB), e.mainB?GRN:AMB); yL += dy;
    Cell(skp, xL, yL, "CNTL",   LiveDark(e.aviPwr), e.aviPwr?GRN:AMB); yL += dy;  // proxy: AVI power
    sprintf(b, "%3.0f PCT", 100.0*e.soc);
    Cell(skp, xL, yL, "BATT", b, e.soc>0.5?GRN:(e.soc>0.15?AMB:RED)); yL += dy;

    HL(xL, yL, "REACTOR");
    Cell(skp, xL, yL, "RCT A", StageName(c.Rct(0).stage),
         c.Rct(0).stage==RctStage::Online?GRN:(c.Rct(0).stage==RctStage::Scram?RED:AMB)); yL += dy;
    Cell(skp, xL, yL, "RCT B", StageName(c.Rct(1).stage),
         c.Rct(1).stage==RctStage::Online?GRN:(c.Rct(1).stage==RctStage::Scram?RED:AMB)); yL += dy;
    sprintf(b, "%.1f", c.Rct(0).Q);
    Cell(skp, xL, yL, "Q", b, c.Rct(0).Q>=5?GRN:AMB); yL += dy;

    HL(xL, yL, "GNC");
    Cell(skp, xL, yL, "IMU", imu.aligned?"ALIGNED":(imu.powered?"NO ALGN":"OFF"),
         imu.aligned?GRN:AMB); yL += dy;
    sprintf(b, "%.1f MIN", imu.drift);
    Cell(skp, xL, yL, "DRIFT", b, imu.drift>8.0?AMB:GRN); yL += dy;
    sprintf(b, "%.1f DEG", imu.middle);
    Cell(skp, xL, yL, "MGMB", b, imu.GimbalLocked()?RED:(imu.GimbalLockWarn()?AMB:GRN)); yL += dy;
    Cell(skp, xL, yL, "GDC", a.GetGdc().aligned?"ALIGNED":"OFF", a.GetGdc().aligned?GRN:DIM); yL += dy;

    // ----- right column: THERMAL / PROP / RCS -----
    HL(xR, yR, "THERMAL");
    sprintf(b, "%.0f K", c.CoolantT());
    Cell(skp, xR, yR, "COOL", b, c.CoolantT()<T_SHED?GRN:(c.CoolantT()<T_SCRAM?AMB:RED)); yR += dy;
    sprintf(b, "%.0f K", c.RadiatorT());
    Cell(skp, xR, yR, "RAD", b, GRN); yR += dy;
    sprintf(b, "%d", e.pumps);
    Cell(skp, xR, yR, "PUMPS", b, e.pumps>0?GRN:AMB); yR += dy;
    Cell(skp, xR, yR, "FLOW", e.pumps>0?"OK":"NONE", e.pumps>0?GRN:AMB); yR += dy;   // no kg/s in model

    HL(xR, yR, "PROP");
    sprintf(b, "%.1f T", c.FuelKg()/1000.0);
    Cell(skp, xR, yR, "FUEL", b, c.FuelKg()>0.10*FUEL0?GRN:AMB); yR += dy;
    sprintf(b, "%.1f T", c.InertKg()/1000.0);
    Cell(skp, xR, yR, "INERT", b, c.InertKg()>0.10*INERT0?GRN:AMB); yR += dy;
    sprintf(b, "%.2f", c.Mixture());
    Cell(skp, xR, yR, "MIX R", b, WHT); yR += dy;
    sprintf(b, "%.0f K", c.TorchT());
    Cell(skp, xR, yR, "DRVR", b, c.TorchT()<T_TORCH_LIM?GRN:RED); yR += dy;

    HL(xR, yR, "RCS");
    const RcsQuad& q = a.Quad(0);
    Cell(skp, xR, yR, "QUAD A", q.Operable()?"OK":(q.Frozen()?"FRZ":"--"),
         q.Operable()?GRN:AMB); yR += dy;
    sprintf(b, "%.1f KG", a.HeReserve());
    Cell(skp, xR, yR, "HE RES", b, GRN); yR += dy;
}

// ---------------------------------------------------------------------------
bool OrionDPS::Update(oapi::Sketchpad* skp) {
    const int w = (int)W, h = (int)H;
    int dy = h / 24; if (dy < 11) dy = 11;
    const int x = 6;
    int y = dy;

    skp->SetFont(GetDefaultFont(0));

    // ---- header ----
    char clk[24]; FormatClock(clk, oapiGetSimTime());
    char hdr[64];
    skp->SetTextColor(CYA);
    sprintf(hdr, "GPC1  OPS 000  MM 000    CRT 1"); skp->Text(x, y, hdr, (int)strlen(hdr)); y += dy;
    sprintf(hdr, "0001/%03d/  ORION DPS  %s", page_, clk); skp->Text(x, y, hdr, (int)strlen(hdr)); y += dy;
    skp->SetTextColor(DIM); skp->Text(x, y, "--------------------------------", 32); y += dy;

    // ---- body by page ----
    switch (page_) {
    case PG_HOME:   DrawHome(skp, x, y, dy); break;
    case PG_DISP78: DrawSysSumm1(skp, x, y, dy, w); break;
    case PG_DISP99: { int yy = y; DrawPlaceholder(skp, x, yy, dy, "DISP 99  FAULT SUMM"); } break;
    default: {
        char t[40]; sprintf(t, "SPEC %d", page_);
        int yy = y; DrawPlaceholder(skp, x, yy, dy, t);
    } break;
    }

    // ---- scratchpad, anchored bottom ----
    y = h - dy * 2;
    skp->SetTextColor(DIM); skp->Text(x, y, "--------------------------------", 32); y += dy;
    std::string sp = "> " + scratch_ + "_";
    skp->SetTextColor(GRN); skp->Text(x, y, sp.c_str(), (int)sp.size());
    return true;
}

// ---------------------------------------------------------------------------
DLLCLBK void InitModule(HINSTANCE hDLL) {
    static char name[] = "Orion DPS";
    MFDMODESPECEX spec;
    spec.name    = name;
    spec.key     = OAPI_KEY_D;
    spec.context = nullptr;
    spec.msgproc = OrionDPS::MsgProc;
    g_dpsMode = oapiRegisterMFDMode(spec);
}
DLLCLBK void ExitModule(HINSTANCE hDLL) {
    oapiUnregisterMFDMode(g_dpsMode);
}
