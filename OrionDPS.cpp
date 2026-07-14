// ============================================================================
//  OrionDPS.cpp -- ORION Data Processing System, Phase 1.
//
//  A Space-Shuttle-GPC/MEDS-style MDU realised as an Orbiter MFD mode. This is
//  Phase 1 of the DPS build order: MDU RENDER + KEYBOARD + SCRATCHPAD, and
//  DELIBERATELY ZERO SUBSYSTEM LOGIC. Item entries do not actuate anything yet;
//  the point is to get the *feel* of the page + keypad before wiring pages to
//  ShipCore (that is Phase 2+).
//
//  Because there is no logic, this file has NO dependency on ShipCore -- it only
//  needs the Orbiter SDK. Phase 2 will add page switching and read ShipCore via
//  the focus vessel.
//
//  BUILD: 64-bit plugin DLL -> Orbiter\Modules\Plugin\OrionDPS.dll, then enable
//  it in the Launchpad "Modules" tab. In flight, an MFD's SEL menu -> "Orion DPS".
// ============================================================================
#define STRICT
#include "orbitersdk.h"
#include <cstdio>
#include <cstring>
#include <string>

static int g_dpsMode = 0;

// ---------------------------------------------------------------------------
class OrionDPS : public MFD2 {
public:
    OrionDPS(DWORD w, DWORD h, VESSEL* v);

    bool  Update(oapi::Sketchpad* skp) override;
    char* ButtonLabel(int bt) override;
    int   ButtonMenu(const MFDBUTTONMENU** menu) const override;
    bool  ConsumeKeyBuffered(DWORD key) override;
    bool  ConsumeButton(int bt, int event) override;

    // NOTE: msgproc returns OAPI_MSGTYPE. On x64 that is LRESULT (pointer-width),
    // so the "MFD opened" reply must cast the new instance to OAPI_MSGTYPE, NOT
    // int -- an (int) cast truncates the 64-bit pointer and crashes on open.
    static OAPI_MSGTYPE MsgProc(UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam);

private:
    bool HandleKey(DWORD key);
    static void FormatClock(char* out, double t);

    VESSEL*     v_;          // focus vessel (unused in Phase 1; Phase 2 reads it)
    std::string scratch_;    // the scratchpad line -- what you are typing
    std::string echo_;       // last entry terminated with EXEC (no-op in Phase 1)
    std::string annun_;      // last annunciator/function key pressed
};

// The DPS keypad, mapped onto the PC keyboard. Command keys double as MFD
// buttons (see ButtonMenu). Digits / sign / decimal are typed directly.
//   OPS=O  SPEC=S  ITEM=I  EXEC=E/Enter  RESUME=R  CLEAR=C
//   FAULT SUMM=F  SYS SUMM=Y  MSG RESET=M  ACK=A  GPC/CRT=G  backspace=Del key
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

OrionDPS::OrionDPS(DWORD w, DWORD h, VESSEL* v) : MFD2(w, h, v), v_(v) {
    echo_  = "--";
    annun_ = "--";
}

OAPI_MSGTYPE OrionDPS::MsgProc(UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam) {
    if (msg == OAPI_MSG_MFD_OPENED)
        return (OAPI_MSGTYPE)(new OrionDPS(LOWORD(wparam), HIWORD(wparam), (VESSEL*)lparam));
    return 0;
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
// All key handling funnels through here so the physical keyboard and the MFD
// buttons behave identically. Returns true if the key was consumed.
//
// PHASE 1: the scratchpad is the whole behaviour. OPS/SPEC/ITEM begin an entry;
// digits/sign/decimal append; EXEC terminates it (into the echo line, doing
// NOTHING else on purpose); CLEAR/backspace edit it. The annunciator keys just
// record that they were pressed so you can see them register.
bool OrionDPS::HandleKey(DWORD key) {
    switch (key) {
    // ---- digits (main row + numpad) ----
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

    // ---- sign / decimal ----
    case OAPI_KEY_EQUALS: case OAPI_KEY_ADD:      scratch_ += '+'; return true;
    case OAPI_KEY_MINUS:  case OAPI_KEY_SUBTRACT: scratch_ += '-'; return true;
    case OAPI_KEY_PERIOD: case OAPI_KEY_DECIMAL:  scratch_ += '.'; return true;

    // ---- scratchpad edit ----
    case OAPI_KEY_BACK:
        if (!scratch_.empty()) scratch_.pop_back();
        return true;
    case OAPI_KEY_C:                                 // CLEAR
        scratch_.clear();
        return true;

    // ---- entry starters ----
    case OAPI_KEY_O: scratch_ = "OPS ";  return true;
    case OAPI_KEY_S: scratch_ = "SPEC "; return true;
    case OAPI_KEY_I: scratch_ = "ITEM "; return true;

    // ---- EXEC: terminate the entry (Phase 1 = NO action) ----
    case OAPI_KEY_E: case OAPI_KEY_RETURN: case OAPI_KEY_NUMPADENTER:
        if (!scratch_.empty()) {
            echo_ = scratch_ + "  (phase 1: no action)";
            scratch_.clear();
        }
        return true;

    // ---- RESUME: back out (Phase 2 returns to the OPS display) ----
    case OAPI_KEY_R:
        scratch_.clear();
        annun_ = "RESUME";
        return true;

    // ---- annunciators: recorded only, no action in Phase 1 ----
    case OAPI_KEY_G: annun_ = "GPC/CRT";    return true;
    case OAPI_KEY_M: annun_ = "MSG RESET";  return true;
    case OAPI_KEY_F: annun_ = "FAULT SUMM"; return true;
    case OAPI_KEY_Y: annun_ = "SYS SUMM";   return true;
    case OAPI_KEY_A: annun_ = "ACK";        return true;

    default:
        return false;
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
bool OrionDPS::Update(oapi::Sketchpad* skp) {
    const int w = (int)W, h = (int)H;
    int dy = h / 24; if (dy < 11) dy = 11;
    const int x = 6;
    int y = dy;
    char buf[96];

    skp->SetFont(GetDefaultFont(0));   // fixed-pitch Courier -- the MEDS look

    auto line = [&](const char* s, DWORD col) {
        skp->SetTextColor(col);
        skp->Text(x, y, s, (int)strlen(s));
        y += dy;
    };

    const DWORD GRN = 0x0000FF00, CYA = 0x00FFFF00, WHT = 0x00FFFFFF,
                AMB = 0x0000BEFF, DIM = 0x00558855;

    // ---- GPC-style header ----
    char clk[24]; FormatClock(clk, oapiGetSimTime());
    sprintf(buf, "GPC1  OPS 000  MM 000    CRT 1"); line(buf, CYA);
    sprintf(buf, "0001/000/  ORION DPS  %s", clk);  line(buf, CYA);
    line("--------------------------------", DIM);

    // ---- body: phase-1 placeholder + live key echo ----
    line("DATA PROCESSING SYSTEM", GRN);
    line("PHASE 1  DISPLAY + KEYBOARD", GRN);
    y += dy / 2;
    line("TYPE:  OPS / SPEC / ITEM", WHT);
    line("       0-9  + - .  EXEC", WHT);
    line("       CLEAR  RESUME", WHT);
    y += dy / 2;
    sprintf(buf, "KEY   %s", annun_.c_str()); line(buf, AMB);
    sprintf(buf, "LAST  %s", echo_.c_str());  line(buf, DIM);

    // ---- scratchpad line, anchored to the bottom ----
    y = h - dy * 2;
    line("--------------------------------", DIM);
    std::string sp = "> " + scratch_ + "_";
    skp->SetTextColor(GRN);
    skp->Text(x, y, sp.c_str(), (int)sp.size());

    return true;
}

// ---------------------------------------------------------------------------
// Module entry points. g_dpsMode holds only the registration handle (assigned
// once at load); there is no mutable global sim state.
DLLCLBK void InitModule(HINSTANCE hDLL) {
    static char name[] = "Orion DPS";
    MFDMODESPECEX spec;
    spec.name    = name;
    spec.key     = OAPI_KEY_D;         // SEL-menu shortcut for this mode
    spec.context = nullptr;
    spec.msgproc = OrionDPS::MsgProc;
    g_dpsMode = oapiRegisterMFDMode(spec);
}
DLLCLBK void ExitModule(HINSTANCE hDLL) {
    oapiUnregisterMFDMode(g_dpsMode);
}
