#!/usr/bin/env python3
"""
panel_gen.py -- ORION 2D panel generator.

THE IDEA
--------
An Orbiter 2D panel is just a BITMAP plus a list of rectangles that are
clickable. The classic bug in every panel project is that the artwork and the
hit-boxes drift apart: you nudge a switch 4 px in Photoshop and now clicking it
does nothing, and you have no idea why.

So we don't hand-author either one. We declare the panel ONCE, as data, and
generate BOTH from it:

    PANEL (this file)  ---> orion_panel.png      (the artwork Orbiter displays)
                       ---> PanelAreas.h         (the C++ RegisterPanelArea IDs)

They can never disagree, because they come from the same list.

AESTHETIC
---------
Deliberately Fasttracker II / DOS-era: hard-edged beveled boxes, a tight
palette, bitmap-ish type, dense readouts. This look is CHEAPER to produce than
photoreal Apollo panels -- a bevel is two 1px lines -- and it reads perfectly at
low resolution. Change PALETTE below to reskin the entire ship in one edit.
"""
from PIL import Image, ImageDraw, ImageFont
import json

W, H = 1280, 720

# ---------------------------------------------------------------- palette
# Swap this block to reskin everything. (Amber CRT variant included below.)
PALETTE = {
    "bg":        (12, 20, 24),     # deep slate
    "panel":     (28, 52, 58),     # panel face
    "bevel_hi":  (86, 148, 152),   # top-left highlight
    "bevel_lo":  (8, 24, 28),      # bottom-right shadow
    "text":      (168, 224, 224),
    "text_dim":  (84, 130, 136),
    "accent":    (255, 190, 60),   # amber
    "good":      (90, 220, 130),
    "warn":      (255, 176, 40),
    "bad":       (255, 78, 66),
    "screen":    (6, 14, 14),      # readout wells
}
AMBER = {  # alternative skin: phosphor terminal
    "bg": (10, 8, 2), "panel": (34, 24, 4), "bevel_hi": (150, 110, 20),
    "bevel_lo": (16, 12, 2), "text": (255, 190, 60), "text_dim": (140, 100, 30),
    "accent": (255, 220, 130), "good": (200, 255, 120), "warn": (255, 180, 40),
    "bad": (255, 90, 50), "screen": (18, 12, 2),
}

P = PALETTE

def font(sz, bold=False):
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
              if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",):
        try: return ImageFont.truetype(p, sz)
        except: pass
    return ImageFont.load_default()

F9, F11, F13, F16 = font(9, True), font(11, True), font(13, True), font(16, True)

def bevel(d, box, raised=True, fill=None):
    """A 1-px bevel. This single function IS the retro aesthetic."""
    x0, y0, x1, y1 = box
    hi, lo = (P["bevel_hi"], P["bevel_lo"]) if raised else (P["bevel_lo"], P["bevel_hi"])
    if fill: d.rectangle(box, fill=fill)
    d.line([(x0, y0), (x1, y0)], fill=hi)      # top
    d.line([(x0, y0), (x0, y1)], fill=hi)      # left
    d.line([(x0, y1), (x1, y1)], fill=lo)      # bottom
    d.line([(x1, y0), (x1, y1)], fill=lo)      # right

# ---------------------------------------------------------------- the panel
# EVERY control is declared here. Nothing is drawn that isn't in this list.
AREAS = []   # -> becomes PanelAreas.h

def group(d, x, y, w, h, title):
    bevel(d, (x, y, x + w, y + h), raised=True, fill=P["panel"])
    d.rectangle((x + 1, y + 1, x + w - 1, y + 14), fill=P["bevel_lo"])
    d.text((x + 6, y + 3), title, font=F9, fill=P["accent"])

def button(d, aid, x, y, w, h, label, state="off"):
    col = {"off": P["panel"], "on": (30, 90, 60), "warn": (110, 70, 10),
           "bad": (110, 30, 26)}[state]
    bevel(d, (x, y, x + w, y + h), raised=True, fill=col)
    tc = {"off": P["text_dim"], "on": P["good"], "warn": P["warn"], "bad": P["bad"]}[state]
    tw = d.textlength(label, font=F9)
    d.text((x + (w - tw) / 2, y + (h - 9) / 2), label, font=F9, fill=tc)
    AREAS.append(dict(id=aid, x=x, y=y, w=w, h=h, kind="button"))

def readout(d, aid, x, y, w, label, value, col=None):
    """A dark 'screen' well with a live value. Orbiter redraws only the well."""
    d.text((x, y), label, font=F9, fill=P["text_dim"])
    bevel(d, (x + 74, y - 2, x + w, y + 12), raised=False, fill=P["screen"])
    d.text((x + 78, y), value, font=F9, fill=col or P["text"])
    AREAS.append(dict(id=aid, x=x + 74, y=y - 2, w=w - 74, h=14, kind="readout"))

def bargraph(d, aid, x, y, w, h, frac, col):
    bevel(d, (x, y, x + w, y + h), raised=False, fill=P["screen"])
    n = int((w - 4) / 5)
    lit = int(n * frac)
    for i in range(n):
        c = col if i < lit else (P["bevel_lo"])
        d.rectangle((x + 2 + i * 5, y + 2, x + 5 + i * 5, y + h - 2), fill=c)
    AREAS.append(dict(id=aid, x=x, y=y, w=w, h=h, kind="bar"))

img = Image.new("RGB", (W, H), P["bg"])
d = ImageDraw.Draw(img)

# ---- title bar
bevel(d, (0, 0, W - 1, 26), raised=True, fill=P["panel"])
d.text((10, 6), "ORION  \u2500  ENGINEERING", font=F13, fill=P["accent"])
d.text((W - 260, 8), "MET 000/00:14:22", font=F11, fill=P["good"])

# ================= REACTOR A =================
group(d, 12, 36, 400, 250, "FUSION REACTOR A")
readout(d, "RA_STAGE", 24, 60, 380, "STAGE",    "IGNITE",     P["warn"])
readout(d, "RA_PRESS", 24, 80, 380, "CHAMBER",  "8.4e-4 PA",  P["good"])
readout(d, "RA_COILI", 24, 100, 380, "COIL I",  "50.0 KA")
readout(d, "RA_BFLD",  24, 120, 380, "B FIELD", "8.00 T",     P["good"])
readout(d, "RA_COILT", 24, 140, 380, "COIL T",  "20.4 K",     P["good"])
readout(d, "RA_TION",  24, 160, 380, "ION TEMP","12.3 KEV",   P["good"])
readout(d, "RA_DENS",  24, 180, 380, "DENSITY", "1.14 e20")
readout(d, "RA_Q",     24, 200, 380, "FUSION Q","4.2",        P["warn"])
d.text((24, 222), "IGNITION", font=F9, fill=P["text_dim"])
bargraph(d, "RA_PROG", 98, 220, 306, 12, 0.70, P["warn"])
button(d, "RA_START", 24, 244, 90, 26, "START A", "warn")
button(d, "RA_SCRAM", 122, 244, 90, 26, "SCRAM", "bad")
button(d, "RA_GEN",   220, 244, 90, 26, "GEN A", "off")

# ================= REACTOR B =================
group(d, 424, 36, 400, 250, "FUSION REACTOR B")
readout(d, "RB_STAGE", 436, 60, 380, "STAGE",   "STANDBY",   P["text_dim"])
readout(d, "RB_PRESS", 436, 80, 380, "CHAMBER", "1.0e+5 PA", P["bad"])
readout(d, "RB_COILI", 436, 100, 380, "COIL I", "0.0 KA")
readout(d, "RB_BFLD",  436, 120, 380, "B FIELD","0.00 T")
readout(d, "RB_COILT", 436, 140, 380, "COIL T", "20.0 K",    P["good"])
readout(d, "RB_TION",  436, 160, 380, "ION TEMP","0.0 KEV")
readout(d, "RB_DENS",  436, 180, 380, "DENSITY","0.00 e20")
readout(d, "RB_Q",     436, 200, 380, "FUSION Q","0.0")
d.text((436, 222), "IGNITION", font=F9, fill=P["text_dim"])
bargraph(d, "RB_PROG", 510, 220, 306, 12, 0.0, P["warn"])
button(d, "RB_START", 436, 244, 90, 26, "START B", "off")
button(d, "RB_SCRAM", 534, 244, 90, 26, "SCRAM", "bad")
button(d, "RB_GEN",   632, 244, 90, 26, "GEN B", "off")

# ================= EPS =================
group(d, 836, 36, 432, 250, "ELECTRICAL POWER")
readout(d, "E_VDC",  848, 60, 412, "DC BUS",   "842 V",   P["warn"])
readout(d, "E_IDC",  848, 80, 412, "CURRENT",  "9,412 A", P["warn"])
readout(d, "E_AC",   848, 100, 412, "AC BUS",  "115V/400HZ", P["good"])
readout(d, "E_GEN",  848, 120, 412, "GEN",     "0.0 MW",  P["bad"])
readout(d, "E_LOAD", 848, 140, 412, "LOAD",    "10.6 MW")
readout(d, "E_NET",  848, 160, 412, "NET",     "-10.6 MW", P["bad"])
for i, (bid, nm, f) in enumerate([("BK_A", "BANK A", 0.58), ("BK_B", "BANK B", 0.58),
                                  ("BK_C", "BANK C", 0.58)]):
    d.text((848, 182 + i * 18), nm, font=F9, fill=P["text_dim"])
    bargraph(d, bid + "_BAR", 912, 180 + i * 18, 348, 12, f, P["warn"])
button(d, "BK_ARM_A", 848, 244, 74, 26, "BANK A", "on")
button(d, "BK_ARM_B", 930, 244, 74, 26, "BANK B", "on")
button(d, "BK_ARM_C", 1012, 244, 74, 26, "BANK C", "on")
button(d, "E_INV1",   1094, 244, 74, 26, "INV 1", "on")
button(d, "E_ESS",    1176, 244, 84, 26, "ESS FEED", "off")

# ================= ECS =================
group(d, 12, 298, 400, 160, "THERMAL / COOLANT")
readout(d, "C_TCOOL", 24, 322, 380, "COOLANT", "412 K", P["good"])
readout(d, "C_TRAD",  24, 342, 380, "RADIATOR","388 K", P["good"])
readout(d, "C_FLOW",  24, 362, 380, "FLOW",    "140 KG/S", P["good"])
readout(d, "C_QIN",   24, 382, 380, "HEAT IN", "12.4 MW")
button(d, "C_PUMP1", 24, 408, 118, 30, "PUMP 1", "on")
button(d, "C_PUMP2", 150, 408, 118, 30, "PUMP 2", "off")
button(d, "C_XFEED", 276, 408, 128, 30, "X-FEED", "off")

# ================= TORCH + MIXING =================
group(d, 424, 298, 400, 160, "TORCH DRIVE  \u2500  MIXING VALVE")
readout(d, "T_MIX",  436, 322, 380, "MIXTURE R", "7.30",  P["accent"])
readout(d, "T_ISP",  436, 342, 380, "ISP",       "1.00 MS")
readout(d, "T_AMAX", 436, 362, 380, "A-MAX",     "0.480 G", P["good"])
readout(d, "T_TRAD", 436, 382, 380, "DRV RAD",   "1503 K", P["good"])
d.text((436, 406), "LEAN", font=F9, fill=P["text_dim"])
bargraph(d, "T_MIXBAR", 478, 404, 288, 14, 0.24, P["accent"])
d.text((772, 406), "RICH", font=F9, fill=P["text_dim"])
button(d, "T_MIX_DN", 436, 426, 60, 24, "\u25c4 R", "off")
button(d, "T_MIX_UP", 504, 426, 60, 24, "R \u25ba", "off")
button(d, "T_DRV",    572, 426, 110, 24, "DRIVE CTL", "off")
button(d, "T_ARM",    690, 426, 126, 24, "ARM TORCH", "warn")

# ================= IMU / GNC =================
group(d, 836, 298, 432, 160, "GUIDANCE  \u2500  IMU / GDC")
readout(d, "I_ALIGN", 848, 322, 412, "PLATFORM", "NOT ALIGNED", P["bad"])
readout(d, "I_DRIFT", 848, 342, 412, "DRIFT",    "0.00 ARCMIN")
readout(d, "I_MIDG",  848, 362, 412, "MID GIMBAL", "12.4 DEG", P["good"])
readout(d, "I_NEXT",  848, 382, 412, "NEXT P52", "05:44:12", P["text_dim"])
button(d, "I_P52",   848, 408, 100, 30, "P52", "warn")
button(d, "I_OPT",   956, 408, 118, 30, "OPT 3 REFS", "off")
button(d, "I_GDC",   1082, 408, 100, 30, "GDC ALIGN", "off")
button(d, "I_LOCK",  1190, 408, 70, 30, "LOCK", "off")

# ================= CAUTION & WARNING =================
group(d, 12, 470, 1256, 68, "CAUTION & WARNING")
warns = [("CW_NOCOOL", "NO COOLANT", "off"), ("CW_BATT", "BATT LOW", "warn"),
         ("CW_GIMBAL", "GIMBAL LOCK", "off"), ("CW_QUENCH", "COIL QUENCH", "off"),
         ("CW_DRVTEMP", "DRV OVERTEMP", "off"), ("CW_HELOW", "RCS HE LOW", "off"),
         ("CW_BUSTRIP", "BUS TRIP", "off"), ("CW_P52", "P52 DUE", "off"),
         ("CW_FUEL", "FUEL LOW", "off"), ("CW_SCRAM", "SCRAM", "bad")]
for i, (aid, lbl, st) in enumerate(warns):
    button(d, aid, 24 + i * 124, 496, 114, 32, lbl, st)

# ================= EVENT LOG =================
group(d, 12, 550, 1256, 158, "EVENT LOG")
bevel(d, (22, 572, 1258, 700), raised=False, fill=P["screen"])
log = [
    ("> RCT A: 8 T FIELD, FUEL INJECTION", P["text"]),
    ("> RCT A: PLASMA AT 10 KEV - IGNITING", P["warn"]),
    ("> WARNING: BATTERY 58% - START REACTOR", P["warn"]),
    ("> PUMP 1 ON - 140 KG/S", P["good"]),
    ("> BANK A/B/C ARMED", P["text_dim"]),
    ("> COLD AND DARK - BEGIN STARTUP", P["text_dim"]),
]
for i, (line, c) in enumerate(log):
    d.text((30, 580 + i * 19), line, font=F11, fill=c)

img.save("/home/claude/orion_panel.png")

# ---------------------------------------------------------------- codegen
with open("/home/claude/PanelAreas.h", "w") as f:
    f.write("// AUTO-GENERATED by panel_gen.py -- DO NOT EDIT BY HAND.\n")
    f.write("// Artwork and hit-boxes come from the same source, so they cannot drift.\n")
    f.write("#ifndef ORION_PANELAREAS_H\n#define ORION_PANELAREAS_H\n\n")
    f.write("enum PanelArea {\n")
    for i, a in enumerate(AREAS):
        f.write(f"    AID_{a['id']:<12} = {i},\n")
    f.write(f"    AID_COUNT = {len(AREAS)}\n}};\n\n")
    f.write("struct AreaDef { int id; int x, y, w, h; const char* kind; };\n")
    f.write(f"static const AreaDef PANEL_AREAS[{len(AREAS)}] = {{\n")
    for a in AREAS:
        f.write(f"    {{ AID_{a['id']}, {a['x']}, {a['y']}, {a['w']}, {a['h']}, \"{a['kind']}\" }},\n")
    f.write("};\n\n#endif\n")

print(f"orion_panel.png  ({W}x{H})")
print(f"PanelAreas.h     ({len(AREAS)} registered areas)")
print(f"  buttons : {sum(1 for a in AREAS if a['kind']=='button')}")
print(f"  readouts: {sum(1 for a in AREAS if a['kind']=='readout')}")
print(f"  bars    : {sum(1 for a in AREAS if a['kind']=='bar')}")
