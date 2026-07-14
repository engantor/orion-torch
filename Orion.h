// ============================================================================
//  Orion.h -- declaration of the Orion VESSEL4 class.
//
//  Extracted from Orion.cpp so that a SEPARATE plugin DLL (the DPS MFD) can see
//  the class layout, cast the focus VESSEL* to Orion*, and read the systems
//  model through Core(). This is the boundary across which the MFD and the
//  vessel talk.
//
//  IMPORTANT: only NON-virtual, header-inline access (Core()) is used across the
//  DLL boundary, and only after verifying GetClassName()=="Orion". Both DLLs must
//  be built with matching settings (x64, /MD) so the layout is identical.
//
//  This header pulls in orbitersdk.h. A translation unit that wants the vessel
//  module boilerplate must #define ORBITER_MODULE *before* including this.
// ============================================================================
#ifndef ORION_ORION_H
#define ORION_ORION_H

#ifndef STRICT
#define STRICT
#endif
#include "orbitersdk.h"
#include "ShipCore.h"

class Orion : public VESSEL4 {
public:
    Orion(OBJHANDLE hVessel, int flightmodel);

    void clbkSetClassCaps(FILEHANDLE cfg) override;
    void clbkPostCreation() override;
    void clbkPreStep(double simt, double simdt, double mjd) override;
    void clbkSaveState(FILEHANDLE scn) override;
    void clbkLoadStateEx(FILEHANDLE scn, void* status) override;
    int  clbkConsumeBufferedKey(DWORD key, bool down, char* kstate) override;

    // MFDs and panels reach the systems model through this. Inline on purpose:
    // callers in other DLLs resolve it without linking Orion.cpp.
    orion::ShipCore& Core() { return core_; }

private:
    orion::ShipCore   core_;
    PROPELLANT_HANDLE ph_main_ = nullptr;
    PROPELLANT_HANDLE ph_rcs_  = nullptr;
    THRUSTER_HANDLE   th_main_ = nullptr;
    THRUSTER_HANDLE   th_rcs_[12] = {};
    double            carry_ = 0.0;   // leftover sim time for fixed-step integration
};

#endif
