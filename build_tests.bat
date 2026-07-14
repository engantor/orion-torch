@echo off
REM ===========================================================================
REM  Build and run the HEADLESS physics tests. No Orbiter needed.
REM  Run this from:  Start > "x86 Native Tools Command Prompt for VS 2022"
REM  (or "Developer Command Prompt for VS 2022")
REM
REM  This is your safety net. If these pass, the physics is correct and any
REM  later problem is an Orbiter integration problem -- which is a much
REM  smaller haystack to search.
REM ===========================================================================
echo.
echo === Building headless core tests ===
cl /nologo /EHsc /O2 /std:c++17 ShipCore.cpp Avionics.cpp test_core.cpp /Fe:test_core.exe
if errorlevel 1 goto :fail
echo.
echo === Building avionics tests ===
cl /nologo /EHsc /O2 /std:c++17 Avionics.cpp test_avionics.cpp /Fe:test_avionics.exe
if errorlevel 1 goto :fail
echo.
echo ============================ CORE TESTS ============================
test_core.exe
echo.
echo ========================= AVIONICS TESTS ==========================
test_avionics.exe
echo.
echo Done. Every RESULT line should say PASS.
goto :eof

:fail
echo.
echo BUILD FAILED. Are you in the "x86 Native Tools Command Prompt for VS 2022"?
exit /b 1
