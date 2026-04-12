@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=amd64
if errorlevel 1 exit /b %errorlevel%
cl /nologo /LD /O2 /EHsc /std:c++17 /openmp /Fo"C:\Users\leifh\Documents\UofA\Senior_Design\Software\Github\ENGR498-GUI-Project-clean\ENGR-498-Project\wire_extraction\native\build\wire_accel.obj" "C:\Users\leifh\Documents\UofA\Senior_Design\Software\Github\ENGR498-GUI-Project-clean\ENGR-498-Project\wire_extraction\native\wire_accel.cpp" /link /OUT:"C:\Users\leifh\Documents\UofA\Senior_Design\Software\Github\ENGR498-GUI-Project-clean\ENGR-498-Project\wire_extraction\native\build\wire_accel.dll"
