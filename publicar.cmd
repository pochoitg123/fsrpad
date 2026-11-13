@echo off
setlocal
cd /d "%~dp0"
set "BIN=%~1"
if not defined BIN for /f "usebackq delims=" %%i in (`powershell -NoProfile -STA -Command "Add-Type -AssemblyName System.Windows.Forms; $d=New-Object System.Windows.Forms.OpenFileDialog; $d.Filter='Aplicacion FSRpad (*.ino.bin)|*.ino.bin'; if($d.ShowDialog() -eq 'OK'){$d.FileName}"`) do set "BIN=%%i"
if not defined BIN exit /b 1
set /p "VER=Numero PAD_BUILD usado al compilar: "
set "PAD_BUILD_INPUT=%VER%"
powershell -NoProfile -Command "if($env:PAD_BUILD_INPUT -notmatch '^[1-9][0-9]{0,8}$'){exit 1}"
if errorlevel 1 exit /b 1
where py >nul 2>nul
if errorlevel 1 (set "PY=python") else (set "PY=py -3")
%PY% tools\preparar_version.py "%BIN%" %VER% --out ota
if errorlevel 1 (pause & exit /b 1)
echo Lista la carpeta ota para subir a GitHub.
pause
