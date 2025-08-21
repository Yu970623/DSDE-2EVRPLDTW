@echo off

SETLOCAL ENABLEDELAYEDEXPANSION

for /f "delims=" %%a in ('dir /b^|findstr "noDSDE"') do (

set name=%%a

set name=!name:noDSDE=nonDSDE!

ren "%%a" "!name!"

)

exit