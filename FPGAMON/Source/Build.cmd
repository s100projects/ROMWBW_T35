@echo off
setlocal

set TOOLS=..\Tools
set PATH=%TOOLS%\zxcc;%TOOLS%\srecord;%PATH%
set CPMDIR80=%TOOLS%/cpm/

srec_cat -generate 0x0 0x4000 --constant 0x00 -o fpgamon.hex -Intel

call :page fpgamona -0xF000 || exit /b	:: 0xF000 - 0x0000 = -0xF000
call :page fpgamonb -0xE000 || exit /b	:: 0xF000 - 0x1000 = -0xE000
call :page fpgamonc -0xD000 || exit /b	:: 0xF000 - 0x2000 = -0xD000
call :page fpgamond -0xC000 || exit /b	:: 0xF000 - 0x3000 = -0xC000

srec_cat fpgamon.hex -Intel -o fpgamon.bin -binary

:: bash -c -- "hexdump -v -e '/1 \""%%02X\\n\""' fpgamon.bin" >fpgamon-old.inithex
PowerShell -Command "[System.IO.File]::ReadAllBytes(\"fpgamon.bin\") | foreach {$_.ToString(\"X2\")}" >fpgamon.inithex

copy fpgamon.inithex ..\..\T35SBCextSD_16\

goto :eof

:page

zxcc z80asm -%1 -FH || exit /b
srec_cat ( fpgamon.hex -Intel -exclude -within ( %1.hex -Intel -offset %2 ) ) ( %1.hex -Intel -offset %2 ) -o fpgamon.hex -Intel

goto :eof
