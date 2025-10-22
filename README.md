# T35 Z80 Firmware for RomWBW

This repository hosts the files required to build firmware for the S100 Computers
Z80 FPGA SBC with a T35 Module.  The resulting firmware can be used to load
RomWBW from a disk image via CF Card or SD Card.

## Building the Firmware

The T35SBCextSD_16 directory contains an Efinity project file set.  Use Efinity to
build the project.  After a successful build, the outflow directory will contain
both T35SBCextSD_16.bit and T35SBCextSD_16.hex files for programming on a T35 FPGA.

The Efinity project already includes the Z80 monitor program.  If you want to modify the
monitor, you will find the source in the FPGAMON directory.  This directory contains
the files required to build the monitor binary file which is called fpgamon.inithex.
To buld the monitor binary, use Windows to run the Build.cmd batch file in the
Source directory of FPGAMON.  The build process will automatically copy the
resulting fpgamon.inithex file into the T35SBCextSD_16.  You will need to run a
new Efinity build of T35SBCextSD_16 to embed the updated monitor binary in the
firmware.  You do **not** need to build the monitor unless you want to make
changes to it.

## Installing & Running RomWBW

In summary, you will need to do two things:

1. Program your T35 with the firmware built from this repository.
2. Write a RomWBW disk image to either an SD or CF Card.

After building the T35 firmware as described above, you will need to use the Efinity
Programmer to program the T35 module.  Use the programmer to write the T35SBCextSD_16.hex
file from the outflow directory.  The process of using Efinity to program your
T35 is described at <https://github.com/s100projects/T35_FPGA_MODULE>.

Once this first step is done, you should be able to boot your T35 FPGA board.
It will boot into a variant of John's monitor running at 9600 baud.  The version
number of the monitor should be v3.08a.  This updated monitor replaces the CP/M load
options with RomWBW load options (N & P).  Loading RomWBW from either SD Card or CF Card
is supported.  You can set the right-most dip switch as desired to boot from either
a Propeller console or the onboard USB serial port.

Now, download a RomWBW disk image from the [RomWBW Repository](https://github.com/wwarthen/RomWBW)
You will need to download RomWBW Version 3.5.0 or later from the [RomWBW Releases Page](https://github.com/wwarthen/RomWBW/releases).
Use the dropdown to access the assets of the release and download the package file which will be
called something similar to RomWBW-v3.5.0-Package.zip.  Within the Binary directory within
the .zip file, you will find a file called "SZ80_t35_hd1k_combo.img".  This is a binary disk image file
that must be written to either a CF or SD Card starting at the first sector.
You can do this with Win32DiskImager on Windows or dd on Linux.

Insert your CF or SD Card into the appropriate slot on your FPGA board.  Boot into John's monitor as before.
Choose either P or N option to load RomWBW from the IDE or SD interface as appropriate.  After it is loaded,
you will be instructed to enter "G0" to launch RomWBW.  Enter this command and RomWBW should start and output
to the same device as the monitor was using.

Below is a session log from boot through the launch of RomWBW.

```
FPGA SBC-Z80 FPGA-ROM MONITOR (@ F000H) V3.08a J.Monahan, 9/4/2024
SP=EFF0  IOBYTE= 11100110

->K

FPGA SBC-Z80 FPGA-ROM MONITOR (@ F000H) V3.08a J.Monahan, 9/4/2024
SP=E000  IOBYTE= 11101110

A=Memmap     D=Show RAM   E=Echo Text   F=Fill RAM
G=Goto       I=IDE Menu   J=Test RAM    K=Menu
L=RTC        M=Move RAM   QI,O=Port     N=Load RomWBW (SD)
R=Ports      S=Subs RAM   T=RAM ASCII   P=Load RomWBW (IDE)
V=Verify RAM X=XModem     Y=Swap RAM    U=SD Card
@=Flush Printer           Z=Top Of RAM


->P
Loading RomWBW.........
Done.  Use 'G0' to launch.
->G0


RomWBW HBIOS v3.5.0, 2025-04-04

S100 FPGA Z80 [SZ80_t35] Z80 @ 8.000MHz
0 MEM W/S, 1 I/O W/S, Z2 MMU
0KB ROM, 512KB RAM, HEAP=0x4CB4

SSER: IO=0x34
DS5RTC: IO=0x68 Tue 2025-04-04 11:30:13
MD: UNITS=1 RAMDISK=256KB
PPIDE: IO=0x30
PPIDE0: ATA LBA BLOCKS=0x003B8E00 SIZE=1905MB
PPIDE1: NO MEDIA
SD: MODE=T35 IO=0x6C DEVICES=2
SD0: SDHC NAME=SD08G BLOCKS=0x00ECE000 SIZE=7580MB
SD1: SDHC NAME=SP32G BLOCKS=0x03B72400 SIZE=30436MB
SCON: IO=0x00 80X40 TEXT (ANSI)

Unit        Device      Type              Capacity/Mode
----------  ----------  ----------------  --------------------
Char 0      SSER0:      RS-232            9600,8,N,1
Char 1      SCON0:      Terminal          Term Module,ANSI
Disk 0      MD0:        RAM Disk          256KB,LBA
Disk 1      PPIDE0:     CompactFlash      1905MB,LBA
Disk 2      PPIDE1:     Hard Disk         --
Disk 3      SD0:        SD Card           7580MB,LBA
Disk 4      SD1:        SD Card           30436MB,LBA


S100 FPGA Z80 [SZ80_fpga] Boot Loader

Boot [H=Help]:
```
