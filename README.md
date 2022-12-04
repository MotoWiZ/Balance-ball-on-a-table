The aim of this project is to balance a ball on a plane surface (table).

Hardware:
- 1 unit - Raspberry Pi 4 Model B Rev 1.4
  - CPU: ARMv7 Processor rev 3 (v7l)
  - Memory: 8 Gb
  - Disk: Sandisk EVO MicroSD 32 Gb Class 10
- 1 unit - Raspberry Pi Camera Module 2 V2.1
- 1 unit - Flat cable to connect the camera
- 1 unit - Power source USB-C
- 1 unit - Case with fan (active cooling)
- 1 set - Adhesive Raspberri Coolers
- 2 units - Servos Futaba S3003
- 1 Small breadboard
- Some dupont extension cables (male and female)


Software:
- Raspberry Pi OS
  - Raspbian ver. 11 (bullseye)
  - 32 bit version
  - kernel ver: 5.15
- Rtems on top of Raspbian followed this toturial with modifications https://mritunjaysharma394.medium.com/installing-rtems-ecosystem-and-building-your-first-bsp-993d1cf38902). This is not needed but I want to play a bit with RTOS, priorities and want to try RTMES kernel for this
- Python language