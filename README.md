## Firmware for SX1276 Development Kit
This repository contains the firmware for the SX1276 Development kit available from Modtronix.com. This development kit consists out of two parts, the master and slave. The master is the larger one with OLED displa. To use the development kit, turn both master and slave on via button on the side of the device. The OLED display on the master device will now display the status. The bottom line will read "Mode: stopped". By pressing the * button, this will change to "Mode: running", and the master will start sending packets every couple of seconds. If the slave device is turned on, it will receive these packets, and reply to the master. On receiving a successful reply from the slave, the Master will display the RSSI(receive signal strength) of the local(master) and remote(slave) device. For example:
L=30, R=28
For this example, it will indicate that the RSSI of the packet received by the master(L for local) was -30 RSSI, and for the slave(R for remote) -28 dBi.

## Building project
This repository contains the firmware for the SX1276 Development kit available from Modtronix.com. It can be built with the free coIDE from http://www.coocox.org
This development kit consists out of two parts, the master and slave.
To build the firmware for the master, ensure that the following is defined on top of the main.c file:

```c
bool isMaster = 1;
//#define DISABLE_OLED
```
To build the firmware for the slave, ensure that the following is defined on top of the main.c file:

```c
bool isMaster = 0;
#define DISABLE_OLED	//Slave does NOT have an OLED display
```

In addition, the following define has to be configured for 433, 868 or 915MHz tests:

```c
#define RF_FREQUENCY 	916700000 // 916.7 kHz
```
For this development kit, 916.7Mhz is used to for 915MHz test, 868.7Mhz for 868Mhz test, and 433.7Mhz for 433Mhz test. 

After building the project, the *.elf, *.bin and *.hex file will be created in the debug folder.


## Upgrading Firmware
This development kit consists out of two parts, the master and slave. The master should be programmed with the **master**.hex file, and slave with the **slave**.hex file. Pre built HEX and DFU files are located in the "dist" folder. The firmware is upgraded via the USB port as follows:

1) Download DFU app (STSW-STM32080). Use v3.0.4 or later! Search for "STSW-STM32080". At time of writing
this, it was located here:
http://www.st.com/web/en/catalog/tools/FM147/CL1794/SC961/SS1533/PF257916

2) Start "DfuSe Demonstaration" application.

3) Enter bootloader mode on NZ32-ST1L. To do this, connect to PC via USB, press and hold "BOOT" button,
   toggle "RESET" button, release "BOOT" button. The "DfuSeDemo" application should now show a device in
   "Available DFU Devices" box.
  
4) Click "Choose" button, and select *.dfu file. Do NOT use th "Choose" button in "Upload Action" section!

5) Click "Upgrade" button, and upgrade firmware. Do NOT use th "Upload" button in "Upload Action" section, this
   is to download the current firmware on the device!  

-- 
Regards, David

=========================================
Modtronix Engineering
Sydney, Australia
Tel: +61 (0)405 353466
Fax: +61 (0)2 82094824
Email: david05@modtronix.com
Web: www.modtronix.com
ABN: 48663118043
