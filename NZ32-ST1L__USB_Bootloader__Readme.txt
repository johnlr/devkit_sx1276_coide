To program the board with DFU programmer, do following:

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