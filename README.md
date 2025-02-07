![image](images/microchip.jpg) 
## Sensorless Field-Oriented Control of 3-phase Induction Motor using PLL estimator: MCHV-230VAC-1.5kW Development Board and dsPIC33CK256MP508 Motor Control DIM


## 1. INTRODUCTION
This document describes the setup requirements for driving a 3 Phase AC Induction Motor (ACIM) using Sensorless Field Oriented Control (FOC) and PLL Estimator on the hardware platform [EV78U65A](https://www.microchip.com/en-us/development-tool/ev78u65a) "MCHV-230VAC-1.5kW Development Board"  and [EV62P66A](https://www.microchip.com/en-us/development-tool/ev62p66a) "dsPIC33CK256MP508 Motor Control Dual In-line Module (DIM)".

For details about PLL estimator, refer to Microchip application note [AN1206](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/01206A.pdf) “Sensorless Field Oriented Control (FOC) of an AC Induction Motor (ACIM) Using Field Weakening”. 

Voltage feedback-based field weakening algorithm is implemented for better utilization of the DC link voltage. 
</br>

## 2. SUGGESTED DEMONSTRATION REQUIREMENTS

### 2.1 Motor Control Application Firmware Required for the Demonstration

To clone or download this application firmware on GitHub, 
- Navigate to the [main page of this repository](https://github.com/microchip-pic-avr-solutions/mchv230vac1.5kw-33ck256mp508-acim-an1206-foc-pll) and 
- On the tab **<> Code**, above the list of files in the right-hand corner, click Code, then from the menu, click **Download ZIP** or copy the repository URL to **clone.**
> **Note:** </br>
>In this document, hereinafter this firmware package is referred as **firmware.**
### 2.2 Software Tools Used for Testing the firmware

- MPLAB® X IDE **v6.20** 
- Device Family Pack (DFP): **dsPIC33CK-MP_DFP v1.13.366**
- Curiosity/Starter Kits Tool Pack : **PKOB4_TP v1.14.1168**
- MPLAB® XC-DSC Compiler **v3.10**
- MPLAB® X IDE Plugin: **X2C-Scope v1.6.6** 
> **Note:** </br>
>The software used for testing the firmware prior to release is listed above. It is recommended to use the version listed above or later versions for building the firmware. All previous versions of Device Family Packs (DFP) and Tool Packs can be downloaded from [Microchip Packs Repository.](https://packs.download.microchip.com/)
### 2.3 Hardware Tools Required for the Demonstration
- MCHV-230VAC-1.5kW Development Board  [(EV78U65A)](https://www.microchip.com/en-us/development-tool/ev78u65a) 
- dsPIC33CK256MP508 Motor Control Dual In-line Module [(EV62P66A)](https://www.microchip.com/en-us/development-tool/ev62p66a)
- 3-phase, 220VAC, 25 Watt 4-Pole Induction Motor - ZD 41K25A-S Motor [(AC300023)](https://www.microchip.com/en-us/development-tool/AC300023)

> **Note:** </br>
> All items listed under the section Hardware Tools Required for the Demonstration are available at [microchip DIRECT](https://www.microchipdirect.com/) 
</br>

## 3. HARDWARE SETUP
This section describes the hardware setup required for the demonstration.
Refer ["Motor Control High Voltage 230VAC-1.5kW Development Board User's Guide"](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/UserGuides/Motor-Control-High-Voltage-230VAC-1.5kW-Dev-Board-Users-Guide-DS70005576.pdf), before operating the unit.
> **Note:** </br>
>In this document, hereinafter the MCHV-230V AC-1.5kW Development Board is referred as **development board**.

1. Motor currents are amplified by the amplifiers internal to the dsPIC33CK256MP508 on the DIM. The firmware and DIM are configured to sample and convert internal amplifier outputs (**'internal op-amp configuration'**) by default to measure the motor currents needed to implement FOC. **Table-1** summarizes the jumper resistors(0R) on the DIM for **‘internal op-amp configuration’**.

     <p align="left" >
     <img  src="images/Table_internalopamp.png"width="800"/></p>

2. Ensure the development board is not powered and it is fully discharged. Verify the LEDs **LD1**(Green) and **LD4**(Red) on Power Factor Correction Board and **LD1**(Green) on Motor Control Inverter Board are not glowing.

     <p align="left" >
     <img  src="images/MCHVnotpowered.jpg" width="600"></p>

3. Remove the thumb screw and open the top cover of the enclosure. Insert the **dsPIC33CK256MP508 Motor Control DIM** into the DIM Interface **connector J2** on the development board. Make sure the DIM is placed correctly and oriented before going ahead. Close the top cover of the enclosure and secure it with the thumb screw.

     <p align="left" >
     <img  src="images/dimconnected.png" width="600"/></p>



4. Connect the 3-phase wires from the motor to **A**, **B**, and **C** (no specific order) of the connector J13(**MOTOR**) on the development board.

     <p align="left" >
      <img  src="images/motorconnection.png" width="600"/></p>

5. Power the PFC board from a controlled AC source by applying voltage of 230V<sub>ac rms</sub> through IEC connector **connector J1** provided on the PFC board.

      <p align="left">
      <img  src="images/mchv_input.png" width="600"/></p>
> **Note:** </br>
>The Development Board is designed to operate in the 90 to 230V<sub>ac rms</sub> voltage range with a maximum input current of 10A<sub>rms</sub>. In the Input AC voltage range of 90 to 150V<sub>ac rms</sub>, the maximum input power to the Development Board must be derated (<1500W) to maintain the input current through the socket to less than or equal to 10A<sub>rms</sub>.

6. The development board has an on-board programming tool called the Isolated PKoB4 Daughter Board. To use the on-board programmer, connect a micro-USB cable between the Host PC and connector J11(**PROGRAM**) on the development board.
      <p align="left">
      <img  src="images/program.png" width="400"/></p>
> **Note:** </br>
> Use only **shielded micro-USB** cables intended for data transfer.

 7. To establish serial communication with the host PC, connect a micro-USB cable between the host PC and connector J8(**USB-UART**) on the development board. 
      <p align="left">
      <img  src="images/usb_uart.png" width="400"/></p>

> **Note:** </br>
> Use only **shielded micro-USB** cables intended for data transfer.
 </br>

## 4. SOFTWARE SETUP AND RUN
### 4.1 Setup: MPLAB X IDE and MPLAB XC-DSC Compiler
Install **MPLAB X IDE** and **MPLAB XC-DSC Compiler** versions that support the device **dsPIC33CK256MP508** and **PKoBv4.** The MPLAB X IDE, MPLAB XC-DSC Compiler, and X2C-Scope plug-in used for testing the firmware are mentioned in the [Motor Control Application Firmware Required for the Demonstration](#21-motor-control-application-firmware-required-for-the-demonstration) section. 

To get help on  
- MPLAB X IDE installation, refer [link](https://microchipdeveloper.com/mplabx:installation)
- MPLAB XC-DSC Compiler installation steps, refer [link](https://developerhelp.microchip.com/xwiki/bin/view/software-tools/xc-dsc/install/)

If MPLAB IDE v8 or earlier is already installed on your computer, then run the MPLAB driver switcher (Installed when MPLAB®X IDE is installed) to switch from MPLAB IDE v8 drivers to MPLAB X IDE drivers. If you have Windows 8 or 10, you must run the MPLAB driver switcher in **Administrator Mode**. To run the Device Driver Switcher GUI application as administrator, right-click on the executable (or desktop icon) and select **Run as Administrator**. For more details, refer to the MPLAB X IDE help topic **“Before You Begin: Install the USB Device Drivers (For Hardware Tools): USB Driver Installation for Windows Operating Systems.”**

### 4.2 Setup: X2C-SCOPE
X2C-Scope is a MPLAB X IDE plugin that allows developers to interact with an application while it runs. X2C-Scope enables you to read, write, and plot global variables (for motor control) in real-time. It communicates with the target using the UART. To use X2C-Scope, the plugin must be installed. To set up and use X2C-Scope, refer to the instructions provided on the [web page](https://x2cscope.github.io/docs/MPLABX_Plugin.html).

## 5.  BASIC DEMONSTRATION
### 5.1 Firmware Description
The firmware version needed for the demonstration is mentioned in the section [Motor Control Application Firmware Required for the Demonstration](#21-motor-control-application-firmware-required-for-the-demonstration) section. This firmware is implemented to work on Microchip’s Digital signal controller (dsPIC® DSC) **dsPIC33CK256MP508**. For more information, see the **dsPIC33CK256MP508 Family datasheet [(DS70005349)](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/DataSheets/dsPIC33CK256MP508-Family-Data-Sheet-DS70005349.pdf)**.

The Motor Control Demo application uses a push button to start or stop the motor and a potentiometer to vary the speed of the motor. This Motor Control Demo Application configures and uses peripherals like PWM, ADC, UART, etc. For more details, refer to Microchip Application note **[AN1206](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/01206A.pdf), “Sensorless Field Oriented Control (FOC) of an AC Induction Motor (ACIM) Using Field Weakening”** available on the [Microchip website](https://www.microchip.com/en-us/application-notes).

> **Note:**</br>
> The project may not build correctly in Windows OS if the Maximum path length of any source file in the project is more than 260 characters. In case the absolute path exceeds or nears the maximum length, do any (or both) of the following:
> - Shorten the directory name containing the firmware used in this demonstration. If you renamed the directory, consider the new name while reading the instructions provided in the upcoming sections of the document.
> - Place firmware in a location such that the total path length of each file included in the projects does not exceed the Maximum Path length specified. </br>
> Refer to MPLAB X IDE help topic **“Path, File, and Folder Name Restrictions”** for details. 

### 5.2 Basic Demonstration
Follow the below instructions, step by step, to set up and run the motor control demo application:

1. Start **MPLAB X IDE** and open the project **acim_mchv.X (File > Open Project)** with device selection **dsPIC33CK256MP508.**  
    <p align="left">
       <img  src="images/deviceidselect.png" width="800"/></p>
  

2. Set the project **acim_mchv.X** as the main project by right-clicking on the project name and selecting **Set as Main Project** as shown. The project **acim_mchv.X** will then appear in **bold.**
    <p align="left">
     <img  src="images/ideprojectsetup.png" width="400"/></p> 

	
3. Open <code>**userparms.h** </code> (**acim_mchv.X > Header Files**) in the project **acim_mchv.X.**  
     - Ensure that the macros <code>**OPEN_LOOP_FUNCTIONING**</code>, and <code>**TORQUE_MODE**</code> are not defined and the macro <code>**FDWEAK_CONTROL_ENABLE**</code> is defined in the header file <code>**userparms.h**</code>.
          <p align="left"><img  src="images/macro_definition.png" width="260"/ ></p>

     - By default the macro <code>**FD_WEAK_TYPE_VOLTAGE_FB**</code> is defined. 
     The <code>**FD_WEAK_TYPE_VOLTAGE_FB**</code> macro is used to select the field weakening control method.
     Define the <code>**FD_WEAK_TYPE_VOLTAGE_FB**</code> macro for voltage feedback based field weakening control. Undefine the macro for conventional inverse-speed based field weakening control
          <p align="left"> <img  src="images/def_fdweak_volt_fb.png" width="260"/></p>

     - Ensure that the macro <code>**INTERNAL_OPAMP_CONFIG**</code> is defined in <code>**userparms.h.**</code>
          <p align="left"> <img  src="images/internalopampconfig.png"></p>

     - Firmware is configured to run with ZD 41K25A-S Motor [(AC300023)](https://www.microchip.com/en-us/development-tool/AC300023) by default.  The macro <code>**ZD_41K25A_MOTOR**</code> should be defined in <code>**userparms.h**</code>.

          <p align="left"> <img  src="images/zd_def.png" width="300"/></p>

> **Note:** </br>
> The firmware also includes the configuration for running LEESON 0.33 HP 3-phase AC Induction Motor(Model: C4T34FB5A). Define the macro <code>**LEESON_MOTOR**</code> and undefine the macro <code>**ZD_41K25A_MOTOR**</code> in <code>**userparms.h**</code> to use the firmware for running LEESON Motor.
          <p align="left"> <img  src="images/leeson_def.png" width="240"/></p>




4. Right-click on the project **acim_mchv.X** and select **Properties** to open its **Project Properties** Dialog. Click the **Conf:[MCHV]** category to reveal the general project configuration information. The development tools used for testing the firmware are listed in section [2.2 Software Tools Used for Testing the firmware.](#22-software-tools-used-for-testing-the-firmware)

     In the **Conf:[MCHV]** category window: 
     - Ensure the selected **Device** is **dsPIC33CK256MP508.**
     - Select the **Connected Hardware Tool** to be used for programming and debugging. 
     - Select the specific Device Family Pack (DFP) from the available list of **Packs.** In this case, **dsPIC33CK-MP_DFP v1.13.366** is selected. 
     - Select the specific **Compiler Toolchain** from the available list of **XC-DSC** compilers. 
     In this case, **XC-DSC(v3.10)** is selected.
     - After selecting Hardware Tool and Compiler Toolchain, Device Pack, click the button **Apply**

     Please ensure that the selected MPLAB® XC-DSC Compiler and Device Pack support the device configured in the firmware

     <p align="left">
     <img  src="images/projectpropertiessettings.png" width="900"/></p>

5. Ensure that the checkbox **Load symbols when programming or building for production (slows process)** is checked under the **Loading** category of the **Project Properties** window.           
      <p align="left">
      <img  src="images/loadvariables.png" width="900"/></p>

     Also, go to **Tools > Options** , and

      <p align="left">
      <img  src="images/tools_options2.png" width="250"/></p>
      
    Open the  **Embedded > Generic Settings** tab and ensure that the **ELF debug session symbol load methodology (MIPS/ARM)** is selected as **Pre-processed (Legacy)** from the drop down.
           
      <p align="left">
      <img  src="images/embedded_legacy.png" width="900"/></p>
6. To build the project (in this case, **acim_mchv.X**) and program the device dsPIC33CK256MP508, click **Make and Program Device Main project** on the toolbar
    <p align="left">
    <img  src="images/deviceprogramming.png" width="700"/></p>
  
7. If the device is successfully programmed, **LED1** will be turned **ON**, indicating that the dsPIC® DSC is enabled.
    <p align="left">
     <img  src="images/LED_1.png" width="350"/></p>


8. Run or stop the motor by pressing the push button **BUTTON 1**. The motor should start spinning smoothly in one direction. Ensure that the motor is spinning smoothly without any vibration. The LED **LED2** is turned **ON** to show the button is pressed to start the motor.
     <p align="left">
     <img  src="images/pushbutton1.png" width="880"/></p>
 

9. The motor speed can be varied using the potentiometer **(POT).**
    <p align="left">
    <img  src="images/potentiometer.png"  width="500"/></p>
 

11. Press the push button **BUTTON 1** to stop the motor.


>**Note:**</br>
>The macros <code>MINIMUM_SPEED_RPM</code>, <code>NOMINAL_SPEED_RPM</code>, and <code>MAXIMUM_SPEED_RPM</code> are specified in the header file <code>**userparms.h**</code> included in the project **acim_mchv.X.** The macros <code>NOMINAL_SPEED_RPM</code> and <code>MAXIMUM_SPEED_RPM</code> are defined as per the Motor manufacturer’s specifications. Exceeding manufacture specifications may damage the motor or the board or both.

## 5.3  Data visualization through X2C-Scope Plug-in of MPLAB X

X2C-Scope is a third-party plug-in in MPLAB X, which helps in real-time diagnostics. The application firmware comes with the initialization needed to interface the controller with the host PC to enable data visualization through the X2C-Scope plug-in. Ensure the X2C-Scope plug-in is installed. For more information on how to set up a plug-in, refer to either the [Microchip Developer Help page](https://microchipdeveloper.com/mplabx:tools-plugins-available) or the [web page.](https://x2cscope.github.io/docs/MPLABX_Plugin.html)
 
1. To establish serial communication with the host PC, connect a micro-USB cable between the host PC and **connector J8** on the development board. 

2. Ensure the application is configured and running as described under section [5.2 Basic Demonstration](#52-basic-demonstration) by following steps 1 through 10.

3. Open the **X2C-Scope** window by selecting **Tools>Embedded>X2CScope.**
      <p align="left">
       <img  src="images/x2cselection.png" width="550"/></p>
 

4. **In the X2C-Scope Configuration** window, open the **Connection Setup** tab and click **Select Project.** This opens the drop-down menu **Select Project** with a list of opened projects. Select the specific project **acim_mchv** from the list of projects and click **OK.**
    <p align="left">
    <img  src="images/x2cprojectselection.png" width="400"/></p>

5. To configure and establish the serial communication for **X2C-Scope**, open the **X2CScope Configuration** window, click on the **Connection Setup** tab and:
     - Set **Baudrate** as **115200**, which is configured in the application firmware. 
     - Click on the **Refresh** button to refresh and update the list of the available Serial COM ports connected to the Host PC. 
     - Select the specific **Serial port** detected when interfaced with the development board. The **Serial port** depends on the system settings

    <p align="left">
     <img  src="images/x2cconnectionsetup.png" width="400"/></p>
 

6. Once the **Serial port** is detected, click on **Disconnected** and turn to **Connected**, to establish serial communication between the Host PC and the board.
     <p align="left">
    <img  src="images/x2cconnectionbutton.png" width="400"/></p>


7. Open the **Project Setup** tab in the **X2CScope Configuration** window and,
     - Set **Scope Sampletime** as the interval at which <code>X2CScopeUpdate()</code> is called. In this application, it is every <code>50µs.</code> 
     - Then, click **Set Values** to save the configuration.

      <p align="left">
      <img  src="images/x2cprojectsetup.png" width="400"/></p>


8.	Click on **Open Scope View** (in the **Data Views** tab of the **X2CScope Configuration** Window); this opens **Scope Window.**
     <p align="left">
      <img  src="images/x2cdataview.png" width="400"/></p>


    	     
9. In the **Scope Window**, select the variables that must be watched. To do this, click on the **Source** against each channel, and a window **Select Variables** opens on the screen. From the available list, the required variable can be chosen. Ensure checkboxes **Enable** and **Visible** are checked for the variables to be plotted.
To view data plots continuously, uncheck **Single-shot.** When **Single-shot** is checked, it captures the data once and stops. The **Sample time factor** value multiplied by **Sample time** decides the time difference between any two consecutive data points on the plot.
    <p align="left">
    <img  src="images/x2cdatapointselection.png" width="800"/></p>

10.	Click on **SAMPLE**, then the X2C-Scope window plots variables in real-time, which updates automatically.
     <p align="left">
     <img  src="images/x2csample.png" width="800"/></p>
 

11.	Click on **ABORT** to stop.
     <p align="left">
     <img  src="images/x2cabort.png" width="800"/></p>
 
 ## 6. REFERENCES:
For additional information, refer following documents or links.
1. AN1206 Application Note [ “Sensorless Field Oriented Control (FOC) of an AC Induction Motor (ACIM) Using Field Weakening”](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/01206A.pdf)
2. Motor Control High Voltage 230VAC-1.5kW Development Board User’s Guide [(DS70005576)](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/UserGuides/Motor-Control-High-Voltage-230VAC-1.5kW-Dev-Board-Users-Guide-DS70005576.pdf)
3. dsPIC33CK256MP508 Motor Control Dual In-Line Module (DIM) Information Sheet [(DS50003063)](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/InformationSheet/dsPIC33CK256MP508-Motor-Control-Dual-In-Line-Module-%28DIM%29-Information-Sheet-DS50003063.pdf)
4. dsPIC33CK256MP508 Family datasheet [(DS70005349H)](https://ww1.microchip.com/downloads/en/DeviceDoc/dsPIC33CK256MP508-Family-Data-Sheet-DS70005349H.pdf)
5. MPLAB® X IDE User’s Guide [(DS50002027)](https://ww1.microchip.com/downloads/en/DeviceDoc/50002027E.pdf) or [MPLAB® X IDE help](https://microchipdeveloper.com/xwiki/bin/view/software-tools/x/)
6. [MPLAB® X IDE installation](http://microchipdeveloper.com/mplabx:installation)
7. [MPLAB® XC-DSC Compiler installation](https://developerhelp.microchip.com/xwiki/bin/view/software-tools/xc-dsc/install/)
8. [Installation and setup of X2Cscope plugin for MPLAB X](https://x2cscope.github.io/docs/MPLABX_Plugin.html)
9. [Microchip Packs Repository](https://packs.download.microchip.com/)
