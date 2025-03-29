
1. Function description
    BLE and 2.4G proprietary application demo, user can switch the funcationality with button 2.
    
    Bluetooth BLE data transmission service (rdts) routine, using 128bit UUID. After the BLE host connects to the device, the downlink data will be transparently transmitted to USART1, and the data received by USART1 will be transparently transmitted to the BLE host
    User can switch the functionality of BLE or 2.4G proprietary with button 2 (PB2).

2. Operating environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board CMT4531_STB

3. Instructions

     push BUTTON1    rdtss <--switch---> RF_TX or RF_RX
     RF_CTRL_MODE    define rf_mode  RF_TX or RF_RX
     PB1 for printf  bps: 115200
     
     test:
     DEV1:  CFG  RF_CTRL_MODE   RF_TX_ONLY,  download program ,reset device, push BUTTON1
     DEV2:  CFG  RF_CTRL_MODE   RF_RX_ONLY,  download program ,reset device, push BUTTON1
     

4. Precautions
    Since the program will enter the sleep mode, the SWD will not be accessible. Press the RESET button to execute the program burning step within one second.