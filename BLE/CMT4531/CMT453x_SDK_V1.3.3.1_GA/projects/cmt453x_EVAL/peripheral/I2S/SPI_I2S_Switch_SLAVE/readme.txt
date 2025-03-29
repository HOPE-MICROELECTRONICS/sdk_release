
1. Function description
    1. After receiving data in I2S peripheral mode, switch to SPI peripheral mode to receive data, and then switch to I2S master mode to receive data

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board CMT4531_Dongle

3. Instructions for use
    System Configuration;
        1. SystemClock: HSI 64MHz
        2. GPIO: (Master mode DEMO board) I2S1: WS--PA0, CK--PA1, SD--PA2
                 (Slave mode DEMO board)  I2S1: WS--PA0, CK--PA1, SD--PA2
        3. Log print: DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control

    Instructions:
        1. After compiling, download the program to reset and run;
        2. The peripheral mode DEMO board enters the debug mode, first press and hold the master mode DEMO board reset button, 
            then the peripheral mode DEMO board runs at full speed,then release the reset button of the master mode DEMO board.
            
4. Matters needing attention
    Two demo boards are required, one board to burn the master mode program, one board to burn the peripheral mode program, connect the VCC and GND of the two boards