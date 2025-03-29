echo off



set HPUtil_path=..\..\HPUtil\HPUtil.exe
:: APP.bin is the update firmware
:: --app_start_address is the address of update firmware
:: --app_version is the version of update firmware
:: --serial_port is the serial port number             
set app_bin=.\Image\APP2.bin
set app_start_address=0x1020000
set app_version=0x01020304
set serial_port=COM5

				  
							 
%HPUtil_path% ius serial %app_bin%                                       ^
                               --app_start_address %app_start_address%         ^
							   --app_version %app_version%                     ^
							   --serial_port %serial_port% ^
							   --serial_baudrate 115200 ^
							   --force_update false


							 
PAUSE
EXIT							 