
echo off


set HPUtil_path=..\..\HPUtil\HPUtil.exe
set private_key=..\..\keys\private_key.pem


set APP_VERSION=0x00000002
set IMAGE_UPDATE_VERSION=0x00000002
set output_ota_zip=Image\ota_dual_bank
set app1_bin=Image\APP1.bin
set app2_bin=Image\APP2.bin
set image_update_bin=Image\ImageUpdate.bin


if defined private_key (

	if defined app2_bin if defined image_update_bin (
	
        %HPUtil_path% pack otawithsigning %output_ota_zip%                                      ^
                                                 --private_key %private_key%                    ^
        										 --app1_start_address 0x01004000                ^
        										 --app1_version %APP_VERSION%                   ^
        										 --app1_bin %app1_bin%                          ^
        										 --app2_start_address 0x01020000                ^
        										 --app2_version %APP_VERSION%                   ^
        										 --app2_bin %app2_bin%  		                ^
										         --image_update_start_address 0x0103C000        ^
										         --image_update_version %IMAGE_UPDATE_VERSION%  ^
										         --image_update_bin %image_update_bin%
										 
										 
	)

	if defined app2_bin if not defined image_update_bin (
	
        %HPUtil_path% pack otawithsigning %output_ota_zip%                                      ^
                                                 --private_key %private_key%                    ^
        										 --app1_start_address 0x01004000                ^
        										 --app1_version %APP_VERSION%                   ^
        										 --app1_bin %app1_bin%                          ^
        										 --app2_start_address 0x01020000                ^
        										 --app2_version %APP_VERSION%                   ^
        										 --app2_bin %app2_bin%
										 
										 
	)

	if not defined app2_bin if defined image_update_bin (
	
        %HPUtil_path% pack otawithsigning %output_ota_zip%                                      ^
                                                 --private_key %private_key%                    ^
        										 --app1_start_address 0x01004000                ^
        										 --app1_version %APP_VERSION%                   ^
        										 --app1_bin %app1_bin%                          ^
										         --image_update_start_address 0x0103C000        ^
										         --image_update_version %IMAGE_UPDATE_VERSION%  ^
										         --image_update_bin %image_update_bin%				 
	)

)

if not defined private_key (

	if defined app2_bin if defined image_update_bin (
	
        %HPUtil_path% pack ota %output_ota_zip%                                      ^
        										 --app1_start_address 0x01004000                ^
        										 --app1_version %APP_VERSION%                   ^
        										 --app1_bin %app1_bin%                          ^
        										 --app2_start_address 0x01020000                ^
        										 --app2_version %APP_VERSION%                   ^
        										 --app2_bin %app2_bin%  		                ^
										         --image_update_start_address 0x0103C000        ^
										         --image_update_version %IMAGE_UPDATE_VERSION%  ^
										         --image_update_bin %image_update_bin%
										 
										 
	)

	if defined app2_bin if not defined image_update_bin (
	
        %HPUtil_path% pack ota %output_ota_zip%                                      ^
        										 --app1_start_address 0x01004000                ^
        										 --app1_version %APP_VERSION%                   ^
        										 --app1_bin %app1_bin%                          ^
        										 --app2_start_address 0x01020000                ^
        										 --app2_version %APP_VERSION%                   ^
        										 --app2_bin %app2_bin%
										 
										 
	)

	if not defined app2_bin if defined image_update_bin (
	
        %HPUtil_path% pack ota %output_ota_zip%                                      ^
        										 --app1_start_address 0x01004000                ^
        										 --app1_version %APP_VERSION%                   ^
        										 --app1_bin %app1_bin%                          ^
										         --image_update_start_address 0x0103C000        ^
										         --image_update_version %IMAGE_UPDATE_VERSION%  ^
										         --image_update_bin %image_update_bin%				 
	)

)



							 
PAUSE
EXIT