/***************************************************************************//**
* # License
* Copyright 2023 Shenzhen HOPE Microelectronics Co., Ltd. 
* All rights reserved.
* 
* IMPORTANT: All rights of this software belong to Shenzhen HOPE 
* Microelectronics Co., Ltd. ("HOPERF"). Your use of this Software is limited 
* to those specific rights granted under the terms of the business contract, 
* the confidential agreement, the non-disclosure agreement and any other forms 
* of agreements as a customer or a partner of HOPERF. You may not use this 
* Software unless you agree to abide by the terms of these agreements. 
* You acknowledge that the Software may not be modified, copied, 
* distributed or disclosed unless embedded on a HOPERF Bluetooth Low Energy 
* (BLE) integrated circuit, either as a product or is integrated into your 
* products.  Other than for the aforementioned purposes, you may not use, 
* reproduce, copy, prepare derivative works of, modify, distribute, perform, 
* display or sell this Software and/or its documentation for any purposes.
* 
* YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE 
* PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
* INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
* NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL 
* CMOSTEK OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT, 
* NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER 
* LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING 
* BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR 
* CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF 
* SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES 
* (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.  
*******************************************************************************/

/**
 * @file app_ble.c
 * @version v1.0.2
 *
 */
#include <string.h>
#include "cmt453x.h"
#include "gapm_task.h"               // GAP Manager Task API
#include "app_ble.h"
#include "hp_sec.h"
#include "app_dis.h"
#include "app_user_config.h"
/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if IBEACON_ENABLE

	struct __attribute__((__packed__)){
    //uint8_t flags_len;     // Length of the Flags field.
    //uint8_t flags_type;    // Type of the Flags field.
    //uint8_t flags;         // Flags field.
    uint8_t mandata_len;   // Length of the Manufacturer Data field.
    uint8_t mandata_type;  // Type of the Manufacturer Data field.
    uint8_t comp_id[2];    // Company ID field.
    uint8_t beac_type[2];  // Beacon Type field.
    uint8_t uuid[16];      // 128-bit Universally Unique Identifier (UUID). The UUID is an identifier for the company using the beacon.
    uint8_t maj_num[2];    // Beacon major number. Used to group related beacons.
    uint8_t min_num[2];    // Beacon minor number. Used to specify individual beacons within a group.
    int8_t tx_power;       // The Beacon's measured RSSI at 1 meter distance in dBm. See the iBeacon specification for measurement guidelines.
  }
  bcn_beacon_adv_data
    = {
    // Flag bits - See Bluetooth 4.0 Core Specification , Volume 3, Appendix C, 18.1 for more details on flags.
    //2,            // Length of field. Stack will fill it automatically
    //0x01,         // Type of field. 
    //0x04 | 0x02,  // Flags: LE General Discoverable Mode, BR/EDR is disabled.

    // Manufacturer specific data.
    26,   // Length of field.
    0xFF, // Type of field.

    // The first two data octets shall contain a company identifier code from
    // the Assigned Numbers - Company Identifiers document.
    // 0x004C = Apple
    { 0x4C, 0x00 },

    // Beacon type.
    // 0x0215 is iBeacon.
    { 0x02, 0x15 },

    // 128 bit / 16 byte UUID
    { 0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, \
      0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0 },

    // Beacon major number.
    // Set to 34987 (0x88AB) and converted to correct format.
    { 0x88, 0xAB },

    // Beacon minor number.
    // Set as 1025 (0x0401) and converted to correct format.
    { 0x04, 0x01 },

    // A dummy value which will be eventually overwritten
    0
    };

#elif EDDYSTONE_URL_ENABLE
	#define EDDYSTONE_DATA_LEN (21)
	static uint8_t eddystone_data[EDDYSTONE_DATA_LEN] = {
		0x03,          // Length of service list
		0x03,          // Service list
		0xAA, 0xFE,    // Eddystone ID
		0x10,          // Length of service data
		0x16,          // Service data
		0xAA,  0xFE,   // Eddystone ID
		0x10,          // Frame type Eddystone-URL, UID	0x00, URL	0x10, TLM	0x20, EID	0x30, RESERVED 0x40
		0x00,          // Tx power
		0x01,          // http://www., 0x01=https://www.
		'h','o','p','e','r','f','.','c','o','m'
	};

#elif EDDYSTONE_TLM_ENABLE
	#define EDDYSTONE_DATA_LEN (22)
	static uint8_t eddystone_data[EDDYSTONE_DATA_LEN] = {
		0x03,          // Length of service list
		0x03,          // Service list
		0xAA, 0xFE,    // Eddystone ID
		0x11,          // Length of service data
		0x16,          // Service data
		0xAA,  0xFE,   // Eddystone ID
		0x20,          // Frame type Eddystone-TLM, UID	0x00, URL	0x10, TLM	0x20, EID	0x30, RESERVED 0x40
		0x00,          // TLV version, value = 0x00
		/* [10] */ 0x0C,  0xE4,   // Battery voltage, 1mV/bit, default 3.3v
		/* [12] */ 0x80,  0x00,	  // Beacon temperature
		/* [14] */ 0x00, 0x00, 0x00, 0x00,	// Advertising PDU count
		/* [18] */ 0x00, 0x00, 0x00, 0x00		// Time since reboot
	};
#endif

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void app_ble_connected(void);
void app_ble_disconnected(void);

/**
 * @brief  update the ibeacon data
 * @param  
 * @return 
 * @note   
 */
void app_update_beacon()
{
#if IBEACON_ENABLE
	uint16_t major, minor;
	major = (uint16_t)(rand() & 0xFFFF); //random 
	minor = (uint16_t)(rand() & 0xFFFF); //random 
	
	memcpy(bcn_beacon_adv_data.maj_num, &major, 2);
	memcpy(bcn_beacon_adv_data.min_num, &minor, 2);
	
	hp_ble_adv_data_set((uint8_t *)(&bcn_beacon_adv_data), sizeof(bcn_beacon_adv_data));

#elif EDDYSTONE_URL_ENABLE

#elif EDDYSTONE_TLM_ENABLE

#endif
	
}


/**
 * @brief  user message handler
 * @param  
 * @return 
 * @note   
 */
void app_user_msg_handler(ke_msg_id_t const msgid, void const *p_param)
{
    switch (msgid)
    {
    	case APP_CUSTS_TEST_EVT:
        {
            app_update_beacon();
					
            //repeat timer
            ke_timer_set(APP_CUSTS_TEST_EVT,TASK_APP,5000);
        }
    		break;
			
    	default:
    		break;
    }
}

/**
 * @brief  ble message handler
 * @param  
 * @return 
 * @note   
 */
void app_ble_msg_handler(struct ble_msg_t const *p_ble_msg)
{
    switch (p_ble_msg->msg_id)
    {
        case APP_BLE_OS_READY:
            HP_LOG_INFO("APP_BLE_OS_READY\r\n");
            ke_timer_set(APP_CUSTS_TEST_EVT,TASK_APP,5000);
            break;
        case APP_BLE_GAP_CONNECTED:
            app_ble_connected();
            break;
        case APP_BLE_GAP_DISCONNECTED:
            app_ble_disconnected();
            break;

        default:
            break;
    }

}


/**
 * @brief  advertising message handler
 * @param  
 * @return 
 * @note   
 */
void app_ble_adv_msg_handler(enum app_adv_mode adv_mode)
{
    switch (adv_mode)
    {
        case APP_ADV_MODE_DIRECTED:
            
            break;
        case APP_ADV_MODE_FAST:
            
            break;
        case APP_ADV_MODE_SLOW:
            
            break;
        default:
            break;
    }

}


/**
 * @brief  ble GAP initialization
 * @param  
 * @return 
 * @note   
 */
void app_ble_gap_params_init(void)
{
    struct hp_gap_params_t dev_info = {0};
    uint8_t *p_mac = SystemGetMacAddr();
    //get UUID from trim stored
    if(p_mac != NULL)
    {
        //set the uuid as mac address
        memcpy(dev_info.mac_addr.addr, p_mac , BD_ADDR_LEN); 
    }
    else{
        memcpy(dev_info.mac_addr.addr, "\x01\x02\x03\x04\x05\x06" , BD_ADDR_LEN);
    }
    
    /* init params*/
    dev_info.mac_addr_type = GAPM_STATIC_ADDR;
    dev_info.appearance = 0;
    dev_info.dev_role = GAP_ROLE_PERIPHERAL;
    
    dev_info.dev_name_len = sizeof(CUSTOM_DEVICE_NAME)-1;
    memcpy(dev_info.dev_name, CUSTOM_DEVICE_NAME, dev_info.dev_name_len); 
   
    dev_info.dev_conn_param.intv_min = MSECS_TO_UNIT(MIN_CONN_INTERVAL,MSECS_UNIT_1_25_MS);
    dev_info.dev_conn_param.intv_max = MSECS_TO_UNIT(MAX_CONN_INTERVAL,MSECS_UNIT_1_25_MS);
    dev_info.dev_conn_param.latency  = SLAVE_LATENCY;
    dev_info.dev_conn_param.time_out = MSECS_TO_UNIT(CONN_SUP_TIMEOUT,MSECS_UNIT_10_MS);
    dev_info.conn_param_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    
    hp_ble_gap_init(&dev_info);
}


/**
 * @brief  Setup the advertising beaconing data for different beacons
 * @param  adv_data The specific advertising data
 * @param  adv_data_len The length of the advertising data
 * @return 
 * @note   
 */
static void setup_adv_beaconing(uint8_t* adv_data, uint8_t* adv_data_len)
{

#if IBEACON_ENABLE
	// Fill the measured RSSI at 1 meter distance in dBm
	bcn_beacon_adv_data.tx_power = 0xD7;

	*adv_data_len = sizeof(bcn_beacon_adv_data);
	//memcpy(user_adv.adv_data, (uint8_t *)(&bcn_beacon_adv_data), sizeof(bcn_beacon_adv_data));
	
	memcpy(adv_data, (uint8_t *)(&bcn_beacon_adv_data), sizeof(bcn_beacon_adv_data));	
	
#elif EDDYSTONE_URL_ENABLE
// Fill the eddystone_URL beacon if needed
	*adv_data_len = EDDYSTONE_DATA_LEN;
	memcpy(adv_data, eddystone_data, EDDYSTONE_DATA_LEN);
	
#elif EDDYSTONE_TLM_ENABLE
// Fill the eddystone_URL beacon if needed
	float temp = 25.0;
	eddystone_data[13] = (uint8_t)(((int)(256.0 * temp)) & 0xFF);
	eddystone_data[12] = (uint8_t)(((int)(256.0 * temp)>>8) & 0xFF);
	
	*adv_data_len = EDDYSTONE_DATA_LEN;
	memcpy(adv_data, eddystone_data, EDDYSTONE_DATA_LEN);	
#endif

}


/**
 * @brief  ble advertising initialization
 * @param  
 * @return 
 * @note   
 */
void app_ble_adv_init(void)
{
    struct hp_adv_params_t user_adv = {0};
		
#if (IBEACON_ENABLE || EDDYSTONE_URL_ENABLE || EDDYSTONE_TLM_ENABLE)
		setup_adv_beaconing(user_adv.adv_data, &user_adv.adv_data_len);
#else
    //init advertising data 
    user_adv.adv_data_len = CUSTOM_USER_ADVERTISE_DATA_LEN;
    memcpy(user_adv.adv_data,CUSTOM_USER_ADVERTISE_DATA,CUSTOM_USER_ADVERTISE_DATA_LEN);
    user_adv.scan_rsp_data_len = CUSTOM_USER_ADV_SCNRSP_DATA_LEN;
    memcpy(user_adv.scan_rsp_data,CUSTOM_USER_ADV_SCNRSP_DATA,CUSTOM_USER_ADV_SCNRSP_DATA_LEN);
#endif
    
    user_adv.attach_appearance  = false;
    user_adv.attach_name        = false;
    user_adv.ex_adv_enable      = false;
    user_adv.adv_phy            = PHY_1MBPS_VALUE;
    user_adv.beacon_enable      = true;
    
    //init advertising params
    user_adv.directed_adv.enable = false;

    user_adv.fast_adv.enable    = true;
    user_adv.fast_adv.duration  = CUSTOM_ADV_FAST_DURATION;
    user_adv.fast_adv.adv_intv  = CUSTOM_ADV_FAST_INTERVAL;
    
    user_adv.slow_adv.enable    = true;
    user_adv.slow_adv.duration  = CUSTOM_ADV_SLOW_DURATION;
    user_adv.slow_adv.adv_intv  = CUSTOM_ADV_SLOW_INTERVAL;
    
    user_adv.ble_adv_msg_handler = app_ble_adv_msg_handler;

    hp_ble_adv_init(&user_adv);
    
}

void app_ble_sec_init(void)
{
    struct hp_sec_init_t sec_init = {0};
    
    sec_init.rand_pin_enable = false;
    sec_init.pin_code = 123456;
    
    sec_init.pairing_feat.auth      = ( SEC_PARAM_BOND | (SEC_PARAM_MITM<<2) | (SEC_PARAM_LESC<<3) | (SEC_PARAM_KEYPRESS<<4) );
    sec_init.pairing_feat.iocap     = SEC_PARAM_IO_CAPABILITIES;
    sec_init.pairing_feat.key_size  = SEC_PARAM_KEY_SIZE;
    sec_init.pairing_feat.oob       = SEC_PARAM_OOB;
    sec_init.pairing_feat.ikey_dist = SEC_PARAM_IKEY;
    sec_init.pairing_feat.rkey_dist = SEC_PARAM_RKEY;
    sec_init.pairing_feat.sec_req   = SEC_PARAM_SEC_MODE_LEVEL;
    
    sec_init.bond_enable            = BOND_STORE_ENABLE;
    sec_init.bond_db_addr           = BOND_DATA_BASE_ADDR;
    sec_init.bond_max_peer          = MAX_BOND_PEER;
    sec_init.bond_sync_delay        = 2000;
    
    sec_init.hp_sec_msg_handler     = NULL;
    
    hp_sec_init(&sec_init);
}

void app_ble_prf_init(void)
{
    //add device informaiton server
    hp_ble_add_prf_func_register(app_dis_add_dis);
}


/**
 * @brief  ble initialization
 * @param  
 * @return 
 * @note   
 */
void app_ble_init(void)
{
    struct hp_stack_cfg_t app_handler = {0};
    app_handler.ble_msg_handler  = app_ble_msg_handler;
    app_handler.user_msg_handler = app_user_msg_handler;
    app_handler.lsc_cfg          = BLE_LSC_LSI_32000HZ;
    //initialization ble stack
    hp_ble_stack_init(&app_handler);
    
    app_ble_gap_params_init();
    app_ble_sec_init();
    app_ble_adv_init();
    app_ble_prf_init();
    //start adv
    hp_ble_adv_start();
}

/**
 * @brief  ble connected
 * @param  
 * @return 
 * @note   
 */
void app_ble_connected(void)
{
 

}

/**
 * @brief  ble disconnected
 * @param  
 * @return 
 * @note   
 */
void app_ble_disconnected(void)
{
    // Restart Advertising
    hp_ble_adv_start();

}





/**
 * @}
 */
