
#ifndef __M_SYSTEM_H__
#define __M_SYSTEM_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble_ftms.h"


typedef union{
	uint8_t byte;
	struct{
		uint8_t b0:1;
		uint8_t b1:1;
		uint8_t b2:1;
		uint8_t b3:1;
		uint8_t b4:1;
		uint8_t b5:1;
		uint8_t b6:1;
		uint8_t b7:1;
	}b;
}u8_flag_def;


typedef struct{
	bool ble_adv_enable;
	bool ble_scan_enable;
	bool ant_scan_enable;
	
	
	bool uart_dkn_cmd_enable;
	
	bool req_reset_after_flash_ok;
}System_ctrl_flag;

typedef struct{
	bool ble_is_adving;
	bool ble_is_scaning;
	bool ant_is_scaning;
	
	bool ble_slave_is_connected;
	bool ble_master_is_connected;
	bool ant_is_connected;
	
	bool ble_is_master_conn_ing;
}System_status_flag;

typedef struct{
	uint8_t uart_delay;
}System_time_def;

typedef struct{
	System_ctrl_flag		ctrl;
	System_status_flag	status;
	
	System_time_def			time;
}System_Def;


typedef struct{
	uint8_t Value[20];
	uint8_t Length;
}ble_dis_data_t;

typedef struct{
	ble_ftms_machine_type_t				type;
	ble_ftms_feature_t						feature;
	
	struct{
		ble_ftms_speed_range_t			speed;
		ble_ftms_incline_range_t 		incline;
		ble_ftms_resist_range_t			resist;
		ble_ftms_power_range_t			power;
		ble_ftms_heart_rate_range_t	heartrate;
	}range;
	
	struct{
		ble_dis_data_t dis_manufacturer_name;
		ble_dis_data_t dis_model_nb;
	}string;

	uint8_t 											enable;
}Flash_FTMS_Data_Def;

typedef struct {
	char 		device_name[20];
	uint8_t name_len;
	Flash_FTMS_Data_Def		ftms;
	uint16_t check; 
}MY_FLASH_DATA_DEF;



typedef struct{
	uint16_t id;
	int8_t	rssi;
	uint8_t pulse;
}Ant_Hrm_Device_t;

typedef struct{
	Ant_Hrm_Device_t	device[10];
	uint8_t 					num;
	uint8_t 					status;
	uint16_t 					select_id;
	uint8_t 					lost_time;
	
	uint8_t 					comfirm_time;
	uint8_t 					pulse;
	
	uint8_t 					send_empty_times;
}Ant_Hrm_Def;

typedef struct{
	ble_gap_addr_t		addr;

	int8_t				 		rssi;
	uint8_t 					comfirm_time;
	
	uint8_t 					pulse;
	uint8_t 					send_empty_times;
	
	uint8_t conn_timeout;
	
}Ble_Hr_Def;


extern MY_FLASH_DATA_DEF my_flash;

extern System_Def	Sys;


extern Ant_Hrm_Def Ant_Hrm;
extern Ble_Hr_Def Ble_Hr;

extern ble_ftms_treadmill_t					treadmill;
extern ble_ftms_indoorbike_t				bike;
extern ble_ftms_cross_trainer_t	cross_trainer;
extern ble_ftms_rower_t					rower;
extern ble_ftms_training_status_t		ftms_training_status;
extern ble_ftms_machine_status_t		ftms_machine_status;

void uart_send_array(const uint8_t* buf, uint8_t len);

void DKN_Send_Ftms_machine_status(void);

void system_reset_after_ms(uint32_t ms);

#endif //__M_SYSTEM_H__
