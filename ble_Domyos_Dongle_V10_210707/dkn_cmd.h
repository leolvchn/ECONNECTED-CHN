
#ifndef __DKN_CMD_H__
#define __DKN_CMD_H__

#include <stdint.h>
#include <stdbool.h>

typedef union{
	uint16_t all;
	struct{
		uint8_t enable_uart_tx_auto:1;
		
		uint8_t req_start:1;
		uint8_t req_stop:1;
		uint8_t req_pause:1;
		
		uint8_t speed_changed:1;
		uint8_t incline_changed:1;
		uint8_t level_changed:1;
		
		uint8_t req_reset:1;
		uint8_t is_ftms_app:1;
	}b;
	
}dkn_cmd_ctrl_def;


typedef struct{
	uint16_t speed;
	int16_t incline;
	
	uint16_t cal_back;
	uint32_t run_time;
	
	bool b_sport;
	
}dkn_status_def;


typedef struct{
	uint8_t speed;
	uint16_t incline;
	int8_t level;
	
}ew_set_data_def;


typedef struct{
	uint8_t speed_min;
	uint8_t speed_max;
	uint8_t incline_max;
	uint8_t level_max;
	
}ew_range_data_def;

typedef enum{
	EWP_Treadmill,
	EWP_Cross_Trainer,
	EWP_Step_Climber,
	EWP_Stair_Climber,
	EWP_Boat,
	EWP_Bike,
	EWP_Others,
}dkn_cmd_protocol_def;



typedef struct{
	dkn_cmd_ctrl_def				ctrl;
	ew_set_data_def					set;
	ew_range_data_def				range;
	dkn_cmd_protocol_def		protc;
	dkn_status_def					sta;

	
	uint8_t conn_sta_send_times;
	uint8_t hr_send_times;
}dkn_cmd_def;



extern dkn_cmd_def 	dkn_cmd;




void dkn_cmd_init(void);
void dkn_cmd_send(void);
void dkn_send_ctrl_cmd(void);
void dkn_cmd_recv_data(uint8_t* rx, uint8_t len);

uint8_t dkn_get_APP_cmd_len(uint8_t cmd);
uint8_t dkn_get_Console_cmd_len(uint8_t cmd);

uint8_t dkn_get_pulse_from_uart(void);
void dkn_set_pulse_value(uint8_t pulse);


dkn_cmd_protocol_def dkn_get_deivce_type_from_name(char* name_str);



void dkn_cal_dell(void);


#endif //__EW_CMD_H__

