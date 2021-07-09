




#ifndef BLE_FTMS_H__
#define BLE_FTMS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_link_ctx_manager.h"

#ifdef __cplusplus
extern "C" {
#endif
	
	
/**@brief   Macro for defining a ble_nus instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _nus_max_clients Maximum number of NUS clients connected at a time.
 * @hideinitializer
 */
#define BLE_FTMS_DEF(_name, _nus_max_clients)                      \
    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
                             (_nus_max_clients),                  \
                             sizeof(ble_nus_client_context_t));   \
    static ble_ftms_t _name	=                                      \
    {                                                             \
        .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
    };                                                            \
//    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
//                         BLE_NUS_BLE_OBSERVER_PRIO,               \
//                         ble_ftms_on_ble_evt,                      \
//                         &_name)
	

	
#define FTMS_UUID_SERVICE															0x1826			
#define FTMS_UUID_FEATURE															0x2ACC			
#define FTMS_UUID_TREADMILL_DATA											0x2ACD			
#define FTMS_UUID_CROSS_TRAINER_DATA									0x2ACE		
#define FTMS_UUID_STEP_CLIMBER_DATA										0x2ACF		
#define FTMS_UUID_STAIR_CLIMBER_DATA									0x2AD0			
#define FTMS_UUID_ROWER_DATA													0x2AD1			
#define FTMS_UUID_INDOOR_BIKE_DATA										0x2AD2			
#define FTMS_UUID_TRAINING_STATUS											0x2AD3			
#define FTMS_UUID_SUPPORTED_SPEED_RANGE								0x2AD4			
#define FTMS_UUID_SUPPORTED_INCLINATION_RANGE					0x2AD5			
#define FTMS_UUID_SUPPORTED_RESISTANCE_LEVEL_RANGE		0x2AD6			
#define FTMS_UUID_SUPPORTED_POWER_RANGE								0x2AD8			
#define FTMS_UUID_SUPPORTED_HEART_RATE_RANGE					0x2AD7			
#define FTMS_UUID_CONTROL_POINT												0x2AD9			
#define FTMS_UUID_STATUS															0x2ADA			
	


/**@brief Fitness Machine Feature bits. */
#define BLE_FTMS_FEATURE_AVGSPEED									((uint32_t)0x00000001<<0)		
#define BLE_FTMS_FEATURE_CADENCE									((uint32_t)0x00000001<<1)		
#define BLE_FTMS_FEATURE_TOTAL_DISTANCE						((uint32_t)0x00000001<<2)		
#define BLE_FTMS_FEATURE_INCLINATION							((uint32_t)0x00000001<<3)		
#define BLE_FTMS_FEATURE_ELEVATION_GAIN						((uint32_t)0x00000001<<4)		
#define BLE_FTMS_FEATURE_PACE											((uint32_t)0x00000001<<5)		
#define BLE_FTMS_FEATURE_STEP_COUNT								((uint32_t)0x00000001<<6)		
#define BLE_FTMS_FEATURE_RESISTANCE_LEVEL					((uint32_t)0x00000001<<7)		
#define BLE_FTMS_FEATURE_STRIDE_COUNT							((uint32_t)0x00000001<<8)		
#define BLE_FTMS_FEATURE_EXPENDED_ENERGY					((uint32_t)0x00000001<<9)		
#define BLE_FTMS_FEATURE_HEART_RATE_MEASUREMENT		((uint32_t)0x00000001<<10)		
#define BLE_FTMS_FEATURE_METABOLIC_EQUIVALENT			((uint32_t)0x00000001<<11)		
#define BLE_FTMS_FEATURE_ELAPSED_TIME							((uint32_t)0x00000001<<12)		
#define BLE_FTMS_FEATURE_REMAINING_TIME						((uint32_t)0x00000001<<13)		
#define BLE_FTMS_FEATURE_POWER_MEASUREMENT				((uint32_t)0x00000001<<14)		
#define BLE_FTMS_FEATURE_FORCE_ON_BELT_AND_POWER_OUTPUT			((uint32_t)0x00000001<<15)		
#define BLE_FTMS_FEATURE_USER_DATA_RETENTION			((uint32_t)0x00000001<<16)		

/**@brief Target Setting Features bits.*/
#define BLE_FTMS_GFEATURE_SETTING_SPEED_TARGET									((uint32_t)0x00000001<<0)		
#define BLE_FTMS_GFEATURE_SETTING_INCLINATION_TARGET						((uint32_t)0x00000001<<1)		
#define BLE_FTMS_GFEATURE_SETTING_RESISTANCE_TARGET						((uint32_t)0x00000001<<2)		
#define BLE_FTMS_GFEATURE_SETTING_POWER_TARGET									((uint32_t)0x00000001<<3)		
#define BLE_FTMS_GFEATURE_SETTING_HEARTRATE_TARGET							((uint32_t)0x00000001<<4)		
#define BLE_FTMS_GFEATURE_SETTING_EXPENDED_ENERGY_CONFIG   		((uint32_t)0x00000001<<5)	
#define BLE_FTMS_GFEATURE_SETTING_STEP_NUMBER_CONFIG						((uint32_t)0x00000001<<6)		
#define BLE_FTMS_GFEATURE_SETTING_STRIDE_NUMBER_CONFIG					((uint32_t)0x00000001<<7)		
#define BLE_FTMS_GFEATURE_SETTING_DISTANCE_CONFIG							((uint32_t)0x00000001<<8)		
#define BLE_FTMS_GFEATURE_SETTING_TRAINING_TIME_CONFIG					((uint32_t)0x00000001<<9)		
#define BLE_FTMS_GFEATURE_SETTING_TIME_IN_2_HEARTRATE_ZONES_CONFIG	((uint32_t)0x00000001<<10)		
#define BLE_FTMS_GFEATURE_SETTING_TIME_IN_3_HEARTRATE_ZONES_CONFIG	((uint32_t)0x00000001<<11)		
#define BLE_FTMS_GFEATURE_SETTING_TIME_IN_5_HEARTRATE_ZONES_CONFIG	((uint32_t)0x00000001<<12)		
#define BLE_FTMS_GFEATURE_SETTING_INDOOR_BIKE_SIMULATE_PARAMETERS	((uint32_t)0x00000001<<13)		
#define BLE_FTMS_GFEATURE_SETTING_WHEEL_CIRCUMFERENCE_CONFIG				((uint32_t)0x00000001<<14)		
#define BLE_FTMS_GFEATURE_SETTING_SPIN_DOWN_CONTROL								((uint32_t)0x00000001<<15)		
#define BLE_FTMS_GFEATURE_SETTING_CADENCE_CONFIG									  ((uint32_t)0x00000001<<16)		



#define BLE_FTMS_ADV_FLAG_AVAILABLE							0x01


#define BLE_FTMS_ADV_MACHINE_TREADMILL					0x01		
#define BLE_FTMS_ADV_MACHINE_CROSS_TRAINER			0x02		
#define BLE_FTMS_ADV_MACHINE_STEP_CLIMBER				0x04		
#define BLE_FTMS_ADV_MACHINE_STAIR_CLIMBER			0x08		
#define BLE_FTMS_ADV_MACHINE_ROWER							0x10		
#define BLE_FTMS_ADV_MACHINE_INDOOR_BIKE				0x20		
	



typedef enum
{
	BLE_FTMS_FEATURE,
	BLE_FTMS_TREADMILL_DATA,
	BLE_FTMS_CROSS_TRAINER_DATA,
	BLE_FTMS_STEP_CLIMBER_DATA,
	BLE_FTMS_STAIR_CLIMBER_DATA,
	BLE_FTMS_ROWER_DATA,
	BLE_FTMS_INDOOR_BIKE_DATA,
	BLE_FTMS_TRAINING_STATUS,
	BLE_FTMS_SUPPORTED_SPEED_RANGE,
	BLE_FTMS_SUPPORTED_INCLINATION_RANGE,
	BLE_FTMS_SUPPORTED_RESISTANCE_LEVEL_RANGE,
	BLE_FTMS_SUPPORTED_POWER_RANGE,
	BLE_FTMS_SUPPORTED_HEART_RATE_RANGE,
	BLE_FTMS_CONTROL_POINT,
	BLE_FTMS_STATUS,
	
} ble_ftms_evt_type_t;



typedef enum
{
	FTMS_TS_Other=0x00,
	FTMS_TS_Idle,
	FTMS_TS_Warming_Up,
	FTMS_TS_Low_Intensity_Interval,
	FTMS_TS_High_Intensity_Interval,
	FTMS_TS_Recovery_Interval,
	FTMS_TS_Isometric,
	FTMS_TS_Heart_Rate_Control,
	FTMS_TS_Fitness_Test,
	FTMS_TS_Speed_Outside_of_Control_Region_Low,
	FTMS_TS_Speed_Outside_of_Control_Region_High,
	FTMS_TS_Cool_Down,
	FTMS_TS_Watt_Control,
	FTMS_TS_Manual_Mode,
	FTMS_TS_Pre_Workout,
	FTMS_TS_Post_Wordout,
	//Reserved
} ble_ftms_training_status_evt_type_t;



typedef struct ble_ftms_s      										ble_ftms_t;
typedef struct ble_ftms_treadmill_s 							ble_ftms_treadmill_t;
typedef struct ble_ftms_cross_trainer_s						ble_ftms_cross_trainer_t;
typedef struct ble_ftms_indoorbike_s 							ble_ftms_indoorbike_t;
typedef struct ble_ftms_rower_s 									ble_ftms_rower_t;
typedef struct ble_ftms_training_status_s					ble_ftms_training_status_t;
typedef struct ble_ftms_machine_status_s					ble_ftms_machine_status_t;
typedef struct ble_ftms_incline_range_s						ble_ftms_incline_range_t;
typedef struct ble_ftms_resist_range_s						ble_ftms_resist_range_t;
typedef struct ble_ftms_speed_range_s							ble_ftms_speed_range_t;
typedef struct ble_ftms_power_range_s							ble_ftms_power_range_t;
typedef struct ble_ftms_heart_rate_range_s				ble_ftms_heart_rate_range_t;
typedef struct ble_ftms_control_point_data_s			ble_ftms_control_point_data_t;



typedef void (*ble_ftms_write_evt_handler_t) (ble_ftms_t * p_ftms, ble_ftms_evt_type_t * p_evt, uint8_t* data);

typedef void (*ble_ftms_control_point_evt_handler_t) (ble_ftms_t * p_ftms, ble_ftms_control_point_data_t* cp_data);


typedef union{
	uint16_t value;			
	struct{
		uint8_t treadmill_support:1;				
		uint8_t cross_trainer_support:1;		
		uint8_t step_climber_support:1;			
		uint8_t stair_climber_support:1;		
		uint8_t rower_support:1;					
		uint8_t indoor_bike_support:1;		
	}b;
}ble_ftms_machine_type_t;



typedef struct{
	uint32_t fitness_machine;	 	
	uint32_t target_setting;		
	
}ble_ftms_feature_t;


typedef union{
	uint16_t all;
	struct{
		uint8_t supported_speed_range:1;
		uint8_t supported_inclination_range:1;
		uint8_t supported_resistance_level_range:1;
		uint8_t supported_power_range:1;
		uint8_t supported_heart_rate_range:1;
	}b;
}ble_ftms_range_serv_suport_t;


typedef union{
	uint16_t word;
	struct{
		uint8_t more_data_present:1;							
		uint8_t avg_speed_present:1;							
		uint8_t total_distance_present:1;					
		uint8_t inclination_and_ramp_angle_setting_present:1;				
		uint8_t elevation_gain_present:1;					
		uint8_t inst_pace_present:1;							
		uint8_t avg_pace_present:1;								
		uint8_t expended_energy_present:1;				
		uint8_t heart_rate_present:1;							
		uint8_t metabolic_equivalent_present:1;		
		uint8_t elapsed_time_present:1;						
		uint8_t remaining_time_present:1;					
		uint8_t forceonbelt_powerout_present:1;		
	}b;
}ble_ftms_treadmill_flag_t;



struct ble_ftms_treadmill_s
{
  ble_ftms_treadmill_flag_t	flags;
	uint16_t inst_speed;					
	uint16_t avg_speed;						
	uint32_t total_distance;			
	int16_t	 inclination;					
	int16_t  ramp_angle;					
	uint16_t pos_elevation_gain;	
	uint16_t neg_elevation_gain;	
	uint8_t  inst_pace;						
	uint8_t  avg_pace;						
	uint16_t total_energy; 				
	uint16_t energy_per_hour;			
	uint8_t  energy_per_minute;		
	uint8_t  heart_rate;					
	uint8_t  metabolic_equivalent;
	uint16_t elapsed_time;				
	uint16_t remaining_time;			
	int16_t  force_on_belt;				
	int16_t  power_output;				
};



typedef union{
	uint32_t word;	
	struct{
		uint8_t more_data_present:1;							
		uint8_t avg_speed_present:1;							
		uint8_t total_distance_present:1;					
		uint8_t step_count_present:1;							
		uint8_t stride_count_present:1;						
		uint8_t elevation_gain_present:1;					
		uint8_t inclination_and_ramp_angle_setting_present:1;
		uint8_t resistance_level_present:1;				
		uint8_t instantaneous_power_present:1;		
		uint8_t average_power_present:1;					
		uint8_t expended_energy_present:1;				
		uint8_t heart_rate_present:1;							
		uint8_t metabolic_equivalent_present:1;		
		uint8_t elapsed_time_present:1;						
		uint8_t remaining_time_present:1;					
		uint8_t movement_direction:1;							
	}b;
}ble_ftms_cross_trainer_flag_t;


struct ble_ftms_cross_trainer_s
{
  ble_ftms_cross_trainer_flag_t			flags;
	uint16_t inst_speed;							
	uint16_t avg_speed;								
	uint32_t total_distance;					
	uint16_t step_per_minute;					
	uint16_t average_step_rate;				
	uint16_t stride_count;						
	uint16_t positive_elevation_gain;	
	uint16_t negative_elevation_gain;	
	int16_t inclination;							
	int16_t ramp_angle_setting;				
	int16_t resistance_level;					
	int16_t inst_power;								
	int16_t average_power;						
	uint16_t total_energy;						
	uint16_t energy_per_hour;					
	uint8_t energy_per_minute;				
	uint8_t heart_rate;								
	uint8_t metabolic_equivalent;			
	uint16_t elapsed_time;						
	uint16_t remaining_time;					
};

typedef union{
	uint16_t word;
	struct{
		uint8_t more_data_present:1;							
		uint8_t avg_stroke_rate_present:1;			
		uint8_t total_distance_present:1;				
		uint8_t inst_pace_present:1;			
		uint8_t average_pace_present:1;						
		uint8_t inst_power_present:1;	
		uint8_t average_power_present:1;					
		uint8_t resistance_level_present:1;								
		uint8_t expended_energy_present:1;				
		uint8_t heart_rate_present:1;							
		uint8_t metabolic_equivalent_present:1;		
		uint8_t elapsed_time_present:1;						
		uint8_t remaining_time_present:1;				
	}b;
}ble_ftms_rower_flag_t;

struct ble_ftms_rower_s
{
  ble_ftms_rower_flag_t	flags;
	uint8_t	stroke_rate;						
	uint16_t stroke_count;					
	uint8_t avg_stroke_rate;				
	uint32_t total_distance;				
	uint16_t inst_pace;							
	uint16_t average_pace;					
	int16_t inst_power;							
	int16_t average_power;					
	int16_t resistance_level;				
	uint16_t  total_energy;					
	uint16_t 	energy_per_hour;			
	uint8_t 	energy_per_minute;		
	uint8_t 	heart_rate;						
	uint8_t 	metabolic_equivalent;	
	uint16_t  elapsed_time;					
	uint16_t  remaining_time;				
};


typedef union{
	uint16_t word;
	struct{
		uint8_t more_data_present:1;							
		uint8_t avg_speed_present:1;							
		uint8_t inst_cadence:1;										
		uint8_t avg_cadence_present:1;						
		uint8_t total_distance_present:1;				
		uint8_t resist_level_present:1;					
		uint8_t inst_power_present:1;							
		uint8_t avg_power_present:1;							
		uint8_t expended_energy_present:1;			
		uint8_t heart_rate_present:1;							
		uint8_t metabolic_equivalent_present:1;		
		uint8_t elapsed_time_present:1;						
		uint8_t remaining_time_present:1;					
	}b;
}ble_ftms_indoorbike_flag_t;


struct ble_ftms_indoorbike_s
{
  ble_ftms_indoorbike_flag_t	flags;
	uint16_t	inst_speed;					
	uint16_t  avg_speed;					
	uint16_t  inst_cadence;				
	uint16_t  avg_cadence;				
	uint32_t  total_distance;			
	uint16_t  resistance_level;	
	uint16_t  inst_power;					
	uint16_t  avg_power;					
	uint16_t  total_energy;				
	uint16_t 	energy_per_hour;		
	uint8_t 	energy_per_minute;	
	uint8_t 	heart_rate;					
	uint8_t 	metabolic_equivalent;
	uint16_t  elapsed_time;					
	uint16_t  remaining_time;				
};





typedef union{
	uint8_t byte;
	struct{
		uint8_t training_status_string_present:1;
		uint8_t extended_string_present:1;
	}b; 
}ble_ftms_training_status_flag_t;



struct ble_ftms_training_status_s
{
	ble_ftms_training_status_flag_t				flags;
	ble_ftms_training_status_evt_type_t 	status;
	char* 																status_string;
	
	uint8_t 														str_len;
}; 


struct ble_ftms_speed_range_s{
	uint16_t min;		
	uint16_t max;		
	uint16_t inc;		
};

struct ble_ftms_incline_range_s{
	int16_t min;								
	int16_t max;								
	int16_t inc;							
};


struct ble_ftms_resist_range_s{
	int16_t min;							
	int16_t max;							
	int16_t inc;							
};


struct ble_ftms_power_range_s{
	int16_t min;	
	int16_t max;	
	uint16_t inc;		
};	


struct ble_ftms_heart_rate_range_s{
	uint8_t min;	
	uint8_t max;		
	uint8_t inc;		
};


typedef enum{
	STATUS_STOP_OR_PAUSE_STOP = 1,
	STATUS_STOP_OR_PAUSE_PAUSE = 2,
}ble_ftms_machine_status_stop_or_pause_def;

typedef enum{
	STATUS_OP_Reserved_for_future_use=0,										
	STATUS_OP_Reset,																				
	STATUS_OP_Stop_or_pause,																
	STATUS_OP_Stop_by_safety,																
	STATUS_OP_Start_or_resumed,															
	STATUS_OP_Target_speed_changed,													
	STATUS_OP_Target_incline_changed,												
	STATUS_OP_Target_resist_level_changed,									
	STATUS_OP_Target_power_changed,													
	STATUS_OP_Target_heart_rate_changed,										
	STATUS_OP_Targeted_expended_energy_changed,							
	STATUS_OP_Targeted_number_of_steps_changed,							
	STATUS_OP_Targeted_number_of_strides_changed,						
	STATUS_OP_Targeted_distance_changed,										
	STATUS_OP_Targeted_training_time_changed,								
	STATUS_OP_Targeted_time_in_2_heart_rate_zones_changed,	
	STATUS_OP_Targeted_time_in_3_heart_rate_zones_changed,	
	STATUS_OP_Targeted_time_in_5_heart_rate_zones_changed,	
	STATUS_OP_Indoor_bike_simulation_parameters_changed,		
	STATUS_OP_Wheel_circumference_changed,									
	STATUS_OP_Spin_down_status,															
	STATUS_OP_Targeted_cadence_changed,											
	
	STATUS_OP_CONTROL_PERMISSION_LOST=0xff,									
}ble_ftms_machine_status_op_data_t;

struct ble_ftms_machine_status_s
{
	ble_ftms_machine_status_op_data_t				op_data;
	uint8_t 										parameter[19];
	uint8_t 										par_len;
};



typedef enum{
	CTRL_OP_Request_Control = 0, 							
	CTRL_OP_Request_Reset, 										
	CTRL_OP_Set_Target_Speed, 								
	CTRL_OP_Set_Target_Inclination, 					
	CTRL_OP_Set_Target_Resistance_Level, 			
	CTRL_OP_Set_Target_Power, 								
	CTRL_OP_Set_Target_Heart_Rate, 					
	CTRL_OP_Start_or_Resume, 									
	CTRL_OP_Stop_or_Pause, 										
	CTRL_OP_Set_Targeted_Expended_Energy, 		
	CTRL_OP_Set_Targeted_Number_of_Steps, 		
	CTRL_OP_Set_Targeted_Number_of_Strides, 	
	CTRL_OP_Set_Targeted_Distance, 						
	CTRL_OP_Set_Targeted_Training_Time, 			
	CTRL_OP_Set_Targeted_Time_in_2_Heart_Rate_Zones, 			
	CTRL_OP_Set_Targeted_Time_in_3_Heart_Rate_Zones, 			
	CTRL_OP_Set_Targeted_Time_in_5_Heart_Rate_Zones, 		
	CTRL_OP_Set_Indoor_Bike_Simulation_Parameters, 			
	CTRL_OP_Set_Wheel_Circumference, 					
	CTRL_OP_Spin_Down_Control, 								
	CTRL_OP_Set_Targeted_Cadence, 						

	CTRL_OP_Response_Code=0x80,								
}ble_ftms_control_point_op_code_t;

struct ble_ftms_control_point_data_s{
	ble_ftms_control_point_op_code_t 	op_code;
	uint8_t 													pram[18];
	uint8_t 													pram_len;
};


typedef enum{
	CTRL_RESULT_Reserved_for_Future_Use = 0,					
	CTRL_RESULT_Success,															
	CTRL_RESULT_Op_Code_not_supported,								
	CTRL_RESULT_Invalid_Parameter,										
	CTRL_RESULT_Operation_Failed,											
	CTRL_RESULT_Control_Not_Permitted,								
	
}ble_ftms_control_point_response_result_def;

typedef struct{
	ble_ftms_control_point_op_code_t 							op_code;
	ble_ftms_control_point_response_result_def		result;
	uint8_t 																			param[17];
	uint8_t 																			param_len;
}ble_ftms_control_point_response_t;




typedef void (*ftms_control_point_cb_fuc_t) (uint8_t conidx, ble_ftms_control_point_data_t* cp_data);



typedef struct 
{
		ble_ftms_write_evt_handler_t       	write_evt_handler;           
		ble_ftms_control_point_evt_handler_t	control_point_evt_handle;

    ble_ftms_feature_t           				feature;                                    
		ble_ftms_range_serv_suport_t				range_suport;
    ble_ftms_treadmill_t*         			initial_treadmill;                          
		ble_ftms_cross_trainer_t*						initial_cross_trainer;
		ble_ftms_indoorbike_t*							initial_indoorbike;
		ble_ftms_rower_t*										initial_rower;
		ble_ftms_training_status_t*					initial_training_status;
		ble_ftms_machine_status_t*					initial_machine_status;
		ble_ftms_incline_range_t						incline_range;
		ble_ftms_resist_range_t							resist_range;
		ble_ftms_speed_range_t							speed_range;
		ble_ftms_power_range_t							power_range;
		ble_ftms_heart_rate_range_t					heart_rate_range;
} ble_ftms_init_t;


struct ble_ftms_s
{
		ble_ftms_write_evt_handler_t write_evt_handler;           
		ble_ftms_control_point_evt_handler_t	control_point_evt_handle;
		uint16_t                     service_handle;                            
		ble_gatts_char_handles_t     feature_handles;                          
		ble_gatts_char_handles_t     treadmill_handles;                          
		ble_gatts_char_handles_t     cross_trainer_handles;                           
		ble_gatts_char_handles_t     indoorbike_handles;                           
		ble_gatts_char_handles_t     rower_handles;                           
		ble_gatts_char_handles_t     training_status_handles;                           
		ble_gatts_char_handles_t     control_point_handles;                          
	  ble_gatts_char_handles_t     machine_status_handles;                           
		ble_gatts_char_handles_t		 speed_range_handles;
		ble_gatts_char_handles_t		 inclination_range_handles;								 
		ble_gatts_char_handles_t		 resistance_level_range_handles;								
		ble_gatts_char_handles_t		 power_range_handles;
		ble_gatts_char_handles_t		 heart_rate_range_handles;
		uint16_t                     conn_handle;                             
		blcm_link_ctx_storage_t * const p_link_ctx_storage; 
		bool 												 is_treamill_notify_enable;
		bool 												 is_cross_trainer_notify_enable;
		bool 												 is_indoorbike_notify_enable;
		bool 												 is_rower_notify_enable;
		bool 												 is_training_status_notify_enable;
		bool 												 is_machine_status_notify_enable;
		
		bool 												 ctrl_point_enable;
};



uint32_t ble_ftms_init(ble_ftms_t * p_ftms, const ble_ftms_init_t * p_ftms_init);



uint32_t ble_ftms_treadmill_send(ble_ftms_t * p_ftms, ble_ftms_treadmill_t * p_treadmill);
uint32_t ble_ftms_cross_trainer_send(ble_ftms_t * p_ftms, ble_ftms_cross_trainer_t * p_cross_trainer);
uint32_t ble_ftms_indoorbike_send(ble_ftms_t * p_ftms, ble_ftms_indoorbike_t * p_indoorbike);
uint32_t ble_ftms_rower_send(ble_ftms_t * p_ftms, ble_ftms_rower_t * p_rower);
uint32_t ble_ftms_training_status_send(ble_ftms_t * p_ftms, ble_ftms_training_status_t * p_training_status);
uint32_t ble_ftms_training_status_value_set(ble_ftms_t * p_ftms, ble_ftms_training_status_t * p_training_status);
uint32_t ble_ftms_machine_status_send(ble_ftms_t * p_ftms, ble_ftms_machine_status_t * p_machine_status);
uint32_t ble_ftms_control_point_send(ble_ftms_t * p_ftms, ble_ftms_control_point_response_t* cp_data);

uint32_t ble_ftms_speed_range_value_set(ble_ftms_t * p_ftms, ble_ftms_speed_range_t * p_speed_range);
uint32_t ble_ftms_inclination_range_value_set(ble_ftms_t * p_ftms, ble_ftms_incline_range_t * p_inc_range);;
uint32_t ble_ftms_resistance_level_range_value_set(ble_ftms_t * p_ftms, ble_ftms_resist_range_t * p_resist_level_range);
uint32_t ble_ftms_power_range_value_set(ble_ftms_t * p_ftms, ble_ftms_power_range_t * p_power_range);
uint32_t ble_ftms_heartrate_range_value_set(ble_ftms_t * p_ftms, ble_ftms_heart_rate_range_t * p_hr_range);


void ble_ftms_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

#endif //BLE_FTMS_H__

