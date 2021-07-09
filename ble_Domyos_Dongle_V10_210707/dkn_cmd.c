
#include <stdio.h>
#include <string.h>

#include "dkn_cmd.h"
#include "ble_dis.h"

#include "m_system.h"

dkn_cmd_def 	dkn_cmd;


const uint8_t T_DKN_CMD_APP[][2]={
//自定义命令	
	{0x50, 5},
	{0x60, 4},
	{0x80, 5},
	
//迪卡侬命令	
	{0xAC, 3},
	{0xA0, 4},
	{0xA2, 4},
	{0xA3, 3},
	{0xA4, 3},
	{0xA5, 3},
	{0xA7, 3},
	{0xA8, 4},
	{0xAB, 3},
	
	{0xAD, 23},
	{0xAF, 4},
	{0xC4, 4},
	{0xC5, 3},
	{0xC6, 3},
	{0xC7, 3},
	{0xC8, 4},
	{0xC9, 3},
	{0xCA, 4},
	{0xCB, 27},
	{0xCC, 9},
	{0xCD, 27},
	{0xCE, 9},
	{0xCF, 4},
	{0xE0, 4},
	{0xE1, 4},
};

const uint8_t T_DKN_CMD_CONSOLE[][2]={
//迪卡侬命令	
	{0xBC, 26},
	{0xB0, 4},
	{0xB2, 4},
	{0xB3, 7},
	{0xB4, 13},
	{0xB5, 7},
	{0xB7, 16},
	{0xB8, 4},
	{0xBB, 8},
	
	{0xBD, 23},
	{0xBF, 4},
	{0xD4, 4},
	{0xD5, 3},
	{0xD6, 4},
	{0xD7, 13},
	{0xD8, 4},
	{0xD9, 8},
	{0xDA, 4},
	{0xDB, 27},
	{0xDC, 9},
	{0xDD, 27},
	{0xDE, 9},
	{0xDF, 5},
	{0xF0, 13},
	{0xF1, 4},
};


unsigned char DKN_CheckSum(unsigned char*p,unsigned char length);
void dkn_treadmill_tx(void);

void dkn_treadmill_rx(uint8_t* rx, uint8_t len);
void dkn_bike_el_rx(uint8_t* rx, uint8_t len);

dkn_cmd_protocol_def dkn_get_deivce_type_from_name(char* name_str);

/**
dkn协议命令初始化
*/
void dkn_cmd_init(void)
{
//	uint8_t i;
	memset(&dkn_cmd, 0, sizeof(dkn_cmd));

	dkn_cmd.protc = dkn_get_deivce_type_from_name(my_flash.device_name);
	
}



uint8_t dkn_get_APP_cmd_len(uint8_t cmd)
{
	uint8_t i, size = sizeof(T_DKN_CMD_APP)>>1;
	for(i=0;i<size;i++)
	{
		if(cmd == T_DKN_CMD_APP[i][0])
		{
			return T_DKN_CMD_APP[i][1];
		}
	}
	
	return 0;
}

uint8_t dkn_get_Console_cmd_len(uint8_t cmd)
{
	uint8_t i, size = sizeof(T_DKN_CMD_CONSOLE)>>1;
	for(i=0;i<size;i++)
	{
		if(cmd == T_DKN_CMD_CONSOLE[i][0])
		{
			return T_DKN_CMD_CONSOLE[i][1];
		}
	}
	
	return 0;
}



/**
根据广播名称返回器材类型
dkn_cmd_protocol_def		器材类型结构体
只能识别以"EW-XX"命名的机型，否则返回其他
目前支持4种机型
EW-TM-跑步机
EW-EL-椭圆机
EW-ST-划船器
EW-BK-自行车
*/
dkn_cmd_protocol_def dkn_get_deivce_type_from_name(char* name_str)
{
	dkn_cmd_protocol_def  type;
	if(memcmp("Domyos-", name_str, 7) == 0)
	{
		if(memcmp("TC", &name_str[7], 2) == 0)
			type = EWP_Treadmill;
		else if(memcmp("Bike", &name_str[7], 4) == 0)
			type = EWP_Bike;
		else if(memcmp("ROW", &name_str[7], 3) == 0)
			type = EWP_Boat;
		else if(memcmp("EL", &name_str[7], 2) == 0)
			type = EWP_Cross_Trainer;
		else
			type = EWP_Treadmill;
	}
	else
		type = EWP_Others;
	
	return type;
}

/**

*/
void dkn_cmd_send(void)
{
	dkn_treadmill_tx();

}

/*ew协议命令接收函数，放在接收超时回调*/
void dkn_cmd_recv_data(uint8_t* rx, uint8_t len)
{

	if(DKN_CheckSum(rx, len-1) == rx[len-1])
	{
		
		switch(dkn_cmd.protc)
		{
			case EWP_Treadmill:
				dkn_treadmill_rx(rx, len);
			break;
			case EWP_Boat:
			case EWP_Cross_Trainer:
			case EWP_Bike:
				dkn_bike_el_rx(rx, len);
			break;
			default:break;
		}
	}
}


void dkn_send_ctrl_cmd(void)
{
	uint8_t tx_buf[30]={0};
	uint16_t temp16;
	
	if(dkn_cmd.ctrl.b.is_ftms_app)
	{
		tx_buf[0] = 0xF0;
		tx_buf[1] = 0xA1;
		tx_buf[2] = 0x31;
		tx_buf[3] =  DKN_CheckSum(tx_buf, 3);

		uart_send_array(tx_buf, 4);
		
		dkn_cmd.ctrl.b.is_ftms_app = 0;
	}
	else if(dkn_cmd.ctrl.b.req_start)
	{
		tx_buf[0] = 0xF0;
		tx_buf[1] = 0xC8;
		tx_buf[2] = 1;
		tx_buf[3] =  DKN_CheckSum(tx_buf, 3);

		uart_send_array(tx_buf, 4);
		
		dkn_cmd.ctrl.b.req_start = 0;
	}
	else if(dkn_cmd.ctrl.b.req_stop || dkn_cmd.ctrl.b.req_pause)
	{
		tx_buf[0] = 0xF0;
		tx_buf[1] = 0xC8;
		tx_buf[2] = 0;
		tx_buf[3] =  DKN_CheckSum(tx_buf, 3);

		uart_send_array(tx_buf, 4);
		
		dkn_cmd.ctrl.b.req_stop = 0;
		dkn_cmd.ctrl.b.req_pause = 0;
	}
	else if(dkn_cmd.ctrl.b.req_reset)
	{
		tx_buf[0] = 0xF0;
		tx_buf[1] = 0xA8;
		tx_buf[2] = 1;
		tx_buf[3] =  DKN_CheckSum(tx_buf, 3);

		uart_send_array(tx_buf, 4);
		
		dkn_cmd.ctrl.b.req_reset = 0;
	}
	else if(dkn_cmd.ctrl.b.level_changed || dkn_cmd.ctrl.b.incline_changed || dkn_cmd.ctrl.b.speed_changed)
	{
		tx_buf[0] = 0xF0;
		tx_buf[1] = 0xAD;
		
		tx_buf[4] = dkn_cmd.set.speed >> 8;
		tx_buf[5] = dkn_cmd.set.speed;

		tx_buf[10] = dkn_cmd.set.level;
		
		temp16 = 1000 + dkn_cmd.set.incline;
		tx_buf[13] = temp16 >> 8;
		tx_buf[14] = temp16;
		
		if(Sys.status.ble_slave_is_connected)
			tx_buf[18] = 1;
		else
			tx_buf[18] = 0;
		
		tx_buf[22] =  DKN_CheckSum(tx_buf, 22);

		uart_send_array(tx_buf, 23);
		
		dkn_cmd.ctrl.b.level_changed = 0;
		dkn_cmd.ctrl.b.incline_changed = 0;
		dkn_cmd.ctrl.b.speed_changed = 0;
	}
}

/**
跑步机发送
*/
//迪卡侬
void dkn_treadmill_tx(void)
{
	uint8_t tx_buf[4]={0};
	
	tx_buf[0] = 0xF0;
	tx_buf[1] = 0xAC;
	tx_buf[2] =  DKN_CheckSum(tx_buf, 2);

	uart_send_array(tx_buf, 3);
}

/**
跑步机接收处理
*/
//迪卡侬
void dkn_treadmill_rx(uint8_t* rx, uint8_t len)
{
	uint16_t temp16;
	int16_t  tmp_int16;
	bool b_machine_sta_change = false;
	
	switch(rx[1])
	{
		case 0xBC:
			tmp_int16 = ((uint16_t)rx[2] << 8) + rx[3];
			if(tmp_int16 > 1000)
				tmp_int16 -= 1000;
			else
				tmp_int16 = -tmp_int16;
			
			if(tmp_int16 != dkn_cmd.sta.incline)
			{
				dkn_cmd.sta.incline = tmp_int16;
				
				dkn_cmd.set.incline = dkn_cmd.sta.incline;
				
				treadmill.inclination = dkn_cmd.sta.incline;
				
				
				if(dkn_cmd.sta.b_sport)
				{
					ftms_machine_status.op_data = STATUS_OP_Target_incline_changed;
					ftms_machine_status.parameter[0] = treadmill.inclination;
					ftms_machine_status.parameter[1] = treadmill.inclination >> 8;
					ftms_machine_status.par_len = 2;
					b_machine_sta_change = true;
				}
			}
			
			treadmill.ramp_angle = treadmill.inclination;
		
			treadmill.power_output = ((uint16_t)rx[4] << 8) + rx[5];
		
			temp16 = ((uint16_t)rx[6] << 8) + rx[7];
			
			if(temp16 != dkn_cmd.sta.speed)
			{
				dkn_cmd.sta.speed = temp16;
				
				dkn_cmd.set.speed = dkn_cmd.sta.speed;
				
				treadmill.inst_speed = dkn_cmd.sta.speed * 10;
				
				
				if(dkn_cmd.sta.b_sport)
				{
					ftms_machine_status.op_data = STATUS_OP_Target_speed_changed;
					ftms_machine_status.parameter[0] = treadmill.inst_speed;
					ftms_machine_status.parameter[1] = treadmill.inst_speed >> 8;
					ftms_machine_status.par_len = 2;
					b_machine_sta_change = true;
				}
			}
			
			
			temp16 = 6000 / dkn_cmd.sta.speed; 
			if(temp16 > 255){
				temp16 = 255;
			}
			treadmill.inst_pace = temp16;
		
			treadmill.total_energy = ((uint16_t)rx[10] << 8) + rx[11];
			treadmill.total_distance = ((uint16_t)rx[12] << 8) + rx[13];//0.1km
			treadmill.total_distance *= 100;//0.001km
			treadmill.heart_rate = ((uint16_t)rx[17] << 8) + rx[18];
			treadmill.avg_speed = ((uint16_t)rx[19] << 8) + rx[20];
		
			if((rx[15]==0)&&(rx[21])&&(rx[16]))   //15-err  16-tablet on equipment or not		21 safekey
			{
				if(rx[24])//sport
				{
					//dkn_cmd.ctrl.b.req_start = 0;
					if(!b_machine_sta_change)
					{
						ftms_machine_status.op_data = STATUS_OP_Start_or_resumed;
						ftms_machine_status.par_len = 0;
						ftms_training_status.status = FTMS_TS_Manual_Mode;
						
						if(!dkn_cmd.sta.b_sport)
						{
							dkn_cmd.sta.b_sport = true;
							b_machine_sta_change = true;
						}
					}
					
					dkn_cmd.ctrl.b.req_start = 0;
				}
				else
				{
					dkn_cmd.ctrl.b.req_stop = 0;

					ftms_machine_status.op_data = STATUS_OP_Stop_or_pause;
					ftms_machine_status.parameter[0] = STATUS_STOP_OR_PAUSE_STOP;
					ftms_machine_status.par_len = 1;
					
					ftms_training_status.status = FTMS_TS_Idle;
					
					if(dkn_cmd.sta.b_sport)
					{
						dkn_cmd.sta.b_sport = false;
						b_machine_sta_change = true;
					}
	
					dkn_cmd.ctrl.b.req_stop = 0;
				}
			}
			else
			{
				if(rx[21] == 0)
				{
					ftms_machine_status.op_data = STATUS_OP_Stop_by_safety;
					ftms_machine_status.par_len = 0;
				}
				else
				{
					dkn_cmd.ctrl.b.req_stop = 0;
					ftms_machine_status.op_data = STATUS_OP_Stop_or_pause;
					ftms_machine_status.parameter[0] = STATUS_STOP_OR_PAUSE_STOP;
					ftms_machine_status.par_len = 1;
				}
				if(dkn_cmd.sta.b_sport)
				{
					dkn_cmd.sta.b_sport = false;
					b_machine_sta_change = true;
				}
			}
			
			if(!dkn_cmd.sta.b_sport)
			{
				if(treadmill.total_energy == 0)
				{
					treadmill.energy_per_hour = 0;
					treadmill.energy_per_minute = 0;
				}
			}
			
			if(b_machine_sta_change){
				DKN_Send_Ftms_machine_status();
			}
		break;
		case 0xB1:
			dkn_cmd.ctrl.b.is_ftms_app = 0;
		break;
		case 0xB8:
			dkn_cmd.ctrl.b.req_reset = 0;
		break;
		case 0xD8:
			dkn_cmd.ctrl.b.req_start = 0;
		break;
		case 0xBD:
//			if(rx[10] == dkn_cmd.set.level)
			{
				dkn_cmd.ctrl.b.level_changed = 0;
				dkn_cmd.ctrl.b.speed_changed = 0;
				dkn_cmd.ctrl.b.incline_changed = 0;
			}
		break;
		
		case 0x70:
			if(dkn_cmd.conn_sta_send_times < 3)
				dkn_cmd.conn_sta_send_times = 0;

		break;
	}
}


void dkn_bike_el_rx(uint8_t* rx, uint8_t len)
{
	bool b_machine_sta_change = false;
	switch(rx[1])
	{
		case 0xBC:
			
//=================================================================================		
				cross_trainer.total_distance = ((uint16_t)rx[12] << 8) + rx[13];//0.1km
				cross_trainer.total_distance *= 100;//0.001km

				cross_trainer.total_energy = ((uint16_t)rx[10] << 8) + rx[11];
			
				cross_trainer.inst_power = ((uint16_t)rx[4] << 8) + rx[5];
			
				cross_trainer.heart_rate = ((uint16_t)rx[17] << 8) + rx[18];
			
				cross_trainer.inst_speed = ((uint16_t)rx[6] << 8) + rx[7];
				cross_trainer.inst_speed *= 10;
		
				cross_trainer.avg_speed = ((uint16_t)rx[19] << 8) + rx[20];
			
				cross_trainer.resistance_level = (int16_t)rx[14] * 10;//level
			
				cross_trainer.step_per_minute = ((uint16_t)rx[8] << 8) + rx[9];
			
//=================================================================================
				rower.stroke_rate = cross_trainer.step_per_minute * 2;
				rower.stroke_count = ((uint16_t)rx[2] << 8) + rx[3];
				rower.inst_power = cross_trainer.inst_power;
			
				rower.inst_pace = ((uint16_t)rx[6] << 8) + rx[7];
			
				rower.resistance_level = rx[14];
				
				rower.total_energy = cross_trainer.total_energy;

				rower.total_distance = cross_trainer.total_distance;
			
				rower.heart_rate = cross_trainer.heart_rate;

//=================================================================================
				bike.inst_power = cross_trainer.inst_power;
				bike.inst_speed = cross_trainer.inst_speed;
			
				bike.inst_cadence = cross_trainer.step_per_minute * 2;
			
				bike.total_energy = cross_trainer.total_energy;
				bike.total_distance = cross_trainer.total_distance;
				
				bike.resistance_level = rx[14];
			
				bike.heart_rate = cross_trainer.heart_rate;
				bike.avg_speed = cross_trainer.avg_speed;
				
//					printf("power:%d, speed:%d, rpm:%d, level:%d, heart:%d\n", bike.inst_power, bike.inst_speed, bike.inst_cadence/2, bike.resistance_level, bike.heart_rate);
//				break;
//				default:break;
//			}
			
		
			if(rx[15]==0)   //15-err  16-tablet on equipment or not
			{
				if(rx[24])//sport
				{
					if(!b_machine_sta_change)
					{
						ftms_machine_status.op_data = STATUS_OP_Start_or_resumed;
						ftms_machine_status.par_len = 0;
						ftms_training_status.status = FTMS_TS_Manual_Mode;
						
						if(!dkn_cmd.sta.b_sport)
						{
							dkn_cmd.sta.b_sport = true;
							b_machine_sta_change = true;
						}
					}
					
					dkn_cmd.ctrl.b.req_start = 0;
				}
				else
				{
					dkn_cmd.ctrl.b.req_stop = 0;
					ftms_machine_status.op_data = STATUS_OP_Stop_or_pause;
					ftms_machine_status.parameter[0] = STATUS_STOP_OR_PAUSE_STOP;
					ftms_machine_status.par_len = 1;
					
					dkn_cmd.ctrl.b.req_start = 1;
					if(dkn_cmd.sta.b_sport)
					{
						dkn_cmd.sta.b_sport = false;
						b_machine_sta_change = true;
					}
				}
			}
			else
			{
				dkn_cmd.ctrl.b.req_stop = 0;
				ftms_machine_status.op_data = STATUS_OP_Stop_or_pause;
				ftms_machine_status.parameter[0] = STATUS_STOP_OR_PAUSE_STOP;
				ftms_machine_status.par_len = 1;
				
				if(dkn_cmd.sta.b_sport)
				{
					dkn_cmd.sta.b_sport = false;
					b_machine_sta_change = true;
				}
			}
			
			if(!dkn_cmd.sta.b_sport)
			{
				if(cross_trainer.total_energy == 0)
				{
					cross_trainer.energy_per_hour = 0;
					cross_trainer.energy_per_minute = 0;
					rower.energy_per_hour = 0;
					rower.energy_per_minute = 0;
					bike.energy_per_hour = 0;
					bike.energy_per_minute = 0;
				}
			}
	
			
			if(b_machine_sta_change){
				DKN_Send_Ftms_machine_status();
			}
		break;
		case 0xB1:
			dkn_cmd.ctrl.b.is_ftms_app = 0;
		break;
		case 0xB8:
			dkn_cmd.ctrl.b.req_reset = 0;
		break;
		case 0xD8:
			dkn_cmd.ctrl.b.req_start = 0;
		break;
		case 0xBD:
//			if(rx[10] == dkn_cmd.set.level)
			{
				dkn_cmd.ctrl.b.level_changed = 0;
			}
		break;
		
		case 0x70:
			if(dkn_cmd.conn_sta_send_times < 3)
				dkn_cmd.conn_sta_send_times = 0;

		break;
	}
}



/*分钟热量和小时热量的计算

放入250ms

*/
void dkn_cal_dell(void)
{
	uint32_t temp32;
	
	if(dkn_cmd.sta.b_sport) //运动中
	{
		dkn_cmd.sta.run_time ++;  //计时
		
		if(dkn_cmd.sta.run_time >= 120)	//至少30s，才开始计算
		{
			if(dkn_cmd.protc == EWP_Treadmill)
			{
				if((treadmill.total_energy != dkn_cmd.sta.cal_back) && (treadmill.total_energy > dkn_cmd.sta.cal_back))  //有变化且被减数大
				{
					temp32 = treadmill.total_energy - dkn_cmd.sta.cal_back;  //计算差值
					
					temp32 = temp32 * 14400 / dkn_cmd.sta.run_time;		//1小时的热量消耗  =  变化量/时间*一小时的秒数 (14400=3600*4-由于计时是250ms，所以计算时扩大4倍)
					
					treadmill.energy_per_hour = temp32;
					treadmill.energy_per_minute = temp32 / 60;
					
					dkn_cmd.sta.run_time = 0;
					dkn_cmd.sta.cal_back = treadmill.total_energy;
				}
			}
			else
			{
				if((cross_trainer.total_energy != dkn_cmd.sta.cal_back) && (cross_trainer.total_energy > dkn_cmd.sta.cal_back))
				{
					temp32 = cross_trainer.total_energy - dkn_cmd.sta.cal_back;
					
					temp32 = temp32 * 14400 / dkn_cmd.sta.run_time;
					
					cross_trainer.energy_per_hour = temp32;
					cross_trainer.energy_per_minute = temp32 / 60;
					
					dkn_cmd.sta.run_time = 0;
					dkn_cmd.sta.cal_back = cross_trainer.total_energy;
				}
			}
		}
	}
	else
	{
		if(dkn_cmd.protc == EWP_Treadmill)
		{
			dkn_cmd.sta.cal_back = treadmill.total_energy;
		}
		else
		{
			dkn_cmd.sta.cal_back = cross_trainer.total_energy;
		}
		
		dkn_cmd.sta.run_time = 0;
	}
}



uint8_t dkn_get_pulse_from_uart(void)
{
	uint8_t pulse = 0;
	switch(dkn_cmd.protc)
	{
		case EWP_Treadmill:
			pulse = treadmill.heart_rate;
		break;
		case EWP_Cross_Trainer:
//			pulse = cross_trainer.heart_rate;
		break;
		case EWP_Boat:
//			pulse = rower.heart_rate;
		break;
		case EWP_Bike:
			pulse = bike.heart_rate;
		break;
		default:break;
	}
	
	return pulse;
}

void dkn_set_pulse_value(uint8_t pulse)
{
	switch(dkn_cmd.protc)
	{
		case EWP_Treadmill:
			treadmill.heart_rate = pulse;
		break;
		case EWP_Cross_Trainer:
//			cross_trainer.heart_rate = pulse;
		break;
		case EWP_Boat:
//			rower.heart_rate = pulse;
		break;
		case EWP_Bike:
			bike.heart_rate = pulse;
		break;
		default:break;
	}
}


/*==================================================================
* Function	:unsigned char F_CheckoutCal(unsigned char*p)
* Description	:和校验
* Input Para	: void
* Output Para	: void
* Return Value  : void
==================================================================*/
unsigned char EW_CheckoutCal(unsigned char*p)
{
  unsigned char summation=0,temp=0,i;
  temp=*(p+2)+3;
  for(i=0;i<temp;i++)
      summation+=*(p+i);
 return summation;
}

/*==================================================================
* Function	:
* Description	:
* Input Para	: void
* Output Para	: void
* Return Value  : void
==================================================================*/
unsigned char DKN_CheckSum(unsigned char*p,unsigned char length)
{
  unsigned char i;
  unsigned int sum=0;
  for(i=0;i<length;i++)
      sum+=*(p+i);
 return (unsigned char)(sum&0xff);
}

