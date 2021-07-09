
#include <stdint.h>
#include <string.h>
#include "app_error.h"
#include "app_timer.h"
#include "app_uart.h"
#include "bsp.h"

#include "m_system.h"
#include "dkn_cmd.h"
#include "ble_dis.h"


#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define UART_TX_BUF_SIZE                128                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                64                                         /**< UART RX buffer size. */

#define UART_RXEND_Delay_INTERVAL     	APP_TIMER_TICKS(20)
#define UART_SEND_Delay_INTERVAL     		APP_TIMER_TICKS(250)
#define UART_AFTER_NUS_INTERVAL     		APP_TIMER_TICKS(50)

struct{
	uint8_t Data[UART_RX_BUF_SIZE];
	uint8_t index;
	
	uint8_t dkn_buf[UART_RX_BUF_SIZE];
	uint8_t dkn[UART_RX_BUF_SIZE];
	uint8_t dkn_index;
	uint8_t dkn_len;
	uint8_t dkn_target_len;
//	uint8_t sData[UART_RX_BUF_SIZE];
//	uint8_t sadd;
	
	uint8_t send_time_cnt;
	uint8_t wait_time;

}Uart;


APP_TIMER_DEF(m_timer_uart_rx_end_id);
APP_TIMER_DEF(m_timer_uart_send_id);
APP_TIMER_DEF(m_timer_send_cmd_after_nus_id);


bool b_uart_rx_dkn_cmd_ok = false;

extern volatile uint16_t  m_conn_handle_peripheral;

void A0_CMD_Dell(void);
void uart_send_conn_sta(void);
void uart_send_hr(void);

extern uint32_t DKN_nus_Send(uint8_t* buf, uint16_t len);
extern unsigned char DKN_CheckSum(unsigned char*p,unsigned char length);
extern int8_t mflash_write(void);
extern int8_t mflash_rewrite(void);

extern unsigned char EW_CheckoutCal(unsigned char*p);

static void timer_uart_rx_end_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	app_timer_stop(m_timer_uart_rx_end_id);
	
	if(Uart.index)
	{
		NRF_LOG_INFO("URX:%d", Uart.index);
		if(Uart.Data[0] == 0xA0)
		{
			A0_CMD_Dell();
		}
		else
		{
			Uart.dkn_len = Uart.index;
			memcpy(Uart.dkn, Uart.Data, Uart.dkn_len);
			b_uart_rx_dkn_cmd_ok = true;
		}
		Uart.index = 0;
	}
	
//	Uart.sadd = 0;
}


static void timer_uart_send_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);

	Uart.send_time_cnt ++;
	
	if((Uart.send_time_cnt & 0x01) == 0)
	{
		Uart.send_time_cnt = 0;

		if(Uart.wait_time)
		{
			Uart.wait_time --;
		}
		else
		{
			dkn_cmd.ctrl.b.enable_uart_tx_auto = 1;
		}
		

		if(dkn_cmd.ctrl.b.enable_uart_tx_auto)				//如果允许自动发送
		{
			dkn_cmd_send();
		}
	}
	else if(dkn_cmd.ctrl.b.enable_uart_tx_auto)
	{
		if(dkn_cmd.conn_sta_send_times)
		{
			dkn_cmd.conn_sta_send_times--;
			uart_send_conn_sta();
		}
		else if(dkn_cmd.hr_send_times)
		{
			dkn_cmd.hr_send_times --;
			uart_send_hr();
		}
		else
		{
			dkn_send_ctrl_cmd();
		}
	}
	
	
	dkn_cal_dell();
}


static void timer_send_cmd_after_nus_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	
	if(dkn_cmd.conn_sta_send_times)
	{
		dkn_cmd.conn_sta_send_times--;
		uart_send_conn_sta();
	}
	else if(dkn_cmd.hr_send_times)
	{
		dkn_cmd.hr_send_times --;
		uart_send_hr();
	}
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
//    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
//    static uint8_t index = 0;
//    uint32_t       err_code;
		uint8_t data;
		uint8_t sta[3];

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
					
					if(Uart.index >= UART_RX_BUF_SIZE)
						Uart.index = 0;
					
					UNUSED_VARIABLE(app_uart_get(&data));
					
					if(b_uart_rx_dkn_cmd_ok)
					{
						Uart.index = 0;
						Uart.dkn_index = 0;
						return;
					}
					
					Uart.Data[Uart.index] = data;
					Uart.index++;
					
					Uart.dkn_buf[Uart.dkn_index] = data;

					app_timer_stop(m_timer_uart_rx_end_id);
						
					app_timer_start(m_timer_uart_rx_end_id, UART_RXEND_Delay_INTERVAL, NULL);

					
					if(Uart.dkn_buf[0] == 0xF0)
					{
						Uart.dkn_index++;
						if(Uart.dkn_index == 2)
						{
							Uart.dkn_target_len = dkn_get_Console_cmd_len(Uart.dkn_buf[1]);
							if(Uart.dkn_target_len == 0)
								Uart.dkn_index = 0;
						}
						else if(Uart.dkn_index > 2)
						{
							if(Uart.dkn_index >= Uart.dkn_target_len)
							{
								Uart.dkn_len = Uart.dkn_index;
								memcpy(Uart.dkn, Uart.dkn_buf, Uart.dkn_len);
								b_uart_rx_dkn_cmd_ok = true;
								
								Uart.index = 0;
								Uart.dkn_index = 0;
								
								app_timer_stop(m_timer_uart_rx_end_id);
							}
						}
					}
					else
						Uart.dkn_index = 0;
            break;

        case APP_UART_COMMUNICATION_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
//.rx_pin_no    = 8,
//.tx_pin_no    = 6,
        .rx_pin_no    = 29,
        .tx_pin_no    = 28,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
		
		//rx timeout timer
		err_code = app_timer_create(&m_timer_uart_rx_end_id,   
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_uart_rx_end_handler);
		APP_ERROR_CHECK(err_code);
		//auto send cmd timer
		err_code = app_timer_create(&m_timer_uart_send_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_uart_send_handler);
		APP_ERROR_CHECK(err_code);
		//send cmd after nus write timer
		err_code = app_timer_create(&m_timer_send_cmd_after_nus_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_send_cmd_after_nus_handler);
		APP_ERROR_CHECK(err_code);
		
//=============================================================			
		app_timer_start(m_timer_uart_send_id, UART_SEND_Delay_INTERVAL, NULL);
		
		Uart.wait_time = 12;
		
		NRF_LOG_INFO("uart init\n");
}


void uart_send_array(const uint8_t* buf, uint8_t len)
{
	uint32_t err_code;
	if(len > UART_TX_BUF_SIZE)len = UART_TX_BUF_SIZE;
	for (uint32_t i = 0; i < len; i++)
	{
			do
			{
					err_code = app_uart_put(buf[i]);
//					if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
//					{
//							NRF_LOG_ERROR("Uart Error 0x%x. ", err_code);
//							APP_ERROR_CHECK(err_code);
//					}
			} while (err_code == NRF_ERROR_BUSY);
	}

}


void uart_send_cmd_after_nus_write(void)
{
	if(dkn_cmd.conn_sta_send_times || dkn_cmd.hr_send_times)
	{
		app_timer_start(m_timer_send_cmd_after_nus_id, UART_AFTER_NUS_INTERVAL, NULL);
	}
}


void uart_auto_hold(void)
{
	Uart.send_time_cnt = 0;

	Uart.wait_time = 12;

	dkn_cmd.ctrl.b.enable_uart_tx_auto = 0;
}


void uart_conn_sta_changed(void)
{
	dkn_cmd.conn_sta_send_times = 3;
}

void uart_has_hr_recv(uint8_t times)
{
	dkn_cmd.hr_send_times = times;
}


void uart_send_conn_sta(void)
{
	uint8_t tx_buf[10]={0};
	uint8_t temp = 0;

	tx_buf[0] = 0xF0;
	tx_buf[1] = 0x60;
	
	if(Sys.status.ble_slave_is_connected)
		temp |= 0x01;
	if(Sys.status.ble_master_is_connected)
		temp |= 0x02;
	if(Sys.status.ant_is_connected)
		temp |= 0x04;
	
	tx_buf[2] = temp;
	tx_buf[3] =  DKN_CheckSum(tx_buf, 3);

	uart_send_array(tx_buf, 4);
	
}

void uart_send_hr(void)
{
	uint8_t tx_buf[10]={0};

	tx_buf[0] = 0xF0;
	tx_buf[1] = 0x80;
	if(Ant_Hrm.pulse)
	{
		tx_buf[2] = 2;
		tx_buf[3] = Ant_Hrm.pulse;
	}
	else if(Ble_Hr.pulse)
	{
		tx_buf[2] = 1;
		tx_buf[3] = Ble_Hr.pulse;
	}
	else
	{
		tx_buf[2] = 0;
		tx_buf[3] = 0;
	}
	tx_buf[4] = DKN_CheckSum(tx_buf, 4);
	
	uart_send_array(tx_buf, 5);
	
}


/**
A0命令处理
*/
void A0_CMD_Dell(void)
{
//	show_reg(Uart.Data, Uart.rx_index, 1);
	uint8_t tx_buf[50]={0};
	uint8_t temp,addr;
	
	if(Uart.Data[Uart.Data[2]+3] == EW_CheckoutCal(Uart.Data))//校验
	{
		NRF_LOG_INFO("A0 cmd:%02X\n", Uart.Data[1]);
		
		switch(Uart.Data[1])
		{
			case 0x50://读名称
				tx_buf[0] = 0xA0;
				tx_buf[1] = 0x60;
				tx_buf[2] = my_flash.name_len;
				memcpy(&tx_buf[3], my_flash.device_name, my_flash.name_len);
				tx_buf[3 + my_flash.name_len] = EW_CheckoutCal(tx_buf);
				
				uart_send_array(tx_buf, 4 + my_flash.name_len);
			break;
			case 0x51://写名称
				if(Uart.Data[2])
				{
					my_flash.name_len = Uart.Data[2];
					if(my_flash.name_len > 26)
						my_flash.name_len = 26;
					memset(my_flash.device_name, 0, sizeof(my_flash.device_name));
					memcpy(my_flash.device_name, &Uart.Data[3], my_flash.name_len);
					
					
					switch(dkn_get_deivce_type_from_name(my_flash.device_name))
					{
						case EWP_Treadmill:
							my_flash.ftms.type.value = 0;
							my_flash.ftms.type.b.treadmill_support = 1;
						break;
						case EWP_Cross_Trainer:
							my_flash.ftms.type.value = 0;
							my_flash.ftms.type.b.cross_trainer_support = 1;
						break;
						case EWP_Step_Climber:
							my_flash.ftms.type.value = 0;
							my_flash.ftms.type.b.step_climber_support = 1;
						break;
						case EWP_Stair_Climber:
							my_flash.ftms.type.value = 0;
							my_flash.ftms.type.b.stair_climber_support = 1;
						break;
						case EWP_Boat:
							my_flash.ftms.type.value = 0;
							my_flash.ftms.type.b.rower_support = 1;
						break;
						case EWP_Bike:
							my_flash.ftms.type.value = 0;
							my_flash.ftms.type.b.indoor_bike_support = 1;
						break;
						case EWP_Others:break;
					}
					
					if(mflash_write() == 0)//如果flash重写了
					{
							Sys.ctrl.req_reset_after_flash_ok = true;
						
					}
					
					tx_buf[0] = 0xA0;
					tx_buf[1] = 0x61;
					tx_buf[2] = 1;
					tx_buf[3] = 0;
					tx_buf[4] = EW_CheckoutCal(tx_buf);
					
					uart_send_array(tx_buf, 5);
				}
			break;
			case 0x52://读器材类型
				tx_buf[0] = 0xA0;
				tx_buf[1] = 0x62;
				tx_buf[2] = 1;
				tx_buf[3] = my_flash.ftms.type.value;
				tx_buf[4] = EW_CheckoutCal(tx_buf);
			
				uart_send_array(tx_buf, 5);
			break;
			case 0x56://写器材类型
				
				tx_buf[3] = 2;
			
				if(Uart.Data[2] == 1)//长度固定为1
				{
					u8_flag_def		flag;
//					uint8_t i,num;
					
					flag.byte = Uart.Data[3];
					

					if(flag.byte != 0)
					{
						my_flash.ftms.enable = 1;
						my_flash.ftms.type.value = 0;
						if(flag.b.b0)
							my_flash.ftms.type.b.treadmill_support = 1;
						else if(flag.b.b1)
							my_flash.ftms.type.b.cross_trainer_support = 1;
						else if(flag.b.b2)
							my_flash.ftms.type.b.step_climber_support = 1;
						else if(flag.b.b3)
							my_flash.ftms.type.b.stair_climber_support = 1;
						else if(flag.b.b4)
							my_flash.ftms.type.b.rower_support = 1;
						else if(flag.b.b5)
							my_flash.ftms.type.b.indoor_bike_support = 1;
					}
					else
					{
						my_flash.ftms.type.value = 0;
						my_flash.ftms.enable = 0;
					}
				}
				
				if(mflash_write() == 0)  //如果flash重写了，则重启芯片
				{
					Sys.ctrl.req_reset_after_flash_ok = true;
							
				}
				
				tx_buf[0] = 0xA0;
				tx_buf[1] = 0x63;
				tx_buf[2] = 1;
				tx_buf[3] = 0;
				tx_buf[4] = EW_CheckoutCal(tx_buf);
				
				uart_send_array(tx_buf, 5);
			break;
			case 0x54: //设置/读取广播和扫描开关
				if(Uart.Data[2] == 0x01)
				{
					if(Uart.Data[3] & 0x80)//设置有效
					{
						if(Uart.Data[3] & 0x01)
							Sys.ctrl.ble_adv_enable = 1;
						else
							Sys.ctrl.ble_adv_enable = 0;
						
						if(Uart.Data[3] & 0x02)
							Sys.ctrl.ble_scan_enable = 1;
						else
							Sys.ctrl.ble_scan_enable = 0;
					
						if(Uart.Data[3] & 0x20)
						{
							system_reset_after_ms(500);
						}	
					}
					
					tx_buf[0] = 0xA0;
					tx_buf[1] = 0x64;
					tx_buf[2] = 1;
			
					tx_buf[3] = (Sys.status.ant_is_connected << 4) | (Sys.status.ble_master_is_connected << 3) | (Sys.status.ble_slave_is_connected << 2) | (Sys.status.ble_is_scaning << 1) | (Sys.status.ble_is_adving);
					
					tx_buf[4] = EW_CheckoutCal(tx_buf);
					uart_send_array(tx_buf, 5);
				}
			break;
			case 0x5E:	//恢复出厂设置
				if((Uart.Data[3]==0x12) && (Uart.Data[4] == 0x34))
				{
					mflash_rewrite();

					tx_buf[0] = 0xA0;
					tx_buf[1] = 0x6E;
					tx_buf[2] = 1;
					tx_buf[3] = 0;
					tx_buf[4] = EW_CheckoutCal(tx_buf);
					
					uart_send_array(tx_buf, 5);
					
					Sys.ctrl.req_reset_after_flash_ok = true;
					
				}
			break;
			case 0x93: //读FTMS相关信息
				tx_buf[0] = 0xA0;
				tx_buf[1] = 0xA3;

				addr = 3;
				addr += uint32_encode(my_flash.ftms.feature.fitness_machine, &tx_buf[addr]);
			
				addr += uint32_encode(my_flash.ftms.feature.target_setting, &tx_buf[addr]);
			
				memcpy(&tx_buf[addr], &my_flash.ftms.range.speed, sizeof(my_flash.ftms.range.speed));
				addr += sizeof(my_flash.ftms.range.speed);
			
				memcpy(&tx_buf[addr], &my_flash.ftms.range.incline, sizeof(my_flash.ftms.range.incline));
				addr += sizeof(my_flash.ftms.range.incline);
			
				memcpy(&tx_buf[addr], &my_flash.ftms.range.power, sizeof(my_flash.ftms.range.power));
				addr += sizeof(my_flash.ftms.range.power);
			
				memcpy(&tx_buf[addr], &my_flash.ftms.range.resist, sizeof(my_flash.ftms.range.resist));
				addr += sizeof(my_flash.ftms.range.resist);
			
				memcpy(&tx_buf[addr], &my_flash.ftms.range.heartrate, sizeof(my_flash.ftms.range.heartrate));
				addr += sizeof(my_flash.ftms.range.heartrate);
				
				tx_buf[2] = addr - 3;
				
				tx_buf[addr] = EW_CheckoutCal(tx_buf);
					
				uart_send_array(tx_buf, addr+1);

			break;
			case 0x94:	//设置FTMS相关信息
				if(Uart.Data[2] == 35)
				{
					addr = 3;
					memcpy(&my_flash.ftms.feature.fitness_machine, &Uart.Data[addr], 4);
					addr += 4;
					memcpy(&my_flash.ftms.feature.target_setting, &Uart.Data[addr], 4);
					addr += 4;
					
					memcpy(&my_flash.ftms.range.speed, &Uart.Data[addr], 6);
					addr += 6;
					memcpy(&my_flash.ftms.range.incline, &Uart.Data[addr], 6);
					addr += 6;
					memcpy(&my_flash.ftms.range.power, &Uart.Data[addr], 6);
					addr += 6;
					memcpy(&my_flash.ftms.range.resist, &Uart.Data[addr], 6);
					addr += 6;
					memcpy(&my_flash.ftms.range.heartrate, &Uart.Data[addr], 3);
					
					if(mflash_write() == 0)  //如果flash重写了，则重启芯片
					{
						Sys.ctrl.req_reset_after_flash_ok = true;
					}
					
					NRF_LOG_INFO("featus:%08X, %08X\n", my_flash.ftms.feature.fitness_machine, my_flash.ftms.feature.target_setting);
					
					tx_buf[0] = 0xA0;
					tx_buf[1] = 0xA4;
					tx_buf[2] = 1;
					tx_buf[3] = 0;
					tx_buf[4] = EW_CheckoutCal(tx_buf);
					
					uart_send_array(tx_buf, 5);
				}
			break;
				
			case 0x95:	//设置厂家名称及机型名称字符串
				
				temp = 0;
				addr = 4;
				if((Uart.Data[3] > 0) && (Uart.Data[3] <= 20))
				{
					my_flash.ftms.string.dis_manufacturer_name.Length = Uart.Data[3];
					memset(my_flash.ftms.string.dis_manufacturer_name.Value, 0, sizeof(my_flash.ftms.string.dis_manufacturer_name.Value));
					memcpy(my_flash.ftms.string.dis_manufacturer_name.Value, &Uart.Data[4], my_flash.ftms.string.dis_manufacturer_name.Length);
					addr += my_flash.ftms.string.dis_manufacturer_name.Length;
					
					
					if((Uart.Data[addr] > 0) && (Uart.Data[addr] <= 20))
					{
						my_flash.ftms.string.dis_model_nb.Length = Uart.Data[addr];
						memset(my_flash.ftms.string.dis_model_nb.Value, 0, sizeof(my_flash.ftms.string.dis_model_nb.Value));
						memcpy(my_flash.ftms.string.dis_model_nb.Value, &Uart.Data[addr + 1], my_flash.ftms.string.dis_model_nb.Length);
						
						if(mflash_write() == 0)
						{
							Sys.ctrl.req_reset_after_flash_ok = true;
						}
					}
					else
						temp = 1;
				}
				else
					temp = 1;

				tx_buf[0] = 0xA0;
				tx_buf[1] = 0xA5;
				tx_buf[2] = 1;
				tx_buf[3] = temp;
				tx_buf[4] = EW_CheckoutCal(tx_buf);
				
				uart_send_array(tx_buf, 5);
			break;
		}
	}
}



void dkn_cmd_dell_in_main_loop(void)
{
	uint16_t length;
	if(b_uart_rx_dkn_cmd_ok)
	{
		if(Uart.dkn_len > 20)
		{
			length = 20;
			DKN_nus_Send(Uart.dkn, length);
			length = Uart.dkn_len-20;
			DKN_nus_Send(&Uart.dkn[20], length);
		}
		else
		{
			DKN_nus_Send(Uart.Data, Uart.dkn_len);
		}

		dkn_cmd_recv_data(Uart.dkn, Uart.dkn_len);
		
		b_uart_rx_dkn_cmd_ok = false;
	}
}

