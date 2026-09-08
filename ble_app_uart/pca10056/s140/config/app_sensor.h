

#ifndef __APP_SENSOR_H__
#define __APP_SENSOR_H__

#include "stdint.h"

#define FUNC_CFG_ACCESS 			0x01
#define EMB_FNC_REG_ACCESS 			(1<<7)
#define SHUB_REG_ACCESS 			(1<<6)
#define FSM_WR_CTRL_EN				(1<<3)
#define SW_POR						(1<<2)	
#define SPI2_RESET					(1<<1)
#define OIS_CTRL_FROM_UI			(1<<0)
		
#define PAGE_RW       0x17

#define PAGE_SEL      0x02

#define PAGE_ADDR     0x08

#define PAGE_DATA     0x09		
		
#define PIN_CTRL					0x02
#define OIS_PU_DIS 					(1<<7)
#define SDO_PU_EN 					(1<<6)
#define IBHR_POR_EN					(1<<3)
		
#define IF_CFG 						0x03
#define SDA_PU_EN 					(1<<7)
#define SHUB_PU_EN 					(1<<6)
#define ASF_CTRL 					(1<<5)
#define H_LACTIVE 					(1<<4)
#define PP_OD						(1<<3)
#define SIM							(1<<2)	
#define I2C_I3C_disable				(1<<0)
		
#define EMB_FUNC_EN_A                           0x04
#define EMB_FUNC_INIT_A                         0x66
#define SFLP_ODR                                0x5e
#define TDM_CFG1                                0x6D
#define EMB_FUNC_FIFO_EN_A                      0x44

#define ODR_TRIG_CFG 						0x06
		
#define FIFO_CTRL1 					0x07
		
#define FIFO_CTRL2 					0x08
		
#define STOP_ON_WTM 				(1<<7)
#define FIFO_COMPR_RT_EN 			(1<<6)
#define ODR_CHG_EN 					(1<<4)
#define PP_OD						(1<<3)
#define UNCOMPR_RATE_1				(1<<2)	
#define UNCOMPR_RATE_0				(1<<1)	
#define XL_DualC_BATCH_FROM_FSM		(1<<0)

#define FIFO_CTRL3 					0x09
#define BDR_GY_						0xF0
#define BDR_XL_						0x0f

#define FIFO_CTRL4 					0x0A
#define DEC_TS_BATCH_1 				(1<<7)
#define DEC_TS_BATCH_0	 			(1<<6)
#define ODR_T_BATCH_1 				(1<<5)
#define ODR_T_BATCH_0 				(1<<4)
#define G_EIS_FIFO_EN				(1<<3)
#define FIFO_MODE_2					(1<<2)	
#define FIFO_MODE_1					(1<<1)	
#define FIFO_MODE_0					(1<<0)

#define COUNTER_BDR_REG1 			0x0B
#define TRIG_COUNTER_BDR_1	 		(1<<6)
#define TRIG_COUNTER_BDR_0 			(1<<5)
#define CNT_BDR_TH_9				(1<<1)	
#define CNT_BDR_TH_8				(1<<0)

#define COUNTER_BDR_REG2 			0x0C

#define INT1_CTRL 					0x0D
#define INT1_CNT_BDR	 			(1<<6)
#define INT1_FIFO_FULL 				(1<<5)
#define INT1_FIFO_OVR 				(1<<4)
#define INT1_FIFO_TH				(1<<3)
#define INT1_DRDY_G					(1<<1)	
#define INT1_DRDY_XL				(1<<0)

#define INT2_CTRL 					0x0E
#define INT2_EMB_FUNC_ENDOP 		(1<<7)
#define INT2_CNT_BDR	 			(1<<6)
#define INT2_FIFO_FULL 				(1<<5)
#define INT2_FIFO_OVR 				(1<<4)
#define INT2_FIFO_TH				(1<<3)
#define INT2_DRDY_G_EIS				(1<<2)	
#define INT2_DRDY_G					(1<<1)	
#define INT2_DRDY_XL				(1<<0)


#define WHO_AM_I 					0x0F


#define CTRL1 					0x10

#define CTRL2 					0x11

#define CTRL3 					0x12

#define CTRL4 					0x13

#define CTRL5					0x14
#define CTRL6 					0x15
#define CTRL7 					0x16
#define CTRL8 					0x17
#define CTRL9 					0x18
#define CTRL10 					0x19
#define CTRL_STATUS 			0x1A

#define FIFO_STATUS1 			0x1B
#define FIFO_STATUS2			0x1C



#define OUTX_L_A                        0x28

#define FIFO_DATA_OUT_TAG               0x78
void Sensor_Get_Id(void);
uint16_t Sensor_fifo_data_size(void);
void Sensor_init(void);
void sensor_enable(void);
void Sensor_update(void);
#endif
