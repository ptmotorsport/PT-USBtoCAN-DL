#pragma once
#include <Arduino.h>

using fsp_err_t=int;
using IRQn_Type=int;
constexpr int FSP_SUCCESS=0,FSP_INVALID_VECTOR=-1;
constexpr int I2C_MASTER_RATE_STANDARD=100000,I2C_MASTER_ADDR_MODE_7BIT=0;
constexpr int IIC_MASTER_TIMEOUT_MODE_SHORT=1,IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED=1;
enum i2c_master_event_t { I2C_MASTER_EVENT_TX_COMPLETE,I2C_MASTER_EVENT_RX_COMPLETE,I2C_MASTER_EVENT_ABORTED };
struct i2c_master_callback_args_t { const void *p_context; i2c_master_event_t event; };
struct i2c_master_cfg_t {
  int channel,rate,addr_mode,slave,ipl;
  void (*p_callback)(i2c_master_callback_args_t *);
  const void *p_context;
  int rxi_irq,txi_irq,tei_irq,eri_irq;
  void *p_extend;
};
struct iic_master_extended_cfg_t {
  int timeout_mode,timeout_scl_low;
  struct { uint8_t brl_value,brh_value,cks_value; } clock_settings;
};
struct iic_master_instance_ctrl_t { int unused; };
fsp_err_t R_IOPORT_PinCfg(int *,unsigned,uint32_t);
fsp_err_t R_IIC_MASTER_Open(iic_master_instance_ctrl_t *,const i2c_master_cfg_t *);
fsp_err_t R_IIC_MASTER_Close(iic_master_instance_ctrl_t *);
fsp_err_t R_IIC_MASTER_Abort(iic_master_instance_ctrl_t *);
fsp_err_t R_IIC_MASTER_SlaveAddressSet(iic_master_instance_ctrl_t *,uint32_t,int);
fsp_err_t R_IIC_MASTER_Write(iic_master_instance_ctrl_t *,uint8_t *,uint32_t,bool);
fsp_err_t R_IIC_MASTER_Read(iic_master_instance_ctrl_t *,uint8_t *,uint32_t,bool);
