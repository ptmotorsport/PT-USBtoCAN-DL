#pragma once

#include <Arduino.h>
#include <IRQManager.h>
#include <r_iic_master.h>

class InsI2C {
public:
  enum class Error : uint8_t { None, Aborted, Timeout, Driver, NotOpen };
  bool begin() {
    if (opened_) return true;
    config_.channel=1; // Uno R4 Minima A4/P101 and A5/P100 use IIC1.
    config_.rate=I2C_MASTER_RATE_STANDARD;
    config_.addr_mode=I2C_MASTER_ADDR_MODE_7BIT;
    config_.slave=0;
    config_.ipl=12;
    config_.p_callback=callback;
    config_.p_context=this;
    config_.rxi_irq=config_.txi_irq=config_.tei_irq=config_.eri_irq=FSP_INVALID_VECTOR;
    extension_.timeout_mode=IIC_MASTER_TIMEOUT_MODE_SHORT;
    extension_.timeout_scl_low=IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED;
    extension_.clock_settings.brl_value=27;
    extension_.clock_settings.brh_value=26;
    extension_.clock_settings.cks_value=2;
    config_.p_extend=&extension_;
    const uint32_t pins=IOPORT_CFG_DRIVE_MID|IOPORT_CFG_PERIPHERAL_PIN|IOPORT_PERIPHERAL_IIC;
    if (R_IOPORT_PinCfg(&g_ioport_ctrl,g_pin_cfg[SDA].pin,pins)!=FSP_SUCCESS ||
        R_IOPORT_PinCfg(&g_ioport_ctrl,g_pin_cfg[SCL].pin,pins)!=FSP_SUCCESS) return driverError();
    if (!irqsAssigned_) {
      I2CIrqReq_t request={&config_,nullptr};
      if (!IRQManager::getInstance().addPeripheral(IRQ_I2C_MASTER,&request)) return driverError();
      irqsAssigned_=true;
      irq_[0]=config_.rxi_irq; irq_[1]=config_.txi_irq;
      irq_[2]=config_.tei_irq; irq_[3]=config_.eri_irq;
    } else {
      config_.rxi_irq=irq_[0]; config_.txi_irq=irq_[1];
      config_.tei_irq=irq_[2]; config_.eri_irq=irq_[3];
    }
    if (R_IIC_MASTER_Open(&control_,&config_)!=FSP_SUCCESS) return driverError();
    opened_=true; error=Error::None;
    return true;
  }
  bool read(uint8_t address,uint8_t reg,uint8_t *bytes,size_t size) {
    if (!select(address)) return false;
    event_=0;
    if (!complete(R_IIC_MASTER_Write(&control_,&reg,1,true),I2C_MASTER_EVENT_TX_COMPLETE)) return false;
    event_=0;
    return complete(R_IIC_MASTER_Read(&control_,bytes,size,false),I2C_MASTER_EVENT_RX_COMPLETE);
  }
  bool write(uint8_t address,uint8_t reg,uint8_t value) {
    if (!select(address)) return false;
    uint8_t bytes[2]={reg,value};
    event_=0;
    return complete(R_IIC_MASTER_Write(&control_,bytes,2,false),I2C_MASTER_EVENT_TX_COMPLETE);
  }
  bool ready() const { return opened_; }
  Error error=Error::None;
  uint32_t failures=0,maxTransactionUs=0;
private:
  iic_master_instance_ctrl_t control_={};
  i2c_master_cfg_t config_={};
  iic_master_extended_cfg_t extension_={};
  IRQn_Type irq_[4]={};
  volatile uint32_t event_=0;
  bool opened_=false,irqsAssigned_=false;
  static void callback(i2c_master_callback_args_t *args) {
    auto *self=static_cast<InsI2C *>(const_cast<void *>(args->p_context));
    self->event_=static_cast<uint32_t>(args->event)+1;
  }
  bool driverError() { ++failures; error=Error::Driver; return false; }
  bool select(uint8_t address) {
    if (!opened_) { error=Error::NotOpen; return false; }
    if (R_IIC_MASTER_SlaveAddressSet(&control_,address,I2C_MASTER_ADDR_MODE_7BIT)!=FSP_SUCCESS)
      return driverError();
    return true;
  }
  bool complete(fsp_err_t result,i2c_master_event_t expected) {
    const uint32_t start=micros();
    if (result==FSP_SUCCESS) {
      while (event_==0 && micros()-start<5000) {}
    }
    const uint32_t elapsed=micros()-start;
    if (elapsed>maxTransactionUs) maxTransactionUs=elapsed;
    if (result==FSP_SUCCESS && event_==static_cast<uint32_t>(expected)+1) {
      error=Error::None; return true;
    }
    ++failures;
    error=result!=FSP_SUCCESS?Error::Driver:event_==0?Error::Timeout:Error::Aborted;
    const fsp_err_t aborted=R_IIC_MASTER_Abort(&control_);
    if (error!=Error::Aborted || aborted!=FSP_SUCCESS) {
      // Stop using a failed controller; explicit SENSOR INIT is required.
      R_IIC_MASTER_Close(&control_);
      opened_=false;
    }
    return false;
  }
};
