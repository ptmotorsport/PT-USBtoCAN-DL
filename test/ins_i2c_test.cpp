#include "ins_i2c.h"
#include <assert.h>
#include <stdio.h>

namespace {
uint32_t now=0;
const i2c_master_cfg_t *config=nullptr;
bool timeOut=false,abortEvent=false,driverError=false;
unsigned closes=0,aborts=0,writes=0,reads=0;
void complete(i2c_master_event_t event) {
  if (timeOut || driverError) return;
  i2c_master_callback_args_t args={config->p_context,abortEvent?I2C_MASTER_EVENT_ABORTED:event};
  config->p_callback(&args);
}
}
unsigned long micros() { now+=100; return now; }
fsp_err_t R_IOPORT_PinCfg(int *,unsigned,uint32_t flags) {
  assert(flags==(IOPORT_CFG_DRIVE_MID|IOPORT_CFG_PERIPHERAL_PIN|IOPORT_PERIPHERAL_IIC));
  return FSP_SUCCESS;
}
fsp_err_t R_IIC_MASTER_Open(iic_master_instance_ctrl_t *,const i2c_master_cfg_t *c) {
  config=c;
  assert(c->channel==1);
  assert(c->rxi_irq==1 && c->eri_irq==4 && c->rate==100000);
  return FSP_SUCCESS;
}
fsp_err_t R_IIC_MASTER_Close(iic_master_instance_ctrl_t *) { ++closes; return FSP_SUCCESS; }
fsp_err_t R_IIC_MASTER_Abort(iic_master_instance_ctrl_t *) { ++aborts; return FSP_SUCCESS; }
fsp_err_t R_IIC_MASTER_SlaveAddressSet(iic_master_instance_ctrl_t *,uint32_t address,int) {
  assert(address==0x68);
  return FSP_SUCCESS;
}
fsp_err_t R_IIC_MASTER_Write(iic_master_instance_ctrl_t *,uint8_t *,uint32_t,bool) {
  ++writes; complete(I2C_MASTER_EVENT_TX_COMPLETE);
  return driverError?1:FSP_SUCCESS;
}
fsp_err_t R_IIC_MASTER_Read(iic_master_instance_ctrl_t *,uint8_t *bytes,uint32_t n,bool restart) {
  assert(!restart);
  ++reads;
  for (uint32_t i=0;i<n;++i) bytes[i]=static_cast<uint8_t>(i);
  complete(I2C_MASTER_EVENT_RX_COMPLETE);
  return FSP_SUCCESS;
}

int main() {
  InsI2C bus;
  uint8_t bytes[14]={};
  assert(!bus.read(0x68,0x3B,bytes,sizeof(bytes)));
  assert(bus.error==InsI2C::Error::NotOpen);
  assert(bus.begin());
  assert(bus.read(0x68,0x3B,bytes,sizeof(bytes)));
  assert(bytes[13]==13 && writes==1 && reads==1 && aborts==0);
  assert(bus.write(0x68,0x6B,1));
  abortEvent=true;
  assert(!bus.read(0x68,0x3B,bytes,sizeof(bytes)));
  assert(bus.error==InsI2C::Error::Aborted && bus.ready() && aborts==1 && closes==0);
  abortEvent=false; timeOut=true;
  const uint32_t start=now;
  assert(!bus.read(0x68,0x3B,bytes,sizeof(bytes)));
  assert(bus.error==InsI2C::Error::Timeout && !bus.ready());
  assert(now-start<=5500 && closes==1 && aborts==2);
  const unsigned previous=writes;
  assert(!bus.read(0x68,0x3B,bytes,sizeof(bytes)));
  assert(writes==previous);
  timeOut=false;
  assert(bus.begin());
  assert(IRQManager::getInstance().allocations==1);
  assert(bus.read(0x68,0x3B,bytes,sizeof(bytes)));
  driverError=true;
  assert(!bus.write(0x68,0x6B,1));
  assert(bus.error==InsI2C::Error::Driver && !bus.ready() && closes==2);
  assert(bus.failures==3);
  puts("ins_i2c_test: PASS");
}
