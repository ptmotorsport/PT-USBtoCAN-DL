#pragma once
#include <r_iic_master.h>
struct I2CIrqReq_t { i2c_master_cfg_t *mcfg; void *scfg; };
constexpr int IRQ_I2C_MASTER=1;
class IRQManager {
public:
  static IRQManager &getInstance() { static IRQManager instance; return instance; }
  unsigned allocations=0;
  bool addPeripheral(int,I2CIrqReq_t *r) {
    ++allocations;
    r->mcfg->rxi_irq=1; r->mcfg->txi_irq=2;
    r->mcfg->tei_irq=3; r->mcfg->eri_irq=4;
    return true;
  }
};
