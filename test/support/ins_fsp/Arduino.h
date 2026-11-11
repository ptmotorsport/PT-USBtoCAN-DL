#pragma once
#include <stddef.h>
#include <stdint.h>

constexpr unsigned SDA=18,SCL=19;
constexpr uint32_t IOPORT_CFG_DRIVE_MID=1,IOPORT_CFG_PERIPHERAL_PIN=2,IOPORT_PERIPHERAL_IIC=4;
struct Pin { unsigned pin; };
static const Pin g_pin_cfg[20]={};
static int g_ioport_ctrl=0;
unsigned long micros();
