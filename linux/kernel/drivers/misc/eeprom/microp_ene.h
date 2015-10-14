#ifndef MICROP_ENE_H
#define MICROP_ENE_H

#define PWR_BTN_PRESS  BIT(0)
#define VOL_UP_PRESS   BIT(1)
#define VOL_DOWN_PRESS BIT(2)

void TriggerPadStationPowerOff(void);
bool pad_exist(void);
void microp_recheck_interrupt(void);
#endif
