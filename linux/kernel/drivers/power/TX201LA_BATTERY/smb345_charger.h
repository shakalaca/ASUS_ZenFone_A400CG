#ifndef _SMB345_CONTROL_H_
#define _SMB345_CONTROL_H_

int smb345_charger_toggle(bool on);
void smb345_dump_registers(struct seq_file *s);
void smb345_config_max_current_twinheadeddragon(int inok_gpio);
#endif
