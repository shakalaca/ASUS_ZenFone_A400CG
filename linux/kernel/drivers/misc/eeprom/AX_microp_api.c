#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/microp.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#include <linux/microp_notify.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <microp_ene.h>

extern int uP_i2c_read_reg(int cmd, void *data);
extern int uP_i2c_write_reg(int cmd, void *data);
extern int isFirmwareUpdating(void);
extern void msleep(unsigned int msecs);
extern unsigned int g_b_isP01Connected;
extern unsigned int g_i2c_bus_suspended;
extern unsigned int g_i2c_microp_busy;
unsigned char isregister = false;
uint8_t speaker_en = 0;
uint8_t recevier_en = 0;

void AX_MicroP_Bus_Suspending(int susp){
    if(susp)
        g_i2c_bus_suspended=1;
    else
        g_i2c_bus_suspended=0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_Bus_Suspending);

/*
*       Check the status of P01 connectness
*       return value: 1: P01 connected
*/
int AX_MicroP_IsP01Connected(void){
       return (g_b_isP01Connected==1 || g_b_isP01Connected==3)?1:0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_IsP01Connected);

/*
*       Check the status of AC/USB if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err
*/
int AX_MicroP_IsACUSBIn(void){
    int pin=-1;
    pin=AX_MicroP_getGPIOPinLevel(IN_AC_USB_IN);
    if(pin<0)
        return pin;
    return (pin==1)?1:0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_IsACUSBIn);

/*
*   @AX_MicroP_get_ChargingStatus
*  input: target
*           0: p01 battery
*           1: dock battery
*    return: 0 for 'no charging', 1 for 'charging', 2 for 'charged full', <0 value means something error
*/
int AX_MicroP_get_ChargingStatus(int target){
    int regval=0;
    int ret=0;

    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }

    if(isFirmwareUpdating()){
        printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
        return -2;
    }

    if(target==Batt_P01){
        // redundant read to wakeup microp from power-down mode and trigger microp to read h/w gauge again
        ret=uP_i2c_read_reg(MICROP_CHARGING_STATUS, &regval);
        ret=uP_i2c_read_reg(MICROP_CHARGING_STATUS, &regval);
                                
        if(ret <= 0 || regval==255)
            regval=P01_CHARGING_ERR;
        else if(regval==0)
            regval=P01_CHARGING_NO;
        else if(regval==1)
            regval=P01_CHARGING_ONGOING;
        else if(regval==2)
            regval=P01_CHARGING_FULL;
        //printk("%s: charging status: %d\r\n",__FUNCTION__,  regval);
    }
    else{
        printk(KERN_ERR "%s: known target %d\r\n", __FUNCTION__, target);
    }

    return regval;
}

/*
*   @AX_MicroP_get_USBDetectStatus
*  input: target
*           0: p01 battery
*           1: dock battery
*    return: 0 for 'no charger/usb', 1 for 'charger', 2 for 'USB', <0 value means something error
*
*/
int AX_MicroP_get_USBDetectStatus(int target){
    int regval=0;
    int ret=0;

    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }

    if(isFirmwareUpdating()){
        printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
        return -2;
    }

    if(target==Batt_P01){
        ret=uP_i2c_read_reg(MICROP_USB_DET,&regval);
        if(ret <= 0 || regval==255)
            regval=P01_CABLE_UNKNOWN;
        else if(regval==0)
            regval=P01_CABLE_NO;
        else if(regval==1)
            regval=P01_CABLE_CHARGER;
        else if(regval==2)
            regval=P01_CABLE_USB;
        else if(regval==7)
            regval=P01_CABLE_OTG;
        else
            regval=P01_CABLE_UNKNOWN;
    }
    else{
        printk(KERN_ERR "%s: known target %d\r\n", __FUNCTION__, target);
    }

    return regval;
}

/*
*  GPIO direct control
*  @ AX_MicroP_getGPIOPinLevel
*  input:
            - pinID
*  return: 0 for low, 1 for high, <0 value means something error
*
*/
int AX_MicroP_getGPIOPinLevel(int pinID){
    int regval=0;
    int ret=0;
    int gpiolevel=0;
//    unsigned int sel_offset=1<<pinID;

    pr_debug("[MicroP] try to get GPIO pin=%x\r\n",pinID);
    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }
       
    ret=uP_i2c_read_reg(MICROP_GPIO_INPUT_LEVEL,&regval);
    if(ret > 0){
        gpiolevel=(regval & pinID)?1:0;
    }

    return ((ret < 0)?ret:gpiolevel);
}

EXPORT_SYMBOL_GPL(AX_MicroP_getGPIOPinLevel);
/*
*  @ AX_MicroP_setGPIOOutputPin
*  input: 
*           - pinID
*           - level: 0 for low, 1 for high
*  return: the status of operation. 0 for success, <0 value means something error
*/
int AX_MicroP_setGPIOOutputPin(unsigned int pinID, int level){
   unsigned int sel_offset=0;
   int ret=0;
   int level_pinID=0;
   unsigned int out_reg=0;
//   printk("[MicroP] set GPIO pin=0x%x, level=%d\r\n",pinID, level);

    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }

    if(isFirmwareUpdating()){
        printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
        return -2;
    }
#if 0
    if(pinID >= OUT_uP_SIZE){
        printk("%s: index error =%d\r\n",__FUNCTION__, pinID);
        return -1;
    }
#endif
//    pinID=1<<pinID;
    if(level)
        ret=uP_i2c_write_reg(MICROP_GPIO_OUTPUT_BIT_SET,&pinID);
    else
        ret=uP_i2c_write_reg(MICROP_GPIO_OUTPUT_BIT_CLR,&pinID);

    // check after set /clear
    ret=uP_i2c_read_reg(MICROP_GPIO_OUTPUT_LEVEL, &out_reg);
    if(ret >=0){
        level_pinID=(out_reg & pinID)?1:0;
        printk("[MicroP] AfterSet GPIO pin=0x%x, orig=0x%x,level=%d\r\n", pinID, out_reg,level_pinID);
        if(level_pinID!=level){
            printk("[MicroP] [Debug] cur state =%d\r\n", AX_MicroP_getOPState());
            ret=-3;
        }
    }
    return ((ret < 0)?ret:0);
}

EXPORT_SYMBOL_GPL(AX_MicroP_setGPIOOutputPin);

/*
*  @ AX_MicroP_getGPIOOutputPinLevel
*  input:
*       - pinID
*  return: 0 for low, 1 for high, <0 value means something error
*/
int AX_MicroP_getGPIOOutputPinLevel(int pinID){
    int regval=0;
    int ret=0;
    int gpiolevel=0;

    pr_debug("[MicroP] try to get GPIO_OutPut pin=%d\r\n",pinID);

    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }

    ret=uP_i2c_read_reg(MICROP_GPIO_OUTPUT_LEVEL,&regval);
    if(ret > 0){
        gpiolevel=(regval & pinID)?1:0;
    }

    return ((ret < 0)?ret:gpiolevel);
}
EXPORT_SYMBOL_GPL(AX_MicroP_getGPIOOutputPinLevel);


/*
*  @AX_MicroP_enableInterrupt
*  input: 
*            - intrpin: input pin id
*            -  enable: 0 for 'disable', 1 for 'enable'
*  return: 0 for success, <0 value means something error
*/

int AX_MicroP_enablePinInterrupt(unsigned int pinID, int enable){

       int ret=0;
       int st_microp;       
       printk("[MicroP] enable Pin Intr pin=0x%x, enable=%d\r\n",pinID, enable);


       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }
       
        uP_i2c_read_reg(MICROP_OPERATING_STATE, &st_microp);
        if(st_MICROP_Off==st_microp){
            printk("%s: microp is under off state, skip intr set/clr operations\r\n",__FUNCTION__);
            return -3;
        }
            
        if(enable){
                ret=uP_i2c_write_reg(MICROP_INTR_EN_BIT_SET,&pinID);
        }
        else{
                ret=uP_i2c_write_reg(MICROP_INTR_EN_BIT_CLR,&pinID);                                                
        }
        
	return ((ret < 0)?ret:0);

}

EXPORT_SYMBOL_GPL(AX_MicroP_enablePinInterrupt);




// return = 0: success
//           <0: error
int AX_MicroP_setPWMValue(uint8_t value){
    int flag=0;
    if(value < 0 && value > 255){
        printk("PWM set value not valid\r\n");
        return -1;
    }
    printk("%s: set value=%d\r\n", __FUNCTION__, value);
    flag=uP_i2c_write_reg(MICROP_PWM, &value);
    return (flag>=0)?0:flag;
}

// return >=0: success
//           <0: error
int AX_MicroP_getPWMValue(void){
    uint8_t value=0;
    int flag=0;
    flag=uP_i2c_read_reg(MICROP_PWM, &value);
    if(flag>=0){
            printk("%s: get value=%d\r\n", __FUNCTION__, value);
            return (int)value;
    }
    else
            return flag;
}

EXPORT_SYMBOL_GPL(AX_MicroP_setPWMValue);
EXPORT_SYMBOL_GPL(AX_MicroP_getPWMValue);


int AX_MicroP_enterSleeping(void){
    int flag=0;
    uint16_t  value=0x0055;
    printk("%s\r\n", __FUNCTION__);
    flag=uP_i2c_write_reg(MICROP_IND_PHONE_SLEEP, &value);
    return (flag>=0)?0:flag;
}

int AX_MicroP_enterResuming(void){
    int flag=0;
    int retries=15;
    int cur_state=st_MICROP_Unknown;
    uint16_t  value=0x0069;
    uint32_t st_jiffies=jiffies;
    printk("%s\r\n", __FUNCTION__);
    flag=uP_i2c_write_reg(MICROP_IND_PHONE_RESUME, &value);
    if(flag){
        g_i2c_microp_busy=1;
        flag=uP_i2c_read_reg(MICROP_OPERATING_STATE, &cur_state);        
    }
    
    while (flag>=0 && st_MICROP_Active != cur_state && retries-- > 0) {            
            msleep(30);
            flag=uP_i2c_read_reg(MICROP_OPERATING_STATE, &cur_state);        
    }
    if(flag >=0 && retries > 0)
        printk("Success: takes %lu jiffies~~\r\n", jiffies - st_jiffies);
    else
        printk("\r\nFailed!!\r\n");

    g_i2c_microp_busy=0;
    return (flag>=0)?0:flag;
}

EXPORT_SYMBOL_GPL(AX_MicroP_enterSleeping);
EXPORT_SYMBOL_GPL(AX_MicroP_enterResuming);

int AX_MicroP_getOPState(void){
       int regval=0;
       int ret=0;

       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }

       ret=uP_i2c_read_reg(MICROP_OPERATING_STATE,&regval);

       return ((ret < 0)?ret:regval);
}

EXPORT_SYMBOL_GPL(AX_MicroP_getOPState);

int AX_MicroP_writeKDataOfLightSensor(uint32_t data){
       int ret=0;

       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }

       ret=uP_i2c_write_reg(MICROP_CALIBRATION_DATA,&data);

       return ((ret < 0)?ret:0);
}

EXPORT_SYMBOL_GPL(AX_MicroP_writeKDataOfLightSensor);

uint32_t AX_MicroP_readKDataOfLightSensor(void){
       int ret=0;
       uint32_t kdata=0;
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }

       ret=uP_i2c_read_reg(MICROP_CALIBRATION_DATA,&kdata);

       return ((ret < 0)?ret:kdata);

}
EXPORT_SYMBOL_GPL(AX_MicroP_readKDataOfLightSensor);


/*int AX_MicroP_writeCompassData(char* data, int length)
{
	int ret = -1;
	int i = 0;
	int j = 0;
	char write_buf[9];
	char read_buf[512];
	
	if(data == NULL){
		printk("%s: data is null\r\n",__FUNCTION__);
		return -1;
	}
	
	if( ((length%8)!=0) || length>512){
		printk("%s: length is wrong (%d)\r\n",__FUNCTION__, length);
		return -1;
	}
	
	if(AX_MicroP_IsP01Connected()==0){
		printk("%s: Pad is not connected\r\n",__FUNCTION__);
		return -1;
	}
	
	if(isFirmwareUpdating()){
		printk("%s: Pad is updating, retry later\r\n",__FUNCTION__);
		return -2;
	}
	
	for(i=0; i<length; i+=8){
		write_buf[0] = i/4;
		memcpy(write_buf+1, data+i, 8);
		
		ret = uP_i2c_write_reg(MICROP_COMPASS_PARAMETER, write_buf);
		if(ret < 0){
			printk("%s: write reg error\r\n",__FUNCTION__);
			return -1;
		}
		
		memset(read_buf, 0, 9);
		read_buf[0] = i/4;
		ret = uP_i2c_read_reg(MICROP_COMPASS_PARAMETER, read_buf);
		if(ret < 0){
			printk("%s: read reg error\r\n",__FUNCTION__);
			return -1;
		}
		else{
			for(j=0; j<8; j++){
				if(read_buf[j] != write_buf[j]){
					printk("%s: check compass data error, i = %d\r\n",__FUNCTION__, i);
					return -1;
				}
			}
		}
	}
	
	return 0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_writeCompassData);*/

/*int AX_MicroP_readCompassData(char* data, int length)
{
	int ret = -1;
	int i = 0;
	char read_buf[9];
	
	if(data == NULL){
		printk("%s: data is null\r\n",__FUNCTION__);
		return -1;
	}
	
	if( ((length%8)!=0) || length>512){
		printk("%s: length is wrong (%d)\r\n",__FUNCTION__, length);
		return -1;
	}
	
	if(AX_MicroP_IsP01Connected()==0){
		printk("%s: Pad is not connected\r\n",__FUNCTION__);
		return -1;
	}
	
	if(isFirmwareUpdating()){
		printk("%s: Pad is updating, retry later\r\n",__FUNCTION__);
		return -2;
	}
	
	for(i=0; i<length; i+=8){
		memset(read_buf, 0, 9);
		
		read_buf[0] = (i/4) | 0x80;
		ret = uP_i2c_write_reg(MICROP_COMPASS_PARAMETER, read_buf);
		if(ret < 0){
			printk("%s: write reg error\r\n",__FUNCTION__);
			return -1;
		}
		
		read_buf[0] = i/4;
		ret = uP_i2c_read_reg(MICROP_COMPASS_PARAMETER, read_buf);
		if(ret < 0){
			printk("%s: read reg error\r\n",__FUNCTION__);
			return -1;
		}
		else{
			memcpy(data+i, read_buf+1, 8);
		}
	}
	
	return 0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_readCompassData);*/


/*
*  @get_MicroP_HUB_SLEEP_STATUS
*  return: value = 1: turn on, value = 0: turn off
*/
/*     TBD

int get_MicroP_HUB_SLEEP_STATUS(void){
	return AX_MicroP_getGPIOOutputPinLevel(OUT_uP_HUB_SLEEP);
}

EXPORT_SYMBOL_GPL(get_MicroP_HUB_SLEEP_STATUS);
*/


uint8_t AX_MicroP_getHWID(void){
    int ret;
    uint8_t hwid = 0;

    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }

    if(isFirmwareUpdating()){
        printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
        return -2;
    }

    ret = uP_i2c_read_reg(MICROP_PAD_HWID, &hwid);
    if(ret < 0){
        printk("%s: read reg error\r\n",__FUNCTION__);
        return -1;
    }
    else {
        hwid = (hwid & 0x3);
    }

    return hwid;
}
EXPORT_SYMBOL_GPL(AX_MicroP_getHWID);

/*
*  return value
    1: Laibao
    0: Wintek
*/
int AX_MicroP_getTSID(void){
    int ret = 0;
    uint8_t hwid = 0;

    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }

    if(isFirmwareUpdating()){
        printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
        return -2;
    }

    ret = uP_i2c_read_reg(MICROP_PAD_HWID, &hwid);
    if(ret < 0){
        printk("%s: read reg error\r\n",__FUNCTION__);
		return -1;
    }
    else {
        ret = (hwid & 0x18) >> 3;
    }
    return ret;
}

/*
*  return value
*/
int AX_MicroP_getMICROPID(void){
	int ret = 0;
	uint16_t hwid = 0;

	if(AX_MicroP_IsP01Connected()==0){
		printk("%s: P01 removed\r\n",__FUNCTION__);
		return -1;
	}
	
	if(isFirmwareUpdating()){
		printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
		return -2;
	}
       
	ret = uP_i2c_read_reg(MICROP_HW_ID, &hwid);
	
	if(ret < 0){
		printk("%s: read reg error\r\n",__FUNCTION__);
		return -1;
	}
	else {
		ret = hwid;
	}	
	return ret;
}
EXPORT_SYMBOL_GPL(AX_MicroP_getMICROPID);

int AX_MicroP_IsMydpNewSKU(void){
	int ret = 0;
	uint8_t hwid = 0; 
	
	ret = uP_i2c_read_reg(MICROP_PAD_HWID, &hwid);
	
	if(ret < 0){
		printk("%s: read reg error\r\n",__FUNCTION__);
		return -1;
	}
	else {
		ret = (hwid & 0x80) >> 7;
	}
	
	return ret;
}
EXPORT_SYMBOL_GPL(AX_MicroP_IsMydpNewSKU);

/*
*  battInfo should be 2 byte

*/
int AX_MicroP_getBatterySoc(void *battInfo){
	int ret = 0;

	if(AX_MicroP_IsP01Connected()==0){
		printk("%s: P01 removed\r\n",__FUNCTION__);
		return -1;
	}

	if(isFirmwareUpdating()){
		printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
		return -2;
	}

	ret = uP_i2c_read_reg(MICROP_BATTERY_SOC, battInfo);

	if(ret < 0){
		printk("%s: read reg error\r\n",__FUNCTION__);
		return -1;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(AX_MicroP_getBatterySoc);

/*
*  battInfo should be 6 charector

*/
int AX_MicroP_getBatteryInfo(void *battInfo){
	int ret = 0;

	if(AX_MicroP_IsP01Connected()==0){
		printk("%s: P01 removed\r\n",__FUNCTION__);
		return -1;
	}

	if(isFirmwareUpdating()){
		printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
		return -2;
	}

	ret = uP_i2c_read_reg(MICROP_BATTERY_INFO, battInfo);

	if(ret < 0){
		printk("%s: read reg error\r\n",__FUNCTION__);
		return -1;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(AX_MicroP_getBatteryInfo);

// bit[0]: enable OTG power
// bit[1-15]: reserved
int AX_MicroP_setOTGPower(uint16_t value){
    int flag=0;
    if(value < 0 && value > 1){
        printk("OTG Powe set value not valid\r\n");
        return -1;
    }
    printk("%s: set value=%d\r\n", __FUNCTION__, value);
    flag=uP_i2c_write_reg(MICROP_OTG_POWER, &value);
    return (flag>=0)?0:flag;
}

void AX_MicroP_set_VBusPower(int level)
{
    int rt;
    printk("set_microp_vbus = %d\n",level);
    rt = AX_MicroP_setGPIOOutputPin(OUT_uP_VBUS_EN, level);
    if (rt<0){
        printk("MicroP set vbus error\n");
        msleep(20);
        printk("wait for ec change vbus status(20ms)\n");
        printk("vbus status:%d\n",AX_MicroP_get_VBusPower());
    }else if(rt == 0){
        printk("MicroP set success\n");
    }
}

int AX_MicroP_get_VBusPower(void)
{
    return AX_MicroP_getGPIOOutputPinLevel(OUT_uP_VBUS_EN);
}

int AX_MicroP_setSPK_EN(uint8_t enable){
    int ret=0;
    uint8_t hwid;

    if( enable > 1){
        printk("%s:set value(%d) not valid\r\n",__FUNCTION__,enable);
        return -1;
    }

    speaker_en = enable;

    hwid = AX_MicroP_getHWID();
    if (hwid == P72_MP_HWID) {
        if ( enable && AX_MicroP_getGPIOOutputPinLevel(OUT_uP_5V_PWR_EN) ==0) {
            AX_MicroP_setGPIOOutputPin(OUT_uP_5V_PWR_EN,1);
            msleep(100);
        }
    }
    if ( hwid == P72_ER1_2_HWID || hwid == P72_ER2_HWID )
        AX_MicroP_setGPIOOutputPin(OUT_up_SPK_SEL,enable);
    else
        AX_MicroP_setGPIOOutputPin(OUT_up_SPK_SEL,!enable);
    if (ret<0){
        printk("MicroP set SPK_SEL fail\n");
    }else if(ret == 0){
        printk("MicroP set SPK_SEL\n");
    }

    return ret;
}

int AX_MicroP_setRCV_EN(uint8_t enable){
    int ret=0;
    uint8_t hwid;

    if( enable > 1){
        printk("%s:set value(%d) not valid\r\n",__FUNCTION__,enable);
        return -1;
    }

    recevier_en = enable;
    hwid = AX_MicroP_getHWID();
    if (hwid == P72_MP_HWID) {
        if ( enable && AX_MicroP_getGPIOOutputPinLevel(OUT_uP_5V_PWR_EN) ==0) {
            AX_MicroP_setGPIOOutputPin(OUT_uP_5V_PWR_EN,1);
            msleep(100);
        }
    }

    hwid = AX_MicroP_getHWID();
    if ( hwid == P72_ER1_2_HWID || hwid == P72_ER2_HWID )
        AX_MicroP_setGPIOOutputPin(OUT_up_RCV_SEL,enable);
    else
        AX_MicroP_setGPIOOutputPin(OUT_up_RCV_SEL,!enable);
    if (ret<0){
        printk("MicroP set RCV_SEL fail\n");
    }else if(ret == 0){
        printk("MicroP set RCV_SEL\n");
    }

    return ret;
}

int AX_MicroP_set_Proxm_crosstalk(unsigned char* data)
{
    int ret = 0;
    unsigned char buf[PROXIMITY_KDATA_SIZE*2];
    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }

    if(isFirmwareUpdating()){
        printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
        return -2;
    }

    ret = uP_i2c_read_reg(MICROP_PROXIM_KDATA, buf);
    if(ret < 0){
        printk("%s: read reg error\r\n",__FUNCTION__);
        return -1;
    }
    memcpy(buf,data,sizeof(unsigned char)*PROXIMITY_KDATA_SIZE);
    ret = uP_i2c_write_reg(MICROP_PROXIM_KDATA, buf);

    if(ret < 0){
        printk("%s: write reg error\r\n",__FUNCTION__);
        return -1;
    }

    return ret;
}

int AX_MicroP_get_Proxm_crosstalk(unsigned char* data)
{
    int ret = 0;
    unsigned char buf[PROXIMITY_KDATA_SIZE*2];

    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }

    if(isFirmwareUpdating()){
        printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
        return -2;
    }

    ret = uP_i2c_read_reg(MICROP_PROXIM_KDATA, buf);

    if(ret < 0){
        printk("%s: read reg error\r\n",__FUNCTION__);
        return -1;
    }

    memcpy(data,buf,sizeof(unsigned char)*PROXIMITY_KDATA_SIZE);
    return ret;
}

int AX_MicroP_set_Proxm_threshold(unsigned char* data)
{
    int ret = 0;
    unsigned char buf[PROXIMITY_KDATA_SIZE*2];
    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }

    if(isFirmwareUpdating()){
        printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
        return -2;
    }

    ret = uP_i2c_read_reg(MICROP_PROXIM_KDATA, buf);

    if(ret < 0){
        printk("%s: read reg error\r\n",__FUNCTION__);
        return -1;
    }
    memcpy(buf+PROXIMITY_KDATA_SIZE,data,sizeof(unsigned char)*PROXIMITY_KDATA_SIZE);

    ret = uP_i2c_write_reg(MICROP_PROXIM_KDATA, buf);

    if(ret < 0){
        printk("%s: write reg error\r\n",__FUNCTION__);
        return -1;
    }

    return ret;
}

int AX_MicroP_get_Proxm_threshold(unsigned char* data)
{
    int ret = 0;
    unsigned char buf[PROXIMITY_KDATA_SIZE*2];

    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }

    if(isFirmwareUpdating()){
        printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
        return -2;
    }

    ret = uP_i2c_read_reg(MICROP_PROXIM_KDATA, buf);

    if(ret < 0){
        printk("%s: read reg error\r\n",__FUNCTION__);
        return -1;
    }

    memcpy(data,buf+PROXIMITY_KDATA_SIZE,sizeof(unsigned char)*PROXIMITY_KDATA_SIZE);
    return ret;
}

int AX_request_gpio_33(void) {
    int ret = 0;
    if (isregister == false) {
        ret = gpio_request(33, "Microp");
        if (ret < 0){
            printk("gpio_request fail gpio=33\n");
       }
       isregister = true;
    }
    return ret;
}

void AX_setECPowerOff(void) {
    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }
    TriggerPadStationPowerOff();
}
EXPORT_SYMBOL_GPL(AX_setECPowerOff);

bool AX_Is_pad_exist(void) {
    return pad_exist();
}

void AX_Recheck_EC_interrupt(void)
{
    microp_recheck_interrupt();
}
