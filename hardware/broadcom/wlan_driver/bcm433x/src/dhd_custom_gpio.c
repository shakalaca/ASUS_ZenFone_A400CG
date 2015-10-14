/*
* Customer code to add GPIO control during WLAN start/stop
* $Copyright Open Broadcom Corporation$
*
* $Id: dhd_custom_gpio.c 417465 2013-08-09 11:47:27Z $
*/

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <bcmutils.h>

#include <dngl_stats.h>
#include <dhd.h>

#include <wlioctl.h>
#include <wl_iw.h>

#ifdef BOARD_INTEL
#include <asm/intel-mid.h>
#include <linux/gpio.h>
#endif

#define WL_ERROR(x) printf x
#define WL_TRACE(x)

#ifdef CUSTOMER_HW
extern  void bcm_wlan_power_off(int);
extern  void bcm_wlan_power_on(int);
#endif /* CUSTOMER_HW */
#if defined(CUSTOMER_HW2)

#if defined(PLATFORM_MPS)
int __attribute__ ((weak)) wifi_get_fw_nv_path(char *fw, char *nv) { return 0;};
#endif

#ifdef CONFIG_WIFI_CONTROL_FUNC
int wifi_set_power(int on, unsigned long msec);
int wifi_get_irq_number(unsigned long *irq_flags_ptr);
#ifdef BOARD_INTEL
int wifi_get_gpioen_number(void);
#endif
int wifi_get_mac_addr(unsigned char *buf);
void *wifi_get_country_code(char *ccode);
#else
int wifi_set_power(int on, unsigned long msec) { return -1; }
int wifi_get_irq_number(unsigned long *irq_flags_ptr) { return -1; }
#ifdef BOARD_INTEL
int wifi_get_gpioen_number(void) { return -1; }
#endif
int wifi_get_mac_addr(unsigned char *buf) { return -1; }
void *wifi_get_country_code(char *ccode) { return NULL; }
#endif /* CONFIG_WIFI_CONTROL_FUNC */
#endif 

#if defined(OOB_INTR_ONLY)

#if defined(BCMLXSDMMC)
extern int sdioh_mmc_irq(int irq);
#endif /* (BCMLXSDMMC)  */

#if defined(CUSTOMER_HW3) || defined(PLATFORM_MPS)
#include <mach/gpio.h>
#endif

/* Customer specific Host GPIO defintion  */
static int dhd_oob_gpio_num = -1;

module_param(dhd_oob_gpio_num, int, 0644);
MODULE_PARM_DESC(dhd_oob_gpio_num, "DHD oob gpio number");

/* This function will return:
 *  1) return :  Host gpio interrupt number per customer platform
 *  2) irq_flags_ptr : Type of Host interrupt as Level or Edge
 *
 *  NOTE :
 *  Customer should check his platform definitions
 *  and his Host Interrupt spec
 *  to figure out the proper setting for his platform.
 *  Broadcom provides just reference settings as example.
 *
 */
int dhd_customer_oob_irq_map(unsigned long *irq_flags_ptr)
{
	int  host_oob_irq = 0;

//CY+ retry request gpio
	int ret=0;
	int retry;
//CY-


#if defined(CUSTOMER_HW2) && !defined(PLATFORM_MPS)
#ifdef BOARD_INTEL
	int host_gpioen = 0;
#endif
	host_oob_irq = wifi_get_irq_number(irq_flags_ptr);

#ifdef BOARD_INTEL

//CY+ retry request GPIO
#if 0
	if (gpio_request(host_oob_irq, "bcm43xx_irq") < 0) {
		WL_ERROR(("%s: Error on gpio_request bcm43xx_irq: %d\n", __func__, host_oob_irq));
#else
	retry=5;
retry_gpio_irq:
	gpio_free(host_oob_irq);
	ret = gpio_request(host_oob_irq, "bcm43xx_irq");
	if (ret < 0) {
		WL_ERROR(("%s: Error on gpio_request bcm43xx_irq: %d, ret=%d, retry remain %d\n", __func__, host_oob_irq, ret, retry));
		if(retry-- > 0){
			msleep(100);
			goto retry_gpio_irq;
		}
#endif
//CY-

		return 1;
	}
	if (gpio_direction_input(host_oob_irq) < 0) {
		WL_ERROR(("%s: Error on gpio_direction_input\n", __func__));
		return 1;
	}

	if (gpio_set_debounce(host_oob_irq, 0) < 0) {
		WL_ERROR(("%s: Error on gpio_set_debounce\n", __func__));
		return 1;
	}

	host_oob_irq = gpio_to_irq(host_oob_irq);
	sdhci_pci_request_regulators();

	host_gpioen = wifi_get_gpioen_number();

//CY+ retry request gpio
#if 0
	if (gpio_request(host_gpioen, "bcm43xx_en") < 0) {
		WL_ERROR(("%s: Error on gpio_request bcm43xx_en: %d\n", __func__, host_gpioen));
#else
       retry=5;
retry_gpio_en:
	gpio_free(host_gpioen);
	ret = gpio_request(host_gpioen, "bcm43xx_en");
	if (ret < 0) {
		WL_ERROR(("%s: Error on gpio_request bcm43xx_en: %d, ret=%d, retry remain %d\n", __func__, host_gpioen, ret, retry));
		if(retry-- > 0){
			msleep(100);
			goto retry_gpio_en;
		}
#endif
//CY-

		return 1;
	}

	if (gpio_direction_output(host_gpioen, 1) < 0) {
		WL_ERROR(("%s: Error on gpio_direction_output\n", __func__));
		return 1;
	}
#endif

#else
#if defined(CUSTOM_OOB_GPIO_NUM)
	if (dhd_oob_gpio_num < 0) {
		dhd_oob_gpio_num = CUSTOM_OOB_GPIO_NUM;
	}
#endif /* CUSTOMER_OOB_GPIO_NUM */

	if (dhd_oob_gpio_num < 0) {
		WL_ERROR(("%s: ERROR customer specific Host GPIO is NOT defined \n",
		__FUNCTION__));
		return (dhd_oob_gpio_num);
	}

	WL_ERROR(("%s: customer specific Host GPIO number is (%d)\n",
	         __FUNCTION__, dhd_oob_gpio_num));

#if defined CUSTOMER_HW
	host_oob_irq = MSM_GPIO_TO_INT(dhd_oob_gpio_num);
#elif defined CUSTOMER_HW3 || defined(PLATFORM_MPS)
	gpio_request(dhd_oob_gpio_num, "oob irq");
	host_oob_irq = gpio_to_irq(dhd_oob_gpio_num);
	gpio_direction_input(dhd_oob_gpio_num);
#endif /* CUSTOMER_HW */
#endif 

	return (host_oob_irq);
}
#endif 

#ifdef BOARD_INTEL
int dhd_customer_oob_irq_unmap(void)
{
#if defined(CUSTOMER_HW2) || defined(CUSTOMER_HW4)
	int host_gpioen = 0;
	int  host_oob_irq = 0;
	long unsigned int irq_flags_ptr;
	host_oob_irq = wifi_get_irq_number(&irq_flags_ptr);
	if (host_oob_irq < 0)
		WL_ERROR(("%s: wifi_get_irq_number returned %d\n",
				__func__, host_oob_irq));
	else {
		if (gpio_set_debounce(host_oob_irq, 1) < 0)
			WL_ERROR(("%s: Error on gpio_set_debounce\n",
					__func__));
		gpio_free(host_oob_irq);
	}
	gpio_free(host_oob_irq);

	host_gpioen = wifi_get_gpioen_number();
	if (host_gpioen < 0)
		WL_ERROR(("%s: wifi_get_gpioen_number returned %d\n",
				__func__, host_gpioen));
	else
		gpio_free(host_gpioen);
	
#endif
	return 0;
}
#endif

/* Customer function to control hw specific wlan gpios */
void
dhd_customer_gpio_wlan_ctrl(int onoff)
{
	switch (onoff) {
		case WLAN_RESET_OFF:
			WL_TRACE(("%s: call customer specific GPIO to insert WLAN RESET\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_off(2);
#endif /* CUSTOMER_HW */
#if defined(CUSTOMER_HW2)
			wifi_set_power(0, WIFI_TURNOFF_DELAY);
#endif
			WL_ERROR(("=========== WLAN placed in RESET ========\n"));
		break;

		case WLAN_RESET_ON:
			WL_TRACE(("%s: callc customer specific GPIO to remove WLAN RESET\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_on(2);
#endif /* CUSTOMER_HW */
#if defined(CUSTOMER_HW2)
			wifi_set_power(1, 200);
#endif
			WL_ERROR(("=========== WLAN going back to live  ========\n"));
		break;

		case WLAN_POWER_OFF:
			WL_TRACE(("%s: call customer specific GPIO to turn off WL_REG_ON\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_off(1);
#endif /* CUSTOMER_HW */
		break;

		case WLAN_POWER_ON:
			WL_TRACE(("%s: call customer specific GPIO to turn on WL_REG_ON\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_on(1);
			/* Lets customer power to get stable */
			OSL_DELAY(200);
#endif /* CUSTOMER_HW */
		break;
	}
}

#ifdef GET_CUSTOM_MAC_ENABLE

//CY+ porting WIFI driver for intel platform
#define MAC_ADDRESS_LEN 12

int wifi_get_mac_addr_intel(unsigned char *buf){
	int ret = 0;
	int i;
	struct file *fp = NULL;
	unsigned char c_mac[MAC_ADDRESS_LEN];
	char fname[]="/factory/wifi/mac.txt";

	WL_TRACE(("%s Enter\n", __FUNCTION__));

	fp = dhd_os_open_image(fname);
	if (fp== NULL){
		WL_ERROR(("%s: unable to open %s\n",__FUNCTION__, fname));
		return 1;
	}

	if ( dhd_os_get_image_block(c_mac, MAC_ADDRESS_LEN, fp) != MAC_ADDRESS_LEN ){
		WL_ERROR(("%s: Error on reading mac address from %s \n",__FUNCTION__, fname));
		dhd_os_close_image(fp);
		return 1;
	}
	dhd_os_close_image(fp);

	for (i =0; i< MAC_ADDRESS_LEN ; i+=2){
		c_mac[i] = bcm_isdigit(c_mac[i]) ? c_mac[i]-'0' : bcm_toupper(c_mac[i])-'A'+10;
		c_mac[i+1] = bcm_isdigit(c_mac[i+1]) ? c_mac[i+1]-'0' : bcm_toupper(c_mac[i+1])-'A'+10;

		buf[i/2] = c_mac[i]*16 + c_mac[i+1];
	}

	WL_TRACE(("%s: read from file mac address: %x:%x:%x:%x:%x:%x\n",
			 __FUNCTION__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]));

	return ret;
}
//CY-

/* Function to get custom MAC address */
int
dhd_custom_get_mac_address(unsigned char *buf)
{
	int ret = 0;

	WL_TRACE(("%s Enter\n", __FUNCTION__));
	if (!buf)
		return -EINVAL;

	/* Customer access to MAC address stored outside of DHD driver */
#if defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))

//CY+ porting WIFI driver for intel platform
//	ret = wifi_get_mac_addr(buf);
	ret = wifi_get_mac_addr_intel(buf);
//CY-

#endif

#ifdef EXAMPLE_GET_MAC
	/* EXAMPLE code */
	{
		struct ether_addr ea_example = {{0x00, 0x11, 0x22, 0x33, 0x44, 0xFF}};
		bcopy((char *)&ea_example, buf, sizeof(struct ether_addr));
	}
#endif /* EXAMPLE_GET_MAC */

	return ret;
}
#endif /* GET_CUSTOM_MAC_ENABLE */

/* Customized Locale table : OPTIONAL feature */
const struct cntry_locales_custom translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */

//CY+ ASUS's translate_custom_table
	//special country code
	{"XY", "XY", 7},	//for WW SKU(include US)
	{"TW", "TW", 16},	//for TW SKU
	{"JP", "JP", 3},	//for JP SKU
	{"CN", "CN", 11},	//for CN SKU 

	{"XZ", "XZ", 1},
	{"US", "US", 109},
	{"CA", "US", 109},

	{"RU", "RU", 5},
	{"EU", "GB", 4},
	{"AT", "GB", 4},
	{"BE", "GB", 4},
	{"BG", "GB", 4},
	{"CY", "GB", 4},
	{"CZ", "GB", 4},
	{"DK", "GB", 4},
	{"EE", "GB", 4},
	{"FI", "GB", 4},
	{"FR", "GB", 4},
	{"DE", "GB", 4},
	{"GR", "GB", 4},
	{"HU", "GB", 4},
	{"IE", "GB", 4},
	{"IT", "GB", 4},
	{"LV", "GB", 4},
	{"LI", "GB", 4},
	{"LT", "GB", 4},
	{"LU", "GB", 4},
	{"MT", "GB", 4},
	{"NL", "GB", 4},
	{"PL", "GB", 4},
	{"PT", "GB", 4},
	{"RO", "GB", 4},
	{"SK", "GB", 4},
	{"SI", "GB", 4},
	{"ES", "GB", 4},
	{"SE", "GB", 4},
	{"GB", "GB", 4},

	{"AU", "AU", 0},
	{"AR", "AR", 0},
	{"MX", "MX", 0},
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"NO", "NO", 0},
	{"KR", "KR", 25},
	{"BR", "KR", 25},

	{"AQ", "XY", 7},
	{"AX", "XY", 7},
	{"BL", "XY", 7}, 
	{"BV", "XY", 7},
	{"CC", "XY", 7},
	{"CU", "XY", 7},
	{"GL", "XY", 7},
	{"GS", "XY", 7},
	{"HM", "XY", 7},
	{"IR", "XY", 7},
	{"KP", "XY", 7},
	{"MH", "XY", 7},
	{"PN", "XY", 7},
	{"PS", "XY", 7},
	{"SD", "XY", 7},
	{"SH", "XY", 7},
	{"SJ", "XY", 7},
	{"SY", "XY", 7},
	{"TK", "XY", 7},
	{"TL", "XY", 7},

        {"ID", "ID", 1},
};

//CY+ ASUS BCM43362 country code translation table
const struct cntry_locales_custom translate_custom_table_bcm43362[] = {
//BCM43362 only support 2.4G, so only 3 choice(1-11/1-13/1-14)
	//XV/0(1-11 active, 12-14 passive)
	{"XY",   "XV", 0}, //for WW SKU

	//no available channel for these country code in 43362 FW, use default country XV/0
	{"AD",   "XV", 0},
	{"AQ",   "XV", 0},
	{"BV",   "XV", 0},
	{"CC",   "XV", 0},
	{"CK",   "XV", 0},
	{"GL",   "XV", 0},
	{"GS",   "XV", 0},
	{"HM",   "XV", 0},
	{"IR",   "XV", 0},
	{"KP",   "XV", 0},
	{"MH",   "XV", 0},
	{"PN",   "XV", 0},
	{"PS",   "XV", 0},
	{"SD",   "XV", 0},
	{"SH",   "XV", 0},
	{"SJ",   "XV", 0},
	{"SY",   "XV", 0},
	{"TK",   "XV", 0},
	{"TL",   "XV", 0},

	//special case(rev!=0)
	{"JP", "JP", 1}, //for JP SKU
};


/* Customized Locale convertor
*  input : ISO 3166-1 country abbreviation
*  output: customized cspec
*/
#ifdef CHIPNUM
void get_customized_country_code(char *country_iso_code, wl_country_t *cspec, uint chipnum)
#else
void get_customized_country_code(char *country_iso_code, wl_country_t *cspec)
#endif
{
#if 0 && defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39))

	struct cntry_locales_custom *cloc_ptr;

	if (!cspec)
		return;

	cloc_ptr = wifi_get_country_code(country_iso_code);
	if (cloc_ptr) {
		strlcpy(cspec->ccode, cloc_ptr->custom_locale, WLC_CNTRY_BUF_SZ);
		cspec->rev = cloc_ptr->custom_locale_rev;
	}
	return;
#else
	int size, i;
	const struct cntry_locales_custom* translate_table;

	translate_table = translate_custom_table;
	//Cannot get array size from sizeof() for pointor!!!
	size = ARRAYSIZE(translate_custom_table);

#ifdef CHIPNUM
	//BCM43362 use it's own table
	if(chipnum == 43362){
		//printk("############get_customized_country_code: use translate_custom_table_bcm43362");
		
		translate_table = translate_custom_table_bcm43362;
		//Cannot get array size from sizeof() for pointor!!!
		size = ARRAYSIZE(translate_custom_table_bcm43362);
	}
#endif
	
	if (cspec == 0)
		 return;

	if (size == 0)
		 return;

	for (i = 0; i < size; i++) {
		if (strcmp(country_iso_code, translate_table[i].iso_abbrev) == 0) {
			memcpy(cspec->ccode,
				translate_table[i].custom_locale, WLC_CNTRY_BUF_SZ);
			cspec->rev = translate_table[i].custom_locale_rev;
			return;
		}
	}

//CY+ always use translate_custom_table
//#ifdef EXAMPLE_TABLE
	/* if no country code matched return first universal code from translate_custom_table */
//	memcpy(cspec->ccode, translate_custom_table[0].custom_locale, WLC_CNTRY_BUF_SZ);
//	cspec->rev = translate_custom_table[0].custom_locale_rev;
	memcpy(cspec->ccode, country_iso_code, WLC_CNTRY_BUF_SZ);
	cspec->rev = 0;
//#endif /* EXMAPLE_TABLE */
//CY-

	return;
#endif /* defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)) */
}
