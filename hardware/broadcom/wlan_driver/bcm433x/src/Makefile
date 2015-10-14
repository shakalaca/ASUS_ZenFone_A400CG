# bcmdhd
DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                     \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DBCMFILEIMAGE            \
	-DDHDTHREAD -DDHD_DEBUG -DSDTEST -DBDC -DTOE                          \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DBCMDBG -DWL_SCHED_SCAN                \
	-DCUSTOMER_HW2 -DOOB_INTR_ONLY -DDEBUGFS_CFG80211                     \
	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P     \
	-DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT -DSUPPORT_PM2_ONLY             \
	-DKEEP_ALIVE -DGET_CUSTOM_MAC_ENABLE -DPKT_FILTER_SUPPORT             \
	-DEMBEDDED_PLATFORM -DENABLE_INSMOD_NO_FW_LOAD -DPNO_SUPPORT          \
	-DDHD_USE_IDLECOUNT -DROAM_ENABLE -DVSDB                              \
	-DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST                            \
	-DESCAN_RESULT_PATCH  -DSDIO_CRC_ERROR_FIX                            \
	-DDHD_DONOT_FORWARD_BCMEVENT_AS_NETWORK_PKT                           \
	-DCUSTOM_SDIO_F2_BLKSIZE=128 -DWL_SDO                                 \
	-DDHD_USE_IDLECOUNT -DWL_SUPPORT_BACKPORTED_KPATCHES                  \
	-DGET_CUSTOM_MAC_ENABLE                                               \
	-Idrivers/net/wireless/bcmdhd -Idrivers/net/wireless/bcmdhd/include

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o dhd_sdio.o bcmwifi_channels.o bcmevent.o hndpmu.o   \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o linux_osl.o            \
	bcmsdh_sdmmc.o dhd_custom_gpio.o sbutils.o wldev_common.o             \
	wl_android.o dhd_pno.o dhd_wlfc.o dhd_ip.o

#throughput monitor
DHDCFLAGS += -DTPUT_MONITOR -DTPUT_MONITOR_USE_PM_QOS

#CY+ get chipnum
DHDCFLAGS += -DCHIPNUM
#CY-

#Nate+ for set WoWLAN support flags
DHDCFLAGS += -DCONFIG_PM
#Nate-

#WIFI_CHIP, default: 43340, PF400CG/ME175CG/A400CG/A450CG/FE380CG: 43362
ifeq ($(ASUS_PROJECT), PF400CG)
        DHDCFLAGS += -DWIFI_CHIP_43362
endif
ifeq ($(ASUS_PROJECT), ME175CG)
        DHDCFLAGS += -DWIFI_CHIP_43362
endif
ifeq ($(ASUS_PROJECT), A400CG)
        DHDCFLAGS += -DWIFI_CHIP_43362
endif
ifeq ($(ASUS_PROJECT), FE380CG)
        DHDCFLAGS += -DWIFI_CHIP_43362
endif
ifeq ($(ASUS_PROJECT), A450CG)
        DHDCFLAGS += -DWIFI_CHIP_43362
endif

#Nate+ for reduce First scan time
DHDCFLAGS += -DUSE_INITIAL_SHORT_DWELL_TIME
#Nate- for reduce First scan time

#Nate + for add WAPI feature
DHDCFLAGS += -DBCMWAPI_WPI -DBCMWAPI_WAI
#Nate - for add WAPI feature

obj-$(CONFIG_BCMDHD) += bcmdhd.o
bcmdhd-objs += $(DHDOFILES)
ifneq ($(CONFIG_WIRELESS_EXT),)
bcmdhd-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP -DWL_WIRELESS_EXT -DUSE_IW
endif
ifneq ($(CONFIG_CFG80211),)
bcmdhd-objs += wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o dhd_cfg80211.o
DHDCFLAGS += -DWL_CFG80211 -DWL_CFG80211_STA_EVENT
DHDCFLAGS += -DWL_CFG80211_P2P_DEV_IF -DWL_IFACE_COMB_NUM_CHANNELS
endif

DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-85
DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=15

#CY+ increase keep alive packet interval to 55s.
#    BCM: most aggressive AP will kick out idle STA every 1min.
#DHDCFLAGS += -DCUSTOM_KEEP_ALIVE_SETTING=10000
DHDCFLAGS += -DCUSTOM_KEEP_ALIVE_SETTING=55000
#CY-

DHDCFLAGS += -DCUSTOM_PNO_EVENT_LOCK_xTIME=7

#CY+ enable dhcp scan supress and set power saving mode at the same time
DHDCFLAGS += -DDHCP_SCAN_SUPPRESS
#CY-

#CY+ raise BK priority to BE
#  fix low throughput for BK packet(for example, some FTP server)
#  BCM43340 will not use AMPDU for BK packet, so the throughput is limited(about 20Mbps)
DHDCFLAGS += -DRAISE_BK_PRIO
#CY-

## Set dhd_dpd_thread priority to MAX to avoid starvation
DHDCFLAGS += -DCUSTOM_DPC_PRIO_SETTING=99

## Use BOARD_INTEL flag for intel platform project
DHDCFLAGS += -DBOARD_INTEL

ifneq ($(CONFIG_DHD_USE_SCHED_SCAN),)
DHDCFLAGS += -DWL_SCHED_SCAN
endif
ccflags-y = -I$(src)
ccflags-y += -I$(src)/include

EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCMDHD),m)
EXTRA_LDFLAGS += --strip-debug
endif
