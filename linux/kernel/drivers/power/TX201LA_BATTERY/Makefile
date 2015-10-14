ifeq ($(TARGET_BUILD_VARIANT),eng)
        KBUILD_CFLAGS += -DTX201LA_ENG_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),user)
        KBUILD_CFLAGS += -DTX201LA_USER_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),userdebug)
        KBUILD_CFLAGS += -DTX201LA_USERDEBUG_BUILD
else
        KBUILD_CFLAGS += -DTX201LA_OTHER_BUILD
endif

obj-$(CONFIG_TX201LA_BATTERY_EC)	+= asus_battery_ec.o smb345_charger.o bq27520_battery_upt_i2c.o bq27520_battery_core.o util.o

