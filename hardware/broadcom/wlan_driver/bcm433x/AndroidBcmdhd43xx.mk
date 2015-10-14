# This makefile is included from vendor/intel/common/wifi/WifiRules.mk.

.PHONY: build_bcmdhd
build_bcmdhd: build_kernel
	KERNEL_SRC_DIR="$(KERNEL_SRC_DIR)" TARGET_TOOLS_PREFIX="$(ANDROID_BUILD_TOP)/$(TARGET_TOOLS_PREFIX)" vendor/intel/support/bcmdhd-build.sh $(TARGET_DEVICE) bcm43xx $(ASUS_PROJECT)

$(PRODUCT_OUT)/obj/PACKAGING/systemimage_intermediates/system.img : build_bcmdhd
