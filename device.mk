$(call inherit-product, $(SRC_TARGET_DIR)/product/languages_full.mk)

# The gps config appropriate for this device
$(call inherit-product, device/common/gps/gps_us_supl.mk)

$(call inherit-product-if-exists, vendor/samsung/mint/mint-vendor.mk)

DEVICE_PACKAGE_OVERLAYS += device/samsung/mint/overlay

LOCAL_PATH := device/samsung/mint

$(shell mkdir -p $(LOCAL_PATH)/../../../out/target/product/mint/recovery/root/system/bin)
$(shell ln -sf -t $(LOCAL_PATH)/../../../out/target/product/mint/recovery/root/system/bin ../../sbin/sh)

PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/rootdir/init.sp8810.rc:root/init.sp8810.rc \
    $(LOCAL_PATH)/rootdir/init.sp8810.usb.rc:root/init.sp8810.usb.rc

PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/rootdir/fstab.sp8810:root/fstab.sp8810

PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/rootdir/ueventd.sp8810.rc:root/ueventd.sp8810.rc

PRODUCT_COPY_FILES += \
    vendor/cm/prebuilt/common/bootanimation/480.zip:system/media/bootanimation.zip

PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/vold.conf:system/etc/vold.conf

$(call inherit-product, build/target/product/full.mk)

PRODUCT_AAPT_CONFIG := normal ldpi mdpi nodpi
PRODUCT_AAPT_PREF_CONFIG := ldpi

