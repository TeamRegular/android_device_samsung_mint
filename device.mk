#
# Copyright (C) 2013 The CyanogenMod Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

$(call inherit-product, $(SRC_TARGET_DIR)/product/languages_full.mk)

## (2) Also get non-open-source specific aspects if available
$(call inherit-product-if-exists, vendor/samsung/mint/mint-vendor.mk)

# Use the Dalvik VM specific for devices with 512 MB of RAM
$(call inherit-product, frameworks/native/build/phone-hdpi-512-dalvik-heap.mk)

## overlays
DEVICE_PACKAGE_OVERLAYS += device/samsung/mint/overlay

LOCAL_PATH := device/samsung/mint

# Softlink sh
$(shell mkdir -p $(LOCAL_PATH)/../../../out/target/product/mint/recovery/root/system/bin)
$(shell ln -sf -t $(LOCAL_PATH)/../../../out/target/product/mint/recovery/root/system/bin ../../sbin/sh)

# Init Files
PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/rootdir/init.sp8810.rc:root/init.sp8810.rc \
    $(LOCAL_PATH)/rootdir/init.sp8810.usb.rc:root/init.sp8810.usb.rc \
    $(LOCAL_PATH)/rootdir/fstab.sp8810:root/fstab.sp8810 \
    $(LOCAL_PATH)/rootdir/lpm.rc:root/lpm.rc \
    $(LOCAL_PATH)/rootdir/ueventd.sp8810.rc:root/ueventd.sp8810.rc \
    $(LOCAL_PATH)/rootdir/modem_control:root/modem_control \

PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/vold.fstab:system/etc/vold.fstab \

# Recovery
#PRODUCT_COPY_FILES += \
#    $(LOCAL_PATH)/rootdir/init.recovery.sp8810.rc:root/init.recovery.sp8810.rc

# Idc
PRODUCT_COPY_FILES += \
     $(LOCAL_PATH)/idc/sec_touchscreen.idc:system/usr/idc/sec_touchscreen.idc

# Keylayout
PRODUCT_COPY_FILES += \
     $(LOCAL_PATH)/keylayout/sprd-keypad.kl:system/usr/keylayout/sprd-keypad.kl

# Media
PRODUCT_COPY_FILES += \
     $(LOCAL_PATH)/media/media_codecs.xml:system/etc/media_codecs.xml \
     $(LOCAL_PATH)/media/media_profiles.xml:system/etc/media_profiles.xml

# Graphics
PRODUCT_PROPERTY_OVERRIDES += \
    ro.opengles.version=131072 \

# These are the hardware-specific settings that are stored in system properties.
# Note that the only such settings should be the ones that are too low-level to
# be reachable from resources or other mechanisms.
PRODUCT_PROPERTY_OVERRIDES += \
    wifi.interface=wlan0 \
    wifi.supplicant_scan_interval=150 \
    ro.telephony.ril_class=SamsungBCMRIL

# Audio
PRODUCT_PACKAGES += \
    audio.a2dp.default \
    audio.usb.default

PRODUCT_COPY_FILES += \
     $(LOCAL_PATH)/configs/audio_policy.conf:system/etc/audio_policy.conf \
     $(LOCAL_PATH)/configs/default_gain.conf:system/etc/default_gain.conf \
     $(LOCAL_PATH)/configs/tiny_hw.xml:system/etc/tiny_hw.xml \
     $(LOCAL_PATH)/configs/tinyucm.conf:system/etc/tinyucm.conf

PRODUCT_PROPERTY_OVERRIDES := \
	keyguard.no_require_sim=true \
	ro.com.android.dataroaming=false \
	persist.msms.phone_count=1 \
	persist.sys.sprd.modemreset=1

# Sensors
#PRODUCT_PACKAGES += \
#    lights.sc8810

# Set default USB interface
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += \
	persist.sys.usb.config=mtp

# Charger
PRODUCT_PACKAGES += \
    charger \
    charger_res_images

# Permissions
PRODUCT_COPY_FILES += \
    frameworks/native/data/etc/android.hardware.telephony.gsm.xml:system/etc/permissions/android.hardware.telephony.gsm.xml \
    frameworks/native/data/etc/android.hardware.wifi.direct.xml:system/etc/permissions/android.hardware.wifi.direct.xml \
    frameworks/native/data/etc/handheld_core_hardware.xml:system/etc/permissions/handheld_core_hardware.xml \
    frameworks/native/data/etc/android.hardware.bluetooth.xml:system/etc/permissions/android.hardware.bluetooth.xml \
    frameworks/native/data/etc/android.hardware.sensor.accelerometer.xml:system/etc/permissions/android.hardware.sensor.accelerometer.xml \
    frameworks/native/data/etc/android.hardware.touchscreen.multitouch.jazzhand.xml:system/etc/permissions/android.hardware.touchscreen.multitouch.jazzhand.xml \
    frameworks/native/data/etc/android.hardware.usb.accessory.xml:system/etc/permissions/android.hardware.usb.accessory.xml \
    frameworks/native/data/etc/android.hardware.usb.host.xml:system/etc/permissions/android.hardware.usb.host.xml \
    frameworks/native/data/etc/android.hardware.wifi.xml:system/etc/permissions/android.hardware.wifi.xml \
    frameworks/native/data/etc/android.software.sip.voip.xml:system/etc/permissions/android.software.sip.voip.xml \
    packages/wallpapers/LivePicker/android.software.live_wallpaper.xml:system/etc/permissions/android.software.live_wallpaper.xml

# Filesystem management tools
PRODUCT_PACKAGES += \
    setup_fs

# Misc packages
PRODUCT_PACKAGES += \
    com.android.future.usb.accessory

# enable Google-specific location features,
# like NetworkLocationProvider and LocationCollector
PRODUCT_PROPERTY_OVERRIDES += \
    ro.com.google.locationfeatures=1 \
    ro.com.google.networklocation=1

# Extended JNI checks
# The extended JNI checks will cause the system to run more slowly, but they can spot a variety of nasty bugs 
# before they have a chance to cause problems.
# Default=true for development builds, set by android buildsystem.
PRODUCT_PROPERTY_OVERRIDES += \
    ro.kernel.android.checkjni=0 \
    dalvik.vm.checkjni=false

## LDPI assets
PRODUCT_AAPT_CONFIG := normal ldpi mdpi nodpi
PRODUCT_AAPT_PREF_CONFIG := ldpi
$(call inherit-product, device/ldpi-common/ldpi.mk)
