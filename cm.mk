## Specify phone tech before including full_phone
$(call inherit-product, vendor/cm/config/gsm.mk)

# Release name
PRODUCT_RELEASE_NAME := mint

# Inherit some common CM stuff.
$(call inherit-product, vendor/cm/config/common_full_phone.mk)

# Inherit device configuration
$(call inherit-product, device/samsung/mint/device_mint.mk)

## Device identifier. This must come after all inclusions
PRODUCT_DEVICE := mint
PRODUCT_NAME := cm_mint
PRODUCT_BRAND := samsung
PRODUCT_MODEL := mint
PRODUCT_MANUFACTURER := samsung
