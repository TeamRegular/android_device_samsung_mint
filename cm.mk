## Specify phone tech before including full_phone
$(call inherit-product, vendor/cm/config/gsm.mk)

# Release name
PRODUCT_RELEASE_NAME := mint

# Inherit some common CM stuff.
$(call inherit-product, vendor/cm/config/common_full_phone.mk)

# Inherit device configuration
$(call inherit-product, device/samsung/mint/full_mint.mk)

PRODUCT_BUILD_PROP_OVERRIDES += \
    PRODUCT_NAME=mint \
    TARGET_DEVICE=mint \
    BUILD_FINGERPRINT="samsung/mintxx/mint:4.1.2/JZO54K/S5282XXAMEA:user/release-keys" \
    PRIVATE_BUILD_DESC="mintxx-user 4.1.2 JZO54K S5282XXAMEA release-keys"

PRODUCT_NAME := cm_mint
PRODUCT_DEVICE := mint
