# Copyright (C) 2011 The Android Open Source Project
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


ifeq ($(strip $(BOARD_USES_TINYALSA_AUDIO)),true)

LOCAL_PATH := $(call my-dir)

#TinyAlsa audio

include $(CLEAR_VARS)

LOCAL_MODULE := audio.primary.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_CFLAGS := -D_POSIX_SOURCE -Wno-multichar -g

ifeq ($(strip $(BOARD_USES_LINE_CALL)), true)
LOCAL_CFLAGS += -D_VOICE_CALL_VIA_LINEIN
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),sc8810)
LOCAL_CFLAGS += -D_DSP_CTRL_CODEC
endif

LOCAL_C_INCLUDES += \
	external/tinyalsa/include \
	external/expat/lib \
	system/media/audio_utils/include \
	system/media/audio_effects/include \
	device/sprd/common/apps/engineeringmodel/engcs \
	device/sprd/common/libs/audio/vb_effect	\
	device/sprd/common/libs/audio/vb_pga \
	device/sprd/common/libs/audio/record_process


LOCAL_SRC_FILES := audio_hw.c tinyalsa_util.c audio_pga.c \
			record_process/aud_proc_config.c \
			record_process/aud_filter_calc.c

ifeq ($(strip $(AUDIO_MUX_PIPE)), true)
LOCAL_SRC_FILES  += audio_mux_pcm.c
LOCAL_CFLAGS += -DAUDIO_MUX_PCM
endif

LOCAL_SHARED_LIBRARIES := \
	liblog libcutils libtinyalsa libaudioutils \
	libexpat libdl \
	libengclient libvbeffect libvbpga

LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

include $(call all-makefiles-under,$(LOCAL_PATH))
endif

