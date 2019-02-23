LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

TMP_LOCAL_PATH := $(LOCAL_PATH)


ifeq ($(TARGET_USE_QTI_BT_STACK),true)
#include $(TMP_LOCAL_PATH)/wipower-host/Android.mk
include $(TMP_LOCAL_PATH)/bt_logger/Android.mk
include $(TMP_LOCAL_PATH)/libbt-logClient/Android.mk
include $(TMP_LOCAL_PATH)/BluetoothExt/Android.mk
include $(TMP_LOCAL_PATH)/hidtestapp/Android.mk
include $(TMP_LOCAL_PATH)/bttestapp/Android.mk
endif #TARGET_USE_QTI_BT_STACK

ifeq ($(TARGET_SWV8_DISK_ENCRYPTION),true)
    LOCAL_CFLAGS += -DCONFIG_SWV8_DISK_ENCRYPTION
endif
