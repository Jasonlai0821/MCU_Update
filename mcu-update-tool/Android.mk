LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := mcu_update
LOCAL_MODULE_TAGS := optional

LOCAL_C_INCLUDES += system/core/include \
                    frameworks/base/include \
                    frameworks/base/core/jni \


LOCAL_SRC_FILES := mcu_version.c \
                   gpio_control.c \
                   mcu_update.c

LOCAL_SHARED_LIBRARIES := libandroid_runtime \
                          libbinder \
                          libutils \
                          libcutils \
                          libnativehelper

LOCAL_CFLAGS += -Werror
LOCAL_LDLIBS := -llog

LOCAL_MULTILIB := 32

include $(BUILD_EXECUTABLE)
