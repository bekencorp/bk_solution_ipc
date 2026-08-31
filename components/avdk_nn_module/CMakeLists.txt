set(incs
    include
    ../../tflite_micro/gesture_detection/ap/ui/beken_generated
)
set(srcs)

if (CONFIG_TFLITE_MICRO)
	list(APPEND srcs
		src/core/box.c
		src/core/AvdkDetectionModel.cc
		src/core/AvdkMultiModel.cc
		src/core/AvdkVideoReatorOSD.cc
	)
endif()

if (CONFIG_TFLM_PERSON_DETECTION_RGB OR CONFIG_TFLM_PERSON_DETECTION_GRAY)
	list(APPEND srcs
		src/tflm_person_detection/tflm_person_detection.cc
	)
	if (CONFIG_TFLM_PERSON_DETECTION_RGB)
		list(APPEND srcs
			src/tflm_person_detection/person_detect_rgb_model_data.cc
		)
	elseif (CONFIG_TFLM_PERSON_DETECTION_GRAY)
		list(APPEND srcs
			src/tflm_person_detection/person_detect_gray_model_data.cc
		)
	endif()
endif()

if (CONFIG_TFLM_FACE_DETECTION)
	list(APPEND srcs
		src/tflm_face_detection/tflm_face_detection.cc
		src/tflm_face_detection/face_detect_model_data.cc
	)
endif()

set(priv_req bk_common unity include avdk_utils bk_tflite_micro multimedia_device_service bk_peripheral)

if (CONFIG_SDCARD)
	list(APPEND priv_req fatfs)
endif()


armino_component_register(SRCS "${srcs}" INCLUDE_DIRS "${incs}" PRIV_REQUIRES "${priv_req}")

add_definitions(
	-DTF_LITE_STATIC_MEMORY
	-DTF_LITE_DISABLE_X86_NEON
	-DTF_LITE_USE_CTIME
	-DTF_LITE_USE_GLOBAL_CMATH_FUNCTIONS
	-DCMSIS_NN
	-DKERNELS_OPTIMIZED_FOR_SPEED
)
