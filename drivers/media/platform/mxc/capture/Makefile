ifeq ($(CONFIG_VIDEO_MXC_IPU_CAMERA),y)
	obj-$(CONFIG_VIDEO_MXC_CAPTURE) += mxc_v4l2_capture.o
	obj-$(CONFIG_MXC_IPU_PRP_VF_SDC) += ipu_prp_vf_sdc.o ipu_prp_vf_sdc_bg.o
	obj-$(CONFIG_MXC_IPU_DEVICE_QUEUE_SDC) += ipu_fg_overlay_sdc.o ipu_bg_overlay_sdc.o
	obj-$(CONFIG_MXC_IPU_PRP_ENC) += ipu_prp_enc.o ipu_still.o
	obj-$(CONFIG_MXC_IPU_CSI_ENC) += ipu_csi_enc.o ipu_still.o
endif

ov5640_camera_int-objs := ov5640.o
obj-$(CONFIG_MXC_CAMERA_OV5640) += ov5640_camera_int.o

py480_camera_int-objs := python_480_spi.o
obj-$(CONFIG_MXC_CAMERA_PY480) += py480_camera_int.o

se2100_camera_mipi_int-objs := se2100_mipi.o
obj-$(CONFIG_MXC_CAMERA_SE2100_MIPI) += se2100_camera_mipi_int.o

ov5642_camera-objs := ov5642.o
obj-$(CONFIG_MXC_CAMERA_OV5642) += ov5642_camera.o

ov5640_camera_mipi_int-objs := ov5640_mipi.o
obj-$(CONFIG_MXC_CAMERA_OV5640_MIPI) += ov5640_camera_mipi_int.o

adv7180_tvin-objs := adv7180.o
obj-$(CONFIG_MXC_TVIN_ADV7180) += adv7180_tvin.o

obj-$(CONFIG_VIDEO_V4L2_MXC_INT_DEVICE) += v4l2-int-device.o
