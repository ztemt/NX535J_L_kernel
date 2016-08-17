include $(srctree)/drivers/misc/mediatek/Makefile.custom
TMP_CFLAGS := -I$(srctree)/sound/soc/mediatek/mt_soc_audio_6755/
ccflags-y  += $(TMP_CFLAGS)

obj-y += sound_dummy.o
obj-$(CONFIG_SND_SOC_AK4376) += ak4376.o
