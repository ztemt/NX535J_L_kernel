include $(srctree)/drivers/misc/mediatek/Makefile.custom

#microtrust  add for fpc irq driver begin
ifeq ($(strip $(CONFIG_FINGERPRINT_FPC_ISEE)),y)

obj-y +=fpc_irq.o
fpc_irq-y := fpc_irq_main.o fpc_irq_supply.o fpc_irq_ctrl.o fpc_irq_pm.o fpc_irq_common.o
endif
#microtrust  modify for fpc irq driver end
