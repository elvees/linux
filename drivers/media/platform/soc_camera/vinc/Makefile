CFLAGS_vinc-neon.o = -ffreestanding -mfpu=neon -mfloat-abi=softfp -O3

# We don't want symbols from math to be added to vmlinux directly, so use
# subdir instead of obj.
subdir-$(CONFIG_VIDEO_VINC) += math

vinc-objs += vinc-driver.o vinc-ctrls.o vinc-hw.o vinc-neon.o math/math.o

# math.o is built in subdirectory. $(subdir-*) can't be used here, because
# prerequisites are expanded immediately and $(subdir-*) are finally defined
# later.
$(obj)/math/math.o: $(obj)/math

obj-$(CONFIG_VIDEO_VINC) += vinc.o
