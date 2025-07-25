#!/bin/bash
export CROSS_COMPILE=/data/lt4-gcc/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
# export INSTALL_MOD_PATH=$PWD/../rootfs
export KERNEL_HEADERS=$PWD/kernel/kernel-jammy-src
make -C kernel
make dtbs
# sudo -E make install -C kernel
# cp kernel/kernel-jammy-src/arch/arm64/boot/Image  ../kernel/Image
#sudo make modules
