# Building kernel
Before building kernel grab gcc-linaro-6.3.1-2017.05-x86_64_arm-eabi.tar.xz from [here](https://releases.linaro.org/components/toolchain/binaries/6.3-2017.05/arm-eabi)

or if you want 64-bit kernel then gcc-linaro-6.3.1-2017.05-x86_64_aarch64-elf.tar.xz from [here](https://releases.linaro.org/components/toolchain/binaries/6.3-2017.05/aarch64-elf/)

Then run
```bash
# 32-bit
mkdir ../build-aarch32
make O=$(readlink -f ../build-aarch32) -j$(nproc) \
  ARCH=arm CROSS_COMPILE=<full path toolchain directory>/bin/arm-eabi- \
  LGE_TARGET_PLATFORM=mt6750 LGE_TARGET_DEVICE=lv5 MTK_DTBO_FEATURE=yes \
  lv5_global_com_defconfig
make O=$(readlink -f ../build-aarch32) -j$(nproc) \
  ARCH=arm CROSS_COMPILE=<full path toolchain directory>/bin/arm-eabi- \
  LGE_TARGET_PLATFORM=mt6750 LGE_TARGET_DEVICE=lv5 MTK_DTBO_FEATURE=yes \
  all
# 64-bit
mkdir ../build-aarch64
make O=$(readlink -f ../build-aarch64) -j$(nproc) \
  ARCH=arm64 CROSS_COMPILE=<full path toolchain directory>/bin/aarch64-elf- \
  LGE_TARGET_PLATFORM=mt6750 LGE_TARGET_DEVICE=lv5 MTK_DTBO_FEATURE=yes \
  lv5_global_com_defconfig
make O=$(readlink -f ../build-aarch64) -j$(nproc) \
  ARCH=arm64 CROSS_COMPILE=<full path toolchain directory>/bin/aarch64-elf- \
  LGE_TARGET_PLATFORM=mt6750 LGE_TARGET_DEVICE=lv5 MTK_DTBO_FEATURE=yes \
  all
```

# Booting kernel
Use following configuration when building BOOT.IMG

For 64-bit:
```
bootsize = 0x1300000
pagesize = 0x800
kerneladdr = 0x40100000
ramdiskaddr = 0x4A000000
secondaddr = 0x40f00000
tagsaddr = 0x44000000
name =
cmdline = bootopt=64S3,32N2,64N2 androidboot.hardware=mlv5 buildvariant=user androidboot.selinux=permissive
```
For 32-bit:
```
bootsize = 0x1300000
pagesize = 0x800
kerneladdr = 0x40008000
ramdiskaddr = 0x45000000
secondaddr = 0x40f00000
tagsaddr = 0x44000000
name =
cmdline = bootopt=64S3,32N2,32N2 androidboot.hardware=mlv5 buildvariant=user
```

To actually boot kernel you need to inject exploit into BOOT.IMG

See instructions [here](https://github.com/arturkow2000/lgk10exploit)

# Debugging kernel
You can enable UART over USB by flashing LK payload built with ENABLE_UART set.

### KDB/KGDB
If you want to use KDB/KGDB you may have to disable watchdog driver,
otherwise your device will reboot if kept paused for too long.

# What works
- Stock Oreo ROM works fine on both 32-bit and 64-bit kernel
- Audio playback/recording
- Wi-Fi, mobile network, calls, SMS
- Camera

# What does not work
- LG crash handler support is not implemented yet on 64-bit kernel,
  device will enter LK's crash handler after reboot.

# What was not tested
- Bluetooth
- GPS

# Virtualization support
64-bit kernel supports KVM.

If you want to use KVM then build kernel from kvm branch as master currently is broken.

Before starting VM disable HPS and CPU deep sleep,
otherwise you will experience kernel crash when entering sleep mode.
```bash
echo 0 > /proc/hps/enabled
for cpu in {0..7}; do
  echo 1 > "/sys/devices/system/cpu/cpu${cpu}/cpuidle/state0/disable"
  echo 1 > "/sys/devices/system/cpu/cpu${cpu}/online"
done
```
After you done with VM re-enable HPS and deep sleep to avoid draining your battery.
```bash
echo 1 > /proc/hps/enabled
for cpu in {0..7}; do
  echo 0 > "/sys/devices/system/cpu/cpu${cpu}/cpuidle/state0/disable"
done
```
