# EMC2101 FAN Controller on OrangePi Zero2

`note > This was operated on Armbian, with kernel 6.1

+ Install the kernel headers

```shell
sudo dpkg -i /opt/linux-headers-*_arm64.deb
```

+ Clone the repo

```shell
git clone \<REPO\>

cd \<REPO_DIR\>
```

+ Compile the device tree overlays

```shell
dtc -@ -I dts -O dtb -o sun50i-h616-ph-emc2101.dtbo dts/orange-pi-zero2-emc2101.dts
```

+ Compile drivers and install

```shell
cd drivers/hwmon

# compile the kernel driver
make

# install
sudo make install
```

Then you can either use

```shell
sudo modprobe emc2101
```

or add the following content to `/etc/modules-load.d/emc2101.conf`:

```txt
# Load emc2101 fan controller module at boot
emc2101
```

+ Configure device tree overlay with `orangepi-config`

```shell
sudo orangepi-config
```

Navigate to `System` -> `Hardware` and select `ph-emc2101` and `ph-i2c3` (and you can check `spi` if you want to use it).

<div align="center">
  <img
    src="https://raw.githubusercontent.com/parker-int64/emc2101_fan/main/screenshot/orangipi-config-screenshot.webp"
    width="400"
  />
</div>

Follow the prompt to reboot the device, make sure you connect the device correctly.

+ Check the device tree nodes and driver load status

```shell
# By default the i2c3 on OrangePi Zero2
# It should return 'okay'
cat /proc/device-tree/soc/i2c@5002c00/emc2101@4c/status
```

```shell
sudo dmesg | grep emc2101

# [    9.036560] emc2101: loading out-of-tree module taints kernel.
# [    9.036770] emc2101: module verification failed: signature and/or required key missing - tainting kernel
# [   14.997614] emc2101 3-004c: [01, 10]: diode fault (open)
```

And device nodes:

```shell
cat /sys/class/hwmon/hwmon4/name
# emc2101
ls  /sys/class/hwmon/hwmon4/
# device              pwm1                     pwm1_auto_point6_pwm       temp1_input              temp2_max_alarm
# fan1_div            pwm1_auto_channels_temp  pwm1_auto_point6_temp      temp1_label              temp2_min
# fan1_input          pwm1_auto_point1_pwm     pwm1_auto_point7_pwm       temp1_max                temp2_min_alarm
# fan1_min            pwm1_auto_point1_temp    pwm1_auto_point7_temp      temp1_max_alarm          temp2_type
# ...
```


 
