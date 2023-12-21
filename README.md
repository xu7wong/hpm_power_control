# hpm_power_control
HPM6200 based power control

> openocd -f jlink.cfg -f hpm6280-single-core.cfg  -f hpm6200evk.cfg -c 'program build/app.elf verify reset exit'