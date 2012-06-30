#!/bin/bash

for sig_name in mcasp0_fsr ecap0_in_pwm0_out mcasp0_fsx mcasp0_axr0 mcasp0_aclkx;
do
		echo SIGNAL NAME: $sig_name
		cat /sys/kernel/debug/omap_mux/$sig_name
		echo
done
