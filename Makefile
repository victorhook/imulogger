reset:
	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 500" -c "init; reset; exit"
