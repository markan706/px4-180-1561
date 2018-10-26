include(configs/nuttx_px4fmu-v5_default)

set(GENERATE_RTPS_BRIDGE off)  

list(APPEND config_module_list
	modules/micrortps_bridge
)
