#uint8[6] channel_id		# ultrasound channel
#uint64[6] channel_value	# ultrasound channel value in volt. adc_value is 16 bits, so channel_value[i] can stage 4 adc_values once

uint64 timestamp		# time since system start (microseconds)

uint16[200] channel_value  