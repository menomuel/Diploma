import rosmsg

while True:
	print(rosmsg.get_msg_text("sensor_msgs_ext/temperature", raw=True))
