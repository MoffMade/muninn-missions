__author__ = 'moff'
import time
import os
import random
message_file_path = os.path.join(os.getcwd(), 'message.out')
last_message_time = 0
while True:
    with open(message_file_path, 'w') as f:
        hover_alt = str(random.randint(20, 50))
        gps_lat = str(random.random()+100)
        gps_log = str(random.random()+100)
        f.write("flight_mode:hover;hover_distance:"+hover_alt+";GPS:"+gps_lat+","+gps_log+";")
    print "Wrote new message\n"
    time.sleep(20)