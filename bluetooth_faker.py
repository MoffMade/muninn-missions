__author__ = 'moff'
import time
import os
import random
message_file_path = os.path.join(os.getcwd(), 'message.out')
last_message_time = 0
while True:
    with open(message_file_path, 'w') as f:
        hover_alt = str(random.randint(20, 50))
        gps_lat = str(30.622397)
        gps_log = str(-96.335168)
        f.write("launch_land:launch;flight_mode:hover;hover_distance:"+hover_alt+";loop_radius:20;GPS:"+gps_lat+","+gps_log+";")
    print "Wrote new message"
    time.sleep(20)