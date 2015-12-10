__author__ = 'moff'
import time
import os
import random
message_file_path = os.path.join(os.getcwd(), 'message.out')
last_message_time = 0
while True:
    alt = str(30)
    gps_lat = str(30.622397)
    gps_log = str(-96.335168)
    messages = [
        "launch_land:launch;flight_mode:hover;hover_distance:"+alt+";loop_radius:20;GPS:"+gps_lat+","+gps_log+";",
        "launch_land:launch;flight_mode:loop;hover_distance:"+alt+";loop_radius:20;GPS:"+gps_lat+","+gps_log+";",
        "launch_land:launch;flight_mode:hover;hover_distance:"+alt+";loop_radius:30;GPS:30.622902,-96.335163;",
        "launch_land:land;flight_mode:hover;hover_distance:"+alt+";loop_radius:20;GPS:"+gps_lat+","+gps_log+";",
        "launch_land:land;flight_mode:loop;hover_distance:"+alt+";loop_radius:20;GPS:"+gps_lat+","+gps_log+";",
        "launch_land:launch;flight_mode:fawf;hover_distance:"+alt+";loop_radius:20;GPS:"+gps_lat+","+gps_log+";",
        "awdwdwdwaa"
        ]
    selected_message = raw_input("Which message to send? 0-launch/hover 1-launch/loop 2-hover at different location 3-land/hover 4-land/loop 5-invalid fmode 6-gibberish :: ")
    try:
        with open(message_file_path, 'w') as f:
            msg = messages[int(selected_message)]
            f.write(msg)
            print "Wrote message - "+msg+'\n'
    except (IndexError, NameError, ValueError):
        print "Invalid input. Please use 0-6"
