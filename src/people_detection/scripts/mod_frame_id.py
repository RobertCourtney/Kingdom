import rosbag
from copy import deepcopy
import tf

bagInName = '/home/matthew/Desktop/software_eng_interview/2016-10-25-11-09-42.bag'
bagIn = rosbag.Bag(bagInName)

bagOutName = '/home/matthew/Desktop/software_eng_interview/2016-10-25-11-09-42_corrected_frames.bag'
bagOut = rosbag.Bag(bagOutName,'w')

with bagOut as outbag:
    for topic, msg, t in bagIn.read_messages():
        #if '/Multisense' in topic:
        if '/velodyne' in topic or '/Multisense' in topic:
            try:
                if msg.header.frame_id[0] == "/":
                    msg.header.frame_id = msg.header.frame_id[1:len(msg.header.frame_id)]
            except:
                pass
            outbag.write(topic, msg, t)
        else:
            outbag.write(topic, msg, t)

bagIn.close()
bagOut.close()
