import rosbag
from std_msgs.msg import Int32, String

bag = rosbag.Bag('/home/userk/Downloads/augmented_depth.bag', 'r')

print("A")


with rosbag.Bag('/home/userk/Downloads/rename_depth.bag', 'w') as outbag:
    for topic, msg, t in bag.read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/Multisense/left/camera_info" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)


try:
    bag.read_messages()
    for topic, msg, t in bag.read_messages(topics=['/Multisense/left/camera_info']):
        print(msg)


    #bag.write('chatter', s)
    #bag.write('numbers', i)
except Exception as e:
    print(e)
#    bag.close()
