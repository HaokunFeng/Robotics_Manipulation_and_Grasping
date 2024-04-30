import rosbag
bag = rosbag.Bag('/home/haokunfeng/data/table.bag')
for topic, msg, t in bag.read_messages():
   print(msg)
   print(topic)
   print(t)
bag.close()