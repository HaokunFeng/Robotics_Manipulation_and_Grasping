import rosbag
from sensor_msgs.msg import PointCloud2

class MockCamera(object): 
    """A MockCamera reads saved point clouds.
    """
    def __init__(self):
        pass

    def read_cloud(self, path):
        """Returns the sensor_msgs/PointCloud2 in the given bag file.
    
        Args:
            path: string, the path to a bag file with a single
            sensor_msgs/PointCloud2 in it.

        Returns: A sensor_msgs/PointCloud2 message, or None if there were no
            PointCloud2 messages in the bag file.
        """
        bag = rosbag.Bag(path)
        """
        for topic, msg, t in bag.read_messages():
            return msg
        """
        

        
        try:
            for topic, msg, t in bag.read_messages():
                if topic.lower() == 'head_camera/depth_registered/points':
                    return msg
                    
        except Exception as e:
            print("Error reading bag file:", e)
            return None
        
        