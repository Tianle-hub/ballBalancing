import rospy
import rosbag
import pandas as pd
from ball_controller.msg import PosVel2D
from geometry_msgs.msg import Vector3Stamped

class PositionRecorder:

    def  __init__(self, bag_path, csv_path):

        rospy.Subscriber('/ball_pos_vel', PosVel2D, self.callback)
        self.bag = rosbag.Bag(bag_path, 'w')


    def callback(self, PosVel2D):

        self.bag.write('PosVel2D', PosVel2D)
        

    def closeBag(self):

        self.bag.close()


def bagToCsv(bag_path, csv_path):

    bag = rosbag.Bag(bag_path, 'r')

    column_names = ['x', 'y']
    

    data = []

    for topic, msg, t in bag.read_messages(topics = 'PosVel2D'):

        row = {
            'x': msg.position.x,
            'y': msg.position.y}
        
        data.append(row)

    df = pd.DataFrame(data, columns=column_names)
    print(df.head())
    df.to_csv(csv_path)

    print('standard deviation:')
    print(df.std())
    print('variance:')
    print(df.var())

    bag.close()


if __name__ == '__main__':

    rospy.init_node('ball_position_recorder', anonymous=True, disable_signals=True)

    bag_file_path = 'src/ball_controller/data/position.bag'
    csv_file_path = 'src/ball_controller/data/position.csv'
    
    try:
        recorder = PositionRecorder(bag_file_path, csv_file_path)
        print('recording...')
        rospy.spin()

    except KeyboardInterrupt:
        recorder.closeBag()
        print('Closed ros bag successfully.')

        bagToCsv(bag_file_path, csv_file_path)
        print('Converted bag to csv successfully.')



        rospy.signal_shutdown('manually shutdown ros node.')