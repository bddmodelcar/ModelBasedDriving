import rospy
import bdd.msg as BDDMsg

"""Controls Aggregator node
Listens to the controls output
from each neural network topic
and does some logic to combine them
into a single output topic, which
the car then uses as its input
"""

class Aggregator():
    def __init__(self, num_nodes):
        for i in range(num_nodes):
            rospy.Subscriber('bdd/controls/node{0}'.format(i), BDDMsg.BDDControlsMsg, callback = self.callback, callback_args = i)
        self.controls_pub = rospy.Publisher('bdd/controls/car_controls', BDDMsg.BDDControlsMsg, queue_size=0)

    def callback(self, controls, node_index):
        # TODO: actually aggregate
        if node_index == 0:
            self.controls_pub.publish(controls)
