import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msgs import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber

def plot(msg_left, msg_center):

    global plot_teb, plot_pose,  fig

    plot_teb.set_xdata(msg_left.poses.pose.position.x)
    plot_teb.set_ydata(msg_left.poses.pose.position.y)

    plot_pose.set_xdata(msg_center.pose.position.x)
    plot_pose.set_ydata(msg_center.pose.position.y)

    


if __name__ == '__main__':
    rospy.init_node("plotter")

    plt.ion()

    subTeb = Subscriber("/teb_local_planner/local_plan", Path)
    subPose = Subscriber("/robot_pose", PoseStamped)
    

    plt.figure()

    fig = plt.gcf()
    ax = plt.gca()

    ax.set_xlim(0, 5000)
    ax.set_ylim(0, 3000)

    plot_teb, = ax.plot(0, label='Trajectory of the TEB approach')
    plot_pose, = ax.plot(0, label='actual trajectorie of the robot')
    ax.legend()

    fig.show()
    fig.canvas.draw()

    ats = ApproximateTimeSynchronizer([subTeb, subPose], queue_size=5, slop=0.2, allow_headerless=False)
    ats.registerCallback(plot)

    plt.show(block=True)

