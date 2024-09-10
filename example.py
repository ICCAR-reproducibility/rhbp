import rospy
from behaviour_components.managers import Manager

if __name__ == '__main__':
    rospy.init_node('test_sim')
    m = Manager(activationThreshold = 21, prefix = "sim") # The same prefix must be passed to the behaviours and goals that should register themselves at this planner instance. ActivationThreshold is more or less arbitrary as it is self adjusting as described above; it should not be too low.
    rate = rospy.Rate(1) # 1Hz: It does not make sense for the planner to plan faster than the fastest behaviour duration or expected environmental changes
    while(True):
        m.step()
        rate.sleep()

