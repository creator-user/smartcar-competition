from __future__ import print_function
import rospy
import roslib; roslib.load_manifest('teleop_twist_keyboard')
from geometry_msgs.msg import Twist
import time
import sys, select, termios, tty,threading

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)

moveBindings = {
        5:(1,0,0,0),
        11:(1,0,0,-1),
        2:(0,0,0,1),
        4:(0,0,0,-1),
        7:(1,0,0,1),
        6:(-1,0,0,0),
        13:(-1,0,0,1),
        9:(-1,0,0,-1),
        12:(1,-1,0,0),
        1:(0,1,0,0),
        3:(0,-1,0,0),
        8:(1,1,0,0),
        14:(-1,-1,0,0),
        10:(-1,1,0,0),
        15:(0,0,0,1)
    }



if __name__=="__main__":
    key= int(sys.argv[1])
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)

    pub_thread = PublishThread(repeat)

    try:
        x = moveBindings[key][0]
        y = moveBindings[key][1]
        z = moveBindings[key][2]
        th = moveBindings[key][3]
        pub_thread.wait_for_subscribers()
        start_time = time.time()  # 记录开始时间
        while time.time() - start_time < 1.5:  # 在3秒钟内循环
            pub_thread.update(x, y, z, th, speed, turn)
            time.sleep(0.05)
            pass
        x = 0
        y = 0
        z = 0
        th = 0
        pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()