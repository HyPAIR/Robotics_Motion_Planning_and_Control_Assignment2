import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from math import sin, cos, atan2, sqrt, pi, hypot


class PIDTrajectoryTrackingNode(Node):
    def __init__(self, traj):
        super().__init__("pid_trajectory_tracking_node")

        # Publishers and subscribers
        self.robot_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.errors_pub = self.create_publisher(Pose2D, "/errors_pub", 10)
        self.desired_traj_pub = self.create_publisher(Pose2D, "/desired_traj_pub", 10)
        self.create_subscription(Odometry, "/odom", self.robot_odom_callback, 10)

        # Variables
        self.Odom_pos = Pose2D()
        self.Odom_yaw = 0.0
        self.robot_pose = Pose2D()
        self.qd = Pose2D()
        self.err = Pose2D()

        # PID gains
        self.kvp = 2.25
        self.kwp = 1.5
        self.kvd = 0.0
        self.kwd = 0.0
        self.kvi = 0.0
        self.kwi = 0.0

        self.vm = 1.0  # max linear velocity
        self.wm = 2.8   # max angular velocity

        # State variables
        self.ev_prev = 0.0
        self.ew_prev = 0.0
        self.t = 0.0
        self.goal_reached = False
        self.index = 10
        self.freq = 0.01
        self.traj = traj
        # Timer for the control loop
        print("Trajectory tracking starts!")
        self.create_timer(0.1, self.control_loop)

    def get_distance(self, x1, y1, x2, y2):
        return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def robot_odom_callback(self, msg):
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        self.robot_pose.theta = atan2(
            2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z),
            1.0 - 2.0 * (msg.pose.pose.orientation.z ** 2),
        )
        self.Odom_yaw = self.robot_pose.theta

    def control_loop(self):
        if (self.index > len(self.traj.states)):
            self.index = len(self.traj.states) - 1
            print("Goal reached!")
            self.goal_reached = True
        # Calculate desired trajectory
        self.qd.x = self.traj.states[self.index].y
        self.qd.y = self.traj.states[self.index].x
        # xd = -self.freq * sin(self.freq * self.t)
        # yd = self.freq * cos(self.freq * self.t)
        self.qd.theta = self.traj.states[self.index].theta / 360 * 2 * pi

        self.desired_traj_pub.publish(self.qd)

        # Calculate tracking errors
        self.err.x = (self.qd.x - self.robot_pose.x) * cos(self.robot_pose.theta) + \
                     (self.qd.y - self.robot_pose.y) * sin(self.robot_pose.theta)
        self.err.y = -(self.qd.x - self.robot_pose.x) * sin(self.robot_pose.theta) + \
                     (self.qd.y - self.robot_pose.y) * cos(self.robot_pose.theta)
        self.err.theta = atan2(sin(self.qd.theta - self.robot_pose.theta),
                               cos(self.qd.theta - self.robot_pose.theta))
        self.errors_pub.publish(self.err)

        ev = self.get_distance(self.robot_pose.x, self.robot_pose.y, self.qd.x, self.qd.y)
        ew = atan2(self.qd.y - self.robot_pose.y, self.qd.x - self.robot_pose.x) - self.robot_pose.theta
        ew = atan2(sin(ew), cos(ew))

        # PID calculations
        pl = self.kvp * ev
        dl = self.kvd * ((ev - self.ev_prev) / 0.1)
        il = self.kvi * ((ev + self.ev_prev) * 0.1)
        pa = self.kwp * ew
        da = self.kwd * ((ew - self.ew_prev) / 0.1)
        ia = self.kwi * ((ew + self.ew_prev) * 0.1)

        vr = pl + il + dl
        wr = pa + ia + da

        # Limit control inputs
        vr = min(self.vm, max(-self.vm, vr))
        wr = min(self.wm, max(-self.wm, wr))
        if (hypot(self.robot_pose.x - self.traj.states[-1].x, self.robot_pose.y - self.traj.states[-1].y) < 2):
            print("Goal reached!")
            self.goal_reached = True
        if (self.goal_reached):
            vr = 0.0
            wr = 0.0
        # Publish velocity
        vel_msg = Twist()
        vel_msg.linear.x = vr
        vel_msg.angular.z = wr
        self.robot_vel_pub.publish(vel_msg)

        self.ev_prev = ev
        self.ew_prev = ew
        self.t += 0.1
        if (hypot(self.err.x, self.err.y) < 0.3):
            self.index += 10
            