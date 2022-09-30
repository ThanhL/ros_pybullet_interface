#!/usr/bin/env python3
import rospy
import optas
import tf2_ros
import numpy as np
from dmp.data_collector import DMPDataCollector
from dmp.dmp_proc import DMPProcessor
from cob_srvs.srv import SetString
from sensor_msgs.msg import JointState
from ros_pybullet_interface.msg import PybulletObject
from geometry_msgs.msg import TwistStamped, Wrench, WrenchStamped
from custom_ros_tools.ros_comm import get_srv_handler
from custom_ros_tools.config import replace_package
from ros_pybullet_interface.srv import AddPybulletObject, AddPybulletObjectRequest
from ros_pybullet_interface.srv import ResetJointState, ResetJointStateRequest
from ros_pybullet_interface.msg import KeyboardEvent

class KeyboardListener:

    def __init__(self, key_action_map):
        self.key_action_map = key_action_map
        rospy.Subscriber('rpbi/keyboard', KeyboardEvent, self._callback)

    def _callback(self, msg):
        if msg.state_str == 'key_was_released':
            handle = self.key_action_map.get(msg.key)
            if handle:
                handle()

class KukaController:

    def __init__(self, dt, thresh_angle):

        urdf_filename = replace_package('{ik_ros_examples}/robots/kuka_lwr.urdf')
        pi = optas.np.pi  # 3.141...
        T = 1 # no. time steps in trajectory
        link_ee = 'end_effector_ball'  # end-effector link name

        # Setup robot
        kuka = optas.RobotModel(
            urdf_filename=urdf_filename,
            time_derivs=[1],  # i.e. joint velocity
        )
        kuka_name = kuka.get_name()

        # Setup optimization builder
        builder = optas.OptimizationBuilder(T=T, robots=[kuka], derivs_align=True)

        # Setup parameters
        qc = builder.add_parameter('qc', kuka.ndof)  # current robot joint configuration
        pg = builder.add_parameter('pg', 3)  # goal end-effector position

        # Get joint velocity
        dq = builder.get_model_state(kuka_name, t=0, time_deriv=1)

        # Get next joint state
        q = qc + dt*dq

        # Cost: nominal joint configuration
        qnom = optas.DM(np.deg2rad([0., 30., 0., -90., 0., 60., 0.]))
        builder.add_cost_term('qnom', 10.*optas.sumsqr(q - qnom))

        # Get jacobian
        Jl = kuka.get_global_linear_geometric_jacobian(link_ee, qc)

        # Get end-effector velocity
        dp = Jl @ dq

        # Get current end-effector position
        pc = kuka.get_global_link_position(link_ee, qc)

        # Get next end-effector position
        p = pc + dt*dp

        # Cost: match end-effector position
        pnom = kuka.get_global_link_position(link_ee, qnom)
        # diffv = dp - vg
        pg += pnom  # ensure pg is in global frame
        diffp = p[:2] - pg[:2]
        # W_v = optas.diag([50., 50., 1.])
        W_p = optas.diag([1e9, 1e9])
        builder.add_cost_term('match_goal', diffp.T @ W_p @ diffp)

        # Cost: minimize velocity in z
        builder.add_cost_term('min_z', 1e6*optas.sumsqr(p[2] - pnom[2]))

        # Cost: min joint velocity
        w_dq = 10
        builder.add_cost_term('min_dq', w_dq*optas.sumsqr(dq))

        # Get global z-axis of end-effector
        T = kuka.get_global_link_transform(link_ee, q)
        z = T[:3, 2]

        # Constraint: eff orientation
        e = optas.DM([0, 0, -1.])
        builder.add_leq_inequality_constraint('eff_orien', optas.cos(thresh_angle), e.T @ z)

        # Cost: align eff
        w_ori = 1e6
        builder.add_cost_term('eff_orien', w_ori*optas.sumsqr(e.T @ z - 1))

        # Setup solver
        optimization = builder.build()
        # self.solver = optas.CasADiSolver(optimization).setup('sqpmethod')
        self.solver = optas.CasADiSolver(optimization).setup('ipopt')

        # Setup variables required later
        self.kuka_name = kuka_name
        self.actuated_joint_names = kuka.actuated_joint_names
        self.dt = dt
        self.qnom = qnom.toarray().flatten()
        self.ee_pos = kuka.get_global_link_position_function(link_ee)

        # Start ROS subscriber
        self._qc = np.zeros(kuka.ndof)
        self.joint_state_pub = rospy.Publisher('rpbi/kuka_lwr/joint_states/target', JointState, queue_size=1)
        rospy.Subscriber('rpbi/kuka_lwr/joint_states', JointState, self._joint_state_callback)

    def _joint_state_callback(self, msg):
        for i, name in enumerate(self.actuated_joint_names):
            idx = msg.name.index(name)
            self._qc[i] = msg.position[idx]

    def command(self, pg):
        qc = self._qc.copy()
        self.solver.reset_parameters({'qc': optas.DM(qc), 'pg': optas.DM(pg)})
        solution = self.solver.solve()
        qg = qc + self.dt*solution[f'{self.kuka_name}/dq'].toarray().flatten()

        msg = JointState(name=self.actuated_joint_names, position=qg.tolist())
        msg.header.stamp = rospy.Time.now()
        self.joint_state_pub.publish(msg)

class FTSensorListener:

    scale = 0.125
    alpha = 0.925

    def __init__(self):
        self.reset()
        topic = 'rpbi/kuka_lwr/lwr_arm_7_joint/ft_sensor/calibrated'
        rospy.Subscriber(topic, WrenchStamped, self._callback)

    def reset(self):
        self._force = np.zeros(3)

    def _callback(self, msg):
        fx = -self.scale*msg.wrench.force.x
        fy = self.scale*msg.wrench.force.y
        self._force = self.alpha*self._force + (1.-self.alpha)*np.array([-fx, 0, fy])

    def get_force(self):
        return self._force

class EEPosGoalListener:

    scale = 7.5

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

    def get_ee_pos(self):
        try:
            tf = self.tf_buffer.lookup_transform('touch_x_base', 'touch_x_ee', rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            tf = None
        return tf

    def transform_pos_goal(self, p):
        return self.scale*np.array([-p[0], p[2], 0.])

    def get_ee_pos_goal(self):
        ee = self.get_ee_pos()
        if ee is None: return ee
        p = np.array([
            ee.transform.translation.x,
            ee.transform.translation.y,
            ee.transform.translation.z,
        ])
        return self.transform_pos_goal(p)

class EEVelGoalListener:

    K = np.diag([60., 60., 1.])

    def __init__(self):
        self._ee_vel_goal = np.zeros(3)
        rospy.Subscriber('geomagic_touch_x_node/twist', TwistStamped, self.callback)

    def callback(self, msg):
        self._ee_vel_goal = self.K@np.array([
            -msg.twist.linear.x,
            msg.twist.linear.z,
            0.,
        ])

    def get_ee_vel_goal(self):
        return self._ee_vel_goal

class ToCentreForceListener:

    def __init__(self):
        self._f = np.zeros(3)
        rospy.Subscriber('geomagic_touch_x_node/to_centre_force', Wrench, self.callback)

    def callback(self, msg):
        self._f = np.array([msg.force.x, msg.force.y, msg.force.z])

    def get_to_centre_force(self):
        return self._f

class Node:

    hz = 100
    dt = 1.0/float(hz)
    dur = rospy.Duration(dt)
    max_f = 2.5

    def __init__(self):
        rospy.init_node('nsl_node')

        self.ft_sensor_listener = FTSensorListener()

        thresh_angle = np.deg2rad(30.)
        self.kuka_controller = KukaController(self.dt, thresh_angle)

        self.ee_pos_goal_listener = EEPosGoalListener()
        # self.ee_vel_goal_listener = EEVelGoalListener()
        self.to_centre_force_listener = ToCentreForceListener()

        self._executing_dmp = False
        self.dmp = None
        self.goal = self.ee_pos_goal_listener.transform_pos_goal(np.array([-1., 0., 0.15]))
        self.dmp_proc = DMPProcessor()
        self.data_collector = DMPDataCollector(interpolate=50)

        self._teleop_is_on = False
        self._teleop_start_time = None
        self._teleop_no_ff_delay = 1. # sec

        self.move_to_initial_joint_state = get_srv_handler(
            f'rpbi/kuka_lwr/move_to_initial_joint_state', ResetJointState, persistent=True)
        self.move_to_joint_state = get_srv_handler(
            f'rpbi/kuka_lwr/move_to_joint_state', ResetJointState, persistent=True)
        self.add_pybullet_object = get_srv_handler(
            'rpbi/add_pybullet_object', AddPybulletObject, persistent=True)
        self.remove_pybullet_object = get_srv_handler(
            'rpbi/remove_pybullet_object', SetString, persistent=True)

        key_action_map = {
            ord('t'): self.toggle_teleop,
            ord('l'): self.learn_dmp,
            ord('e'): self.exec_learned_plan,
        }
        KeyboardListener(key_action_map)

        self._teleop_timer = None
        self.wrench_pub = rospy.Publisher('geomagic_touch_x_node/cmd_force', Wrench, queue_size=1)
        rospy.Timer(self.dur, self.force_feedback_update)

    def move_robot_to_home(self):
        req = ResetJointStateRequest(duration=.5)
        self.move_to_initial_joint_state(req)

    def add_box(self):
        obj = PybulletObject()
        obj.object_type = PybulletObject.DYNAMIC
        obj.filename = "{rpbi_examples}/configs/nsl/box.yaml"
        req = AddPybulletObjectRequest(pybullet_object=obj)
        resp = self.add_pybullet_object(req)

    def remove_box(self):
        self.remove_pybullet_object('pushing_box')

    def toggle_teleop(self):
        if not self._teleop_is_on:
            # turn on teleop
            self.move_robot_to_home()
            rospy.sleep(1.)
            self.add_box()
            self.data_collector.reset()
            self._teleop_start_time = rospy.Time.now().to_sec()
            self._teleop_timer = rospy.Timer(self.dur, self.teleop_update)
        else:
            # turn off teleop
            self._teleop_timer.shutdown()
            self.ft_sensor_listener.reset()
            self.remove_box()
            rospy.sleep(1)
            self.move_robot_to_home()
            self._teleop_timer = None

        # Update variable
        self._teleop_is_on = not self._teleop_is_on

    def teleop_update(self, event):
        pg = self.ee_pos_goal_listener.get_ee_pos_goal()
        if pg is None: return
        pg = np.clip(pg, [-0.325, -100, -100], [100, 100, 100]) # prevents haptic device going wild near limits
        self.data_collector.log(rospy.Time.now().to_sec(), pg)
        self.kuka_controller.command(pg)

    def learn_dmp(self):
        if self.data_collector.is_empty(): return

        t, pos_traj = self.data_collector.get()
        k_gain = 100.0
        d_gain = 2.0*np.sqrt(k_gain)
        num_bases = 4
        self.dmp = self.dmp_proc.learn_dmp(t, pos_traj, k_gain, d_gain, num_bases)

        rospy.logwarn("Learned DMP")

    def exec_learned_plan(self):
        self._executing_dmp = True
        self.move_robot_to_home()

        # qgoal = self.kuka_controller.qnom + np.random.uniform(-1, 1, size=(7,))
        # js = JointState(name=self.kuka_controller.actuated_joint_names, position=qgoal)
        # req = ResetJointStateRequest(duration=3., joint_state=js)
        # self.move_to_joint_state(req)

        self.add_box()

        # pos0 = self.kuka_controller.ee_pos(qgoal).toarray().flatten()


        pos0 = np.array(self.data_collector.get_first()[1])
        rlim = 0.1
        vel0 = np.zeros(3)
        t0 = 0.0
        goal_thresh = np.array([0.03, 0.03, 0.03])
        seg_length = -1
        tau = 2.0*self.dmp.tau
        dt = 1.0/200.
        integrate_iter = 5
        success, t, pos_traj, vel_traj = self.dmp_proc.generate_plan(
            self.dmp, pos0, vel0, t0, np.array(self.data_collector.get_last()[1]),
            goal_thresh, seg_length, tau, dt,
            integrate_iter,
        )
        self.t_plan = t
        self.pos_traj_plan = pos_traj

        rate = rospy.Rate(int(1.0/dt))
        N = pos_traj.shape[1]
        for i in range(N):
            pg = pos_traj[:,i]
            self.kuka_controller.command(pg)
            rate.sleep()

        self._executing_dmp = False
        rospy.logwarn('Executed trajectory')
        rospy.sleep(2.)
        self.remove_box()

    def force_feedback_update(self, event):
        f2c = self.to_centre_force_listener.get_to_centre_force()
        ff = 0.
        if self._teleop_is_on:
            if (rospy.Time.now().to_sec() - self._teleop_start_time) < self._teleop_no_ff_delay:

                # this is required because otherwise, when teleop is
                # started there is always a spike in the force
                # feedback that can cause the haptic device to go
                # wild, this small hack avoids this issue. I think
                # this has to do with the fact that the robot is
                # tracking the position of the haptic device and at
                # the start the user is never quite at zero so there
                # is an initial high velocity causing the spike. Plus,
                # if the robot is close to the object then there could
                # be an instantaneous high contact force compounding
                # the issue.

                ff = 0.
            else:
                ff = self.ft_sensor_listener.get_force()

        f = np.clip(f2c + ff, -self.max_f, self.max_f)

        msg = Wrench()
        msg.force.x = f[0]
        msg.force.y = f[1]
        msg.force.z = f[2]
        self.wrench_pub.publish(msg)

    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
