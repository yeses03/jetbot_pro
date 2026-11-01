"""Local test for scripts/odom_ekf.py; stubs ROS so it runs without Melodic."""
import importlib.util
import os
import sys
import types
import unittest


class Msg(object):
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Time(object):
    def __init__(self, t):
        self.t = t

    def __sub__(self, other):
        return Time(self.t - other.t)

    def to_sec(self):
        return self.t


class Odometry(Msg):
    def __init__(self):
        Msg.__init__(self, header=None, child_frame_id='', pose=None, twist='default-twist')


def _load():
    rospy = types.ModuleType('rospy')
    geo = types.ModuleType('geometry_msgs.msg')
    geo.PoseWithCovarianceStamped = Msg
    nav = types.ModuleType('nav_msgs.msg')
    nav.Odometry = Odometry
    for name, mod in (('rospy', rospy), ('geometry_msgs', types.ModuleType('geometry_msgs')),
                      ('geometry_msgs.msg', geo), ('nav_msgs', types.ModuleType('nav_msgs')),
                      ('nav_msgs.msg', nav)):
        sys.modules[name] = mod
    path = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'odom_ekf.py')
    spec = importlib.util.spec_from_file_location('odom_ekf', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


odom_ekf = _load()


def combined():
    cov = [0.0] * 36
    cov[7], cov[35] = 1e-6, 1.5
    cov[14] = cov[21] = cov[28] = 7.8e10
    return Msg(header=Msg(frame_id='/odom', stamp=Time(10.0)),
               pose=Msg(pose='pose', covariance=tuple(cov)))


class BuildOdometryTest(unittest.TestCase):
    def test_frames_have_no_leading_slash(self):
        o = odom_ekf.build_odometry(combined(), None, Time(10.0), Odometry)
        self.assertEqual(o.header.frame_id, 'odom')
        self.assertEqual(o.child_frame_id, 'base_footprint')

    def test_input_header_not_mutated(self):
        c = combined()
        odom_ekf.build_odometry(c, None, Time(10.0), Odometry)
        self.assertEqual(c.header.frame_id, '/odom')

    def test_pose_passthrough(self):
        o = odom_ekf.build_odometry(combined(), None, Time(10.0), Odometry)
        self.assertEqual(o.pose.pose, 'pose')

    def test_unobserved_covariance_is_capped(self):
        o = odom_ekf.build_odometry(combined(), None, Time(10.0), Odometry)
        for i in (14, 21, 28):
            self.assertEqual(o.pose.covariance[i], 1e6)

    def test_observed_covariance_is_untouched(self):
        o = odom_ekf.build_odometry(combined(), None, Time(10.0), Odometry)
        self.assertEqual(o.pose.covariance[7], 1e-6)
        self.assertEqual(o.pose.covariance[35], 1.5)

    def test_input_covariance_not_mutated(self):
        c = combined()
        odom_ekf.build_odometry(c, None, Time(10.0), Odometry)
        self.assertEqual(c.pose.covariance[14], 7.8e10)

    def test_fresh_raw_twist_is_used(self):
        raw = Msg(header=Msg(stamp=Time(9.9)), twist='raw-twist')
        o = odom_ekf.build_odometry(combined(), raw, Time(10.0), Odometry)
        self.assertEqual(o.twist, 'raw-twist')

    def test_stale_raw_twist_is_ignored(self):
        raw = Msg(header=Msg(stamp=Time(5.0)), twist='raw-twist')
        o = odom_ekf.build_odometry(combined(), raw, Time(10.0), Odometry)
        self.assertEqual(o.twist, 'default-twist')


if __name__ == '__main__':
    unittest.main()
