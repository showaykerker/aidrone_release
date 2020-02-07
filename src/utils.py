import numpy as np
import rospy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Twist, TwistStamped, Quaternion, Vector3
from mav_msgs.msg import Actuators
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState

##################### Type Transformation #####################

def to_numpy(o): 
	"""
		Convert ROS message to numpy array.
		May be replace by ros_numpy package in the future.

		Argument:
			o : Type listed below.

		Returen:
			ret [np.array]: An array that extract value from the input message.

	"""
	if type(o) == PointStamped: return np.array([o.point.x, o.point.y, o.point.z])
	elif type(o) == Actuators: return np.array(o.angular_velocities)
	elif type(o) == Point: return np.array([o.x, o.y, o.z])
	elif type(o) == Quaternion: return np.array([o.x, o.y, o.z, o.w])
	elif type(o) == Vector3: return np.array([o.x, o.y, o.z])
	elif type(o) == Pose: return np.array( [*to_numpy(o.position)]+[*to_numpy(o.orientation)] )
	elif type(o) == PoseStamped: return to_numpy(o.pose)
	elif type(o) in [np.array, np.ndarray]: return o
	elif type(o) in [tuple, list]: return np.array(o)
	else: raise TypeError(str(type(o))+', '+str(o))


def move(model_name, to, ref='world', service_proxy=None):
		"""
			Move [model_name] to assigned pose/ position

			Argumet:
				model_name [str]: The model name in Gazebo to be moved.
				to [Pose/PoseStamped/Point/ModelState/np.array]: The assigned pose/ position
				ref [str]: Coordinate reference. [default='world']

			Return:
				res
				to: as type ModelState
		"""
		assert type(model_name) is str, type(model_name)
		assert type(to) in [Pose, PoseStamped, Point, ModelState, np.ndarray], type(to)
		assert service_proxy is None or service_proxy,resolved_name == '/gazebo/set_model_state'
		if type(to) is Pose: to = ModelState(model_name, to, Twist(), ref)
		if type(to) is PoseStamped: to = ModelState(model_name, to.pose, Twist(), ref)
		if type(to) is Point: to = ModelState(model_name, Pose(to, Quaternion(0, 0, 0, 1)), Twist(), ref)
		if type(to) is np.ndarray:
			to_pose = pose_nparray_to_msg(src=to)
			to = ModelState(model_name, to_pose, Twist(), ref)

		if service_proxy is None:
			service_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

		res = service_proxy(to)
		if res.success: pass # rospy.loginfo('Move Model %s to \n%s success.' % (model_name, str(to)))
		else: rospy.logwarn('Move Model %s to %s Failed. %s' % (model_name, str(to), res.status_message))
		return res, to


def pose_nparray_to_msg(src, type_="Pose", header=None):
		assert type(src) == np.ndarray, type(src)
		assert src.shape == (3, ) or src.shape == (4, ) or src.shape == (6, ), src.shape
		if src.shape == (3, ):
			target = Pose(Point(*src), Quaternion(0, 0, 0, 1))
		elif src.shape == (4, ):
			quat = to_quaternion(0, 0, src[-1])
			target = Pose(Point(*src[:3]), quat)
		elif src.shape == (6, ):
			quat = ut.to_quaternion(*src[-3:])
			target = Pose(Point(*src[:3]), quat)
		if type_ == "Pose":
			return target
		elif type_ == "PoseStamped":
			if header is None:
				header = Header(0, rospy.Time.now(), "world")
			return PoseStamped(header, target)


##################### Rotation Transformation #####################

def to_quaternion(roll, pitch, yaw, get_myQuaternion=False, get_msgQuaternion=False):
	"""
		Convert roll pitch yaw to quaternion.

		Arguments:
			roll, pitch, yaw [float]
			get_myQuaternion  [bool]: Returns a utils.myQuaternion object if set to True
			get_msgQuaternion [bool]: Returns a geometry_msgs/Quaternion object if set to True

		Returens:
			quaternion either as type np.array of utils.myQuaternion or geometry_msgs/Quaternion

	"""
	cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
	cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
	cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)

	qx = cy * cp * sr - sy * sp * cr
	qy = sy * cp * sr + cy * sp * cr
	qz = sy * cp * cr - cy * sp * sr
	qw = cy * cp * cr + sy * sp * sr

	if get_msgQuaternion:
		return Quaternion(qx, qy, qz, qw)
	elif get_myQuaternion:
		return myQuaternion(qx, qy, qz, qw)
	else:
		return np.array([qx, qy, qz, qw])


def to_eularian_angles(x, y=None, z=None, w=None, type_='RPY'):
	"""
		Convert quaternion to roll pitch yaw.

		Arguments:
			x [float or Quaternion or list or np.array]:
				If x is set to be Quaternion / list / array,
				it means x is the quaternion itself,
				y, z, w will be overwritten.
			y z w [float]: Only functional when x is a float
			type_   [str]: Either 'RPY' or 'PRY', specified the return order of (roll, pitch, yaw) or (pitch, roll, yaw)

		Returens:
			ret [np.array]: Eularian angle in 'PRY' or 'RPY' that represents the value of input quaternion.

	"""
	if type(x) in [list, np.ndarray]:
		x, y, z, w = x
	elif type(x) == Quaternion:
		x, y, z, w = to_numpy(x)

	ysqr = y**2

	# roll (x-axis rotation)
	t0, t1 = 2.0 * (w*x + y*z), 1.0 - 2.0*(x*x + ysqr)
	roll = np.arctan2(t0, t1)

	# pitch (y-axis rotation)
	t2 = +2.0 * (w*y - z*x)
	t2 = np.clip(t2, -1.0, 1.0)
	pitch = np.arcsin(t2)

	# yaw (z-axis rotation)
	t3, t4 = 2.0 * (w*z + x*y), 1.0 - 2.0 * (ysqr + z*z)
	yaw = np.arctan2(t3, t4)

	if type_ == 'PRY': return np.array([pitch, roll, yaw])
	elif type_ == 'RPY': return np.array([roll, pitch, yaw])
	else: raise ValueError(str(type_))


##################### Coordinate Transformation #####################

def Body_To_Inertia(observed_from_body_frame, body_frame_pose):
	'''
		Observe target from body frame and transform to inertia frame
		Note that only relative yaw will be calculated.

		Arguments:
			observed_from_body_frame [Pose, PoseStamped]: Target pose observed on body frame
			body_frame_pose          [Pose, PoseStamped]: Observer's pose observed on inertia frame

		Returens:
			rotated_poseI [Pose]: Target pose observed on inertia frame

	'''
	assert type(observed_from_body_frame) in [Pose, PoseStamped], type(observed_from_body_frame)
	assert type(body_frame_pose) in [Pose, PoseStamped], type(body_frame_pose)
	if type(observed_from_body_frame) is PoseStamped: observed_from_body_frame = observed_from_body_frame.pose
	if type(body_frame_pose) is PoseStamped: body_frame_pose = body_frame_pose.pose
	body_positionI = to_numpy(body_frame_pose.position)
	body_orientationI = myQuaternion(msg=body_frame_pose.orientation)
	target_positionB = to_numpy(observed_from_body_frame.position)
	target_orientationB = myQuaternion(msg=observed_from_body_frame.orientation)
	target_rollpitchyawB = target_orientationB.to_eularian_angles()

	if target_rollpitchyawB[0] or target_rollpitchyawB[1]:
		rospy.logwarn_throttle(60, 'Only yaw will be calculated.')

	rotated_positionI = body_orientationI.rotate(target_positionB) + body_positionI
	target_yaw = body_orientationI.to_eularian_angles()[2] + target_rollpitchyawB[2]
	rotated_orientationI = to_quaternion(0, 0, target_yaw, get_myQuaternion=True)
	rotated_positionI_msg = Point(*rotated_positionI)
	rotated_orientationI_msg = Quaternion(*rotated_orientationI.numpy())
	rotated_poseI = Pose(rotated_positionI_msg, rotated_orientationI_msg)

	return rotated_poseI


def Inertia_To_Body(observed_from_inertia_frame, body_frame_pose):
	'''
		Observe target from inertia frame and transform to body frame
		Note that only relative yaw will be calculated.

		Arguments:
			observed_from_inertia_frame [Pose]: Target pose observed on inertia frame
			body_frame_pose             [Pose]: Observer's pose observed on inertia frame

		Returens:
			rotated_poseB [Pose]: Target pose observed on body frame
	'''
	assert type(observed_from_inertia_frame) in [Pose, PoseStamped], type(observed_from_inertia_frame)
	assert type(body_frame_pose) in [Pose, PoseStamped], type(body_frame_pose)
	if type(observed_from_inertia_frame) is PoseStamped: observed_from_inertia_frame = observed_from_inertia_frame.pose
	if type(body_frame_pose) is PoseStamped: body_frame_pose = body_frame_pose.pose
	body_positionI = to_numpy(body_frame_pose.position)
	body_orientationI = myQuaternion(msg=body_frame_pose.orientation)
	target_positionI = to_numpy(observed_from_inertia_frame.position)
	target_orientationI = myQuaternion(msg=observed_from_inertia_frame.orientation)
	target_rollpitchyawI = target_orientationI.to_eularian_angles()

	if target_rollpitchyawI[0] or target_rollpitchyawI[1]:
		rospy.logwarn_throttle(60, 'Only yaw will be calculated.')


	rotated_positionB = body_orientationI.inverse_rotate(target_positionI - body_positionI)

	diff_yaw = target_rollpitchyawI[2] - body_orientationI.to_eularian_angles()[2]

	rotated_orientationB = to_quaternion(0, 0, diff_yaw, get_myQuaternion=True)
	rotated_positionB_msg = Point(*rotated_positionB)
	rotated_orientationB_msg = Quaternion(*rotated_orientationB.numpy())
	rotated_poseB = Pose(rotated_positionB_msg, rotated_orientationB_msg)

	return rotated_poseB


def SphereToCartesian(val):  # r, theta, phi-, psi -> x, y, z, yaw
	"""
		Coordinate transform from Sphere to Catesian

		Argument:
			val [list of np.array]: an array of 4 elements, (r, theta, phi-, psi)

		Return:
			ret [np.array]: an array of shape (4, ). Representing (x, y, z, yaw)
	"""
	r, the, phi_, psi = val
	return np.array([
		r * np.cos(phi_) * np.cos(the),
		r * np.cos(phi_) * np.sin(the),
		r * np.sin(phi_),
		psi
	])


def CartesianToSphere(val):  # (x, y, z, yaw) -> r, theta, phi-, psi
	"""
		Coordinate transform from Catesian to Sphere

		Argument:
			val (one of the above)
				1. an array of 4 elements, (x, y, z, yaw)
				2. geometry_msgs/Pose
				3. geometry_msgs/PoseStamped

		Return:
			ret [np.array]: an array of shape (4, ). Representing (r, theta, phi-, psi)
	"""
	if type(val) == Pose:
		x, y, z = val.position.x, val.position.y, val.position.z
		yaw = to_eularian_angles(to_numpy(val.orientation))[-1]
		val = x, y, z, yaw
	elif type(val) == PoseStamped:
		return CartesianToSphere(val.pose)

	x, y, z, yaw = val
	r = (x**2 + y**2 + z**2) ** 0.5
	return np.array([
		r,
		np.arctan2(y, x),
		np.arctan2(z, (x**2+y**2)**0.5),
		yaw
	])


##################### Quaternion Class for rotation calculations #####################

def _div(a, b):
	return a/b


def _mul(a, b):
	return a * b


class myQuaternion:
	"""
		A class that helps doing calculations of Quaternion.

		Usage:
			* 2 ways to do construction:
				* Specify x, y, z, w seperately.
					q = myQuaternion(x=0, y=0, z=0, w=1)
				* Specify using ROS message geometry_msgs/Quaternion
					q = myQuaternion(msg=Quaternion())
			* rotation(self, p)
				Quaternion can do rotation on vecotor or another quaternion.
				The calculation is p_rotated = qpq**-1
				Input p can be
					* np.ndarray of shape (3, )
					* np.ndarray of shape (4, )
					* geometry_msgs/Quaternion
					* myQuaternion
			* inverse_rotation(self, p)
				Same as 'Rotation' but with inverse direction.
				p_inverse_rotated = q**-1pq
			* conjugate(self)
				return the conjugate of self.
			* __add__(self, other)
				return myQuaternion that is the sum of self and other
			* __sub__(self, other)
			* __mul__(self, other)
			* __floordiv__(self, s)
			* __eq__(self, other)
				Defines if self is equivalent with other
			* dot(self, other)
			* norm(self)
			* normalize(self)
			* reciprocal(self)
			* numpy(self)
				Return a (4, ) shaped numpy.ndarray with [x, y, z, w]
			* vec(self)
				Return a (3, ) shaped numpy.ndarray with [x, y, z]
			* to_eularian_angles(self)
				Return [Roll, Pitch, Yaw] that is equivalent to self rotation.

	"""
	div = np.vectorize(_div)
	mul = np.vectorize(_mul)
	def __init__(self, x=0, y=0, z=0, w=1, msg=None):
		self.x = x if msg is None else msg.x
		self.y = y if msg is None else msg.y
		self.z = z if msg is None else msg.z
		self.w = w if msg is None else msg.w

	def rotate(self, p): # p' = qpq**-1
		if type(p) == list: p = np.array(v).flatten()
		if type(p) is Quaternion: p = myQuaternion(msg=p)
		if type(p) in [np.array, np.ndarray]:
			if p.shape == (3, ):
				quat = myQuaternion(p[0], p[1], p[2], 0)
				return (self * quat * self.reciprocal()).vec()
			else:
				quat = myQuaternion(*p)
				return self * quat * self.reciprocal()
		elif type(p) == type(self):
			return self * p * self.reciprocal()
		else: assert False, '%s, %s' % (type(p), type(self))

	def inverse_rotate(self, p): # p' = q**-1pq
		return self.reciprocal().rotate(p)

	def conjugate(self):
		return myQuaternion(-self.x, -self.y, -self.z, self.w)

	def __add__(self, other):
		return myQuaternion(self.x+other.x, self.y+other.y, self.z+other.z, self.w+other.w)

	def __sub__(self, other):
		return myQuaternion(self.x-other.x, self.y-other.y, self.z-other.z, self.w-other.w)

	def __mul__(self, other):
		arr = np.zeros((4, 4))
		arr[:, 0] = self.mul([ other.w,  other.x,  other.y,  other.z], self.w).reshape(1, 4)
		arr[:, 1] = self.mul([-other.x,  other.w, -other.z,  other.y], self.x).reshape(1, 4)
		arr[:, 2] = self.mul([-other.y,  other.z,  other.w, -other.x], self.y).reshape(1, 4)
		arr[:, 3] = self.mul([-other.z, -other.y,  other.x,  other.w], self.z).reshape(1, 4)

		w = np.sum(arr[0,:])
		x = np.sum(arr[1,:])
		y = np.sum(arr[2,:])
		z = np.sum(arr[3,:])

		# Command this after first test
		w_ = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
		x_ = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
		y_ = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
		z_ = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
		assert x==x_ and y==y_ and z==z_ and w==w_, ((x, x_),(y, y_),(z, z_),(w, w_))
		# Command this after first test

		return myQuaternion(x, y, z, w)

	def __floordiv__(self, s):
		return myQuaternion(*self.div(self.numpy(), s))

	def __eq__(self, other):
		return self.x == other.x and self.y == other.y and self.z == other.z and self.w == other.w

	def dot(self, other):
		return self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z

	def norm(self):
		return (self.x**2 + self.y**2 + self.z**2 + self.w**2)**0.5

	def normalize(self):
		n = self.norm()
		return self // n

	def reciprocal(self):
		return self.conjugate() // self.dot(self)

	def numpy(self):
		return np.array([self.x, self.y, self.z, self.w])

	def vec(self):
		return np.array([self.x, self.y, self.z])

	def to_eularian_angles(self):
		x, y, z, w = self.numpy()
		return to_eularian_angles(x, y, z, w, type_='RPY')

	def __str__(self):
		return 'myQuaternion(x=%f, y=%f, z=%f, w=%f) \nPRY(%s)' % (self.x, self.y, self.z, self.w, str(self.to_eularian_angles()))
