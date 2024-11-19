#!/usr/bin/python3
import rospy
import numpy
import scipy.spatial
from matplotlib import pyplot
from hdl_localization.msg import *

from std_msgs.msg import Bool, Float32, String, Int32MultiArray


class Plotter(object):
	def __init__(self, ratio):
		pyplot.ion()
		pyplot.show(block=False)

		self.status_buffer = []
		# self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
		self.status_sub = rospy.Subscriber('/status', ScanMatchingStatus, self.status_callback)
		self.status_pub = rospy.Publisher('/localization_matching_status', Bool, queue_size=1)
		self.launcher_pub = rospy.Publisher("/rdv_launcher/cmd", String, queue_size=1)
		self.pub_neon = rospy.Publisher('/mcu/neon', Int32MultiArray, queue_size=1)
		
		self.ratio =ratio


	def status_callback(self, status_msg):
		self.status_buffer.append(status_msg)

		if len(self.status_buffer) > 50:
			self.status_buffer = self.status_buffer[-50:]
			
		# rospy.loginfo(f"matcing ratio: {status_msg.inlier_fraction}")
		
		# self.status_pub.publish(Float32(data=status_msg.inlier_fraction))
		
		if status_msg.inlier_fraction < self.ratio:
			self.status_pub.publish(False)
			# self.launcher_pub.publish(String(data="navigation shutdown"))
		elif status_msg.inlier_fraction >= self.ratio:
			self.status_pub.publish(True)
			
		

	# def timer_callback(self, event):
	# 	if len(self.status_buffer) < 2:
	# 		return
	#
	# 	errors = {}
	# 	for status in self.status_buffer:
	# 		for label, error in zip(status.prediction_labels, status.prediction_errors):
	# 			if label.data not in errors:
	# 				errors[label.data] = []
	#
	# 			quat = [error.rotation.x, error.rotation.y, error.rotation.z, error.rotation.w]
	# 			trans = [error.translation.x, error.translation.y, error.translation.z]
	#
	# 			t = status.header.stamp.secs + status.header.stamp.nsecs / 1e9
	# 			t_error = numpy.linalg.norm(trans)
	# 			r_error = numpy.linalg.norm(scipy.spatial.transform.Rotation.from_quat(quat).as_rotvec())
	#
	# 			if len(errors[label.data]) and abs(errors[label.data][-1][0] - t) > 1.0:
	# 				errors[label.data] = []
	#
	# 			errors[label.data].append((t, t_error, r_error))
	#
	# 	pyplot.clf()
	# 	for label in errors:
	# 		errs = numpy.float64(errors[label])
	# 		pyplot.subplot('211')
	# 		pyplot.plot(errs[:, 0], errs[:, 1], label=label)
	#
	# 		pyplot.subplot('212')
	# 		pyplot.plot(errs[:, 0], errs[:, 2], label=label)
	#
	# 	pyplot.subplot('211')
	# 	pyplot.ylabel('trans error')
	# 	pyplot.subplot('212')
	# 	pyplot.ylabel('rot error')
	#
	# 	pyplot.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=len(errors))
	# 	pyplot.gcf().canvas.flush_events()
	# 	# pyplot.pause(0.0001)


def main():
	rospy.init_node('status_plotter')
	
	ratio = rospy.get_param('~ratio', 0.85)
	node = Plotter(ratio)
	rospy.spin()

if __name__ == '__main__':
	main()
