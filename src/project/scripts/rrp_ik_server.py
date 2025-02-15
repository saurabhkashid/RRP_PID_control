#!/usr/bin/env python3

from math import cos, atan2, sqrt

import rospy
from rbe500_project.srv import rrpIK

# RRP bot constants
l1 = 0.425
l2 = 0.345


def get_rrp_ik(request):
	x = request.x
	y = request.y
	z = request.z

	num = x**2 + y**2 - l1**2 - l2**2
	denom = 2 * l1 * l2
	cos_term = num/denom
	sin_term = sqrt(1 - cos_term ** 2)
	theta_2 = atan2(sin_term, cos_term)

	alpha = atan2(y, x)
	num_ = l1 + l2 * cos(theta_2)
	denom_ = sqrt(x**2 + y**2)
	cos_term_ = num_/denom_
	sin_term_ = sqrt(1 - cos_term_ ** 2)
	beta = atan2(sin_term_, cos_term_)

	theta_1 = beta - alpha

	d3 = 0.39 - z
	return theta_1, theta_2, d3

# def get_rrp_ik(request):
#     x = request.x
#     y = request.y
#     z = request.z

#     # Compute theta_2 (elbow angle)
#     num = x**2 + y**2 - l1**2 - l2**2
#     denom = 2 * l1 * l2
#     cos_term = num / denom

#     # Ensure cos_term is within valid range [-1,1] to prevent sqrt() domain errors
#     cos_term = max(-1, min(1, cos_term))

#     sin_term = sqrt(1 - cos_term ** 2)
#     theta_2 = atan2(sin_term, cos_term)  # Choose the "elbow-up" solution

#     # Compute theta_1 (shoulder angle)
#     alpha = atan2(y, x)
#     beta = atan2(l2 * sin_term, l1 + l2 * cos_term)  # Corrected beta computation

#     theta_1 = alpha - beta

#     # Compute d3 (prismatic joint extension)
#     d3 = 0.39 - z

#     return theta_1, theta_2, d3

# def get_rrp_ik(request):
# 	x = request.x
# 	y = request.y
# 	z = request.z

# 	theta_1 = atan2(y/x)

# 	s_ = z - l1
# 	r = sqrt(pow(x,2)+pow(y,2))
# 	theta_2 = atan2(s_/r)
# 	# Compute d3 (prismatic joint extension)
# 	d3 = 0.39 - z

# 	return theta_1, theta_2, d3

def rrp_ik_server():
	rospy.init_node("rrp_ik_server")
	_ = rospy.Service("calc_rrp_ik", rrpIK, get_rrp_ik)
	rospy.spin()


if __name__ == '__main__':
	rrp_ik_server()
