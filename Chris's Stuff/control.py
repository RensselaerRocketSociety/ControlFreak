#===================================================================
# Quick Active Energy Control
# ------------------------------------------------------------------
# Author: Chris Andre (chrisandre01@gmail.com)
#===================================================================

from numpy.random import normal
from math import *
from vec import *
from variations import *
from param import *

TARGET = 1609.344 # meters, 1658.112 meters unaltered
#===================================================================
# REQUIRED INTERFACE
#===================================================================
def fuzz(rx, ry, vx, vy, ax, ay, dx, dy, t, dt):
	"""
	Simulate noisy sensor data here.
	"""
	ry = normal(ry, 5.0e-9 * dt) # fuzzy altimeter
	rx = 0.0 # No data on this!
	vx = 0.0 # No data on this!
	vy = 0.0 # No data on this!
	ax = ModulateGaussian(ax, 2.0) # fuzzy accelerometer
	ay = ModulateGaussian(ay, 2.0) # fuzzy accelerometer
	dx = ModulateGaussian(dx, 2.0) # fuzzy gyroscope
	dy = ModulateGaussian(dy, 2.0) # fuzzy gyroscope
	dt = 0.0 # No data on this!
	return (rx, ry, vx, vy, ax, ay, dx, dy, t, dt)

def control(rx, ry, vx, vy, ax, ay, dx, dy, t, dt, mem):
	"""
	Controller loop function. Called every loop.
	This directly controls u, the fraction of the drag flap
	maximum angle (~40 degrees). It is neccessary to
	incorporate a motor model for best accuracy. I do not
	think we can do this since someone else will have to
	translate that to C/C++ without my assistance. K.I.S.S.
	-------------------------------------
	Right now it simply calls the user-defined functions to get
	the best control and then serves that after 4 seconds.
	"""
	#==================
	# Initialization of memory - it is an empty object at this point.
	if t is None:
		mem.last_t = -dt
		mem.last_v = 0.0
		return
	#==================
	# Actual loop
	#==================
	# Integrate accelerometer +Z reading (y since this is a 2D sim)
	# This should be changed so that it only starts integrating
	# velocity after about 4 seconds because the G force on the
	# rocket is too high at the beginning for accurate results.
	# I suggest that you set mem.last_v to the derivative of your
	# altimeter reading at the 4.0 second mark and integrate from
	# there. So you need to add mem.last_ry and differentiate that.
	vy = ay * (t - mem.last_t) + mem.last_v
	mem.last_t = t
	mem.last_v = vy
	#==================
	# Get min and max heights with max and min controls for graphing
	# (not needed in the actual controller)
	y0, t0 = proj_alt_time(ry, vy, 0.0) # max height, u = 0.0
	y1, t1 = proj_alt_time(ry, vy, 1.0) # min height, u = 1.0
	#==================
	# Only start at t > 4.0
	if t > 4.0:
		u = optimal_control(ry, vy)
	else:
		u = 0.0
	#==================
	# Expect apogee height and time with given control
	eH, eT = proj_alt_time(ry, vy, u)
	#==================
	# Export control and some extra data to graph.
	return u, (ry, vy, y0, y1, eH)
#===================================================================
# USER-DEFINED FUNCTIONS
#===================================================================
def proj_alt_time(ry, vy, u, xCD_C=1.28):
	"""
	Projected apogee and time of the rocket given a control.
	"""
	# Reference: http://physics.eou.edu/opensource/physics/drag/drag_math.pdf
	eCD = xCD * xA + xCD_C * xA_C * u
	vt = sqrt(xM * xG / (0.5 * xRho * eCD))
	ttop = vt / xG * atan(vy / vt)
	ytop = ry + vt**2 / 2 / xG * log(1.0 + vy**2 / vt**2)

	return ytop, ttop

def optimal_control(ry, vy):
	"""
	Search for best control for target apogee.
	"""
	# Simple application of false position method
	a, b = 0.0, 1.0
	while fabs(a - b) > 1.0e-3:
		fa, _ = proj_alt_time(ry, vy, a)
		fb, _ = proj_alt_time(ry, vy, b)
		if TARGET <= fb:
			return 1.0
		elif TARGET >= fa:
			return 0.0
		c = 0.5 * (a + b)
		fc, _ = proj_alt_time(ry, vy, c)
		if TARGET < fc:
			a = c
		elif TARGET > fc:
			b = c
		else:
			return c # edge edge edge case
	return 0.5 * (a + b)

def clamp(u):
	if u < 0.0:
		return 0.0
	elif u > 1.0:
		return 1.0
	else:
		return u
#===================================================================
# MOTOR MODEL
#===================================================================
"""
I think we should just ignore this for now, unless Paul wants to
figure something out.
"""