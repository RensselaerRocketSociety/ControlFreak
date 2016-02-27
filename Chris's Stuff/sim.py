#===================================================================
# Quick Active Energy Control
# ------------------------------------------------------------------
# Line of thought for ACTEC condensed into a focused script for the
# 2016 USLI competition. Data represents RPI 2016 rocket.
#
# Author: Chris Andre (chrisandre01@gmail.com); Team Red Gemini, RPI
#===================================================================
#===================================================================
# TODO
# ------------------------------------------------------------------
# * Hand this thing off to Philip and Paul. Chris is employed full-
#   time at the moment.
#===================================================================
from numpy.random import normal
from math import *
from vec import *
from thrustcurve import *
import numpy as np
from matplotlib import pyplot
import time
from variations import *

class Mem: pass # Memory =D!
#===================================================================
# ROCKET PARAMETERS
#===================================================================
THRUSTER = build_thrust_model('data/AeroTech_K1103.eng')
tmass, pmass, mmass, burntime, curve = THRUSTER
from param import *
#===================================================================
# CONTROLLER
#===================================================================
from control import *
#===================================================================
# INTEGRATOR
#-------------------------------------------------------------------
# There's nothing to change here.
#===================================================================
DT = 0.01

def integrate(r0, v0, m0):
	t = 0.0
	rx, ry = r0
	vx, vy = v0
	dx, dy = v0
	ax, ay = 0.0, -9.8066
	hist = []
	CTRL_MEM = Mem()
	control(rx, ry, vx, vy, ax, ay, dx, dy, t=None, dt=DT, mem=CTRL_MEM) # init controller
	while vy >= 0.0:
		# DRAG
		fuzz_pts = fuzz(rx, ry, vx, vy, ax, ay, dx, dy, t, DT)
		frx, fry, fvx, fvy, fax, fay, fdx, fdy, ft, fdt = fuzz_pts
		u, ctrl_pts = control(frx, fry, fvx, fvy, fax, fay, fdx, fdy, ft, fdt, CTRL_MEM)
		m = get_mass(t, burntime, mmass, pmass) + m0 - tmass
		rho = 1.225 * exp(-1.0/8000.0 * ry)
		vm = mag(vx, vy)
		dynp = 0.5 * rho * vm**2
		drag_r = mem.CD * mem.A * dynp
		uf = mem.CD_C * mem.A_C * u * dynp
		drag = drag_r + uf
		dx, dy = unit(vx, vy)
		a_drag_x, a_drag_y = scale(dx, dy, -drag/m)
		# GRAVITY
		a_grav_x, a_grav_y = 0.0, -9.8066
		# THRUST
		thr = get_thrust(curve, t)
		a_thrust_x, a_thrust_y = scale(dx, dy, thr / m)
		# STATE INTEGRATION
		ax = a_drag_x + a_grav_x + a_thrust_x
		ay = a_drag_y + a_grav_y + a_thrust_y

		rxi = rx + vx * DT + ax * 0.5 * DT * DT
		ryi = ry + vy * DT + ay * 0.5 * DT * DT
		vxi = vx + ax * DT
		vyi = vy + ay * DT

		hist.append( (rx, ry, vx, vy, m, ax, ay, t, dynp, drag, thr, u, uf, ctrl_pts, fuzz_pts) )
		rx, ry, vx, vy = rxi, ryi, vxi, vyi
		t += DT
	return hist
#===================================================================
# ANALYSIS AND PLOTTING
#===================================================================
from plotting import *
#===================================================================
# MONTE-CARLO SIMULATION
#===================================================================
# Perform simulations
hists = []
for i in range(10):
	mem = fuzzy_rocket()
	start = time.clock()
	hist = integrate(xR0, mem.V0, mem.M)
	hists.append(hist)
	elapsed = time.clock() - start

	# Pull out the data points
	rx, ry, vx, vy, m, ax, ay, t, dynp, drag, thr, u, uf, ctrl_pts, fuzz_pts = pull(hist, indices=range(len(hist[0])))

	# Some stats
	#print 'Integration time:', elapsed, 's'
	print 'Apogee:', ry[-1], 'm'
	#print 'Apogee time:', t[-1], 's'
	#maxg = max(hist, key=lambda x: x[6])
	#print 'Max G force: %f Gees at t = %f s' % (maxg[6] / 9.8066, maxg[7])
	#maxq = max(hist, key=lambda x: x[8])
	#print 'Max Q: %f N/m^2 at t = %f s' % (maxq[8], maxq[7])

plot_state_history(hists)
plot_control_program(hists)
plot_ctrl_data(hists)
plot_feas_region(hist=hists)
#===================================================================
# OPTIMIZATION
#-------------------------------------------------------------------
# I haven't explored this just yet. I think we won't need this.
#===================================================================
def score(apogee):
	feet = 3.28084 * apogee
	if feet > 5600: # 5600 feet correct?
		return 0.0
	if feet > 5280.0:
		return 5280.0 - 2.0 * (feet - 5280.0)
	elif feet <= 5280.0:
		return feet