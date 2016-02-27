#===================================================================
# Quick Active Energy Control
# ------------------------------------------------------------------
# Author: Chris Andre (chrisandre01@gmail.com)
#===================================================================
from matplotlib import pyplot

def pull(records, index=None, indices=None):
	if not index is None:
		return [r[index] for r in records]
	else:
		return [[r[i] for r in records] for i in indices]

def plot_state_history(hist):
	f, (ax1, ax2, ax3) = pyplot.subplots(3, sharex=True, sharey=False)
	for h in hist:
		rx, ry, vx, vy, m, ax, ay, t, dynp, drag, thr, u, uf, ctrl_pts, fuzz_pts = pull(h, indices=range(len(h[0])))
		ax1.plot(t, ay)
		ax2.plot(t, vy)
		ax3.plot(t, ry)

	ax1.set_title('Simulation state history')
	ax1.grid(True)
	ax1.set_ylabel('Acceleration (m/s^2)')
	ax2.grid(True)
	ax2.set_ylabel('Velocity (m/s)')
	ax3.grid(True)
	ax3.set_ylabel('Altitude (m)')
	pyplot.xlabel('Time (s)')
	pyplot.show()

def plot_control_program(hist):
	f, (ax1, ax2, ax3) = pyplot.subplots(3, sharex=True, sharey=False)
	for h in hist:
		rx, ry, vx, vy, m, ax, ay, t, dynp, drag, thr, u, uf, ctrl_pts, fuzz_pts = pull(h, indices=range(len(h[0])))
		ax1.plot(t, u)
		ax2.plot(t, dynp)
		ax3.plot(t, uf)
	
	ax1.set_title('Simulation control program')
	ax1.axis([0.0,t[-1],0.0,1.0])
	ax1.grid(True)
	ax1.set_ylabel('Control input')
	ax2.set_ylabel('Dynamic pressure (Pa)')
	ax2.grid(True)
	ax3.grid(True)
	ax3.set_ylabel('Control contribution (N)')
	pyplot.xlabel('Time (s)')
	pyplot.show()

def plot_ctrl_data(hist):
	f, (ax1, ax2, ax3, ax4) = pyplot.subplots(4, sharex=True, sharey=False)
	for h in hist:
		rx, ry, vx, vy, m, ax, ay, t, dynp, drag, thr, u, uf, ctrl_pts, fuzz_pts = pull(h, indices=range(len(h[0])))
		cry, cvy, cy0, cy1, ceH = pull(ctrl_pts, indices = range(5))
		frx, fry, fvx, fvy, fax, fay, fdx, fdy, ft, fdt = pull(fuzz_pts, indices=range(10))
		ax1.plot(t, cry)
		ax2.plot(t, cvy)
		ax3.plot(t, fay)
		ax4.plot(t, cy0, 'r-', t, cy1, 'r-', t, ry, 'k-', t, ceH, 'b--')
	
	ax1.set_title('Controller data')
	ax1.grid(True)
	ax1.set_ylabel('Measured altitude (m)')
	ax2.grid(True)
	ax2.set_ylabel('Inferred speed (m/s)')
	ax3.grid(True)
	ax3.set_ylabel('Measured acceleration (m/s^2)')
	ax4.grid(True)
	ax4.set_ylabel('Control authority')
	pyplot.xlabel('Time (s)')
	pyplot.show()

def plot_feas_region(target_alt=1609.344, xCD_C=1.28, hist=None):
	from control import proj_alt_time
	import numpy as np
	import math
	alt_hint = target_alt
	vel_hint = 500.0
	pyplot.title('Region of feasibility for CD_C = %0.3f' % xCD_C)
	pyplot.xlabel('Altitude (m)')
	pyplot.ylabel('Velocity (m/s)')
	alts = np.arange(0.0, alt_hint, 1.0)
	minr, maxr = [], []
	for alt in alts: # this can be done by propagating backwards...
		# u = 0.0, lower bound
		va, vb = 0.0, vel_hint
		while math.fabs(va - vb) > 1.0e-3:
			ra, _ = proj_alt_time(alt, va, 0.0)
			rb, _ = proj_alt_time(alt, vb, 0.0)
			if target_alt >= rb:
				# Not enough energy
				minr.append(vel_hint)
				break
			vc = 0.5 * (va + vb)
			rc, _ = proj_alt_time(alt, vc, 0.0)
			if target_alt < rc:
				vb = vc
			elif target_alt > rc:
				va = vc
			else:
				pass # edge edge edge case
		vc = 0.5 * (va + vb)
		minr.append(vc)
		# u = 1.0, upper bound
		va, vb = 0.0, vel_hint
		while math.fabs(va - vb) > 1.0e-3:
			ra, _ = proj_alt_time(alt, va, 1.0)
			rb, _ = proj_alt_time(alt, vb, 1.0)
			if target_alt <= ra:
				# Too much energy
				maxr.append(0.0)
				break
			vc = 0.5 * (va + vb)
			rc, _ = proj_alt_time(alt, vc, 1.0)
			if target_alt < rc:
				vb = vc
			elif target_alt > rc:
				va = vc
			else:
				pass # edge edge edge case
		vc = 0.5 * (va + vb)
		maxr.append(vc)
	pyplot.fill_between(alts, 0, minr, facecolor='grey', interpolate=True)
	pyplot.fill_between(alts, maxr, vel_hint, facecolor='grey', interpolate=True)
	if not hist is None:
		for h in hist:
			rx, ry, vx, vy, m, ax, ay, t, dynp, drag, thr, u, uf, ctrl_pts, fuzz_pts = pull(h, indices=range(len(h[0])))
			pyplot.plot(ry, vy)
	pyplot.grid(True)
	pyplot.axis([0, alt_hint, 0, vel_hint])
	pyplot.yticks(np.arange(0.0,vel_hint, 20.0))
	pyplot.xticks(np.arange(0.0,alt_hint, 50.0))
	pyplot.show()