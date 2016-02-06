#===================================================================
# Quick Active Energy Control
# ------------------------------------------------------------------
# Author: Chris Andre (chrisandre01@gmail.com)
#===================================================================

from numba import jit

def build_thrust_model(filepath):
	f = open(filepath)
	s = f.readlines()
	s = [line for line in s if not line[0] is ';']
	name, dia, length, delays, pwght, twght, manu = s[0].split(' ')
	tmass = float(twght)
	pmass = float(pwght)
	mmass = tmass - pmass
	curve = []
	# Thrust curve
	for line in s[1:]:
		time, thrust = line.split()
		curve.append((float(time), float(thrust)))
	curve.insert(0, (0.0, curve[0][1]))
	burntime = curve[-1][0]
	return tmass, pmass, mmass, burntime, curve

@jit
def get_thrust(curve, time):
	if time >= curve[-1][0]:
		return 0.0
	if time <= curve[0][0]:
		return curve[0][1]
	i = 0
	while time > curve[i][0]:
		i += 1
	p0, p1 = i-1, i
	t0, t1 = curve[p0][0], curve[p1][0]
	f0, f1 = curve[p0][1], curve[p1][1]
	frac = (time-t0)/(t1-t0)
	f = frac * (f1-f0) + f0
	return f

@jit
def get_mass(time, burntime, mmass, pmass):
	# not the most correct way of doing it...
	if time >= burntime:
		return mmass
	return (burntime - time) / burntime * pmass + mmass