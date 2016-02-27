#===================================================================
# Quick Active Energy Control
# ------------------------------------------------------------------
# Author: Chris Andre (chrisandre01@gmail.com)
#===================================================================

from numba import jit
from math import sqrt

@jit
def mag(x, y):
	return sqrt(x*x + y*y)

@jit
def scale(x, y, a):
	return x*a, y*a

@jit
def unit(x, y):
	m = mag(x, y)
	return x/m, y/m

@jit
def dot(ax, ay, bx, by):
	return ax*bx + ay*by

@jit
def cross(ax, ay, bx, by):
	return ax*by-bx*ay

@jit
def add(ax, ay, bx, by):
	return ax+bx, ay+by