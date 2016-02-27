#===================================================================
# Quick Active Energy Control
# ------------------------------------------------------------------
# Author: Chris Andre (chrisandre01@gmail.com)
#===================================================================

from numpy.random import normal
from math import *
from vec import *
#===================================================================
# VARIABLE METHODS
#===================================================================
def PercentTolerance(val,tol,std=None):
	if std is None:
		std = tol / 2.0
	tol = tol / 100.0
	std = std / 100.0
	mod = normal(1.0, std)
	if mod < 1.0 - tol:
		mod = 1.0 - tol
	if mod > 1.0 + tol:
		mod = 1.0 + tol
	return mod * val
def ModulateGaussian(value, std_percent):
	return value * normal(1.0, std_percent / 100.0)

def AddGaussian(value, std):
	return normal(value, std)