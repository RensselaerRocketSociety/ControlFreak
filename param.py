#===================================================================
# Quick Active Energy Control
# ------------------------------------------------------------------
# Author: Chris Andre (chrisandre01@gmail.com); Team Red Gemini, RPI
#===================================================================
import math
import variations

class Mem: pass # Memory =D!
#===================================================================
# ROCKET PARAMETERS
# ------------------------------------------------------------------
# xValue = expected Value, average Value
# vValue = variation in Value, or tolerance
#===================================================================
xCD,   vCD   = 0.78,     1.0e-9   # percent variation
xA,    vA    = 8.107e-3, 1.0e-9   # percent variation
xM,    vM    = 4.99,     10.0     # percent variation
xCD_C, vCD_C = 1.28,     1.0e-9   # percent variation
xA_C,  vA_C  = 0.0103,   1.0e-9   # percent variation
xG           = 9.8066             # no variation
xRho         = 1.225              # no variation

xY0, vY0 = math.pi/2.0, 1.0e-9    # percent variation
xV0 = [math.cos(xY0), math.sin(xY0)]
xR0 = [0.0, 0.0]
#-----------------------------
# Construct a rocket with slightly fuzzed parameters.
def fuzzy_rocket():
	mem = Mem()
	mem.CD = variations.PercentTolerance(xCD, vCD)
	mem.A = variations.PercentTolerance(xA, vA)
	mem.M = variations.PercentTolerance(xM, vM)
	mem.CD_C = variations.PercentTolerance(xCD_C, vCD_C)
	mem.A_C = variations.PercentTolerance(xA_C, vA_C)
	mem.Y0 = variations.PercentTolerance(xY0, vY0)
	mem.V0 = [math.cos(mem.Y0), math.sin(mem.Y0)]
	return mem