from interp_logic import Interpolation

# Test to show the interpolation logic working

# Start by creating an instance of the interpolation class
interp = Interpolation()

# Use in boundary conditions for position, velocity, acceleration and time
#
p0 = [0.5]
pf = [-0.5]
v0 = [0.0]
vf = [0.1]
a0 = [0.0]
af = [0.0]
t0 = 0.0
tf = 10.0
# Use compute_interpolation_params to computer the Time Mat and Computation Params implicitly in the class
interp.compute_interpolation_params(p0, pf, v0, vf, a0, af, t0, tf)
# get the value of p, v or a at any time instance
# t = 0.5
# interp.pva_interpolated(t)
interp.plot_trajectory(500)


p0 = [ 0.5, 1.0,-0.3]
pf = [-0.5, 0.0, 0.2]
v0 = [ 0.0, 0.0,-0.2]
vf = [ 0.5, 0.0, 0.0]
a0 = [ 0.0, 0.0, 0.0]
af = [-0.3, 0.0, 0.0]
t0 = 0.0
tf = 10.0

interp.compute_interpolation_params(p0, pf, v0, vf, a0, af, t0, tf)

# interp.pva_interpolated(t)
interp.plot_trajectory(50)