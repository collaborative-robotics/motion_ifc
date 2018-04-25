from interp_logic import Interpolation
import matplotlib.pyplot as plt
import numpy as np

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
t = 0.5
p = interp.pva_interpolated(t)
print p

# We can plot the entire trajectory by using the method below. The argument is the desired number of data points,
# it defaults to 50
interp.plot_trajectory(500)

# Similarly, we can pass any sized boundary conditions to the interpolater, the dimensions should be consistent among
# all boundary conditions
p0 = [ 0.5, 1.0,-0.3]
pf = [-0.5, 0.0, 0.2]
v0 = [ 0.0, 0.0,-0.2]
vf = [ 0.5, 0.0, 0.0]
a0 = [ 0.0, 0.0, 0.0]
af = [-0.3, 0.0, 0.0]
t0 = 0.0
tf = 10.0

interp.compute_interpolation_params(p0, pf, v0, vf, a0, af, t0, tf)

# Lets try to plot 100 data points for above BCs
interp.plot_trajectory(100)

# We can also specify a list of t and get a list of p, v and a rather than the interpolated value at a single t
t = [0.1, 0.5, 0.9, 5.6, 5.9, 9.9]

p = interp.p_interpolated(t)
v = interp.v_interpolated(t)
a = interp.a_interpolated(t)

plt.plot(t, p, '.')
plt.legend(['pos'], loc='best')
plt.show()

# Lastly, we can use linspace to get an array of times for getting the trajectory

t = np.linspace(5.0,9.0, 30)
p = interp.p_interpolated(t)

# Here is the plotted result
plt.plot(t, p, '.')
plt.legend(['pos'], loc='best')
plt.show()
