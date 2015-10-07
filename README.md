# kalman_filter

This respository is only meant for storing the files on the implementation of "kalman filter" for "TEAM AUV (IIT Kharagpur)" for testing purposes only.

Used the following linear equations :
x(k) = x(k-1) + v(k-1)*dt + 0.5*a(k-1)*dt*dt
v(k) = v(k-1) + a(k-1)*dt
a(k) = a(k-1)

taking measuremnts for only 'v' and 'a' but tracking all three state variables.
