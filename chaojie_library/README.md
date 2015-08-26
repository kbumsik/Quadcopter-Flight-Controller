This is a place where I put library I wrote.

### cj_state.h
Untested. State estimation based on sequentially updating the speed from the angular velocity.

### cj_state2.h
Tested. Another implementation of state estimation based on this [paper](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=4419916&tag=1). This one works well if quadcopter only has small acceleration, in other words, it is in slow motion. Note that this implementation can be combined with first implementation to deal with angle estimation for sudden change in motion.

### cj_helper.h
Tested. Important helper functions. 

### cj_ctrl.h
Untested. This is a Linear-quadratic regulator(LQR). It looks like a PD(proportional-derivative) controller, in which each contant K is determined by using the Linear-quadratic regulator. Consult my review for determine that(some quantities needs to be measured, like moment of inertia, length of wings).
