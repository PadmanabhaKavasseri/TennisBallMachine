Still running mc_v1.ino

TODO:
Make guage control stepper
    But: Make sure it doesn't go out of bounds
    Need to find angle limits
    Less than min angle pos, it should stay at limit switch. More than angle max, it should stay at other side. 
    Need to keep track of how many steps we have actually moved.
        Does Arduino send status messages back that we update on the screen?
    It would be cool to see guage dial move while homing was done

Understand message protocol and document it. Clean it up if required.