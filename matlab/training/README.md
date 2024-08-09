
# State variables

(Kinematic model)
model_home: lengths based on our model
model_comp: compressed lengths based on our model
comp_delta: compression delta

all other delta the model predicts should be deltas that are applied from the compressed state 

(Actual robot)
home: cable lengths for arm to be at minimal tension
comp: compressed lengths for the arm (home + comp_delta)

# Repetability

## Ordered

## Unordered


# Teaching

## program_robot

Uses keyboard inputs to record a set of waypoints that are compiled into a *positions.csv* file.

## program2path

Takes a recorded set of waypoints $(X,Q) \in \mathbb{R}^n$ and generates an interpolated (linear or cubic) trajectory $(X,Q) \in \mathbb{R}^m$. 

## teach_analysis

Compares a set of input waypoints to the measured waypoints infered from the model. 

## mocap_analysis