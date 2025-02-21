
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


# Waypoint Teaching

## program_robot

Uses keyboard inputs to record a set of waypoints that are compiled into a *positions.csv* file.

## program2path

Takes a recorded set of waypoints $(X,Q) \in \mathbb{R}^n$ and generates an interpolated (linear or cubic) trajectory $(X,Q) \in \mathbb{R}^m$. 

## follow_trajectory

Collect data on this new path

## teach_analysis

Compares a set of input waypoints to the measured waypoints infered from the model. 

# Mocap Teaching

## motive

Record trajectory in motive. Save to folder in record/mocap/date. Format as csv called *'mocap_take.csv'* 

## mocap_analysis

Run first two cells to generate a trajectory of waypoints. Adjust downsampling rate as seen fit.

## follow_trajectory

Collect data on this new path

## mocap_analysis

Compares a set of input waypoints to the measured waypoints infered from the model. 

# Inference pipeline (Scrubbing)

1. Visualize the workspace based on the mass to find a trajectory within the workspace.
    - Go to *compile_dataset.m* and set mass_vis to be the index of which mass you are using to refer to that dataset.
    - Using this data, find your x,y,z bounds for the path
2.  Create the path
    - Go to *generate_trajectory.m* to set appropriate bounds and specify your waypoints.
    - Run the first section and the section of your bounds to create your waypoints.
3. Run inference on the trajectory
    - Open *bot_DNN.py*
        - Verify the correct model being used for the section titled "Load inverse model instead" on or around line 54.
    - Go to *run_bot.ipynb*
        - And either use the section labelled Weighted inference data, or create a separate section using this.
        - Be sure to change the path of the training data and trajectory. Change the naming for the new path as well.
4. Following the trajectory
    - Open *follow_trajectory.m*
    - Rename the "programmed_path", "trajectory_name", and "inputs_name".
    - Run this while by the "e-stop" to ensure the arm does not move into a compromising position.
    - If "record" is not set to true, change it and re-run the arm. Once complete, set record to true.
5. Evaluate the trajectory
    - Open *evaluate_trajectory.m*.
    - Change "T_ideal" and "T_model" to the appropriate files.
    - Run and compare.


# Data Structure

The bot is currently trained on data relative to the instron's coordinate frame. Positional values are scaled by 1000 to have units of millimeters.
