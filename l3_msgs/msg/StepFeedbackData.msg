l3_msgs/FootStepData foot_step
l3_msgs/BaseStepData base_step

bool changeable            # indicates if step can still be changed
bool executing             # indicates if step is currently executed
bool finished              # indicates if step has been executed

time execution_start       # timestamp when execution of step will start
time execution_end         # timestamp when execution of step will be completed
