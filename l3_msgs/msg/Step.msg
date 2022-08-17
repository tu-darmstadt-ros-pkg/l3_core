l3_msgs/FootStepData[] foot_steps          # stored step data (moving legs)
l3_msgs/Foothold[]     support_feet        # supporting foot (limbs which are not moving during this step)
l3_msgs/BaseStepData[] moving_bases        # stored base step data (moving floating bases)
l3_msgs/FloatingBase[] resting_bases       # supporting base (floating bases which are not moving during this step)

int32 idx                                  # index number of step; all succeeding steps must have an unique increasing index

l3_msgs/VariantDataSet data                # may contain user specific data
