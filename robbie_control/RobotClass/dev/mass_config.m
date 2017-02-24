disp('configuring masses and relative positions')
obj.com_shank_footprint = com_data
obj.com_shank_footprint.rel = [0.163037 0 0.159341 1]'
obj.com_thigh_link.rel = [0 0 0.1815 1]'
obj.com_torso_link.rel = [0 0 0.192 1]'
obj.com_lhm_link.rel = [0 0 0.02 1]'
obj.com_lhm_wheel_right_link.rel = [0 0 0 1]'
obj.com_lhm_wheel_left_link.rel = [0 0 0 1]'
obj.com_shoulder_left_link.rel = [0 0.16 -0.15 1]'
obj.com_shoulder_right_link.rel = [0 -0.16 -0.15 1]'
obj.com_arm_left_link.rel = [0 0 -0.1325 1]'
obj.com_arm_right_link.rel = [0 0 -0.1325 1]'
obj.com_stab_link.rel = [0 0 0.04625 1]'
obj.com_stab_wheel.rel = [0 0 0 1]'
obj.com_wheel_left_link.rel = [0 0 0 1]'
obj.com_wheel_right_link.rel = [0 0 0 1]'


obj.com_shank_footprint.mass = 7
obj.com_thigh_link.mass = 2
obj.com_lhm_link.mass = 2.25
obj.com_torso_link.mass = 16
obj.com_lhm_wheel_right_link.mass = 0.2
obj.com_lhm_wheel_left_link.mass = 0.2
obj.com_shoulder_left_link.mass = 0.25
obj.com_shoulder_right_link.mass = 0.25
obj.com_arm_left_link.mass = 0.25
obj.com_arm_right_link.mass = 0.25
obj.com_stab_link.mass = 0.04625
obj.com_stab_wheel.mass = 0.75
obj.com_wheel_left_link.mass = 0.25
obj.com_wheel_right_link.mass = 0.25

% obj.total_mass = obj.com_shank_footprint.mass + obj.com_thigh_link.mass + obj.com_lhm_link.mass + obj.com_torso_link.mass + obj.com_lhm_wheel_right_link.mass + obj.com_lhm_wheel_left_link.mass + obj.com_shoulder_left_link.mass + obj.com_shoulder_right_link.mass + obj.com_arm_left_link.mass + obj.com_arm_right_link.mass + obj.com_stab_link.mass + obj.com_stab_wheel.mass + obj.com_wheel_left_link.mass + obj.com_wheel_right_link.mass


         % obj.com_shank_footprint.rel = [0.163037 0 0.159341 1]'
         % obj.com_thigh_link.rel = [0 0 0.1815 1]'
         % obj.com_torso_link.rel = [0 0 0.192 1]'
         % obj.com_lhm_link.rel = [0 0 0.02 1]'
         % obj.com_lhm_wheel_right_link.rel = [0 0 0 1]'
         % obj.com_lhm_wheel_left_link.rel = [0 0 0 1]'
         % obj.com_shoulder_left_link.rel = [0 0.16 -0.15 1]'
         % obj.com_shoulder_right_link.rel = [0 -0.16 -0.15 1]'
         % obj.com_arm_left_link.rel = [0 0 -0.1325 1]'
         % obj.com_arm_right_link.rel = [0 0 -0.1325 1]'
         % obj.com_stab_link.rel = [0 0 0.04625 1]'
         % obj.com_stab_wheel.rel = [0 0 0 1]'
         % obj.com_wheel_left_link.rel = [0 0 0 1]'
         % obj.com_wheel_right_link.rel = [0 0 0 1]'


         % obj.com_shank_footprint.mass = 7
         % obj.com_thigh_link.mass = 2
         % obj.com_lhm_link.mass = 2.25
         % obj.com_torso_link.mass = 16
         % obj.com_lhm_wheel_right_link.mass = 0.2
         % obj.com_lhm_wheel_left_link.mass = 0.2
         % obj.com_shoulder_left_link.mass = 0.25
         % obj.com_shoulder_right_link.mass = 0.25
         % obj.com_arm_left_link.mass = 0.25
         % obj.com_arm_right_link.mass = 0.25
         % obj.com_stab_link.mass = 0.04625
         % obj.com_stab_wheel.mass = 0.75
         % obj.com_wheel_left_link.mass = 0.25
         % obj.com_wheel_right_link.mass = 0.25

         % obj.com_shank_footprint.setMatrix
         % obj.com_thigh_link.mat(:, 4) = obj.com_thigh_link.rel
         % obj.com_lhm_link.mat(:, 4) = obj.com_thigh_link.rel
         % obj.com_torso_link.mat(:, 4) = obj.com_torso_link.rel
         % obj.com_lhm_wheel_right_link.mat(:, 4) = obj.com_lhm_wheel_right_link.rel
         % obj.com_lhm_wheel_left_link.mat(:, 4) = obj.transform_matrix
         % obj.com_shoulder_left_link.mat(:, 4) = obj.transform_matrix
         % obj.com_shoulder_right_link.mat(:, 4) = obj.transform_matrix
         % obj.com_arm_left_link.mat(:, 4) = obj.transform_matrix
         % obj.com_arm_right_link.mat(:, 4) = obj.transform_matrix
         % obj.com_stab_link.mat(:, 4) = obj.transform_matrix
         % obj.com_stab_wheel.mat(:, 4) = obj.transform_matrix
         % obj.com_wheel_left_link.mat(:, 4) = obj.transform_matrix
         % obj.com_wheel_right_link.mat(:, 4) = obj.transform_matrix

         % obj.com_.mat = obj.tranform_matrix;
         % arg.mat(:, 4) = obj.;
         % end
