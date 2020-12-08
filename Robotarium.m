%% Robotarium
% A class that models your communications with the GRITSbots!
%   This class handles retrieving the poses of agents, setting their
%   velocities, iterating the simulation, and saving data.
%% Method Description
% * get_poses(): $\emptyset \to \mathbf{R}^{3 \times N}$ retrieves the
% poses of the agents in a 3 x N vector, where each column contains the
% pose of an agent.
% * set_velocities(): $\mathbf{R}^{2 \times N} \to Robotarium$ sets the
% velocities of each agent using a 2 x N vector.  Each column represents
% the linear and angular velocity of an agent.
% * step(): $\emptyset \to \emptyset$ iterates the simulation, updating the
% state of each agent.  This function should be called for each "iteration"
% of your experiment.  Additionally, it should only be called once per call
% of get_poses().

classdef Robotarium < ARobotarium
    %Robotarium This is the Robotarium simulator object that represents
    %your communications with the GRITSbots.
    %   This class handles retrieving the poses of agents, setting their
    %   velocities, iterating the simulation, and saving data.

    % THIS CLASS SHOULD NEVER BE MODIFIED

    properties (GetAccess = private, SetAccess = private)
        previous_timestep
        checked_poses_already = false
        called_step_already = true
        x_lin_vel_coef = 1;
        y_lin_vel_coef = 1;
        ang_vel_coef = 1; %1
    end

    methods

        function this = Robotarium(number_of_agents, save_data, show_figure, initial_poses)
            this = this@ARobotarium(number_of_agents, save_data, show_figure, initial_poses);
            this.previous_timestep = tic;
        end

        function poses = get_poses(this)

            assert(~this.checked_poses_already, 'Can only call get_poses() once per call of step()!');

            poses = this.poses;

            %Include delay to mimic behavior of real system
            this.previous_timestep = tic;

            %Make sure it's only called once per iteration
            this.checked_poses_already = true;
            this.called_step_already = false;
        end
        
        function maxLinearVelocity = set_maxLinearVelocity(this, new_maxLinearVelocity)
            this.maxLinearVelocity = new_maxLinearVelocity;
            maxLinearVelocity = this.maxLinearVelocity;
        end
        
        function poses = set_poses(this, new_pos)
            this.poses = new_pos;
            poses = this.poses;
        end
        
        function bot_true_path = set_bot_true_path(this, new_pos)
            this.bot_true_path(:,:,end+1) = new_pos; % 2d
            bot_true_path = this.bot_true_path;         
        end
        
        function bot_true_path = set_init_bot_true_path(this, new_pos)
            this.bot_true_path = new_pos;
            bot_true_path = this.bot_true_path;
        end
        
        function bot_observe_path = set_bot_observe_path(this, observe_pos)
            this.bot_observe_path(:,:,end+1) = observe_pos; % 2d
            bot_observe_path = this.bot_observe_path;         
        end
        
        function bot_observe_path = set_init_bot_observe_path(this, observe_pos)
            this.bot_observe_path = observe_pos;
            bot_observe_path = this.bot_observe_path;
        end
        
        function set_true_traj_flag(this, true_traj_flag)
            this.true_traj_flag = true_traj_flag;
        end
        
        function set_ghost_traj_flag(this, ghost_traj_flag)
            this.ghost_traj_flag = ghost_traj_flag;
        end
   
        function pos_error = get_ghost_poses_error(this)
            pos_error = this.ghost_poses_error;
        end
        
        function ghost_poses_error = set_ghost_poses_error(this, new_ghost_pos_error)
            this.ghost_poses_error = new_ghost_pos_error;
            ghost_poses_error = this.ghost_poses_error;
        end
        
        function ghost_error_box = set_ghost_error_box(this, new_ghost_error_box)
            this.ghost_error_box = new_ghost_error_box;
            ghost_error_box = this.ghost_error_box;
        end
        
        function vel_error = set_vel_error(this, new_vel_error)
            this.vel_error = new_vel_error;
            vel_error = this.vel_error;
        end
        
        function safe_radius = set_radius(this, s_radius)
            this.safe_radius = s_radius;
            safe_radius = this.safe_radius;
        end
                        
        function videoobj = get_videoobj(this) 
            videoobj = this.video_obj;
        end
        
        function set_video_flag(this,  video_flag)
            this.video_flag = video_flag;
        end
        
        function set_ghost_flag(this, ghost_flag)
            this.ghost_flag = ghost_flag;
        end
        
        function set_dynamics_flag(this, dynamics_flag)
            this.dynamics_flag = dynamics_flag;
        end
        
        function set_arrow_flag(this, new_arrow_flag)
            this.arrow_flag = new_arrow_flag;
        end
        
        function set_replay_flag(this, replay_flag)
            this.replay_flag = replay_flag;
        end
        
        function set_prsbc_off_flag(this, new_prsbc_off_flag)
            this.prsbc_off_flag = new_prsbc_off_flag;
        end
        
        function set_ctrl(this, ctrl_flag)
            this.ctrl_flag = ctrl_flag;
        end
        
        function set_video_obj(this, video_obj)
            this.video_obj = video_obj;
        end       
        
        function set_ctrl_flag(this, ctrl_flag)
            this.ctrl_flag = ctrl_flag;
        end
        
        function set_safety_radius_flag(this, safety_radius_flag)
            this.safety_radius_flag = safety_radius_flag;
        end

        function step(this)

            assert(~this.called_step_already, 'Make sure you call get_poses before calling step!');

            %Vectorize update to states
            i = 1:this.number_of_agents;

            total_time = this.time_step;

            %Update velocities using unicycle dynamics
            % taken out the unicycle dynamics for now
            
            if ~this.replay_flag
                if this.dynamics_flag
                    this.poses(1, i) = this.poses(1, i) + this.x_lin_vel_coef*total_time.*this.velocities(1, i).*cos(this.poses(3, i));
                    this.poses(2, i) = this.poses(2, i) + this.y_lin_vel_coef*total_time.*this.velocities(1, i).*sin(this.poses(3, i));% this is for kinematic mapping
                    this.quiver_u(i) = this.x_lin_vel_coef*this.velocities(1, i).*cos(this.poses(3, i));
                    this.quiver_v(i) = this.y_lin_vel_coef*this.velocities(1, i).*sin(this.poses(3, i));
                else
                    this.poses(1, i) = this.poses(1, i) + this.x_lin_vel_coef*total_time.*this.velocities(1, i);%.*cos(this.poses(3, i));
                    this.poses(2, i) = this.poses(2, i) + this.y_lin_vel_coef*total_time.*this.velocities(2, i);%this.velocities(1, i).*sin(this.poses(3, i)); this is for single integrater dynamics only - no kinematic model assumed
                    this.quiver_u(i) = this.x_lin_vel_coef*this.velocities(1, i); 
                    this.quiver_v(i) = this.y_lin_vel_coef*this.velocities(2, i);    
                end
                
                if ~this.prsbc_off_flag
                    this.poses(3, i) = this.poses(3, i) + this.ang_vel_coef*total_time.*this.velocities(2, i);           
                end
                

                %Ensure that we're in the right range
                this.poses(3, i) = atan2(sin(this.poses(3, i)), cos(this.poses(3, i)));
                
                %Introduce process noise
                this.poses(1:2,i) = this.poses(1:2,i) + this.vel_error(1:2,i);
            end

            %Allow getting of poses again
            this.checked_poses_already = false;
            this.called_step_already = true;
            
            % Set LEDs
            led_commands(1:3, :) = this.led_commands(1:3, :)/255;            
            
            for i = 1:this.number_of_agents
               if(this.led_commands(4, i) == 0)
                   to_set = [4]; % 18 
               else
                   to_set = [5];
               end
               this.robot_handle{i}.FaceVertexCData(to_set, :) = repmat(led_commands(1:3, i)', numel(to_set), 1);
            end

            if(this.save_data)
                this.save();
            end            
            
            if(this.show_figure)
                this.draw_robots();
            end
        end

        function call_at_scripts_end(this)
            if(this.save_data)
                this.mat_file_path.robotarium_data = this.mat_file_path.robotarium_data(:, 1:(this.current_saved_iterations-1));
            end
        end
    end
end
