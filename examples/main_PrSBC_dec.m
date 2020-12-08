%Initializing multi-robot system with decentralized probabilistic safety barrier
%certificates to avoid inter-robot collisions under uncertainty
%in this example, black robots 6 and 7 act as non-cooperative moving
%obstacles that do not consider collision avoidance with others
%Wenhao Luo (whluo12@gmail.com)
%Last modified: 5/25/2020

%skeleton code used from Georgia Tech Robotarium Repo at 
%https://github.com/robotarium/robotarium-matlab-simulator

figure(99)
clf
warning('off');
% close all;
% clear all;
% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = rb.get_available_agents(); 

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
N = 7;
confidence_level = 0.8;
SafetyRadius = 0.2; % should manually keep consistent with the initial value in ARobotarium
r = rb.set_number_of_agents(N).set_save_data(false).build();

r.set_dynamics_flag(true);
r.set_ghost_flag(true);
% rng(99);
x_rand_span_x = 0.02*randi([3 4],1,N); % setting up position error range for each robot, rand_span serves as the upper bound of uncertainty for each of the robot
x_rand_span_y = 0.02*randi([1 4],1,N); %
v_rand_span = 0.005*ones(2,N); % setting up velocity error range for each robot

r.set_ghost_error_box([x_rand_span_x;x_rand_span_y]);
r.set_vel_error(zeros(2,N));

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.get_poses();
ctrl_flag = [1 10 2 20 3 0 0];

r.set_radius(SafetyRadius);
r.set_ghost_poses_error(zeros(2,N));
r.step();
r.set_ctrl_flag(ctrl_flag);

x = [-1.3 2.4 -1.1 1.7 1.6 -1.3 0.5;...
    0 -0.2 1.7 -1.7 1.2 -1.8 -2;...
    0 0 0 0 0 0 0]; %

goal_condition = [x(1:2,[2 1 4 3 6 5]) [0.5;2]];% robots move to swap their positions   

obs_robot_idx = [6 7];

r.set_poses(x);


hold on;
handle_targets_dummy = cell(0,1);
for ijk_goal = 1:N
    
    if ctrl_flag(ijk_goal) == 0 % light black
        ctrl_color = [0 0 0];
        radius_color = [0 0 0];
    elseif ctrl_flag(ijk_goal) == 3   % MSFT Blue
        ctrl_color = [0 161 241]/255;
        radius_color = ctrl_color;
    elseif ctrl_flag(ijk_goal) == 2   % MSFT Green
        ctrl_color = [124 187 0]/255;
        radius_color = ctrl_color;
    elseif ctrl_flag(ijk_goal) == 4   % MSFT Yellow
        ctrl_color = [255 187 0]/255;
        radius_color = ctrl_color;
    elseif ctrl_flag(ijk_goal) == 1   % MSFT Red
        ctrl_color = [246 83 20]/255;
        radius_color = ctrl_color;
    elseif ctrl_flag(ijk_goal) == 10   % Dark Red
        ctrl_color = [255 0 0]/255;
        radius_color = ctrl_color;
    elseif ctrl_flag(ijk_goal) == 20   % Dark Green
        ctrl_color = [0 102 0]/255;
        radius_color = ctrl_color;
    elseif ctrl_flag(ijk_goal) == 30   % Dark Blue
        ctrl_color = [0 0 255]/255;
        radius_color = ctrl_color;
    end
    
    x_pri = [goal_condition(1, ijk_goal)-0.04 goal_condition(1, ijk_goal)+0.04 goal_condition(1, ijk_goal)+0.04 goal_condition(1, ijk_goal)-0.04];
    y_pri = [goal_condition(2,ijk_goal)-0.04 goal_condition(2,ijk_goal)-0.04 goal_condition(2,ijk_goal)+0.04 goal_condition(2,ijk_goal)+0.04];
    
    text(goal_condition(1, ijk_goal)-0.1, goal_condition(2,ijk_goal)+0.3, num2str(ijk_goal),'FontSize',30, 'Color','r');    

    handle_targets_dummy{ijk_goal} = patch(x_pri,y_pri,ctrl_color,'EdgeColor','none','FaceAlpha',0.6);

end

% Create a barrier certificate so that the robots don't collide
si_barrier_certificate = create_si_pr_barrier_certificate_decentralized('SafetyRadius', 2*SafetyRadius, 'Confidence', confidence_level, 'obs_robot_idx', obs_robot_idx);%
si_to_uni_dynamics = create_si_to_uni_mapping2();
fun_rand = @(a,B) [a.*B(1,:);a.*B(2,:)];
        
%Get randomized initial conditions in the robotarium arena
initial_conditions = generate_initial_conditions(N, 'Width', r.boundaries(2)-r.boundaries(1)-0.1, 'Height', r.boundaries(4)-r.boundaries(3)-0.1, 'Spacing', 0.2);

% We'll make the rotation error huge so that the initialization checker
% doesn't care about it
args = {'PositionError', 0.01, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();
timer_to_stop = 3300;
timer_count = 1;

record_video_flag = false;
if record_video_flag
    writerObj = VideoWriter('PrSBC_decentralized_obstacles.mp4', 'MPEG-4');
    open(writerObj);
    set(gca,'nextplot','replacechildren');
    set(gcf,'Renderer','zbuffer');
    r.set_video_obj(writerObj);
end

rng(150)

font_size = 25;

%% prepare for computing the performance-related metrics
min_bot_dist_hist = zeros(timer_to_stop,1);
min_bot_dist_observe_hist = zeros(timer_to_stop,1);
min_bot_safe_ratio_hist = zeros(timer_to_stop,1);

pertub_bot_hist = zeros(timer_to_stop,1);
perf_bot_hist = zeros(timer_to_stop,1);

while timer_count< timer_to_stop  %(~init_checker(x(1:2,:), goal_condition)) %
    
    if timer_count == 1
        text(-1.5, -2.5, strcat('Collision-free Confidence Level: ', num2str(confidence_level)),'FontSize',font_size);
        
    end
    handle_timestep = text(-0.5,-2.8,strcat('Time Step = ',num2str(timer_count)),'FontSize',font_size);
    
    x = r.get_poses();
    
    
    bot_dist_tmp = pdist2(x(1:2,:)', x(1:2,:)');
    bot_dist_tmp = triu(bot_dist_tmp, 1);
    min_bot_dist_tmp = min(bot_dist_tmp(find(bot_dist_tmp(:)>0)));
    min_bot_dist_hist(timer_count) = min_bot_dist_tmp;
    
    min_bot_safe_ratio_tmp = [];
    for i_hehe = 1:(N-1)
            for j_hehe = (i_hehe+1):N
                boxA = [x(1:2,i_hehe)'-([x_rand_span_x(i_hehe);x_rand_span_y(i_hehe)])'-SafetyRadius 2*x_rand_span_x(i_hehe)+2*SafetyRadius 2*x_rand_span_y(i_hehe)+2*SafetyRadius];
                boxB = [x(1:2,j_hehe)'-([x_rand_span_x(j_hehe);x_rand_span_y(j_hehe)])'-SafetyRadius 2*x_rand_span_x(j_hehe)+2*SafetyRadius 2*x_rand_span_y(j_hehe)+2*SafetyRadius];
                area_tmp = rectint(boxA,boxB);
                area_boxA = (2*x_rand_span_x(i_hehe)+2*SafetyRadius)*(2*x_rand_span_y(i_hehe)+2*SafetyRadius);
                area_boxB = (2*x_rand_span_x(j_hehe)+2*SafetyRadius)*(2*x_rand_span_y(j_hehe)+2*SafetyRadius);
                min_bot_safe_ratio_tmp = [min_bot_safe_ratio_tmp; 1-area_tmp/min(area_boxA, area_boxB)];
            end
    end
    
    
    min_bot_safe_ratio_hist(timer_count) = min(min_bot_safe_ratio_tmp);
    
    
    pos_error = r.get_ghost_poses_error();
    
    % add noise to observation

    x_observe = [x(1:2,:)+pos_error; x(3,:)]; % put noise over ground-truth position to get noisy measurements.
    
    
    bot_dist_tmp = pdist2(x_observe(1:2,:)', x_observe(1:2,:)');
    bot_dist_tmp = triu(bot_dist_tmp, 1);
    min_bot_dist_observe_tmp = min(bot_dist_tmp(find(bot_dist_tmp(:)>0)));
    min_bot_dist_observe_hist(timer_count) = min_bot_dist_observe_tmp;
    
    vel_error = (rand(2,N)-0.5).*2;
    vel_error = v_rand_span.*vel_error;
       
    r.set_vel_error(vel_error);
    % finish add noise to motion
    
    if timer_count==2
        r.set_init_bot_true_path(x(1:2,:));
        r.set_init_bot_observe_path(x_observe(1:2,:));
    elseif timer_count>2
        r.set_bot_true_path(x(1:2,:));
        r.set_bot_observe_path(x_observe(1:2,:));
    end
    
    
%     dxi = controller(x(1:2, :), initial_conditions(1:2, :));
    dxi = controller(x_observe(1:2, :), goal_condition);
    
    dxi_r = si_barrier_certificate(dxi, x_observe(1:2, :), 'XRandSpan', [x_rand_span_x;x_rand_span_y],'URandSpan', v_rand_span);
%     fval_r
    
    delta_dxi = dxi_r-dxi;
    
    pertub_bot_tmp = sum(sum(delta_dxi.^2, 1))/N;
    pertub_bot_hist(timer_count) = pertub_bot_tmp; % 

    dxu = si_to_uni_dynamics(dxi_r, x_observe); 

    r.set_velocities(1:N, dxu);%+vel_error
    
    pos_error = (rand(2,N)-0.5).*2; % get uniformly distributed noise at 2 dimensions for all robots
    pos_error = [x_rand_span_x;x_rand_span_y].*pos_error; %bsxfun(fun_rand,[x_rand_span_x;x_rand_span_y],pos_error); % get the final noise of observation
    
    r.set_ghost_poses_error(pos_error);
    
    r.step();

   
    r.set_video_flag(record_video_flag);
    timer_count = timer_count + 1
    delete(handle_timestep)
 
end
if record_video_flag
    writerObj = r.get_videoobj();
    close(writerObj);
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

