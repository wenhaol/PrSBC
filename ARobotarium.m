classdef ARobotarium < handle
    %APIAbstract This is an interface for the Robotarium class that
    %ensures the simulator and the robots match up properly.

    properties (GetAccess = protected, SetAccess = protected)
        robot_handle
        robot_body
        robot_radius
        robot_id_handle
        robot_arrow_handle
        ghost_radius
        ghost_box
        bot_true_path_handle
        bot_observe_path_handle

        % Stuff for saving data
        file_path
        current_file_size
        current_saved_iterations
        % Path to mat file to keep this in memory
        mat_file_path
        boundary_patch
        boundary_points = {[-2.5, 3, 3, -2.5], [-3, -3, 2.5, 2.5]};%{[-3, 3, 3, -3], [-2, -2, 2, 2]}; %
    end

    properties (GetAccess = public, SetAccess = protected)
        % Time step for the Robotarium
        time_step = 0.033
        maxLinearVelocity = 0.1
        maxAngularVelocity = 2*pi
        robot_diameter = 0.08
        number_of_agents
        velocities
        poses
        ghost_poses_error = zeros(2,100);
        ghost_error_box = zeros(2,100);
        vel_error = zeros(2,100);
        led_commands
        safe_radius = 0.2
        max_arrow = 0.8 % specify the maximum length of arrow on figure, if moving at maxLinearVelocity
        bot_true_path = []
        bot_observe_path = [] % 2 x N x T
        quiver_u = zeros(1,100)
        quiver_v = zeros(1,100)
        
        ctrl_flag = 0 % ctrl the color of the bots 0 - black
        video_flag = false 
        video_obj 
        dynamics_flag = false 
        arrow_flag = false
        safety_radius_flag = true
        ghost_radius_flag = true
        ghost_box_flag = true
        label_flag = true
        boundary_flag = 'off'
        ghost_flag = true
        replay_flag = false
        true_traj_flag = false
        ghost_traj_flag = false
        prsbc_off_flag = false
        
        %Saving parameters
        save_data

        % Figure handle for simulator
        figure_handle
        show_figure

        % Arena parameters
        boundaries = [-2.5, 3, -3, 2.5];%[-3 3 -2 2]; %
    end

    methods (Abstract)

        %Try this one out...
        % We can use this to finish saving / clean up after MQTT
        call_at_scripts_end(this)

        % Getters
        % Get poses must be implemented independently
        get_poses(this)

        %Update functions
        step(this);
    end

    methods
        function this = ARobotarium(number_of_agents, save_data, show_figure, initial_poses)
            this.number_of_agents = number_of_agents;
            this.save_data = save_data;
            this.show_figure = show_figure;

            this.velocities = zeros(2, number_of_agents);
            this.led_commands = zeros(4, number_of_agents);
            this.poses = initial_poses;

            % If save data, set up the file saving variables
            if(save_data)
                date = datetime('now');
                this.file_path = 'robotarium_data';
                this.file_path = strcat(this.file_path, '_', num2str(date.Month), '_', num2str(date.Day), '_', ...
                num2str(date.Year), '_', num2str(date.Hour), '_', ...
                num2str(date.Minute), '_', num2str(round(date.Second)), '.mat');

                this.current_file_size = 100;
                this.current_saved_iterations = 1;

                % Plus one for timestamp
                robotarium_data = zeros(5*number_of_agents+1, this.current_file_size);
                save(this.file_path, 'robotarium_data', '-v7.3')

                this.mat_file_path = matfile(this.file_path, 'Writable', true);
            end

            if(show_figure)
                this.initialize_visualization()
            end
        end

        function agents = get_number_of_agents(this)
           agents = this.number_of_agents;
        end

        function this = set_velocities(this, ids, vs)
            N = size(vs, 2);

            assert(N<=this.number_of_agents, 'Column size of vs (%i) must be <= to number of agents (%i)', ...
                N, this.number_of_agents);

            % Threshold velocities
            for i = 1:N
                if(abs(vs(1, i)) > this.maxLinearVelocity)
                   vs(1, i) = this.maxLinearVelocity*sign(vs(1,i));
                end

                if(abs(vs(2, i)) > this.maxAngularVelocity)
                   vs(2, i) = this.maxAngularVelocity*sign(vs(2, i));
                end
            end

            this.velocities(:, ids) = vs;
        end
        
        % Commands is [r g b index] x N
        function this = set_leds(this, ids, commands)
            N = size(commands, 2);

            assert(N<=this.number_of_agents, 'Column size of vs (%i) must be <= to number of agents (%i)', ...
                N, this.number_of_agents);
            
            assert(all(all(commands(1:3, :) <= 255)) && all(all(commands(1:3, :) >= 0)), 'RGB commands must be between 0 and 255');
            assert(all(commands(4, :) >= 0) && all(commands(4, :) <= 1), 'LED index must be 0 or 1');                              
            
            % Only set LED commands for the selected robots
            this.led_commands(:, ids) = commands;
        end

        function iters = time2iters(this, time)
           iters = time / this.time_step;
        end
    end

    methods (Access = protected)

        % Initializes visualization of GRITSbots
        function initialize_visualization(this)
            % Initialize variables
            N = this.number_of_agents;
            offset = 0.05;

            % Scale factor (max. value of single Gaussian)
            scaleFactor = 50;
            figPhi = figure(99);
            this.figure_handle = figPhi;

            % Plot Robotarium boundaries %Maria
            this.boundary_patch = patch('XData', this.boundary_points{1}, 'YData', this.boundary_points{2}, ...
            'FaceColor', 'none', ...
            'LineWidth', 3, ...,
            'EdgeAlpha', 0.5, ...
            'EdgeColor', [0, 0, 0]);
        
            set(this.boundary_patch,'Visible',this.boundary_flag); % NeurIPS_20_Author

            %plot(im)
            set(figPhi,'color','white');

            % Set axis
            robotPlaneAxes = gca;

            % Limit view to xMin/xMax/yMin/yMax
            axis(robotPlaneAxes, [this.boundaries(1) - offset,this.boundaries(2)+offset,this.boundaries(3)-offset,this.boundaries(4)+offset])
            caxis([0,1.5*scaleFactor])
            set(robotPlaneAxes,'PlotBoxAspectRatio',[1 1 1],'DataAspectRatio',[0.96 1 1])

            % Store axes
            axis(robotPlaneAxes,'on')
            set(robotPlaneAxes,'position',[0.05 0.05 0.95 0.95],'units','normalized','YDir','normal')
            set(robotPlaneAxes, 'FontSize',25)     % added by NeurIPS_20_Author
            hold on % "This ride's about to get bumpy!"

            % Let's jump through hoops to make the robot diameter look to data scale
            curUnits = get(robotPlaneAxes, 'Units');
            set(robotPlaneAxes, 'Units', 'Points');
            set(robotPlaneAxes, 'Units', curUnits);

            offset = [-0.1 0.1];
            xlim(this.boundaries(1:2)+offset); ylim(this.boundaries(3:4)+offset);

            % Static legend
            setappdata(gca,'LegendColorbarManualSpace',1);
            setappdata(gca,'LegendColorbarReclaimSpace',1);

            assert(N <= 100, 'Number of robots (%i) must be <= 100', N);

            this.robot_handle = cell(1, N);
            this.robot_radius = cell(1, N);
            this.robot_id_handle = cell(1, N);
            this.robot_arrow_handle = cell(1, N);
            this.ghost_radius = cell(1,N);
            this.ghost_box = cell(1,N);
            this.bot_true_path_handle = cell(1,N);
            this.bot_observe_path_handle = cell(1,N);
            for ii = 1:N
                data = gritsbot_patch;%drone_patch; %
                this.robot_body = data.vertices;
                x  = this.poses(1, ii);
                y  = this.poses(2, ii);
                th = this.poses(3, ii) - pi/2;
                
                if this.ghost_flag
                    x_observe = x + this.ghost_poses_error(1,ii);
                    y_observe = y + this.ghost_poses_error(2,ii);
                    errorbox_l = this.ghost_error_box(1,ii)+this.safe_radius;
                    errorbox_w = this.ghost_error_box(2,ii)+this.safe_radius;
                end
                
                
                rotation_matrix = [...
                    cos(th) -sin(th) x;
                    sin(th)  cos(th) y;
                    0 0 1];
                transformed = this.robot_body*rotation_matrix';
                
                
                % insert code here to control the color
                
                if numel(this.ctrl_flag)> 1               
                    ctrl_flag = this.ctrl_flag(ii);
                else
                    ctrl_flag = this.ctrl_flag;                       
                end
                
                if ctrl_flag == 0 % light black
                    ctrl_color = 'flat';
                    radius_color = [0 0 0];
                elseif ctrl_flag == 3   %   Blue
                    ctrl_color = [0 161 241]/255;
                    radius_color = ctrl_color;
                elseif ctrl_flag == 2   %   Green
                    ctrl_color = [124 187 0]/255;
                    radius_color = ctrl_color;
                elseif ctrl_flag == 4   %   Yellow
                    ctrl_color = [255 187 0]/255;
                    radius_color = ctrl_color;
                elseif ctrl_flag == 1   %   Red
                    ctrl_color = [246 83 20]/255;
                    radius_color = ctrl_color;
                elseif ctrl_flag == 10   % Dark Red
                    ctrl_color = [255 0 0]/255;
                    radius_color = ctrl_color;
                elseif ctrl_flag == 20   % Dark Green
                    ctrl_color = [0 102 0]/255;
                    radius_color = ctrl_color;
                elseif ctrl_flag == 30   % Dark Blue
                    ctrl_color = [0 0 255]/255;
                    radius_color = ctrl_color;    
                end
                
                this.robot_handle{ii} = patch(...
                          'Vertices', transformed(:, 1:2), ...
                          'Faces', data.faces, ...
                          'FaceAlpha', 0.6, ...
                          'FaceColor', ctrl_color, ...
                          'FaceVertexCData', data.colors, ...
                          'EdgeColor','none');
%                       if this.safety_radius_flag
                          this.robot_radius{ii} = plot_circle(x, y, this.safe_radius, 'Color', radius_color); % assume homogeneous robot team with same safety radius
                          set(this.robot_radius{ii},'Visible',this.safety_radius_flag);  
%                       end
                      if this.label_flag
                          this.robot_id_handle{ii} = text(x, y, num2str(ii),'FontSize',30);
                      end
                      
                      
%                       if this.arrow_flag
%                           quiver_u = this.velocities(1,ii)/this.maxLinearVelocity*this.max_arrow;
%                           quiver_v = this.velocities(2,ii)/this.maxLinearVelocity*this.max_arrow;
                          this.quiver_u(ii) = this.quiver_u(ii)/this.maxLinearVelocity;
                          this.quiver_v(ii) = this.quiver_v(ii)/this.maxLinearVelocity;
                          hold on;
                          this.robot_arrow_handle{ii} = quiver(x,y,this.quiver_u(ii),this.quiver_v(ii),'MaxHeadSize',100,'LineWidth',2);
                          set(this.robot_arrow_handle{ii},'Visible',this.arrow_flag);  
%                       end
                      
                      if this.ghost_flag
                          this.ghost_radius{ii} = plot_circle(x_observe, y_observe, this.safe_radius, 'Color', 'k','LineStyle','--'); % assume homogeneous robot team with same safety radius
                          set(this.ghost_radius{ii},'Visible',this.ghost_radius_flag);        
                          
                          this.ghost_box{ii} = rectangle('Position',[-this.ghost_error_box(1,ii)-this.safe_radius+x -this.ghost_error_box(2,ii)-this.safe_radius+y 2*this.safe_radius+2*this.ghost_error_box(1,ii) 2*this.safe_radius+2*this.ghost_error_box(2,ii)], 'EdgeColor','r','LineWidth',2);
                          set(this.ghost_box{ii},'Visible',this.ghost_box_flag);
                      end
                      
            end
        end

        function draw_robots(this)
            for ii = 1:this.number_of_agents
                x  = this.poses(1, ii);
                y  = this.poses(2, ii);
                th = this.poses(3, ii) - pi/2;
                rotation_matrix = [...
                    cos(th) -sin(th) x;
                    sin(th)  cos(th) y;
                    0 0 1
                ];
                transformed = this.robot_body*rotation_matrix';
                
                if this.ghost_flag
                    x_observe = x + this.ghost_poses_error(1,ii);
                    y_observe = y + this.ghost_poses_error(2,ii);
                    errorbox_l = this.ghost_error_box(1,ii)+this.safe_radius;
                    errorbox_w = this.ghost_error_box(2,ii)+this.safe_radius;
                end
                
                if numel(this.ctrl_flag)> 1               
                    ctrl_flag = this.ctrl_flag(ii);
                else
                    ctrl_flag = this.ctrl_flag;                       
                end
                
                if ctrl_flag == 0 % light black
                    ctrl_color = [0 0 0];
                    radius_color = [0 0 0];
                elseif ctrl_flag == 3   %   Blue
                    ctrl_color = [0 161 241]/255;
                    radius_color = ctrl_color;
                elseif ctrl_flag == 2   %   Green
                    ctrl_color = [124 187 0]/255;
                    radius_color = ctrl_color;
                elseif ctrl_flag == 4   %   Yellow
                    ctrl_color = [255 187 0]/255;
                    radius_color = ctrl_color;
                elseif ctrl_flag == 1   %   Red
                    ctrl_color = [246 83 20]/255;
                    radius_color = ctrl_color;
                elseif ctrl_flag == 10   % Dark Red
                    ctrl_color = [255 0 0]/255;
                    radius_color = ctrl_color;
                elseif ctrl_flag == 20   % Dark Green
                    ctrl_color = [0 102 0]/255;
                    radius_color = ctrl_color;
                elseif ctrl_flag == 30   % Dark Blue
                    ctrl_color = [0 0 255]/255;
                    radius_color = ctrl_color;    
                end
                
                set(this.robot_handle{ii},'Vertices', transformed(:, 1:2),'FaceColor', ctrl_color);
                
                if this.safety_radius_flag
                    x_offset = x - mean(this.robot_radius{ii}.XData);
                    y_offset = y - mean(this.robot_radius{ii}.YData);
                    set(this.robot_radius{ii},'XData', this.robot_radius{ii}.XData+x_offset, 'YData', this.robot_radius{ii}.YData+y_offset);
                    set(this.robot_radius{ii},'Visible',this.safety_radius_flag,'Color',radius_color);
                end
                
                if this.ghost_flag                                
                    x_offset_observe = x_observe - mean(this.ghost_radius{ii}.XData);
                    y_offset_observe = y_observe - mean(this.ghost_radius{ii}.YData);
                    set(this.ghost_radius{ii},'XData', this.ghost_radius{ii}.XData+x_offset_observe, 'YData', this.ghost_radius{ii}.YData+y_offset_observe);
                    set(this.ghost_radius{ii},'Visible',this.ghost_radius_flag,'Color','k');                
                    
                    set(this.ghost_box{ii},'Visible',this.ghost_box_flag, 'Position', [-this.ghost_error_box(1,ii)-this.safe_radius+x -this.ghost_error_box(2,ii)-this.safe_radius+y 2*this.safe_radius+2*this.ghost_error_box(1,ii) 2*this.safe_radius+2*this.ghost_error_box(2,ii)]);
                end
                
                if this.label_flag
                    set(this.robot_id_handle{ii},'Position', this.robot_id_handle{ii}.Position+[x_offset y_offset 0]);
                end
                
                if this.arrow_flag                   
                    this.quiver_u(ii) = this.quiver_u(ii)/this.maxLinearVelocity;
                    this.quiver_v(ii) = this.quiver_v(ii)/this.maxLinearVelocity;
                    hold on;
%                     quiver_u = this.velocities(1,ii)/this.maxLinearVelocity*this.max_arrow;
%                     quiver_v = this.velocities(2,ii)/this.maxLinearVelocity*this.max_arrow;
                    set(this.robot_arrow_handle{ii},'XData',x,'YData',y,'UData',this.quiver_u(ii),'VData',this.quiver_v(ii));
                    set(this.robot_arrow_handle{ii},'Visible',this.arrow_flag);  
                end
                
                if ~isempty(this.bot_true_path)
                    hold on;
%                     this.bot_true_path_handle{ii}=plot(squeeze(this.bot_true_path(1,ii,:)), squeeze(this.bot_true_path(2,ii,:)),'Color',ctrl_color,'LineWidth',2);
                    delete(this.bot_true_path_handle{ii}) 
                    delete(this.bot_observe_path_handle{ii})
                    this.bot_true_path_handle{ii}=plot(squeeze(this.bot_true_path(1,ii,:)), squeeze(this.bot_true_path(2,ii,:)),'Color',ctrl_color,'LineWidth',2);
                    this.bot_observe_path_handle{ii}=scatter(squeeze(this.bot_observe_path(1,ii,1:10:end)), squeeze(this.bot_observe_path(2,ii,1:10:end)),10,ctrl_color,'filled');
                end
                
                
            end

            if(this.number_of_agents <= 6)
                drawnow
            else
                drawnow limitrate
            end
            
            if  this.video_flag
                frame = getframe(gcf);
                writeVideo(this.video_obj,frame);                
            end
            
        end

        function save(this)
            
            this.mat_file_path.robotarium_data(:, this.current_saved_iterations) = ...
                [reshape([this.poses ; this.velocities], [], 1) ; double(tic())];
            
            this.current_saved_iterations = this.current_saved_iterations + 1;
        end
    end
end
