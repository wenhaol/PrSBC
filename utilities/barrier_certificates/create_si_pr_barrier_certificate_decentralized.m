%% create_si_barrier_certificate
% Returns a single-integrator barrier certificate function ($f :
% \mathbf{R}^{2 \times N} \times \mathbf{R}^{2 \times N} \to \mathbf{R}^{2
% \times N}$).  This function takes a 2 x N, 2 x N single-integrator
% velocity and state vector, respectively, and returns a single-integrator
% velocity vector that does not induce collisions in the agents.
%% Detailed Description
%%
% * BarrierGain - affects how quickly the agents can approach each other
% * SafetyRadius - affects the distance the agents maintain
%%
% A good rule of thumb is to make SafetyRadius a bit larger than the agent
% itself (0.08 m for the GRITSbot).

% introduce decentralized probablistic Control Barrier Function to deal with uncertainty
% and obstacles
%Wenhao Luo (whluo12@gmail.com)
%Last modified: 5/25/2020

%skeleton code used from Georgia Tech Robotarium Repo at
%https://github.com/robotarium/robotarium-matlab-simulator

%% Implementation
function [ si_barrier_certificate ] = create_si_pr_barrier_certificate_decentralized(varargin)

parser = inputParser;
parser.addParameter('BarrierGain', 1e4);
parser.addParameter('SafetyRadius', 0.1);
parser.addParameter('Confidence', 1);
parser.addParameter('obs_robot_idx', []);


parse(parser, varargin{:})
opts = optimoptions(@quadprog,'Display','off');

gamma = parser.Results.BarrierGain;
safety_radius = parser.Results.SafetyRadius;
Confidence = parser.Results.Confidence;
obs_robot_idx_set = parser.Results.obs_robot_idx;

si_barrier_certificate = @barrier_certificate;

    function [ dx, fval ] = barrier_certificate(dxi, x, varargin)
        %BARRIERCERTIFICATE Wraps single-integrator dynamics in safety barrier
        %certificates
        %   This function accepts single-integrator dynamics and wraps them in
        %   barrier certificates to ensure that collisions do not occur.  Note that
        %   this algorithm bounds the magnitude of the generated output to 0.1.
        %
        %   dx = BARRIERCERTIFICATE(dxi, x, safetyRadius)
        %   dx: generated safe, single-integrator inputs
        %   dxi: single-integrator synamics
        %   x: States of the agents (Noisy)
        %   safetyRadius:  Size of the agents (or desired separation distance)
        
        
        %         parser.addParameter('XRandSpan',0); % XRandSpan should be 1 x N vectors containing vector of upper bound of the
        %         parser.addParameter('XRandSpan',0);
        
        parser = inputParser;
        parser.addParameter('XRandSpan', 0); % should be a 1 x N vector containing upper bounds of each robot's position
        parser.addParameter('URandSpan', 0); % should be a 1 x N vector containing upper bounds of each robot's velocity (control inputs)
        
        parse(parser, varargin{:})
        XRandSpan = parser.Results.XRandSpan;
        URandSpan = parser.Results.URandSpan;
        
        N = size(dxi, 2);
        
        if(N < 2)
            dx = dxi;
            return
        end
        
        if (numel(XRandSpan)==1)&&(XRandSpan==0)
            XRandSpan = zeros(2,N);
        end
        
        if (numel(URandSpan)==1)&&(URandSpan==0)
            URandSpan = zeros(2,N);
        end
        
        x = x(1:2, :);
        robot_idx_set = setdiff(1:N, obs_robot_idx_set);
        % N = N_bot + N_bot_obs and assume bot: 1:N_bot and bot_obs: (N_bot+1):N
        
        %Generate constraints for barrier certificates based on the size of
        %the safety radius
        num_constraints = 2*nchoosek(N, 2);
        
        
        % introduce recursive policy with "stop action"
        
        start_i = 0;
        loop_flag = true; % initialize
        while loop_flag
            loop_flag = false;
            A = zeros(num_constraints, 2*N);
            b = zeros(num_constraints, 1);
            count = 1;
            
            for i = (start_i+1):(N-numel(obs_robot_idx_set))
                for j = (i+1):N
                    
                    max_dvij_x = norm(URandSpan(1,i)+URandSpan(1,j));
                    max_dvij_y = norm(URandSpan(2,i)+URandSpan(2,j));
                    max_dxij_x = norm(x(1,i)-x(1,j)) + norm(XRandSpan(1,i)+XRandSpan(1,j));
                    max_dxij_y = norm(x(2,i)-x(2,j)) + norm(XRandSpan(2,i)+XRandSpan(2,j));
                    
                    BB_x = -safety_radius^2-2/gamma*max_dvij_x*max_dxij_x;
                    BB_y = -safety_radius^2-2/gamma*max_dvij_y*max_dxij_y;
                    
                    [b2_x, b1_x, sigma] = trap_cdf_inv(XRandSpan(1,i), XRandSpan(1,j), x(1,i)-x(1,j), Confidence);
                    [b2_y, b1_y, sigma] = trap_cdf_inv(XRandSpan(2,i), XRandSpan(2,j), x(2,i)-x(2,j), Confidence);
                    
                    
                    
                    % for x
                    ratio_x = sqrt((XRandSpan(1,i)+XRandSpan(1,j))^2+(XRandSpan(2,i)+XRandSpan(2,j))^2)/(XRandSpan(1,i)+XRandSpan(1,j));
                    ratio_y = sqrt((XRandSpan(1,i)+XRandSpan(1,j))^2+(XRandSpan(2,i)+XRandSpan(2,j))^2)/(XRandSpan(2,i)+XRandSpan(2,j));
                    ratio = 1;
                    
                    if ((b2_x<0)&&(b1_x>0))||((b2_x>0)&&(b1_x<0))
                        warning('distance between robots on x smaller than error bound!');
                        b_x = 0;
                        ratio = ratio_y;          
                    elseif ((b1_x<0)&&(b2_x<b1_x))||((b2_x<0)&&(b2_x>b1_x))
                        b_x = b1_x;
                    elseif ((b2_x>0)&&(b2_x<b1_x))||((b1_x>0)&&(b2_x>b1_x))
                        b_x = b2_x;
                    else
                        b_x = b1_x;warning('no uncertainty or sigma = 0.5 on x'); % b1 = b2 or no uncertainty
                    end
                    
                    if ((b2_y<0)&&(b1_y>0))||((b2_y>0)&&(b1_y<0))
                        warning('distance between robots on y smaller than error bound!');
                        b_y = 0;
                        ratio = ratio_x;
                    elseif ((b1_y<0)&&(b2_y<b1_y))||((b2_y<0)&&(b2_y>b1_y))
                        b_y = b1_y;
                    elseif ((b2_y>0)&&(b2_y<b1_y))||((b1_y>0)&&(b2_y>b1_y))
                        b_y = b2_y;
                    else
                        b_y = b1_y;warning('no uncertainty or sigma = 0.5 on y');
                    end
                    
                    
                    if j<(N-numel(obs_robot_idx_set))+1
                        A(2*count-1, (2*i-1):(2*i)) = -2*([b_x;b_y]);
                        A(2*count, (2*j-1):(2*j)) =  2*([b_x;b_y])';
                        h = norm([b_x;b_y])^2-2*safety_radius^2-2*norm([max_dvij_x;0])*norm([max_dxij_x;0])/gamma-2*norm([0;max_dvij_y])*norm([0;max_dxij_y])/gamma;
                        b((2*count-1):(2*count)) = 1/2*gamma*h; %^3;
                    else
                        A(2*count-1, (2*i-1):(2*i)) = -2*([b_x;b_y]);
                        %     A(2*count, (2*j-1):(2*j)) =  2*([b_x;b_y])';
                        h = -2*([b_x;b_y])'*dxi(:,j)/gamma+norm([b_x;b_y])^2-2*safety_radius^2-2*norm([max_dvij_x;0])*norm([max_dxij_x;0])/gamma-2*norm([0;max_dvij_y])*norm([0;max_dxij_y])/gamma;
                        b((2*count-1)) = gamma*h; %^3;
                    end
                    
                    
                    count = count + 1;
                end
            end
            
            %Solve QP program generated earlier
            vhat = reshape(dxi,2*N,1);
            H = 2*eye(2*N);
            f = -2*vhat;
            
            [vnew, fval] = quadprog(sparse(H), double(f), A, b, [],[], [], [], [], opts);
            
            %Set robot velocities to new velocities
            
            if isempty(vnew) % if no solution exists, then iteratively set u_i=0 from i = 1,..N, until solution found
                loop_flag = true;
                start_i = start_i+1;
            else
                dx = reshape(vnew, 2, N);
                loop_flag = false;
                dx(:,1:start_i)=zeros(2,numel(1:start_i));
            end
            
        end
    end
end

