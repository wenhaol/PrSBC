% this function constructs the trap distribution resultant from convolution
% of two different central uniform distribution (i.e. from measurements of two robots positions)
% bot 1 : uniformly distributed between [-a,a]
% bot 2 : uniformly distributed between [-c,c]
% delta: x_bot1 - x_bot2 error between the two noisy measurements
% sigma: requred confidence level (>50%)
% Output: when sigma >.5
%         b2: <b1 whose CDF corresponds to 1-sigma
%         b1: >b2 whose CDF corresponds to sigma
%         when sigma < .5
%         b2: >b1 whose CDF corresponds to 1-sigma
%         b1: <b2 whose CDF corresponds to sigma

function [b2, b1, sigma] = trap_cdf_inv(a, c, delta, sigma)
%   a and c should be positive

    if a>c  % [-A A] is the large one, and [-C C] is the smaller one
        A = a;
        C = c;
    else
        A = c;
        C = a;        
    end
    
    if (A==0)&&(C==0)
        b2=delta;b1=delta;return;
    end
    
    O_vec = [-(A+C) -(A-C) (A-C) (A+C)]; % vector of vertices on the trap distribution pdf
    h = 1/(2*A); % height of the trap distribution
    area_seq = [1/2*2*C*h 2*(A-C)*h 1/2*2*C*h];
    
    area_vec = [area_seq(1) sum(area_seq(1:2))];
    
    
    if abs(A-C) < 1e-5 % then is triangle
        % assuming sigma > 50%
            b1 = (A+C) - 2*C*sqrt((1-sigma)/(1-area_vec(2)));  % 1-area_vec(2) should be very close to 0.5
            b2 = -b1;
            
            b1 = b1 + delta;
            b2 = b2 + delta;% apply shift here due to xi-xj
        
    else % than is trap
        if sigma>area_vec(2) % right triangle area
            b1 = (A+C)-2*C*sqrt((1-sigma)/(1-area_vec(2)));
            b2 = -(A+C) + 2*C*sqrt((1-sigma)/(1-area_vec(2)));
            
            b1 = b1 + delta;
            b2 = b2 + delta; % apply shift here due to xi-xj
  
        elseif (sigma>area_vec(1))&&(sigma<=area_vec(2)) % in between the triangle part
            b1 = -(A-C) + (sigma - area_vec(1))/h; % assuming >50%, then b1 should >0
            b2 = -b1;
            
            b1 = b1 + delta;
            b2 = b2 + delta;      % apply shift here due to xi-xj  
            
            % note that b1 could be > or < b2, depending on whether sigma >
            % or < .5
            
        elseif sigma<=area_vec(1)
            b1 = -(A+C) + 2*C*sqrt(sigma/area_vec(1)); % assuming >50%, then b1 should >0
            b2 = -b1;
            
            b1 = b1 + delta;
            b2 = b2 + delta;      % apply shift here due to xi-xj 
       
        else  % first triangle, which is not allowed as long as we assume sigma > 50%
            error('bug! what is wrong?');          
        end
        
        
    end
    
%     if (b2<0)&&(b1>0)
%         warning('debug intiated');
%         %pause;
%     end
    

end