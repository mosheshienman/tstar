%% dubins path 2 steps
initial_pos = [1,1,pi/2];
final_pos = [2,4,5*pi/4];
mid_pos = [2,1,0];
% mid_pos = [2,2,0;2,2,45;2,2,90;2,2,135;2,2,180;2,2,225;2,2,270;2,2,315];
num_of_orentations = 8;
r = 1;
s2m_dist = zeros(1,8);
m2f_dist = zeros(1,8);
total_dist = zeros(1,8);

for i=1:num_of_orentations
   temp_theta = 2*pi/ num_of_orentations * (i - 1);
   mid_pos(3) = temp_theta;
   s2m = dubins_core(initial_pos,mid_pos,r);
   m2f = dubins_core(mid_pos, final_pos,r);
   s2m_length = 0;
   m2f_length = 0;
   for j=1:3
       s2m_length = s2m_length + s2m.seg_param(j);
       m2f_length = m2f_length + m2f.seg_param(j);
   end
   s2m_dist(i) = s2m_length;
   m2f_dist(i) = m2f_length;
   total_dist(i) = s2m_length + m2f_length;
end
disp(s2m_dist)
disp(m2f_dist)
disp(total_dist)