%% Robotics Final
%  Edwin Bodge (etb10)
%  Fall 2017

clear; format short e

%% Find z_pos conversion equation
%  Plug in pixel and z data
pixel_radius = [15.0,   10.0,   7.5,    6.0,    5.0];
z_Pos        = [ 1.0,    1.5,   2.0,    2.5,    3.0];

%  From cftool
a = 15;
b = -1;
z = @(r) a*r^b;