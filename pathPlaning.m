function varargout = pathPlaning(varargin)
% Description: 机器人路径规划用户界面 (粒子群算法+Bezier)
% Author：Yang
% Version:0.0.1
% Others:
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @pathPlaning_OpeningFcn, ...
                   'gui_OutputFcn',  @pathPlaning_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% --- 界面及数据初始化函数
function pathPlaning_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;

guidata(hObject, handles);
% 设置axes的坐标轴的范围
set(handles.axes1,'XLim',[0 12000],'YLim',[0 12000]);
axis square
hold on
global lastPosi_Draw curenPosi_Draw   %绘制圆时圆心与终止位置的坐标
global draw_ok_flag                   %绘制完成标志
global barriers_num barriers_info_buf % 障碍物的个数 与 障碍物信息数组
global add_barrier_flag               % 按下按钮添加障碍物的标志
global P0 P1                          % 路径的起点与终点
global PSO_barriers_num               % 优化时的障碍物的个数
global x_spots y_spots                % 障碍物的横纵坐标点
global D_safe                         % 安全距离
global num_ba                         % 障碍物的坐标点个数
lastPosi_Draw = zeros(1,2);
curenPosi_Draw = zeros(1,2);
draw_ok_flag = 0;
barriers_info_buf = zeros(20,3);
barriers_num = 0;
add_barrier_flag = 0;
P0 =zeros(1,2);%路径的起始点
P1 =zeros(1,2);%路径的终点
x_spots = [];
y_spots = [];
PSO_barriers_num = 0;
D_safe = 2;
num_ba = 0;
function varargout = pathPlaning_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;

% --- 鼠标按下时的回调函数 (绘制障碍物)
function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
global lastPosi_Draw draw_ok_flag
global add_barrier_flag
if add_barrier_flag
    mouse_posi = get(gca,'CurrentPoint');
    if mouse_posi(1,1)>0 && mouse_posi(1,1)<12000 && mouse_posi(1,2)>0 && mouse_posi(1,2)<12000
        lastPosi_Draw(1,1) = mouse_posi(1,1);%记录鼠标按下的位置
        lastPosi_Draw(1,2) = mouse_posi(1,2);%
    end
    draw_ok_flag = 0;
end
% --- 鼠标发生移动时的回调函数.
function figure1_WindowButtonMotionFcn(hObject, eventdata, handles)
global draw_ok_flag lastPosi_Draw
global barriers_num barriers_info_buf
global add_barrier_flag
if ~draw_ok_flag && add_barrier_flag  %没有添加成功 则显示
    cla
    % 设置GUI的axes控件属性
    axes(handles.axes1);
    set(handles.axes1,'XLim',[0 12000],'YLim',[0 12000]);
    theta = 0:2*pi/50:2*pi;
    %绘制上次的历史数据
    for index = 1:barriers_num
        x_dybamic=barriers_info_buf(index,2)+sin(theta)*barriers_info_buf(index,1);
        y_dybamic=barriers_info_buf(index,3)+cos(theta)*barriers_info_buf(index,1);
        plot(x_dybamic,y_dybamic,'*b');
        hold on
    end
    mouse_posi = get(gca,'CurrentPoint');
    
    r = sqrt((mouse_posi(1,1)-lastPosi_Draw(1,1))^2 + (mouse_posi(1,2)-lastPosi_Draw(1,2))^2);
    x_dybamic=lastPosi_Draw(1,1)+sin(theta)*r;
    y_dybamic=lastPosi_Draw(1,2)+cos(theta)*r;
    plot(x_dybamic,y_dybamic,'*b');
end
% --- 单机鼠标键松手后的回调函数
function figure1_WindowButtonUpFcn(hObject, eventdata, handles)
global curenPosi_Draw lastPosi_Draw draw_ok_flag
global barriers_num barriers_info_buf
global add_barrier_flag
if add_barrier_flag
    mouse_posi = get(gca,'CurrentPoint');
    if mouse_posi(1,1)>0 && mouse_posi(1,1)<12000 && mouse_posi(1,2)>0 && mouse_posi(1,2)<12000
        curenPosi_Draw(1,1) = mouse_posi(1,1);%记录鼠标按下的位置
        curenPosi_Draw(1,2) = mouse_posi(1,2);%
    end
        axes(handles.axes1);
        set(handles.axes1,'XLim',[0 12000],'YLim',[0 12000]);
        theta = 0:2*pi/50:2*pi;
        r = sqrt((curenPosi_Draw(1,1)-lastPosi_Draw(1,1))^2 + (curenPosi_Draw(1,2)-lastPosi_Draw(1,2))^2);
        x_dybamic=lastPosi_Draw(1,1)+sin(theta)*r;
        y_dybamic=lastPosi_Draw(1,2)+cos(theta)*r;
        plot(x_dybamic,y_dybamic,'*b');
        %记录障碍物的信息(圆心坐标 半径)
        barriers_num = barriers_num+1;
        barriers_info_buf(barriers_num,1) = r;
        barriers_info_buf(barriers_num,2) = lastPosi_Draw(1,1);
        barriers_info_buf(barriers_num,3) = lastPosi_Draw(1,2);
        hold on
        draw_ok_flag = 1;
end

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% 添加三角形障碍物
global curenPosi_Draw lastPosi_Draw draw_ok_flag
global barriers_num barriers_info_buf
global add_barrier_flag
global x_spots y_spots % 障碍物的横纵坐标点、
global PSO_barriers_num
global num_ba
add_barrier_flag = 0;
triangle_corner = ginput(3);
% line 1-2 ginput 1-2
axes(handles.axes1);

k1 = (triangle_corner(2,2)- triangle_corner(1,2))/(triangle_corner(2,1)- triangle_corner(1,1));
b1 = triangle_corner(2,2)-k1*triangle_corner(2,1);
x1 = triangle_corner(1,1):(triangle_corner(2,1)-triangle_corner(1,1))/50:triangle_corner(2,1);
y1 = k1.*x1+b1;

% line 1-3
k2 = (triangle_corner(3,2)- triangle_corner(1,2))/(triangle_corner(3,1)- triangle_corner(1,1));
b2 = triangle_corner(3,2)-k2*triangle_corner(3,1);
x2 = triangle_corner(3,1):(triangle_corner(1,1)-triangle_corner(3,1))/50:triangle_corner(1,1);
y2 = k2.*x2+b2;
% line 2-3
k3 = (triangle_corner(2,2)- triangle_corner(3,2))/(triangle_corner(2,1)- triangle_corner(3,1));
b3 = triangle_corner(2,2)-k3*triangle_corner(2,1);
x3 = triangle_corner(2,1):(triangle_corner(3,1)-triangle_corner(2,1))/50:triangle_corner(3,1);
y3 = k3.*x3+b3;

% plot lines
plot(x1,y1,'*b');
hold on

plot(x2,y2,'*b');
hold on

plot(x3,y3,'*b');
hold on

x1 = x1./100;
y1 = y1./100;

x2 = x2./100;
y2 = y2./100;

x3 = x3./100;
y3 = y3./100;
x_spots = [x_spots,x1];
y_spots = [y_spots,y1];

x_spots = [x_spots,x2];
y_spots = [y_spots,y2];

x_spots = [x_spots,x3];
y_spots = [y_spots,y3];

[m,n] = size(x_spots);
num_ba = num_ba+n


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% 添加矩形障碍物
global curenPosi_Draw lastPosi_Draw draw_ok_flag
global barriers_num barriers_info_buf
global add_barrier_flag
global x_spots y_spots % 障碍物的横纵坐标点、
global PSO_barriers_num
global num_ba
add_barrier_flag = 0;
corners = ginput(2);
% line 1-2 ginput 1-2
axes(handles.axes1);
y1 = corners(1,2):(corners(2,2)-corners(1,2))/20:corners(2,2);
[m n] = size(y1);
x1 = ones(1,n)*corners(1,1);

x2 = corners(1,1):(corners(2,1)-corners(1,1))/20:corners(2,1);
[m n] = size(x2);
y2 = ones(1,n)*corners(1,2);

y3 = corners(1,2):(corners(2,2)-corners(1,2))/20:corners(2,2);
[m n] = size(y3);
x3 = ones(1,n)*corners(2,1);

x4 = corners(1,1):(corners(2,1)-corners(1,1))/20:corners(2,1);
[m n] = size(x4);
y4 = ones(1,n)*corners(2,2);

plot(x1,y1,'*b');
hold on

plot(x2,y2,'*b');
hold on

plot(x3,y3,'*b');
hold on

plot(x4,y4,'*b');
hold on

x1 = x1./100;
y1 = y1./100;

x2 = x2./100;
y2 = y2./100;

x3 = x3./100;
y3 = y3./100;

x4 = x4./100;
y4 = y4./100;
x_spots = [x_spots,x1];
y_spots = [y_spots,y1];

x_spots = [x_spots,x2];
y_spots = [y_spots,y2];

x_spots = [x_spots,x3];
y_spots = [y_spots,y3];

x_spots = [x_spots,x4];
y_spots = [y_spots,y4];
% 
[m,n] = size(x_spots);
num_ba = num_ba+n

% --- 按键按下开始添加障碍物
function pushbutton1_Callback(hObject, eventdata, handles)
global add_barrier_flag
add_barrier_flag = 1;
% --- 按键按下停止添加障碍物
function endAddBarrier_Callback(hObject, eventdata, handles)
global add_barrier_flag
add_barrier_flag = 0;

% 退出操作
function exit_Callback(hObject, eventdata, handles)
button=questdlg('Are you sure to Exit？','Close message','Yes','No','Yes');
switch button
    case 'Yes'
        clear global
        clear
        close
    case 'No'
        return
end
% --- 按键按下执行路径规划主函数
function pushbutton3_Callback(hObject, eventdata, handles)
global P0 P1
N = 30;   %种群规模
D = 4;    %种群维数
Iter_max = 50; 
c1 = 2;
c2 = 2;
v_max = 4;
W_max = 0.9;
W_min = 0.1;
%______________________________________________
Period = zeros(1,D);  % 记录上次迭代的最优适应度值
n = 6;   % 速度变异的配比
Iter_current = 0;  % 当前迭代次数初始化

%% -------------- 初始化种群个体（位置和速度）---------------------
x = randn(N,D);
v = randn(N,D);
%--------Compute Particle's fitness>>By the function of my_fitness------
P_best = zeros(1,N);
y =zeros(N,D);
for i = 1:N
    P_best(i) = my_fitness( x(i,:));  % 个体极值
    y(i,:) = x(i,:);                  %最优解的位置
end
G_best = x(N,:);      % Global optimum 全局最优解
for i=1:(N-1)
    if my_fitness(x(i,:)) < my_fitness(G_best)
        G_best = x(i,:);
    end
end
%% Process of iteration. main code! 
h = waitbar(0,'Please wait...');
while Iter_current < Iter_max
    Iter_current = Iter_current+1;
    w = W_max-(W_max-W_min)./Iter_max.*Iter_current;
    for i=1:N
        v(i,:) = w.*v(i,:)+c1.*rand.*(y(i,:)-x(i,:))+c2.*rand.*(G_best-x(i,:));
        x(i,:) = x(i,:)+v(i,:);
        v(i,:) = check_v(v(i,:),v_max); % 检查粒子的速度，并且把速度限制在v_max与-v_max之间
        if my_fitness(x(i,:))<P_best(i)
            P_best(i) = my_fitness(x(i,:));
            y(i,:) = x(i,:);
        end
        if P_best(i)<my_fitness(G_best)
            Period = G_best;   % 记录上次迭代的最优解，为了优化速度
            G_best = y(i,:);
%                     suoyou = [suoyou,G_best];
        end
        if my_fitness(G_best)<my_fitness(Period)
           v(i,:) = n.*v(i,:); 
        else
            v(i,:) = -n.*v(i,:);
        end
    end
        waitbar(Iter_current/Iter_max,h,['Please wait...',num2str(2*Iter_current),'%']);
end
close(h);
x11 = G_best(1)*100; x21 = G_best(3)*100;
y11 = G_best(2)*100; y21 = G_best(4)*100;
x0 = P0(1)*100;
y0 = P0(2)*100;
x1 = P1(1)*100;
y1 = P1(2)*100;
tt = 0:1/79:1;
x_t1 = (-x0+3.*x11-3.*x21+x1).*tt.^3+(3.*x0-6.*x11+3.*x21).*tt.^2+(-3.*x0+3.*x11).*tt+x0;
y_t1 = (-y0+3.*y11-3.*y21+y1).*tt.^3+(3.*y0-6.*y11+3.*y21).*tt.^2+(-3.*y0+3.*y11).*tt+y0;
        %% 绘制路线数据和场地信息数据
        plot(x_t1,y_t1,'.r')    %  路线数据
        hold on
function [ f ] = my_fitness( x )
warning off
global x_pass x_spots y_spots num_ba P0 P1 D_safe
x_pass = x;
a1  = 1;
a2 = 1;
x0 = P0(1);
y0 = P0(2);
x1 = P1(1);
y1 = P1(2);
x11 = x(1); x21 = x(3);
y11 = x(2); y21 = x(4); 
tt = 0:1/99:1;
x_t1 = (-x0+3.*x11-3.*x21+x1).*tt.^3+(3.*x0-6.*x11+3.*x21).*tt.^2+(-3.*x0+3.*x11).*tt+x0;  
y_t1 = (-y0+3.*y11-3.*y21+y1).*tt.^3+(3.*y0-6.*y11+3.*y21).*tt.^2+(-3.*y0+3.*y11).*tt+y0;
% ---------------------距离约束-----------------
d_min_oi = zeros(1,num_ba);
    for i=1:num_ba
        d_oi1 = sqrt((x_t1-x_spots(i)).^2+(y_t1-y_spots(i)).^2);
        d_min_oi(i) = min(d_oi1);
    end    
d_min = min(d_min_oi); % 最小距离
     if d_min>D_safe
         f_safe = 0;                
     else
          f_safe =(1-d_min./D_safe).^2;
     end
% -----------------长度约束 -------------------- 
%   优化积分函数，改为直线段的长度相加
tt = 0:1/19:1;
x_suml = (-x0+3.*x11-3.*x21+x1).*tt.^3+(3.*x0-6.*x11+3.*x21).*tt.^2+(-3.*x0+3.*x11).*tt+x0;  
y_suml = (-y0+3.*y11-3.*y21+y1).*tt.^3+(3.*y0-6.*y11+3.*y21).*tt.^2+(-3.*y0+3.*y11).*tt+y0;
com_arr = [x_suml',y_suml'];
X = zeros(1,length(com_arr)-1);
for i=1:length(com_arr)-1
        X(i)=norm(com_arr(i,:)-com_arr(i+1,:));
end
L = sum(X(:));
L_min = sqrt((x1-x0)^2+(y1-y0)^2);
%     L = integral(@Tran_fcn,0,1);
f_len = (1-L_min/L)^2;
% -----------------总的罚函数为-----------------
f = a1*f_safe+a2*f_len;

function v = check_v( v,v_max)
v(v > v_max ) = v_max;
v(v < -v_max ) = -v_max;


% --- 选择路径的起始点
function pushbutton4_Callback(hObject, eventdata, handles)
global x_spots y_spots % 障碍物的横纵坐标点、
global barriers_num barriers_info_buf
global PSO_barriers_num
global num_ba
set_Start_End = ginput(2);
global P0 P1
P0(1,1) = set_Start_End(1,1)/100;
P0(1,2) = set_Start_End(1,2)/100;
P1(1,1) = set_Start_End(2,1)/100;
P1(1,2) = set_Start_End(2,2)/100;
if PSO_barriers_num~=barriers_num
    % 统计障碍物信息
    theta = 0:2*pi/50:2*pi;
    [temp spot_num] = size(theta);
    %绘制上次的历史数据
    for index = 1:barriers_num
        x_dybamic=(barriers_info_buf(index,2)+sin(theta)*barriers_info_buf(index,1))./100;
        y_dybamic=(barriers_info_buf(index,3)+cos(theta)*barriers_info_buf(index,1))./100;
        x_spots = [x_spots,x_dybamic];
        y_spots = [y_spots,y_dybamic];
    end
    PSO_barriers_num = barriers_num;
    num_ba = length(x_spots);           %记录障碍物的坐标点数    
end
