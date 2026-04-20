% =========================================================
%  Theo Jansen Walking Mechanism Simulation (Merged Final)
%  IE410 - Introduction to Robotics, Winter 2026
% =========================================================

clear; clc; close all;

%% ---- LINK LENGTHS (Jansen canonical, mm) ----
a  = 15;    % crank radius
b  = 38;    % ground link (O1 to O2, fixed horizontal)
c  = 41.5;  % A -> B
d  = 39.3;  % O2 -> B
e  = 55.8;  % A -> C
f  = 39.4;  % B -> C
g  = 36.7;  % M -> D  
h  = 65.7;  % C -> D
i  = 49.0;  % C -> Foot
j  = 50.0;  % D -> Foot
p  = 20.1;  % O1 -> M 
q  = 15.0;  % A -> M  

cos_delta = (a^2 + p^2 - q^2) / (2*a*p);
delta = acos(min(1, max(-1, cos_delta))); 

%% ---- SIMULATION PARAMETERS ----
N     = 720;
theta = linspace(0, 2*pi, N);
foot_x = NaN(1, N);
foot_y = NaN(1, N);
ALL = cell(1, N);

for k = 1:N
    th = theta(k);
    
    O1 = [0, 0];
    O2 = [b, 0];
    A = [a*cos(th), a*sin(th)];
    M = [p*cos(th - delta), p*sin(th - delta)];
    
    B = intersect2c(A, c, O2, d, -1);
    if isempty(B), continue; end
    
    C = intersect2c(A, e, B, f, 1);
    if isempty(C), continue; end
    
    D = intersect2c(M, g, C, h, -1);
    if isempty(D), continue; end
    
    Foot = intersect2c(C, i, D, j, 1);
    if isempty(Foot), continue; end
    
    foot_x(k) = Foot(1);
    foot_y(k) = Foot(2);
    ALL{k} = struct('O1',O1,'O2',O2,'A',A,'M',M,'B',B,'C',C,'D',D,'Foot',Foot);
end

valid = ~isnan(foot_x);
fx = foot_x(valid);
fy = foot_y(valid);
th_v = theta(valid);

crank_x = a * cos(th_v);
crank_y = a * sin(th_v);

%% ---- FIGURE 1: GAIT TRAJECTORY ----
figure('Name','Theo Jansen - Foot Trajectory','Color','w','Position',[80 80 700 440]);

y_thresh = min(fy) + 0.18 * (max(fy) - min(fy));
is_stance = fy <= y_thresh;
hold on;

plot(fx, fy, '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 3);

swx = fx; swx(is_stance)  = NaN;
swy = fy; swy(is_stance)  = NaN;
plot(swx, swy, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Swing phase');

stx = fx; stx(~is_stance) = NaN;
sty = fy; sty(~is_stance) = NaN;
plot(stx, sty, 'r-', 'LineWidth', 3, 'DisplayName', 'Stance phase');

plot(fx(1), fy(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start (\theta = 0)');

yg = min(fy);
plot([min(fx)-20, max(fx)+20], [yg yg], '--', 'Color', [0.3 0.65 0.2], 'LineWidth', 1.2, 'DisplayName', 'Ground reference');

idx_arr = round(linspace(1, sum(valid)-1, 8));
for ia = idx_arr
    dx = fx(min(ia+5,end)) - fx(ia);
    dy = fy(min(ia+5,end)) - fy(ia);
    if norm([dx dy]) > 0.5
        quiver(fx(ia), fy(ia), dx*0.4, dy*0.4, 0, 'Color', [0.5 0.5 0.9], 'MaxHeadSize', 2, 'LineWidth', 0.8, 'HandleVisibility', 'off');
    end
end

grid on; axis equal; box on;
xlabel('x  (mm)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('y  (mm)', 'FontSize', 12, 'FontWeight', 'bold');
title('Theo Jansen Mechanism — Foot-Point Gait Trajectory', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 10);

xl = xlim; yl = ylim;
step_len   = max(fx) - min(fx);
step_ht    = max(fy) - min(fy);
ann_str = sprintf('Step length:  %.1f mm\nStride height:  %.1f mm', step_len, step_ht);
text(xl(1)+0.55*(xl(2)-xl(1)), yl(1)+0.08*(yl(2)-yl(1)), ann_str, 'FontSize', 10, 'BackgroundColor', [0.93 0.95 1], 'EdgeColor', [0.7 0.7 0.9], 'Margin', 4);
hold off;

%% ---- FIGURE 2: TIME HISTORY ----
figure('Name','Theo Jansen - Time History','Color','w','Position',[800 80 680 400]);
subplot(2,1,1);
plot(rad2deg(th_v), fx, 'b-', 'LineWidth', 1.8);
yline(mean(fx), '--r', 'Mean', 'LabelHorizontalAlignment','left','FontSize',9);
xlabel('Crank angle (deg)'); ylabel('Foot X (mm)');
title('Foot X vs crank angle'); grid on; xlim([0 360]);

subplot(2,1,2);
plot(rad2deg(th_v), fy, 'r-', 'LineWidth', 1.8);
yline(mean(fy), '--b', 'Mean', 'LabelHorizontalAlignment','left','FontSize',9);
xlabel('Crank angle (deg)'); ylabel('Foot Y (mm)');
title('Foot Y vs crank angle'); grid on; xlim([0 360]);
sgtitle('Foot-Point Cartesian Coordinates over One Crank Cycle', 'FontSize', 12, 'FontWeight', 'bold');

%% ---- FIGURE 3: CRANK LENGTH SENSITIVITY ----
figure('Name','Theo Jansen - Crank Sensitivity','Color','w','Position',[80 540 680 380]);
crank_vals = [8, 11, 15, 20, 26];
cmap = lines(length(crank_vals));
hold on;

for ci = 1:length(crank_vals)
    a_mod = crank_vals(ci);
    cos_d = (a_mod^2 + p^2 - q^2) / (2*a_mod*p);
    d_mod = acos(min(1,max(-1,cos_d)));
    fxm = NaN(1,N); fym = NaN(1,N);
    for k = 1:N
        th = theta(k);
        A_m=[a_mod*cos(th), a_mod*sin(th)];
        M_m=[p*cos(th-d_mod), p*sin(th-d_mod)];
        B_m=intersect2c(A_m,c,[b,0],d,-1); if isempty(B_m), continue; end
        C_m=intersect2c(A_m,e,B_m,f,1);    if isempty(C_m), continue; end
        D_m=intersect2c(M_m,g,C_m,h,-1);   if isempty(D_m), continue; end
        Foot_m=intersect2c(C_m,i,D_m,j,1); if isempty(Foot_m), continue; end
        fxm(k)=Foot_m(1); fym(k)=Foot_m(2);
    end
    vld = ~isnan(fxm);
    plot(fxm(vld), fym(vld), '-', 'Color', cmap(ci,:), 'LineWidth', 1.8, 'DisplayName', sprintf('a = %d mm', crank_vals(ci)));
end

grid on; axis equal; box on;
xlabel('x (mm)', 'FontSize', 12); ylabel('y (mm)', 'FontSize', 12);
title('Effect of Crank Length on Foot Trajectory Shape', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location','northwest','FontSize',10);
hold off;

%% ---- FIGURE 4: CONTINUOUS ANIMATION ----
fig4 = figure('Name','Theo Jansen - Animation','Color','k', 'Position',[800 540 640 460]);

% Wide-angle camera view so the mechanism is fully visible
xl4 = [-60, 120];  
yl4 = [-40, 140];  

speed_factor = 12;   % INCREASED: Set to 12 for much faster animation (was 4)
k = 1;

% Infinite Loop for video recording
while true
    if ~ishandle(fig4), break; end
    
    if ~isempty(ALL{k})
        s = ALL{k};
        clf(fig4); ax4 = axes('Parent',fig4,'Color','k');
        set(ax4,'XColor','w','YColor','w','GridColor',[0.3 0.3 0.3]);
        hold(ax4,'on'); grid(ax4,'on');
        
        % Background paths
        plot(ax4, crank_x, crank_y, '--', 'Color', [1.0 0.4 0.4], 'LineWidth', 1.5, 'HandleVisibility','off');
        plot(ax4, fx, fy, '-', 'Color', [0.25 0.25 0.5], 'LineWidth', 1.2, 'HandleVisibility','off');
        
        if s.Foot(2) <= y_thresh
            trail_col = [0.9 0.3 0.2];
        else
            trail_col = [0.3 0.6 1.0];
        end
        
        % Links 
        lc = [0.7 0.7 0.7];   
        dk = [0.4 0.7 1.0];   
        pairs = {s.O1,s.O2; s.O1,s.A; s.O1,s.M; s.A,s.M;
                 s.A,s.B; s.O2,s.B; s.A,s.C; s.B,s.C;
                 s.M,s.D; s.C,s.D; s.C,s.Foot; s.D,s.Foot};
        colors_lnk = {lc;dk;dk;dk; lc;lc;lc;lc; lc;lc;lc;lc};
        widths_lnk  = {1.5;3;2;2; 2;2;2;2; 2;2;2;2};
        
        for li = 1:size(pairs,1)
            p1=pairs{li,1}; p2=pairs{li,2};
            plot(ax4,[p1(1) p2(1)],[p1(2) p2(2)],'-', 'Color',colors_lnk{li},'LineWidth',widths_lnk{li}, 'HandleVisibility','off');
        end
        
        % Ground pivots
        for gp = {s.O1, s.O2}
            pp=gp{1};
            plot(ax4,pp(1),pp(2),'s','MarkerSize',10, 'MarkerFaceColor',[0.6 0.3 0.9],'MarkerEdgeColor','w', 'LineWidth',1.5,'HandleVisibility','off');
        end
        
        % Regular joints
        for jp = {s.A, s.B, s.C, s.D, s.M}
            pp=jp{1};
            plot(ax4,pp(1),pp(2),'o','MarkerSize',6, 'MarkerFaceColor',[0.5 0.5 0.6],'MarkerEdgeColor','w', 'LineWidth',1,'HandleVisibility','off');
        end
        
        % Foot point
        plot(ax4,s.Foot(1),s.Foot(2),'o','MarkerSize',12, 'MarkerFaceColor',trail_col,'MarkerEdgeColor','w', 'LineWidth',1.5,'HandleVisibility','off');
        
        % Ground reference
        plot(ax4,[xl4(1) xl4(2)],[yg yg],'--','Color',[0.3 0.7 0.25], 'LineWidth',1,'HandleVisibility','off');
        
        axis(ax4,'equal');
        xlim(ax4,xl4); ylim(ax4,yl4);
        title(ax4, sprintf('Theo Jansen Mechanism  |  \\theta = %.0f°', rad2deg(theta(k))), 'Color','w','FontSize',12,'FontWeight','bold');
        xlabel(ax4,'x (mm)','Color','w','FontSize',10);
        ylabel(ax4,'y (mm)','Color','w','FontSize',10);
        
        % Legend proxies
        plot(ax4,NaN,NaN,'-','Color',dk,'LineWidth',2.5,'DisplayName','Crank');
        plot(ax4,NaN,NaN,'-','Color',lc,'LineWidth',2,'DisplayName','Linkage');
        plot(ax4,NaN,NaN,'o','MarkerFaceColor',[0.3 0.8 0.3],'MarkerEdgeColor','w', 'MarkerSize',8,'DisplayName','Foot (swing)');
        plot(ax4,NaN,NaN,'o','MarkerFaceColor',[0.9 0.3 0.2],'MarkerEdgeColor','w', 'MarkerSize',8,'DisplayName','Foot (stance)');
        legend(ax4,'Location','northwest','FontSize',8,'TextColor','w','Color','k');
        hold(ax4,'off');
        
        drawnow;
    end
    
    k = mod(k + speed_factor - 1, N) + 1;
end

%% =========================================================
%  LOCAL FUNCTIONS
%% =========================================================
function P = intersect2c(A, rA, B, rB, sgn)
    d2 = sum((B-A).^2);
    d  = sqrt(d2);
    if d > rA+rB+1e-9 || d < abs(rA-rB)-1e-9 || d < 1e-9
        P = []; return;
    end
    a2 = (rA^2 - rB^2 + d2) / (2*d);
    h  = sqrt(max(0, rA^2 - a2^2));
    dx = (B(1)-A(1))/d;  dy = (B(2)-A(2))/d;
    Mx = A(1) + a2*dx;   My = A(2) + a2*dy;
    P  = [Mx + sgn*h*dy,  My - sgn*h*dx];
end