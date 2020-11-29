function plotResult(nav, cfg)
% -------------------------------------------------------------------------
%PLOTRESULT plot the results
% -------------------------------------------------------------------------
% Author:
% Qijin Chen, GNSS Research Center, Wuhan University, China.;
% chenqijin@whu.edu.cn;
% Nov. 2019;
% -------------------------------------------------------------------------

t0 = floor(nav(1,1)/100)*100;

%%  Figure1, plot the mounting angle estimate
if cfg.plot_ma
    figure,plot(nav(:,1)-t0, nav(:,6:7)*180/pi, 'linewidth', 2.0);
    set(gca, 'fontsize', 12);
    xlabel(['Time -' num2str(t0) ' / s'], 'fontsize', 12);
    ylabel('deg', 'fontsize', 12);
    legend({'\delta\theta', '\delta\psi'}, 'orientation', 'horizontal', 'Box', 'off', 'fontsize', 13);
    title('Mounting Angle Estimate', 'fontsize', 12, 'color', 'k');
    ylim([0 4]);
end

%% figure2, plot the estimated errors in the GNSS/INS smoothing attitude 
if cfg.plot_att_err
    figure,
    f1 = subplot(311); plot(nav(:,1)-t0, nav(:,8)*180/pi, 'color', [0.3 0.3 0.3]);
    set(gca, 'fontsize', 12);
    ylabel('\phi / deg', 'fontsize', 12, 'color', 'r');
    title('Attitude Error Estimate', 'fontsize', 12, 'color', 'k');
    f2 = subplot(312); plot(nav(:,1)-t0, nav(:,9)*180/pi, 'color', [0.3 0.3 0.3]);
    set(gca, 'fontsize', 12);
    ylabel('\theta / deg', 'fontsize', 12, 'color', 'r');
    f3 = subplot(313); plot(nav(:,1)-t0, nav(:,10)*180/pi, 'color', [0.3 0.3 0.3]);
    set(gca, 'fontsize', 12);
    ylabel('\psi / deg', 'fontsize', 12, 'color', 'r');
    xlabel(['Time -' num2str(t0) ' / s'], 'fontsize', 12);
    linkaxes([f1 f2 f3], 'x');
end

%% figure3, plot the estimated scale factor of the distance measurements.
if cfg.plot_odosf
    figure,plot(nav(:,1)-t0, nav(:,11)*1.0e6, 'color', [0.3 0.3 0.3]);
    set(gca, 'fontsize', 12);
    xlabel(['Time -' num2str(t0) ' / s'], 'fontsize', 12);
    ylabel('PPM', 'fontsize', 12);
    title('Odometer Scale Factor Error Estimate', 'fontsize', 12, 'color', 'k');
end
end