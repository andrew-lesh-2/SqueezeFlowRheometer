%% Load Data

sfrDataFolder = "C:\Users\rcmoo\Documents\GitHub\SqueezeFlowRheometer\data\";

sfrFiles = ["2023-07-13_11-38-52_PID_squeeze_flow_1_Test1a-Carbopol_1mL_5g-data.csv";
    "2023-07-13_12-34-44_PID_squeeze_flow_1_Test2a-Carbopol_1mL_5g-data.csv";
    "2023-07-13_12-56-20_PID_squeeze_flow_1_Test3a-Carbopol_1mL_30g-data.csv";
    "2023-07-13_14-33-28_PID_squeeze_flow_1_Test4a-Carbopol_5mL_10g-data.csv";
    "2023-07-18_10-21-01_PID_squeeze_flow_1_Test1a-Carbopol_1mL_5g-data.csv";
    % "2023-07-18_13-36-55_PID_squeeze_flow_1_Test3a-Carbopol_1mL_5g-data.csv"; % force signal was very noisy due to control system issues
    "2023-07-18_14-28-17_PID_squeeze_flow_1_Test4a-Carbopol_2mL_5g-data.csv";
    "2023-07-18_15-18-45_PID_squeeze_flow_1_Test5a-Carbopol_4mL_5g-data.csv";
    % "2023-07-19_14-03-21_PID_squeeze_flow_1_Test1a_Carbopol_1mL_5g-data.csv"; % 07-19 data was not good
    % "2023-07-19_15-03-59_PID_squeeze_flow_1_Test2c-Carbopol_1mL_5g-data.csv";
    % "2023-07-19_15-50-08_PID_squeeze_flow_1_Test3a_Carbopol_1mL_5g-data.csv";
    % "2023-07-19_16-17-19_PID_squeeze_flow_1_Test4a_Carbopol_1mL_5g-data.csv";
    % "2023-07-20_10-30-09_PID_squeeze_flow_1_Test1a_KI=0.7_KP=0.005_decay_rate=-0.1507_carbopol_1mL_5g-data.csv";
    "2023-07-20_10-51-52_PID_squeeze_flow_1_Test2a_carbopol_KI=0.01_power=2.5_1mL_5g-data.csv";
    "2023-07-20_11-22-20_PID_squeeze_flow_1_Test3a_carbopol_KP=3_KI=0.03_power=1_1mL_5g-data.csv";
    "2023-07-20_11-51-48_PID_squeeze_flow_1_Test4a_carbopol_limited_interr_influence_1mL_5g-data.csv";
    "2023-07-20_13-28-05_PID_squeeze_flow_1_Test5a_carbopol_controlled_KP_error_1mL_5g-data.csv";
    "2023-07-20_13-51-13_PID_squeeze_flow_1_Test6a_carbopol_smaller_limitation_carbopol_1mL_5g-data.csv";
    "2023-07-20_14-13-09_PID_squeeze_flow_1_Test7a_carbopol_big_v_test_for_changed_limitations_6mL_5g-data.csv";
    % "2023-07-27_13-50-25_PID_squeeze_flow_1_Test1a-CarbopolA_1mL_5g-data.csv"; % abnormally small volume
    "2023-07-27_14-37-43_PID_squeeze_flow_1_Test2a-CarbopolA_1mL_5g-data.csv";
    "2023-07-28_12-32-43_PID_squeeze_flow_1_Test1a-CarbopolB_1mL_5g-data.csv";
    % "2023-07-28_13-30-08_PID_squeeze_flow_1_Test2a-CarbopolB_1mL_5g-data.csv"; % fiddling with control system, data was weird
    % "2023-07-28_14-18-07_PID_squeeze_flow_1_Test3a-CarbopolB_1mL_5g-data.csv"; % fiddling with control system, data was weird
    % "2023-07-31_11-35-36_PID_squeeze_flow_1_Test1a-CarbopolB_0mL_5g-data.csv"; % this was when it was slowed down and so it didn't maintain constant force as usual
    "2023-07-31_15-20-49_PID_squeeze_flow_1_Test2a-CarbopolB_2mL_5g-data.csv";
    "2023-07-31_16-40-45_PID_squeeze_flow_1_Test3a-CarbopolB_2mL_5g-data.csv";
    % "2023-07-31_16-54-45_PID_squeeze_flow_1_Test4b-CarbopolB_2mL_20g-data.csv"; % it was not allowed to reach equilibrium and is not valid
    "2023-08-01_12-01-47_PID_squeeze_flow_1_Test1a-CarbopolA_2mL_5g-data.csv";
    "2023-08-01_12-46-21_PID_squeeze_flow_1_Test2a-CarbopolA_3mL_5g-data.csv";
    "2023-08-01_13-47-01_PID_squeeze_flow_1_Test3a-CarbopolA_4mL_5g-data.csv";
    "2023-08-01_14-48-46_PID_squeeze_flow_1_Test4a-CarbopolA_5mL_5g-data.csv";
    "2023-08-01_15-58-22_PID_squeeze_flow_1_Test5a-CarbopolB_1mL_5g-data.csv";
    "2023-08-02_13-12-39_PID_squeeze_flow_1_Test1a-CarbopolB_2mL_5g-data.csv";
    "2023-08-02_14-19-49_PID_squeeze_flow_1_Test2b-CarbopolB_3mL_5g-data.csv";
    "2023-08-02_15-26-10_PID_squeeze_flow_1_Test3a-CarbopolB_4mL_5g-data.csv";
    "2023-08-02_16-28-59_PID_squeeze_flow_1_Test4a-CarbopolB_5mL_5g-data.csv";
    "2023-08-04_13-56-50_set_gap_squeeze_flow_Test1c-CarbopolC1_1mL_10g-data.csv";
    "2023-08-04_14-19-02_set_gap_squeeze_flow_Test2a-CarbopolC1_1mL_10g-data.csv";
    "2023-08-04_14-37-02_set_gap_squeeze_flow_Test3a-CarbopolC1_1mL_10g-data.csv";
    "2023-08-07_12-54-25_set_gap_squeeze_flow_Test5a-CarbopolC1_2mL_12g-data.csv";
    % "2023-08-07_16-02-30_set_gap_squeeze_flow_Test6a-CarbopolA_1mL_11g-data.csv";
    "2023-08-07_16-26-50_set_gap_squeeze_flow_Test7a-CarbopolA_1mL_11g-data.csv";
    "2023-08-07_16-55-47_set_gap_squeeze_flow_Test8a-CarbopolA_1mL_10g-data.csv";
    "2023-08-14_11-48-43_PID_squeeze_flow_1_Test1a-CarbopolA_0mL_5g-data.csv";
    "2023-08-14_12-32-48_PID_squeeze_flow_1_Test2a-CarbopolA_0mL_5g-data.csv";
    ];

s = sfrEmptyStructGenerator();
sfrStructs = repmat(s,length(sfrFiles),1);
for i = 1:length(sfrFiles)
    filePath = sfrDataFolder + sfrFiles(i);
    % sfrStructs(i) = sfrStructGenerator(filePath,xq,yq);
    sfrStructs(i) = sfrStructGenerator(filePath);
    % sfrFiles(i)
    % sfrStructs(i)
end

%%% Exclude bad data

% 2023-07-20 Test3a had very bad noise from control system in last 2 steps.
    % Exclude last 2 steps
idx = find(strcmp(sfrFiles,"2023-07-20_11-22-20_PID_squeeze_flow_1_Test3a_carbopol_KP=3_KI=0.03_power=1_1mL_5g-data.csv"));
if ~isempty(idx)
    sfrStructs(idx).StepEndIndices = sfrStructs(idx).StepEndIndices(1:2,:);
end

% 2023-07-20 Test4a was abruptly ended before the last step could finish.
    % Exclude last step
idx = find(strcmp(sfrFiles,"2023-07-20_11-51-48_PID_squeeze_flow_1_Test4a_carbopol_limited_interr_influence_1mL_5g-data.csv"));
if ~isempty(idx)
    sfrStructs(idx).StepEndIndices = sfrStructs(idx).StepEndIndices(1:3,:);
end

% 2023-07-20 Test7a had very bad noise from control system in last 2 steps,
    % unclear why. Exclude last 2 steps
idx = find(strcmp(sfrFiles,"2023-07-20_14-13-09_PID_squeeze_flow_1_Test7a_carbopol_big_v_test_for_changed_limitations_6mL_5g-data.csv"));
if ~isempty(idx)
    sfrStructs(idx).StepEndIndices = sfrStructs(idx).StepEndIndices(1:3,:);
end

% 2023-07-27 Test1a weird behavior in step 5 unclear why.
    % Exclude step 5
idx = find(strcmp(sfrFiles,"2023-07-27_13-50-25_PID_squeeze_flow_1_Test1a-CarbopolA_1mL_5g-data.csv"));
if ~isempty(idx)
    sfrStructs(idx).StepEndIndices = sfrStructs(idx).StepEndIndices([1:4,6:10],:);
end

% 2023-07-27 Test2a weird behavior in steps 8 and 10 unclear why.
    % Exclude steps 8 and 10
idx = find(strcmp(sfrFiles,"2023-07-27_14-37-43_PID_squeeze_flow_1_Test2a-CarbopolA_1mL_5g-data.csv"));
if ~isempty(idx)
    sfrStructs(idx).StepEndIndices = sfrStructs(idx).StepEndIndices([1:7,9],:);
end

% 2023-07-28 Test1a weird behavior from step 7 onward
idx = find(strcmp(sfrFiles,"2023-07-28_12-32-43_PID_squeeze_flow_1_Test1a-CarbopolB_1mL_5g-data.csv"));
if ~isempty(idx)
    sfrStructs(idx).StepEndIndices = sfrStructs(idx).StepEndIndices(1:6,:);
end

% 2023-07-31 Test1a film became too thin to appropriately reach target
    % force after step 6. Exclude steps 7-10
idx = find(strcmp(sfrFiles,"2023-07-31_11-35-36_PID_squeeze_flow_1_Test1a-CarbopolB_0mL_5g-data.csv"));
if ~isempty(idx)
    sfrStructs(idx).StepEndIndices = sfrStructs(idx).StepEndIndices(1:6,:);
end

% 2023-07-31 Test2a had something weird in step 10. Exclude it.
idx = find(strcmp(sfrFiles,"2023-07-31_15-20-49_PID_squeeze_flow_1_Test2a-CarbopolB_2mL_5g-data.csv"));
if ~isempty(idx)
    sfrStructs(idx).StepEndIndices = sfrStructs(idx).StepEndIndices(1:9,:);
end

% 2023-08-01 Test2a had weird plateau in gap for step 14
idx = find(strcmp(sfrFiles,"2023-08-01_12-46-21_PID_squeeze_flow_1_Test2a-CarbopolA_3mL_5g-data.csv"));
if ~isempty(idx)
    sfrStructs(idx).StepEndIndices = sfrStructs(idx).StepEndIndices([1:13,15],:);
end

% 2023-08-01 Test5a had weird plateaus in gap for steps 6, 8, 9, and 11 on
idx = find(strcmp(sfrFiles,"2023-08-01_15-58-22_PID_squeeze_flow_1_Test5a-CarbopolB_1mL_5g-data.csv"));
if ~isempty(idx)
    sfrStructs(idx).StepEndIndices = sfrStructs(idx).StepEndIndices([1:5,7,10],:);
end

% 2023-08-02 Test1a had weird plateau in gap for step 9
idx = find(strcmp(sfrFiles,"2023-08-02_13-12-39_PID_squeeze_flow_1_Test1a-CarbopolB_2mL_5g-data.csv"));
if ~isempty(idx)
    sfrStructs(idx).StepEndIndices = sfrStructs(idx).StepEndIndices([1:5,7,10],:);
end

% 2023-08-02 Test4a I accidentally bumped the end effector in step 3
idx = find(strcmp(sfrFiles,"2023-08-02_16-28-59_PID_squeeze_flow_1_Test4a-CarbopolB_5mL_5g-data.csv"));
if ~isempty(idx)
    % sfrStructs(idx).StepEndIndices = sfrStructs(idx).StepEndIndices([1:2,4:10],:);
end

%%% Get list of unique dates
date_strs = strings(length(sfrFiles),1);
for i = 1:length(sfrFiles)
    % date_strs(i) = extractAfter(extractBefore(extractBefore(sfrFiles(i),"PID"),"_"),"-");
    date_strs(i) = sfrStructs(i).dateStr;
end
date_strs = unique(date_strs);

%%% Get list of unique samples
% sample_strs = strings(length(sfrFiles),1);
% for i = 1:length(sfrFiles)
%     sample_strs(i) = sfrStructs(i).sampleSubstance;
% end
% sample_strs = unique(sample_strs);
% nice_sample_strs = ["Carbopol", "Carbopol A", "Carbopol B"];

sample_strs = ["carbopola", "carbopolb", "carbopol", "carbopolc1"];
nice_sample_strs = ["1.46wt% Carbopol, 0.12wt% NaOH", "1.46wt% Carbopol, 0.18wt% NaOH", "1.46wt% Carbopol, 0.29wt% NaOH", "1.46wt% Carbopol, 0.29wt% NaOH"];

%% Plot Data
colors = ["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30","#4DBEEE","#A2142F"];
markers = ['o','s','d','^','p','h','<','>','v','*','+'];

colorList = parula();

minVol = sfrStructs(1).V(1);
maxVol = sfrStructs(1).V(1);
for i = 2:length(sfrStructs)
    minVol = min(minVol, sfrStructs(i).V(1));
    maxVol = max(maxVol, sfrStructs(i).V(1));
end


% plot data, changing symbol by day and color by test
if true
figure(1)
for i = 1:length(sfrFiles)
    s = sfrStructs(i);
    DisplayName = s.dateStr + " " + s.testNum + " " + s.volStr;

    plotColor = colors(mod(i - 1, length(colors)) + 1);
    fillColor = plotColor;
    % if i > 7 % make symbols hollow after some point
    %     fillColor = 'auto';
    % end

    markerIdx = find(strcmp(date_strs,s.dateStr));
    markerStr = markers(markerIdx);

    plot(s.aspectRatio(s.StepEndIndices(:,2)),...
        s.MeetenYieldStress(s.StepEndIndices(:,2)),markerStr,...
        'DisplayName',DisplayName,'MarkerEdgeColor',plotColor,...
        'MarkerFaceColor',fillColor);

    hold on
end
hold off
xlabel('h/R [-]')
ylabel('Yield Stress [Pa]')

% Add legend for the first/main plot handle
hLegend = legend('location','northeast');
hLegend.NumColumns = 2;
title("Perfect Slip, Meeten (2000)")
end
sfrPrettyPlot(false)

% plot data, changing symbol by day and color by sample
if true
figure(2)
for i = 1:length(sfrFiles)
    s = sfrStructs(i);
    % DisplayName = s.dateStr + " " + s.testNum + " " + s.volStr;
    sampleStr = upper(extractAfter(s.sampleSubstance,"carbopol"));
    DisplayName = s.dateStr + " " + s.testNum + " " + s.volStr + " " + sampleStr;


    colorIdx = find(strcmp(sample_strs,s.sampleSubstance));
    plotColor = colors(colorIdx);
    fillColor = plotColor;
    % if i > 7 % make symbols hollow after some point
    %     fillColor = 'auto';
    % end

    markerIdx = find(strcmp(date_strs,s.dateStr));
    markerStr = markers(markerIdx);

    plotColor = 'k';
    fillColor = 'k';

    plot(s.aspectRatio(s.StepEndIndices(:,2)),...
        s.MeetenYieldStress(s.StepEndIndices(:,2)),markerStr,...
        'DisplayName',DisplayName,'MarkerEdgeColor',plotColor,...
        'MarkerFaceColor',fillColor);

    hold on
end
hold off
xlabel('h/R [-]')
ylabel('Yield Stress [Pa]')

% Add legend for the first/main plot handle
hLegend = legend('location','northeast');
hLegend.NumColumns = 2;
title("Perfect Slip, Meeten (2000)")
end
sfrPrettyPlot(false)

% make separate subplots for each sample
if true
figure(3)
t = tiledlayout("horizontal","TileSpacing","tight");
axs = gobjects(length(sample_strs),1);
for i = 1:length(sample_strs)
    if(contains(sample_strs(i),'carbopolc',IgnoreCase=true))
        continue
    end
    axs(i) = nexttile;
    plotted_counter = 1;
    for j = 1:length(sfrStructs)
        if ~strcmpi(sfrStructs(j).sampleSubstance,sample_strs(i))
            if ~(strcmpi(sample_strs(i),'carbopol') && contains(sfrStructs(j).sampleSubstance,'carbopolc',IgnoreCase=true))
                continue
            end
        end
        s = sfrStructs(j);

        DisplayName = s.dateStr + " " + s.testNum + " " + s.volStr;
    
        plotColor = colors(mod(plotted_counter - 1, length(colors)) + 1);
        if strcmpi(sample_strs(i),'carbopol')
            plotColor = colors(1);
            if contains(sfrStructs(j).sampleSubstance,'carbopolc',IgnoreCase=true)
                plotColor = colors(2);
            end
        end
        fillColor = plotColor;
    
        markerIdx = find(strcmp(date_strs,s.dateStr));
        markerStr = markers(markerIdx);
    
        plot(s.aspectRatio(s.StepEndIndices(:,2)),...
            s.MeetenYieldStress(s.StepEndIndices(:,2)),markerStr,...
            'DisplayName',DisplayName,'MarkerEdgeColor',plotColor,...
            'MarkerFaceColor',fillColor);
    
        hold on
        plotted_counter = plotted_counter + 1;
    end
    hold off
    xlabel('h/R [-]')

    % Have y-ticks only on far left and right
    if(i == length(sample_strs))
        set(gca,'YAxisLocation','right')
    elseif i ~= 1
        set(gca,'yticklabel',[])
    end
    
    % Add legend for the first/main plot handle
    hLegend = legend('location','best');
    hLegend.NumColumns = 2;
    % title(sample_strs(i))
    title(nice_sample_strs(i))
    grid on
end
linkaxes(axs,'xy')
xlim([0,1])
% ylim([0,250])
title(t,'Perfect Slip, Meeten (2000)')
ylabel(t,'Yield Stress [Pa]')
sfrPrettyPlot(false)
end

%% Sherwood and Durban

% Sherwood and Durban propose F = (pi * D^3 * m * tau_0)/(12*h) +
% (sqrt(3) * pi *D ^2 * tau_0)/8 * (sqrt(1 - m^2) + 1/m * asin(m))

% Call the far right terms with m f(m) = sqrt(1 - m^2) + 1/m * asin(m)
f = @(m) sqrt(1 - m^2) + asin(m)/m;

% with force, volume, and height, this can become
% F = Omega*tau_0/h * [ (m*D)/(3*h) + sqrt(3)/2 * f(m) ]

% Which should give
% F*h / (Omega*tau_0) = (m/3)*(D/h) + sqrt(3)/2 * f(m)
% Which is linear in aspect ratio
% plot F h / Omega vs. R/h

% actually h is on both sides


% plot data, changing symbol by day and color by test
if true
figure(1)
for i = 1:length(sfrFiles)
    s = sfrStructs(i);
    DisplayName = s.dateStr + " " + s.testNum + " " + s.volStr;

    plotColor = colors(mod(i - 1, length(colors)) + 1);
    fillColor = plotColor;
    % if i > 7 % make symbols hollow after some point
    %     fillColor = 'auto';
    % end

    markerIdx = find(strcmp(date_strs,s.dateStr));
    markerStr = markers(markerIdx);

    y = (s.F .* s.h) ./ (s.V);

    plot(s.aspectRatio(s.StepEndIndices(:,2)),...
        y(s.StepEndIndices(:,2)),markerStr,...
        'DisplayName',DisplayName,'MarkerEdgeColor',plotColor,...
        'MarkerFaceColor',fillColor);

    % plot(s.aspectRatio(s.StepEndIndices(:,2)),...
    %     s.MeetenYieldStress(s.StepEndIndices(:,2)),markerStr,...
    %     'DisplayName',DisplayName,'MarkerEdgeColor',plotColor,...
    %     'MarkerFaceColor',fillColor);

    hold on
end
hold off
xlabel('h/R [-]')
ylabel('Yield Stress [Pa]')

% Add legend for the first/main plot handle
hLegend = legend('location','northeast');
hLegend.NumColumns = 2;
title("Perfect Slip, Meeten (2000)")
end
sfrPrettyPlot(false)



% append all carbopol B results
if true

i = 2;

carbopol_B_structs = {};

for j = 1:length(sfrStructs)
    if ~strcmpi(sfrStructs(j).sampleSubstance,sample_strs(i))
        if ~(strcmpi(sample_strs(i),'carbopol') && contains(sfrStructs(j).sampleSubstance,'carbopolc',IgnoreCase=true))
            continue
        end
    end
    s = sfrStructs(j);
    carbopol_B_structs{end + 1} = s;
end

y = [];
x = [];
for i = 1:length(carbopol_B_structs)
    s = carbopol_B_structs{i};
    
    y_ = s.F .* s.h ./ s.V;
    x_ = s.R ./ s.h;

    indices = s.StepEndIndices(:,2);

    y_ = y_(indices);
    x_ = x_(indices);

    y = [y; y_];
    x = [x; x_];
end

end

%% Do linear fit for each sample

% make separate subplots for each sample
if true
figure(3)
t = tiledlayout("horizontal","TileSpacing","tight");
axs = gobjects(length(sample_strs),1);
for i = 1:length(sample_strs)
    axs(i) = nexttile;
    plotted_counter = 1;

    h_R = [];
    yieldStress = [];

    for j = 1:length(sfrStructs)
        if ~strcmpi(sfrStructs(j).sampleSubstance,sample_strs(i))
            continue
        end

        s = sfrStructs(j);
        
        h_R = [h_R; s.aspectRatio(s.StepEndIndices(:,2))];
        yieldStress = [yieldStress; s.MeetenYieldStress(s.StepEndIndices(:,2))];

        DisplayName = s.dateStr + " " + s.testNum + " " + s.volStr;
    
        plotColor = colors(mod(plotted_counter - 1, length(colors)) + 1);
        fillColor = plotColor;
    
        markerIdx = find(strcmp(date_strs,s.dateStr));
        markerStr = markers(markerIdx);
    
        plot(s.aspectRatio(s.StepEndIndices(:,2)),...
            s.MeetenYieldStress(s.StepEndIndices(:,2)),markerStr,...
            'DisplayName',DisplayName,'MarkerEdgeColor',plotColor,...
            'MarkerFaceColor',fillColor);
    
        hold on
        plotted_counter = plotted_counter + 1;
    end

    X = [ones(length(h_R),1), h_R];
    y = yieldStress;
    b = X \ y;
    % yieldStressIntercept = b(1);
    % abs(b(1) / b(2))
    fitdist(yieldStress(h_R < 0.1),'Normal')

    xl = xlim;
    % yl = ylim;
    xq = linspace(min(xl), max(xl));
    trendlineStr = "y = " + num2str(b(2),'%.1f') + "x + " + num2str(b(1),'%.1f');
    plot(xq, xq*b(2) + b(1), 'k-', 'DisplayName', trendlineStr)

    hold off
    xlabel('h/R [-]')

    % Have y-ticks only on far left and right
    if(i == length(sample_strs))
        set(gca,'YAxisLocation','right')
    elseif i ~= 1
        set(gca,'yticklabel',[])
    end
    
    % Add legend for the first/main plot handle
    hLegend = legend('location','best');
    hLegend.NumColumns = 2;
    % title(sample_strs(i))
    title(nice_sample_strs(i))
    grid on
end
linkaxes(axs,'xy')
xlim([0,1])
ylim([0,250])
title(t,'Perfect Slip, Meeten (2000)')
ylabel(t,'Yield Stress [Pa]')
sfrPrettyPlot(false)
end


%% Plot strain rates for everything

if true
figure(4)
for i = 1:length(sfrStructs)
    s = sfrStructs(i);
    % x = s.h;
    % y = abs(s.v ./ s.h);
    % y = abs(s.v);
    x = abs(s.v ./ s.h);
    y = s.F;
    semilogx(x,y)
    hold on
end
hold off
xlabel('Effective Extensional Strain Rate v/s [s^{-1}]')
ylabel('Force [N]')
sfrPrettyPlot
xlim([1e-2,1])
end

%% Look at variance of force signal versus gap

portion_of_step = 0.5; % look at the last __ fraction of the force signal in that step (don't look at the start because it needs a chance to try and equilibrate)

F_vars = [];
F_stds = [];
h_infinitys = [];
for i = 1:length(sfrFiles)
    for j = 1:size(sfrStructs(i).StepEndIndices,1)
        idxs = sfrStructs(i).StepEndIndices(j,:);
        var_indices = floor(idxs(2) - portion_of_step*(idxs(2) - idxs(1))):idxs(2);
        F_var = var(sfrStructs(i).F(var_indices));
        F_vars = [F_vars; F_var];
        F_stds = [F_stds; sqrt(F_var)];
        h_infinitys = [h_infinitys; sfrStructs(i).h(idxs(2))];
    end
end

y = log(F_stds);
X = [ones(length(h_infinitys),1), log(h_infinitys)];
q = X \ y;
c = exp(q(1))
n = q(2)

mean_F_std = mean(F_stds);
SST = sum((F_stds - mean_F_std).^2);
SSR = sum((F_stds - (c * h_infinitys.^n)).^2);
R_squared = 1 - SSR / SST

figure(8)
scatter(h_infinitys, F_stds,'o','filled')
set(gca, 'xscale', 'log', 'yscale', 'log')
hold on
xl = xlim;
xq = linspace(xl(1),xl(2));
yl = ylim;
% plot(xq, c./xq, 'k-');
plot(xq, c * xq.^n, 'k-');
hold off

xlabel('h [m]')
ylabel('Force Standard Deviation \sigma [N]')
title('Force Variation with Gap')


%% Investigate surface tension impact

sigma = 0.066; % From Boujlel & Coussot (2013)
theta = 1.8; % bows outward, which is what we observe in experiments

figure(8)
% plot sfr data
for i = 1:length(sfrFiles)
    testNum = split(sfrFiles(i),"PID_squeeze_flow_1_Test");
    dateStr = extractAfter(extractBefore(testNum(1),"_"),"-"); % get just month and day
    testNum = split(testNum(2), "-");
    testNum = split(testNum(1), "_");
    testNum = testNum(1);
    volStr = num2str(sfrStructs(i).V(1)*10^6,3);
    DisplayName = dateStr + " " + testNum + " " + volStr + "mL";
    plotColor = colors(mod(i - 1, length(colors)) + 1);
    fillColor = plotColor;

    % if i > 7 % make symbols hollow after some point
    %     fillColor = 'auto';
    % end

    markerIdx = find(strcmp(date_strs,dateStr));
    markerStr = markers(markerIdx);

    x = sfrStructs(i).aspectRatio(sfrStructs(i).StepEndIndices(:,2));
    % y = sfrStructs(i).MeetenYieldStress(sfrStructs(i).StepEndIndices(:,2));
    y = (sfrStructs(i).F(sfrStructs(i).StepEndIndices(:,2)) .* sfrStructs(i).h(sfrStructs(i).StepEndIndices(:,2)) ./ sfrStructs(i).V(1)) / sqrt(3);
    % y = y / max(y);

    yyaxis left
    % semilogx(x,y,...
    %     markerStr,'DisplayName',DisplayName,'MarkerEdgeColor',plotColor,...
    %     'MarkerFaceColor',fillColor);
    semilogx(x,y,...
        markerStr,'DisplayName',DisplayName,'MarkerEdgeColor',colors(1),...
        'MarkerFaceColor',colors(1));
    % semilogx(x,y,...
        % markerStr);
    yyaxis right
    y = sfrStructs(i).V(1) ./ sfrStructs(i).h(sfrStructs(i).StepEndIndices(:,2));
    F_sigma = sigma * (-2*cos(theta)*y./ sfrStructs(i).h(sfrStructs(i).StepEndIndices(:,2)) + sqrt(pi * y));
    F_yield_stress = sfrStructs(i).F(sfrStructs(i).StepEndIndices(:,2)) - F_sigma;
    y = (F_yield_stress .* sfrStructs(i).h(sfrStructs(i).StepEndIndices(:,2)) ./ sfrStructs(i).V(1)) / sqrt(3);
    % y = y / max(y);
    % semilogx(x,y,...
    %     markerStr);
    semilogx(x,y,...
        markerStr,'DisplayName',DisplayName,'MarkerEdgeColor',colors(2),...
        'MarkerFaceColor',colors(2));

    hold on
end
hold off
xlabel('h/R [-]')
yyaxis left
ylabel('Yield Stress, Meeten (2000) [Pa]')
ylim([0,250])
yyaxis right
ylabel('Yield Stress with Surface Tension Correction [Pa]')
ylim([0,250])

% Add legend for the first/main plot handle
% hLegend = legend('location','northeast');
% hLegend.NumColumns = 2;
title("Comparison of Surface Tension Importance")

%% Save out figures for each test

saveFig = figure(6);

mkdir(sfrDataFolder + "Figures\");
for i = 1:length(sfrStructs)
    sfrDateStr = extractBefore(sfrFiles(i),"_");
    mkdir(sfrDataFolder + "Figures\" + sfrDateStr + "\");

    clf
    yyaxis left
    plot(sfrStructs(i).t,sfrStructs(i).F)
    ylabel('Force (N)')

    yl = ylim;
    ylim([0, max(yl)]);

    hold on
    yyaxis right
    plot(sfrStructs(i).t,1000*sfrStructs(i).h)
    ylabel('Gap (mm)')

    yl = ylim;
    ylim([0, max(yl)]);

    hold off
    xlabel('Time (s)')
    xlim([min(sfrStructs(i).t), max(sfrStructs(i).t)])
    
    figTitle = replace(replace("Test" + extractAfter(sfrFiles(i),"Test"),"_"," "),"-data.csv","");
    figTitle = "SFR: " + sfrDateStr + " " + figTitle(1);
    title(figTitle)
    
    % figFileName = extractBefore(sfrDataFolder + "Figures\" + sfrDateStr + "\" + sfrFiles(i),".") + ".png";
    figFileName = extractBefore(sfrDataFolder + "Figures\" + "\" + sfrFiles(i),".") + ".png";
    saveas(saveFig,figFileName)
end