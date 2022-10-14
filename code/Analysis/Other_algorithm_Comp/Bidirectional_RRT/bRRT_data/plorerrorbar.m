function plorerrorbar(model_series,model_error,lowerb,upperb)

X = categorical({'map1','map2','map3','map4','map5','map6','map7','map8'});
X = reordercats(X,{'map1','map2','map3','map4','map5','map6','map7','map8'});

b = bar(model_series, 'grouped');
set(gca,'YLim', [lowerb,upperb], 'XTickLabel',{'map1','map2','map3','map4','map5','map6','map7','map8'});
%%For MATLAB R2019a or earlier releases
hold on
% Find the number of groups and the number of bars in each group
ngroups = size(model_series, 1);
nbars = size(model_series, 2);
%legend(legend1,legend2);
legend('boxoff');
ylabel('Trajectory cost');
% Calculate the width for each bar group
groupwidth = min(0.8, nbars/(nbars + 1.5));
% Set the position of each error bar in the centre of the main bar
% Based on barweb.m by Bolu Ajiboye from MATLAB File Exchange
for i = 1:nbars
    % Calculate center of each bar
    x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
    errorbar(x, model_series(:,i), model_error(:,i), 'k', 'linestyle', 'none');
end
hold off
%%For MATLAB 2019b or later releases
hold on
% Calculate the number of bars in each group
nbars = size(model_series, 2);
% Get the x coordinate of the bars
x = [];
for i = 1:nbars
    x = [x ; b(i).XEndPoints];
end
% Plot the errorbars
errorbar(x',model_series,model_error,'k','linestyle','none')'
hold off


end