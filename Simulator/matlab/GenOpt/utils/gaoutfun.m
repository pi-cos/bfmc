function [state,options,optchanged] = gaoutfun(options,state,flag)
persistent h1 pop_history r
persistent score_history rank_history feas_history
optchanged = false;
switch flag
    case 'init'
        h1 = figure;
        ax = gca;
%         ax.XLim = [0 21];
%         ax.YLim = [0 21];
        l1 = min(state.Population(:,1));
        m1 = max(state.Population(:,1));
        l2 = min(state.Population(:,2));
        m2 = max(state.Population(:,2));
        r = rectangle(ax,'Position',[l1 l2 m1-l1 m2-l2]);
        pop_history(:,:,1) = state.Population;
        assignin('base','gapopulationhistory',pop_history);
        score_history(:,:,1) = state.Score;
        assignin('base','gascorehistory',score_history);
        rank_history(:,1) = state.Rank;
        assignin('base','garankhistory',rank_history);
        feas_history(:,1) = state.isFeas;
        assignin('base','gafeashistory',feas_history);
    case 'iter'
        % Update the history every 1 generation.
        if rem(state.Generation,1) == 0
            ss = size(pop_history,3);
            pop_history(:,:,ss+1) = state.Population;
            assignin('base','gapopulationhistory',pop_history);
            score_history(:,:,ss+1) = state.Score;
            assignin('base','gascorehistory',score_history);
            rank_history(:,ss+1) = state.Rank;
            assignin('base','garankhistory',rank_history);
            feas_history(:,ss+1) = state.isFeas;
            assignin('base','gafeashistory',feas_history);
        end
        % Find the best objective function, and stop if it is low.
%         ibest = state.Best(end);
%         ibest = find(state.Score == ibest,1,'last');
%         bestx = state.Population(ibest,:);
%         bestf = gaintobj(bestx);
%         if bestf <= 1e-5
%             state.StopFlag = 'y';
%             disp('Got below 0.1')
%         end
        % Update the plot.
        figure(h1)
        l1 = min(state.Population(:,1));
        m1 = max(state.Population(:,1));
        l2 = min(state.Population(:,2));
        m2 = max(state.Population(:,2));
        r.Position = [l1 l2 m1-l1 m2-l2];
        pause(0.1)
        save partial_ga_results feas_history pop_history rank_history score_history
        % Update the fraction of mutation and crossover after 25 generations.
%         if state.Generation == 25
%             options.CrossoverFraction = 0.8;
%             optchanged = true;
%         end
    case 'done'
        % Include the final population in the history.
        ss = size(pop_history,3);
        pop_history(:,:,ss+1) = state.Population;
        assignin('base','gapopulationhistory',pop_history);
        score_history(:,:,ss+1) = state.Score;
        assignin('base','gascorehistory',score_history);
        rank_history(:,ss+1) = state.Rank;
        assignin('base','garankhistory',rank_history);
        feas_history(:,ss+1) = state.isFeas;
        assignin('base','gafeashistory',feas_history);
end

