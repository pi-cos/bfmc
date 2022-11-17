%% colormap

gen_number = size(score_history,3);
zero2one = (0:gen_number)/gen_number;
one2zero = -(-gen_number:0)/gen_number;
halfXsize = round(gen_number/2);
zero2one2zero = [(0:halfXsize)/halfXsize,-(-(gen_number-halfXsize):0)/(gen_number-halfXsize)]';
custom_colormap = [ one2zero',zero2one2zero(1:end-1),zero2one'];

%%

figure
grid on
hold on
xlabel('OBJ1')
ylabel('OBJ2')
for ii = 1:20%gen_number
    plot(score_history(score_history(:,1,ii)<1e10,1,ii),score_history(score_history(:,2,ii)<1e10,2,ii),'*','color',custom_colormap(ii,:))
end

%%
score_history(score_history(:,1,95)<1e10,1,ii)