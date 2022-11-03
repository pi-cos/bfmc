close all
v = 1./(abs(track.k)+1);
plot(track.s(1:end-1),v)
figure; plot(track.s(1:end-1),track.k); hold on; plot(track.s(1:end-1),abs(track.k)+1)