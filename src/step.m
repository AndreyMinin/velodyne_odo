clear

dataV = load("-ascii", "/home/user/.ros/step_v.log");
dataW = load("-ascii", "/home/user/.ros/step_w.log");

sumV = [0,0,0];
for i=1:length(dataV)
	sumV(i+1,:) = sumV(i,:) + dataV(i,:);
	endfor

sumW = [0,0,0];
for i=1:length(dataW)
	sumW(i+1,:) = sumW(i,:) + dataW(i,:);
	endfor
lim = 0.1
figure (1)
plot3(sumV(:,1), sumV(:,2), sumV(:,3), "o-")
axis([-lim, lim, -lim, lim, -lim, lim])

figure(2)
plot3(sumW(:,1), sumW(:,2), sumW(:,3), "o-")
axis([-lim, lim, -lim, lim, -lim, lim])
