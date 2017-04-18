clear

dataV = load("-ascii", "/home/user/.ros/v.log");
dataW = load("-ascii", "/home/user/.ros/w.log");
datat = load("-ascii", "/home/user/.ros/t.log");

for i=1:length(dataV)
	modV(i) = norm (dataV(i,:));
	endfor

for i=1:length(dataW)
	modW(i) = norm (dataW(i,:));
	endfor

lim = 0.1
figure (1)
plot(dataV(:,1), 'ro-', dataV(:,2), 'go-', dataV(:,3), "bo-", modV, "co-")
#axis([-lim, lim, -lim, lim, -lim, lim])

figure(2)
plot(dataW(:,1), 'ro-', dataW(:,2), 'go-', dataW(:,3), "bo-", modW, "co-")
#axis([-lim, lim, -lim, lim, -lim, lim])

figure(3)
plot(datat);
