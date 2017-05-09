fid=fopen("dumpvars_tab.txt");
line = 0;

data=zeros(2,3);

line_i=1;
while (-1 ~= (line=fgetl(fid)))
 data(line_i,:) = str2num(line);
end

time = (1:214);

figure();
plot(time, data(:,1));

figure();
plot(time, data(:,2));

figure();
plot(time, data(:,3));

fclose(fid);
