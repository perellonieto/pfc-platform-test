%m-file to adquire ERIKA-FLEX FULL data from the serial port using RS232 

clear all
close all

number_of_samples=500;                  %set the number of adquisitions
delete(instrfind)                       %clean all open ports

                                        %configuring serial port
s=serial('/dev/ttyUSB0');                       %creates a matlab object from the serial port 'COM1' 
s.BaudRate=115200;                      %baudrate=115200bps
s.Parity='none';                        %no parity
s.DataBits=8;                           %data sended in 8bits format
s.StopBits=1;                           %1 bit to stop
s.FlowControl='none';                   %no flowcontrol
s.Terminator='LF';                      %LineFeed character as terminator
s.Timeout=10;                           %maximum time in seconds since the data is readed
s.InputBufferSize=100000;               %increment this value when data is corrupted
fopen(s)                                %open serial port object

q = quantizer('float',[32 8]);          %cast 32bits hex to float

disp('Waiting for data... ')

%Read data until header character(0x01) is received 
data=zeros(23,1);
while(data~=1)
    data=fread(s,1,'char');
end
data=fread(s,23-1,'char');

disp('Beginning data adquisition... ')
figure;
for n = 1:number_of_samples  %get data and plot 
    data=fread(s,23,'char');

    t(n,1)= hex2dec([dec2hex(data(2),2) dec2hex(data(3),2) dec2hex(data(4),2) dec2hex(data(5),2)])*25e-9;%25e-9 is 1/Fcy=1/40e6
    r(n,1)=hex2num(q,[dec2hex(data(9),2) dec2hex(data(8),2) dec2hex(data(7),2) dec2hex(data(6),2)]);%joint four bytes, float value
    x1(n,1)=hex2num(q,[dec2hex(data(13),2) dec2hex(data(12),2) dec2hex(data(11),2) dec2hex(data(10),2)]);
    x2(n,1)=hex2num(q,[dec2hex(data(17),2) dec2hex(data(16),2) dec2hex(data(15),2) dec2hex(data(14),2)]);
    u(n,1)=hex2num(q,[dec2hex(data(21),2) dec2hex(data(20),2) dec2hex(data(19),2) dec2hex(data(18),2)]);
    %t(n,1)=bitshift(data(23),24) + bitshift(data(22),16) + bitshift(data(21),8) + data(20); %joint two bytes, unsignded int value
    
    plot(t,r,'g.',t,x1,'b');%,t,x2,'m');
    axis([max(t)-3 max(t) -1.75 1.75]);%DI Plant
    %axis([max(t)-3 max(t) 0 3]);%RCRC Plant
    grid on;

    drawnow; 
end

hold on
stairs(t,u,'r') %Plots u as stair
legend('r','x_1','u')  
xlabel('t(s)')
ylabel('voltage (V)')
title('Adquisition')



