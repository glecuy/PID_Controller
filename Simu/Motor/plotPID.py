import matplotlib.pyplot as plt
import sys


Time = []
SetPoint_input = []
Measurement_output = []
PID_output = []


f = open('samples.txt','r')
for row in f:
    row = row.split(' ')
    Time.append(float(row[0]))
    SetPoint_input.append(float(row[1]))
    Measurement_output.append(float(row[2]))
    PID_output.append(float(row[3]))


# Initialise the subplot function using number of rows and columns
figure, axis = plt.subplots(3, sharex=True)

figure.set_figheight(12)

axis[0].plot(Time, SetPoint_input)
axis[0].set_title('SetPoint', fontsize = 10)

axis[1].plot(Time, Measurement_output)
axis[1].set_title('Measurement', fontsize = 10)

axis[2].plot(Time, PID_output)
axis[2].set_title('PID_output', fontsize = 10)

if ( len(sys.argv) > 1 ):
    figName = format(sys.argv[1])
    figure.suptitle( figName, fontsize=20)
    plt.savefig(figName+'.png')
else:
    figure.suptitle( 'Motor response', fontsize=20)

plt.show()
