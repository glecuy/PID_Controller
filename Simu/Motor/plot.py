import matplotlib.pyplot as plt

Time = []
Signal_input = []
T1_output = []
EFM_output = []
RPM_output = []
Power_output =[]

f = open('samples.txt','r')
for row in f:
    row = row.split(' ')
    Time.append(float(row[0]))
    Signal_input.append(float(row[1]))
    RPM_output.append(float(row[2]))
    EFM_output.append(float(row[3]))
    Power_output.append(float(row[4]))


# Initialise the subplot function using number of rows and columns
figure, axis = plt.subplots(3, sharex=True)

figure.set_figheight(12)

axis[0].plot(Time, Signal_input)
axis[0].set_title('Input', fontsize = 10)

axis[1].plot(Time, RPM_output)
axis[1].set_title('RPM', fontsize = 10)

axis[2].plot(Time, Power_output)
axis[2].set_title('POWER', fontsize = 10)


plt.show()
