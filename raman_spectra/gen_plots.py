import numpy as np
import matplotlib.pyplot as plt
import csv
import time
import joystick as jk

files = ["1000056_2.csv",
         "1000111_2.csv",
         "1000025_2.csv",
         "1000078_2.csv",
         "1000061_2.csv",
         "1000042_2.csv",
         "1000103_2.csv",
         "1000044_2.csv"
         ]
x_files = []
y_files = []
for i in range(len(files)):
    with open(files[i],newline='') as csv_file:
        reader = csv.reader(csv_file, delimiter=' ')
        next(csv_file)  # discard header
        data = list(reader)

    x1 = [float(row[0]) for row in data]
    y1 = [float(row[1]) for row in data]
    
    x_files.append(x1)
    y_files.append(y1)
    

# Generate a Raman spectrum
x = x_files[0]
y = y_files[0]
# y = 100 * np.exp(-(x - 1000)**2 / 2 / 50**2) + 50 * np.exp(-(x - 1400)**2 / 2 / 50**2)

# Add shot noise to the spectrum
noise_sensor = np.random.normal(0, 50, len(y))
noise_map = np.random.normal(0, 20, len(y))

y_sensor = []
# y_sensor.append(y + noise_sensor)
y_map = []
# y_map.append(y + noise_map)

# # Plot the spectra
# plt.plot(x, y_sensor, label='Raw reading', color='blue')
# plt.plot(x, y_map, label='Map spectrum', color='orange')
plt.ion()
# fig = plt.figure()
plt.xlabel('Raman shift (cm$^{-1}$)')
plt.ylabel('Intensity (a.u.)')
plt.legend()
fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(x, y, 'b-')
# plt.show()

for i in range(len(files)):
    y = y_files[i]
    noise_sensor = np.random.normal(0, 50, len(y))
    noise_map = np.random.normal(0, 20, len(y))
    y_sensor.append(y + noise_sensor)
    y_map.append(y + noise_map)
    # plt.plot(x, y_sensor, label='Raw reading', color='blue')
    # plt.plot(x, y_map, label='Map spectrum', color='orange')
    # plt.show()

# i = 0
for i in range(len(files)):
    line1.set_ydata(y_sensor[0])
    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(2.0)
    # ax.plot(x, y_sensor[0], label='Raw reading', color='blue')
    # plt.plot(x, y_map[0], label='Map spectrum', color='orange')
    # plt.show()
exit
# class test(jk.Joystick):
#     # initialize the infinite loop decorator
#     _infinite_loop = jk.deco_infinite_loop()

#     def _init(self, *args, **kwargs):
#         """
#         Function called at initialization, see the doc
#         """
#         self._t0 = time.time()  # initialize time
#         self.xdata = np.array(x)  # time x-axis
#         self.ydata = np.array([0.0])  # fake data y-axis
#         # create a graph frame
#         self.mygraph = self.add_frame(jk.Graph(name="test", size=(500, 500), pos=(50, 50), fmt="go-", xnpts=10000, xnptsmax=10000, xylim=(None, None, 0, 1)))

#     @_infinite_loop(wait_time=2)
#     def _generate_data(self):  # function looped every 0.2 second to read or produce data
#         """
#         Loop starting with the simulation start, getting data and
#     pushing it to the graph every 0.2 seconds
#         """
#         # concatenate data on the time x-axis
#         self.xdata = jk.core.add_datapoint(self.xdata, time.time(), xnptsmax=self.mygraph.xnptsmax)
#         # concatenate data on the fake data y-axis
#         self.ydata = jk.core.add_datapoint(y_map[i], y_sensor[i], xnptsmax=self.mygraph.xnptsmax)
#         self.mygraph.set_xydata(t, self.ydata)
#         i = i + 1
#         if i > len(files):
#             i = 0

# t = test()
# t.start()
# t.stop()