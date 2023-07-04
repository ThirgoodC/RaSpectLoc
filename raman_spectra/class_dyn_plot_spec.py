import matplotlib.pyplot as plt
import numpy as np
import csv
import time
from random import randrange


class SpectraPlotter:
    def __init__(self, files):
        self.files = files
        self.x_files = []
        self.y_files = []
        self.y_sensor = []
        self.y_map = []
        self.colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink', 'gray']
        self.fig, self.ax = plt.subplots()
        self.line1, = self.ax.plot([], [], label='Raw reading', color='blue')
        self.line2, = self.ax.plot([], [], label='Map reading', color='orange')
        
    def read_data(self):
            for i in range(len(self.files)):
                with open(self.files[i], newline='') as csv_file:
                    reader = csv.reader(csv_file, delimiter=' ')
                    next(csv_file)  # discard header
                    data = list(reader)

                x1 = [float(row[0]) for row in data]
                y1 = [float(row[1]) for row in data]

                self.x_files.append(x1)
                self.y_files.append(y1)

                noise_sensor = np.random.normal(0, 50, len(y1))
                noise_map = np.random.normal(0, 20, len(y1))
                self.y_sensor.append(y1 + noise_sensor)
                self.y_map.append(y1 + noise_map)

            min_len = min([len(x) for x in self.x_files])
            self.x_files = [x[:min_len] for x in self.x_files]
            self.y_files = [y[:min_len] for y in self.y_files]
            # (self.line1,) = self.ax.plot(self.x_files[0], self.y_sensor[0], label='Raw reading', color='blue')
            # (self.line2,) = self.ax.plot(self.x_files[0], self.y_map[0][:len(self.x_files[0])], label='Map reading', color='orange')
    
    def update_plot(self, idx, mapidx):

        noise_sensor = np.random.normal(0, 50, len(self.y_files[idx]))
        noise_map = np.random.normal(0, 40, len(self.y_files[mapidx]))
        curSen = self.y_files[idx] + noise_sensor
        curMap = self.y_files[mapidx] + noise_map
        self.ax.clear()
        # idx = 0
        print(idx)            
        self.ax.set_xlabel('Raman shift (cm$^{-1}$)')
        self.ax.set_ylabel('Intensity (a.u.)')
        # set the legend
        self.ax.legend()
        # while True:
        
        # self.line1.set_data(self.x_files[i], self.y_sensor[i])
        # self.line2.set_data(self.x_files[i], self.y_map[i][:len(self.x_files[i])])
        # self.line1.set_color(self.colors[i])
        # self.line2.set_color(self.colors[(i+1)%len(self.colors)])
        (self.line1,) = self.ax.plot(self.x_files[idx], curSen[:len(self.x_files[idx])], label='Raw reading', color='blue')
        (self.line2,) = self.ax.plot(self.x_files[idx], curMap[:len(self.x_files[idx])], label='Map reading', color='orange')


        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.5)
        # i = i + 1
        # if i >= len(files):
        #     i = 0
        return

    def run_update(self, idx, mapidx):
        # i = 0
        self.read_data()
        # for i in range(len(self.files)):
        self.update_plot(idx, mapidx)
        # i = i + 1
        # if i >= len(files):
        #     i = 0



if __name__ == '__main__':
    j = 0
    files = ["1000056_2.csv",
             "1000111_2.csv",
             "1000025_2.csv",
             "1000078_2.csv",
             "1000061_2.csv",
             "1000042_2.csv",
             "1000103_2.csv",
             "1000044_2.csv"
             ]
    plotter = SpectraPlotter(files)
    # for idx in range(len(files)):
    #     plotter.run_update(idx)
    
    while True:
        f = open("label_now.txt", "r")
        # print(f.read()) 
        curLabel = int(f.readline())
        f.close()
        print(curLabel)
        
        i = 0
        if curLabel == 1:
            i = 0
        elif curLabel == 8:
            i = 1
        elif curLabel == 9:
            i = 2
        elif curLabel == 10:
            i = 3
        elif curLabel == 11:
            i = 4
        elif curLabel == 12:
            i = 5
        else:
            i = 0
            
        if j < 20:
            map = randrange(5)
            j = j + 1
        else:
            map = i
            j = 30

        plotter.run_update(i, map)
        # time.sleep(1.0)

