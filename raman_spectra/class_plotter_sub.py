import matplotlib.pyplot as plt
import numpy as np
import csv
import time
import rospy

# def callback(data):
#     i = data.labels[0]

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
            (self.line1,) = self.ax.plot(self.x_files[0], self.y_sensor[0], label='Raw reading', color='blue')
            (self.line2,) = self.ax.plot(self.x_files[0], self.y_map[0][:len(self.x_files[0])], label='Map reading', color='orange')
    
    def update_plot(self, idx):
        # idx = 0
        print(idx)            
        self.ax.set_xlabel('Raman shift (cm$^{-1}$)')
        self.ax.set_ylabel('Intensity (a.u.)')
        # while True:
        self.ax.clear()
        # self.line1.set_data(self.x_files[i], self.y_sensor[i])
        # self.line2.set_data(self.x_files[i], self.y_map[i][:len(self.x_files[i])])
        # self.line1.set_color(self.colors[i])
        # self.line2.set_color(self.colors[(i+1)%len(self.colors)])
        (self.line1,) = self.ax.plot(self.x_files[idx], self.y_sensor[idx], label='Raw reading', color=self.colors[idx])
        (self.line2,) = self.ax.plot(self.x_files[idx], self.y_map[idx][:len(self.x_files[idx])], label='Map reading', color=self.colors[(idx+1)%len(self.colors)])


        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.5)
        # i = i + 1
        # if i >= len(files):
        #     i = 0
        return

    def callback(self, data):
        self.label = data.labels[0]

    def run_update(self, idx):
        # i = 0
        self.read_data()
        # for i in range(len(self.files)):
        self.update_plot(idx)
        # i = i + 1
        # if i >= len(files):
        #     i = 0



if __name__ == '__main__':
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
    rospy.init_node("plot_node", anonymous=False)
    rospy.Subscriber("/raman/semantic/scan", SemanticScan, plotter.callback)
    # for idx in range(len(files)):
    #     plotter.run_update(idx)
    
    while True:
        rospy.spin()
    # plotter.update_plot(0)
    # for idx in range(len(files)):
    #     plotter.update_plot(idx)
