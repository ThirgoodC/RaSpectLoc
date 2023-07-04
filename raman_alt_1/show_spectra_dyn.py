import matplotlib.pyplot as plt
import numpy as np
import csv
import time

i = -1
import rospy
# from std_msgs.msg import String

def callback(data):
    i = data.labels[0]
    
# def listener():

#     rospy.Subscriber("chatter", SemanticScan, callback)

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

# if __name__ == '__main__':
#     listener()
if __name__ == "__main__":
    # rospy.init_node("plot_node", anonymous=False)
    # rospy.Subscriber("/raman/semantic/scan", SemanticScan, callback)
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
        with open(files[i], newline='') as csv_file:
            reader = csv.reader(csv_file, delimiter=' ')
            next(csv_file)  # discard header
            data = list(reader)

        x1 = [float(row[0]) for row in data]
        y1 = [float(row[1]) for row in data]

        x_files.append(x1)
        y_files.append(y1)

    y_sensor = []
    y_map = []
    for i in range(len(files)):
        y = y_files[i]
        noise_sensor = np.random.normal(0, 50, len(y))
        noise_map = np.random.normal(0, 20, len(y))
        y_sensor.append(y + noise_sensor)
        y_map.append(y + noise_map)

    # create the figure and axis objects
    fig, ax = plt.subplots()

    # plot the first spectrum
    (line1,) = ax.plot(x_files[0], y_sensor[0], label='Raw reading', color='blue')

    # plot the second spectrum
    # (line2,) = ax.plot(x_files[0], y_map[0], label='Map reading', color='orange')
    (line2,) = ax.plot(x_files[0], y_map[0][:len(x_files[0])], label='Map reading', color='orange')

    # set the legend and labels
    ax.legend()
    ax.set_xlabel('Raman shift (cm$^{-1}$)')
    ax.set_ylabel('Intensity (a.u.)')

    i = 0
    colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink', 'gray']
    fig.canvas.draw()
    # update the plot every 2 seconds with new data
    while True:
        # clear the axis before each update
        ax.clear()
        # new data for two spectra
        (line1,) = ax.plot(x_files[i], y_sensor[i], label='Raw reading', color=colors[i])
        (line2,) = ax.plot(x_files[i], y_map[i][:len(x_files[i])], label='Map reading', color=colors[(i+1)%len(colors)])

        # set the legend
        ax.legend()
        
        # redraw the plot
        fig.canvas.draw()
        fig.canvas.flush_events()

        # pause for 2 seconds
        plt.pause(2.0)

        # remove old lines
        line1.remove()
        line2.remove()

        # reset index iterator
        i = i + 1
        if i >= len(files):
            i = 0
            
        # rospy.spin()
