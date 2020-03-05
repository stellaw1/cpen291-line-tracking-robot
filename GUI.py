import RPi.GPIO as GPIO
import time
import ctypes as ct

import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter
from tkinter import *
from tkinter import messagebox

def plotSetup():
    GPIO.setmode(GPIO.BCM) # Set the GPIO pins as BCM
    # GPIO.setup(IRTrackingPinLL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(IRTrackingPinL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(IRTrackingPinR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(IRTrackingPinRR, GPIO.IN, pull_up_down=GPIO.PUD_UP)

x = 0
y = 0
xdir = 0
ydir = 1

arr = DynamicArray()

def plot(turnType):

def destroy():
    GPIO.cleanup()  # Release resource




if __name__ == '__main__': # The Program will start from here
    plotSetup()
try:
    plot()
except KeyboardInterrupt: # When control c is pressed child program destroy() will be executed.
    destroy()


class GUIGraph:
    def __init__(self, frame):
        self.fig = Figure(figsize=(5,4))
        self.figCanvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.p = self.fig.add_subplot(1,1,1)

# method for setting the labels of the current plot
    def set_labels(self, title=None, xlabel=None, ylabel=None):
        self.p.set_title(title)
        self.p.set_xlabel(xlabel)
        self.p.set_ylabel(ylabel)

# method for showing the plot at a specific location on the grid
    def show_plot(self, xloc=0, yloc=0, padx=0, pady=0):
        self.figCanvas.get_tk_widget().grid(row=xloc,column=yloc,padx=padx,pady=pady)

# method for making the plot disappear off of the current window
    def hide_plot(self):
        self.figCanvas.get_tk_widget().grid_remove()

# method for clearing the plot and displaying the specified data
    def graph(self, xdata, ydata, title, xtitle, ytitle):
        self.p.clear()
        self.set_labels(title, xtitle, ytitle)
        self.p.plot(xdata, ydata, color='blk')
        self.figCanvas.draw()
        plt.xticks(rotation=45, ha='right')

class TrackPlot:
    def __init__(self):
        self.root = Tk()
        self.measure_rate = "1"
        self.x = 0
        self.y = 0
        self.yDir = 1
        self.xDir = 0
        self.xVals = []
        self.yVals = []
        self.running = False
        self.paused = True
        self.message = StringVar()
        self.message.set("Welcome to PathPlotter!")

        # method to begin measuring sensor values and displaying them on the plot and GUI
        def begin_measure(self):
            self.running = True

            xVal = round(read_temp(), 2)
            yVal = round(read_light(), 2)

            self.xVals.append(xVal)
            self.yVals.append(yVal)

            self.path_plot.graph(self.xVals, self.yVals,"Path Travelled so far", "x", "y")

            self.loop = self.root.after(int(float(self.measure_rate) * 1000), lambda: self.begin_measure())

        # creating plot for x vs y
        self.path_plot = GUIGraph(self.root)
        self.path_plot.show_plot(xloc=50, padx=10)
    # method for terminating the program
    def stop_program(self):
        try:
            if self.running:
                self.root.after_cancel(self.loop)
            self.root.destroy()
        except:
            print("Program terminated")

    # method for running the current program
    def run_program(self):
        self.message.set("Program running...")
        self.paused = False
        if not self.running:
            self.begin_measure()


