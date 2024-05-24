#This code is the Model Predictive Control App builder
#the source code joins the GUI with the controller simulatior API
#
#Author: José Miguel Córdoba Méndez
#Date: May 2024 
from PyQt6.QtGui import *
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *

from pyqtgraph import PlotWidget, mkPen
from threading import Thread

from ui_MPC import Ui_Predictive_Control_API
from MPC import PredictiveControl


class GUI(QMainWindow, Ui_Predictive_Control_API):
    #App initialization
    def __init__(self):
        super().__init__()
        self.ui = Ui_Predictive_Control_API()
        self.ui.setupUi(self) #GUI initialized
        
        #Initialization of simulator parameters
        self.t = 30
        self.Ku = 1
        self.tau = 1
        self.sat = [-10, 10]
        self.ref = [1, 2, 4, 1]
        self.cdt = 0.1
        self.horizon = 30
        self.Lrate = 1e-2
        self.decay = 0
        self.iter = 100
        #Initialization of concurrent "Lock" to force one thread doing simulations at a time
        self.lock = False
        self.predictive = None
        self.sim_begin = False

        #Widget initialization
        #--Use PlotWidget from pyqtgraph
        self.plot_widget_y = PlotWidget()
        self.plot_widget_u = PlotWidget()
        self.graph_y = None
        self.graph_ref = None
        self.graph_u = None
        self.init_plot()
        
        #--Assign the PlotWidget to a GUI Widget
        self.ui.Planta.addWidget(self.plot_widget_y) #Plant behaviour
        self.ui.Entradas.addWidget(self.plot_widget_u) #Control action
        #--Join line edit with simulation variable
        self.ui.LineEdit_simtime.textChanged.connect(self.sim_time)
        self.ui.LineEdit_Gain.textChanged.connect(self.sim_ku)
        self.ui.LineEdit_Tau.textChanged.connect(self.sim_tau)
        self.ui.LineEdit_Saturation.textChanged.connect(self.sim_sat)
        self.ui.LineEdit_Reference.textChanged.connect(self.sim_ref)
        self.ui.LineEdit_Horizon_Size.textChanged.connect(self.sim_horizon)
        self.ui.LineEdit_Learning_Rate.textChanged.connect(self.sim_Lrate)
        self.ui.LineEdit_Step_decay.textChanged.connect(self.sim_decay)
        self.ui.LineEdit_Iterations.textChanged.connect(self.sim_iter)
        #--Horizontal slider
        self.ui.horizontalSlider.valueChanged.connect(self.sim_cdt)
        #--Button
        self.ui.Run_btn.clicked.connect(self.init_simulation)
        
        #refresh the graphics
        self.timer = QTimer()
        self.timer.setInterval(300)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()
        
        
        
    #fcn to store changes in simulation time
    def sim_time(self, data):
        try:
            self.t = float(data)
        except:
            self.t = 30
    
    #fcn to sotore Ku
    def sim_ku(self, data):
        try:
            self.Ku = float(data)
        except:
            self.Ku = 1
    
    #fcn to store tau
    def sim_tau(self, data):
        try:
            self.tau = float(data)
        except:
            self.tau = 1
    
    #fcn to store saturation
    def sim_sat(self, data):
        try:
            data = data.split(",")
            self.sat = [float(data[0]), float(data[1])]
        except:
            self.sat = [-10, 10]
    
    #fcn to store ref
    def sim_ref(self, data):
        try:
            self.ref = []
            data = data.split(",")
            for i in data:
                self.ref.append(float(i))
        except:
            self.ref = [1, 2, 3, 2]
    
    #fcn to store cdt
    def sim_cdt(self, data):
        data = 0.01*data
        self.ui.label.setText(str(data)) #Modify the label value
        try:
            self.cdt = float(data)
        except:
            self.cdt = 0.1
    
    #fcn to store moving horizon number
    def sim_horizon(self, data):
        try:
            self.horizon = int(data)
        except:
            self.horizon = 30
    
    #fcn to store learning rate of gradient descent
    def sim_Lrate(self, data):
        try:
            self.Lrate = float(data)
        except:
            self.Lrate = 1
    
    #fcn to store step decay learning rate
    def sim_decay(self, data):
        try:
            self.decay = float(data)
        except:
            self.decay = 8
    
    #fcn to store iterations
    def sim_iter(self, data):
        try:
            self.iter = int(data)
        except:
            self.iter = 400
    
    #fcn to begin the simulation
    def init_simulation(self):
        if (self.lock == False): #Check if a thread is still executing a simulation
            self.lock = True
            self.sim_begin = True #Once the first simulation has begun, data can be plotted
            thread = Thread(target = self.concurrent_sim)
            thread.start()
            
            
    
    #fcn to do the simulation concurrently with the GUI
    def concurrent_sim(self):
        self.predictive = PredictiveControl(t = self.t, cdt = self.cdt, Lrate = self.Lrate, horizon = self.horizon, iter = self.iter, decay = self.decay, tau = self.tau, Ku = self.Ku, sat = self.sat, ref = self.ref)
        self.predictive.simulation()
        #print("done")
        self.lock = False #Unlock the capability of creating a new thread
    
    def update_plot(self):
        if (self.sim_begin == True): #Without simulations there aren't data to plot
            if (self.lock == False): #Check if a thread is still executing a simulation
                self.plot_data(self.predictive) #plot the data in the GUI
    
    def init_plot(self):
        self.plot_widget_y.setBackground('w') #Set background color to white
        self.plot_widget_u.setBackground('w')
        #Title
        self.plot_widget_y.setTitle("Reference Tracking", color = "k", size = "15pt")
        self.plot_widget_u.setTitle("Controller Action", color = "k", size = "15pt")
        #Labels axis
        self.plot_widget_y.setLabel("left", "y(t)")
        self.plot_widget_u.setLabel("left", "u(t)")
        self.plot_widget_y.setLabel("bottom", "Time (Seconds)")
        self.plot_widget_u.setLabel("bottom", "Time (Seconds)")
        #Legend
        self.plot_widget_y.addLegend()
        self.plot_widget_u.addLegend()
        #Axis scale
        self.plot_widget_y.enableAutoRange(axis='xy')
        self.plot_widget_u.enableAutoRange(axis='xy')
        #Add grid
        self.plot_widget_y.showGrid(x = True, y = True, alpha = 0.3)
        self.plot_widget_u.showGrid(x = True, y = True, alpha = 0.3)
        #Add property lines
        pen_y = mkPen(color = 'r', width = 2)
        pen_ref = mkPen(color = 'k', width = 2)
        pen_u = mkPen(color = 'b', width = 2)
        #Plot
        #-y
        self.graph_y = self.plot_widget_y.plot([0,0], [0,0], pen=pen_y, width=30, name = "y(t)") # Use PlotWidget's plot method
        self.graph_ref = self.plot_widget_y.plot([0,0], [0,0], pen=pen_ref, width=30, name = "Ref(t)")
        #-u
        self.graph_u = self.plot_widget_u.plot([0,0], [0,0], pen=pen_u, width=30, name = "u(t)")
    
    #fcn to plot the data in the GUI
    def plot_data(self, predictive):

        #Plot
        #-y
        self.graph_y.setData(predictive.record_time[0:-1], predictive.y[0:-1])
        if(len(predictive.record_time[0:-1]) == len(predictive.ref)):
            self.graph_ref.setData(predictive.record_time[0:-1], predictive.ref)
        else:
            self.graph_ref.setData(predictive.record_time, predictive.ref)
        #-u
        self.graph_u.setData(predictive.record_time[0:-1], predictive.u[0:-1])

    
app = QApplication([])
window = GUI()
window.show()

app.exec()
#python -m PyQt6.uic.pyuic -x Example_1.ui -o ui_MPC.py
#python -m PyInstaller ./main.py --onefile --exclude-module PyQt5 -w
#python -m PyInstaller ./main.py --exclude-module PyQt5 -w