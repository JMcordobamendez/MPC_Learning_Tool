#This code has been developed to learn more about Model Predictive Control
#the goal is to simulate and visualize how this control algorithm works
#
#Author: José Miguel Córdoba Méndez
#Date: April 2024
import matplotlib.pyplot as plt

class PredictiveControl:
    
    '''
    Model
    '''
    def __init__(self, t = 10, dt = 0.01, cdt = 0.01, ref = [1, 2, 3, 2], horizon = 20, Lrate = 5e-3, decay = 0, iter = 300, Ku = 1, tau = 1, sat = [-10, 10]):
        self.t_end = t #Simulation time
        self.record_time = []
        self.dt = dt #simulation period
        self.cdt = cdt #Control period
        self.time_ratio = int(cdt/dt) #time ratio for the control prediction reference
        N = round(t/dt) #number of periods to analyze
        self.ref = [] #list of the different references to follow
        c, pos = 0, 0
        #Simulation reference builder
        for i in range(N):
            if (c >= round(N/len(ref))):
                c = 0
                pos += 1
            c += 1
            self.ref.append(ref[pos])
        #Model parameters
        self.tau = tau
        self.Ku = Ku
        self.Y = 0
        self.y = []
        self.u = []
        #MPC parameters
        self.h = horizon
        self.Yp = [] #declaration
        self.Up = []
        self.histU = []
        self.histf = [0, 0]
        for i in range(horizon): #definition
            self.Yp.append(0)
            self.Up.append(0)
            self.histU.append([0, 0])
        self.Lrate = Lrate #learning rate descent gradient
        self.decay = (1-decay/100)**(1/10)
        self.iter = iter
        #Actuator Parameters
        self.sat = sat
        
    def f_dev(self, Y, U):
        return (1/self.tau)*(self.Ku*U - Y) # = dy/dt
    
    def simulation_step(self, U):
        #runge-kutta ode4 as solver
        k1 = self.f_dev(self.Y, U)
        k2 = self.f_dev(self.Y + 0.5*k1, U)
        k3 = self.f_dev(self.Y + 0.5*k2, U)
        k4 = self.f_dev(self.Y + k3, U)
        self.Y += (self.dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        
    def prediction(self, Up):
        Yans = self.Y #initial point is the actual state
        for i in range(self.h):
            #Dy = (1/self.tau)*(Up[i] - Yans)
            k1 = self.f_dev(Yans, Up[i])
            k2 = self.f_dev(Yans + 0.5*k1, Up[i])
            k3 = self.f_dev(Yans + 0.5*k2, Up[i])
            k4 = self.f_dev(Yans + k3, Up[i])
            #self.Yp[i] = Yans + self.dt*Dy
            self.Yp[i] = Yans +(self.cdt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            Yans = self.Yp[i]
        
        
    def sum_err(self, Yp, Ref):
        sum_err = 0
        for i in range(len(Ref)):
            sum_err += (Yp[i] - Ref[i])**2
        
        return sum_err
    
    def descent_gradient(self, Lrate):
        for i in range(self.h):
            #U_{i}(k)=U_{i}(k-1)-\varepsilon\cdot\frac{\Delta F}{\Delta U_{i}}
            self.Up[i] += - Lrate*(self.histf[0] - self.histf[1])/(self.histU[i][0] - self.histU[i][1] + 1e-25)
            self.Up[i] = self.windup_check(self.Up[i]) #Avoids the saturation of the actuator
            #Actualize last value to compute next step gradient
            self.histU[i][1] = self.histU[i][0]
            self.histU[i][0] = self.Up[i]
     
    def simulation(self):
        t = 0 #simulation time
        ct = 0 #controller time
        N = 0
        while(t < self.t_end):
            self.y.append(self.Y) #History of outputs
            self.record_time.append(t)
            if (ct <= t): #discrete controller action
                U = self.mpc(self.y[N], self.ref, N) #Calculate control action
                ct += self.cdt
            self.u.append(U) #History of inputs
            self.simulation_step(U)
            t += self.dt
            N += 1
        #self.plot()
    
    def mpc(self, y, ref, N):
        #future reference generation
        ref_h = []
        for i in range(self.h):
            if((N + i*self.time_ratio) < len(ref)): #check if the prediction is over the last reference
                ref_h.append(ref[N + i*self.time_ratio])
            else: #if it is the case take the last reference value as predicted reference
                ref_h.append(ref[-1])
        
        #-Here begins the iterative optimization algorithm :)
        Lrate = self.Lrate
        for i in range(self.iter):
            self.prediction(self.Up) #Output prediction with given input
            self.histf[0] = self.sum_err(self.Yp, ref_h) #error summatory, this is the function to minimize
            self.descent_gradient(Lrate) #calculate the input which minimize the error summation
            self.histf[1] = self.histf[0] #Store the error of the last prediction
            Lrate = Lrate*self.decay #Step decay learning rate
        
        return self.Up[0]
        
    def windup_check(self, Up):
        if (Up >= self.sat[1]):
            return self.sat[1]
        elif (Up <= self.sat[0]):
            return self.sat[0]
        else:
            return Up
    
    def plot(self):
        fig, (ax1, ax2) = plt.subplots(2, 1)
        ax1.plot(self.record_time[0:-1], self.y[0:-1], "r")
        try:
            ax1.plot(self.record_time[0:-1], self.ref, "k")
        except:
            ax1.plot(self.record_time, self.ref, "k")
        #ax1.set_xlabel("Time (Seconds)")
        ax1.set_ylabel("y(t)")
        ax1.set_title("Model Predictive Control")
        ax1.grid(True)
        ax1.legend(['y(t)', 'Ref(t)'])
        ax2.plot(self.record_time[0:-1], self.u[0:-1], "b")
        ax2.set_xlabel("Time (Seconds)")
        ax2.set_ylabel("u(t)")
        ax2.grid(True)
        ax2.legend(['u(t)'])
        plt.show()
        
    def plot_y(self):
        fig, ax1 = plt.subplots()
        ax1.plot(self.record_time[0:-1], self.y[0:-1], "r")
        try:
            ax1.plot(self.record_time[0:-1], self.ref, "k")
        except:
            ax1.plot(self.record_time, self.ref, "k")
        ax1.set_ylabel("y(t)")
        ax1.set_title("Model Predictive Control")
        ax1.grid(True)
        ax1.legend(['y(t)', 'Ref(t)'])
    
    def plot_u(self):
        fig, ax2 = plt.subplots()
        ax2.plot(self.record_time[0:-1], self.u[0:-1], "b")
        ax2.set_xlabel("Time (Seconds)")
        ax2.set_ylabel("u(t)")
        ax2.grid(True)
        ax2.legend(['u(t)'])
    
#predictive = PredictiveControl(dt=0.01, Lrate = 5e-3, horizon = 10)