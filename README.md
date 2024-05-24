# Model Predictive Control Interactive Tool
## Index

 - Introduction
 - Installation Guide
 - Tool Manual

## Introduction
This interactive tool has been developed to help students during the process of learning Model Predictive Control, the student can learn how to design the different control parameters and it effects over the plant without doing a single mathematical calculation.

The student can modify also the plant parameters (first order SISO system), simulation duration and controller references.

First order SISO system:
$$Y(s) = \frac{K}{\tau\cdot s+1}\cdot U(s)$$

## Installation Guide
The interactive tool is programmed in python so the user must have python installed. If it is not the case the easiest way to install it is:

**Windows:**

 1. Left click over the Windows icon.
 2. Write "cmd" over the Windows search and open system symbol, a black terminal will be opened.
 3. Write "**python3**" and **press Enter**, if python is not installed the Microsoft Store window will appear with the option to install it (do it). If python is installed the terminal will change to the python terminal having this ">>>" symbol before the things we write, type "**exit()**" and **press Enter** to exit.
 4. Once it is installed, download the following python modules to make the toolbox work. Type in the terminal and wait until it is automatically downloaded and installed:
 ```bash
pip install PyQt6
 ```
  ```bash
pip install pyqtgraph
 ```
  ```bash
pip install matplotlib
 ```
 5. Once all the python modules are installed download the files of the repository or clone it.
 6. Type in the terminal "cd C:\path_till_the_directory\MPC_Learning_Tool".
 
 **Linux:**
 
 1. Press **Ctrl+Alt+T** or open the Linux terminal.
 2. Write "**python3**" and **press Enter**, if python is installed the python terminal appears with ">>>" symbol before the things we write. To exit write "**exit()**" and **press Enter**. if python in not installed write:
   ```bash
sudo apt install python3
 ```
 
 3. Once it is installed, download the following python modules to make the toolbox work. Type in the terminal and wait until it is automatically downloaded and installed:
  ```bash
pip install PyQt6
 ```
  ```bash
pip install pyqtgraph
 ```
  ```bash
pip install matplotlib
 ```
  4. Once all the python modules are installed download the files of the repository or clone it.
  5. Type in the terminal "cd /path_till_the_directory/MPC_Learning_Tool".
 ## Tool Manual
Once the user is inside the "MPC_Learning_Tool" directory with the terminal, to execute the tool, type:
```bash
python3 main.py
```
 After some seconds, the tool window will be opened with some default values.
 
![menu_parts](https://github.com/JMcordobamendez/MPC_Learning_Tool/assets/79694677/4a5e5c4b-4030-4a35-967a-60a89fb9df25)

In the tool window, the "**Model Parameters**" (red square) are configured modifying the tex editor, the user must write a **real number** using "**.**" for decimal point. The saturation of the actuator has an inferior and superior limit that are defined separating them with "**,**".

Inside "**Controller Parameters**" there are 2 types of parameters, one related with the "**MPC controller**" (blue square) and other related with the "**Optimization Algorithm**" (orange square) used.

 - **MPC controller**: The first parameter is the reference given to the controller, the user can modify the default one replacing or increasing the number of values (there must be a "**,**" between them). The second parameter is the control time, as the controller is a discrete time controller the user can modify the period of it between [0.01, 1]. The last parameter is the prediction window of the MPC algorithm.
 
*As a thumb rule, the moving horizon size must be big enough to characterize the dynamics of the plant. If the time constant is "1" the main dynamic of the system is characterized in the first 3 seconds (95% of the steady state value is reached in 3 times the time constant). So, with a time constant of "1", a controller time of "0.25", a moving horizon of 12 is enough.* 

 - **Optimization Algorithm**: The optimization algorithm used has been "**Gradient Descent**". It is an iterative algorithm which numerically converge to a local optimum solution. The main task of the algorithm in MPC is to minimize the error (cost function) between the future references and the predicted output modifying the control action to the plant. So, the algorithm calculates the "optimal" controller action to achieve the references (present and future). The algorithm is (where "h" is the horizon and "k" the actual iteration):
 
 **Cost function**
 $$f(k) = \sum_{i=1}^{h}\left(Ref_{p}(i)-y_{p}(i)\right )^{2}$$
 **Gradient**
 $$\frac{\Delta f}{\Delta u_{i}} = \frac{f(k)-f(k-1)}{u_{i}(k)-u_{i}(k-1)}$$
 **Step Decay Learning Rate**
 $$\epsilon(k) =D\cdot\epsilon(k-1) $$
 **Decay**
 $$D = \left (1-\frac{decay(\% )}{100}\right )^{\left(\frac{1}{10
 }\right)}$$
 **Gradient Descent**
 $$u_{i}(k)=u_{i}(k-1)-\epsilon(k)\cdot\frac{\Delta f}{\Delta u_{i}}$$
 
 *The "step decay learning rate" reduces the learning rate value with each iteration to achieve thinner approximation to the "optimal" value. The idea is to begin with "big" learning rates to move faster to the solution and once the algorithm is near the solution, to make smoother the approximation.*

The "Run" button (green square) initializes the simulation, once the simulation is done it is plotted as can be seen in the following Figure:

![Last_example](https://github.com/JMcordobamendez/MPC_Learning_Tool/assets/79694677/6eecfe91-50da-4ee5-832c-9cc94921e3b2)
