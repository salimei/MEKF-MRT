

from matplotlib import pyplot as plt


def plotKF(time, position, ylabel, color): #Plots KF

    plt.scatter(time, position, color=color, linestyle='--', linewidths=0.1, 
                marker='o', label=ylabel)
    plt.legend(loc="upper left")
    plt.xlabel('time (centiseconds)')
    plt.ylabel(ylabel)


    #plot a single axis with covariance and true value
def plotMEKF1axis(xt, state_pred, state_real, cov1, cov2, N):
     
    plotKF(xt[0:N], state_pred[0:N], "predicted position (m)", "blue")
    plotKF(xt[0:N], state_real[0:N], "real position (m)", "red")
    plt.fill_between(xt[0:N], cov1[0:N], cov2[0:N], alpha=.5, linewidth=0)
    plt.show()
    
def plotMEKF1angle(xt, state_pred, state_real, cov1, cov2, N):
     
    plotKF(xt[0:N], state_pred[0:N], "predicted angle (rad)", "blue")
    plotKF(xt[0:N], state_real[0:N], "real angle (rad)", "red")
    plt.fill_between(xt[0:N], cov1[0:N], cov2[0:N], alpha=.5, linewidth=0)
    plt.show()  
    
    
#error plotters 

def plotMEKF1axisError(xt, state_pred, cov1, cov2, N):
     
    plotKF(xt[0:N], state_pred[0:N], "error position (m)", "blue")
    plt.fill_between(xt[0:N], cov1[0:N], cov2[0:N], alpha=.5, linewidth=0)
    plt.show()

def plotMEKF1angleError(xt, state_pred, cov1, cov2, N):
     
    plotKF(xt[0:N], state_pred[0:N], "error angle (rad)", "blue")
    plt.fill_between(xt[0:N], cov1[0:N], cov2[0:N], alpha=.5, linewidth=0)
    plt.show()  