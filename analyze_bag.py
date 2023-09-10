from bagpy import bagreader
import matplotlib.pyplot as plt
import pandas as pd
import sys

def Plot_2D_trajectory(ax, df):
    ax.plot(df['x'], df['y'],label="Followed path", linewidth=0.7)
    ax.plot(df['xref'], df['yref'],label="Reference path", linestyle=(0,(1,1)))
    
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Car Trajectory")
    ax.legend(title="Paths")

def Plot_dim_wrt_time(ax, df, dimension):
    ax.plot(df['Time'], df[dimension],label="Followed path", linewidth=0.7)
    ax.plot(df['Time'], df[dimension + 'ref'],label="Reference path", linestyle=(0,(1,1)))
    
    ax.set_xlabel("t [s]")
    ax.set_ylabel(dimension + " [m]")
    ax.set_title(dimension + " Trajectory")
    #ax.legend(title="Paths")

def RMSE(df):
    return ((df['x'] - df['xref']) ** 2 + \
           (df['y'] - df['yref']) ** 2).mean() ** 0.5
    
def ISE(df):
    return ((df['x'] - df['xref']) ** 2 + \
           (df['y'] - df['yref']) ** 2).sum()
        
#open bag file and retrieve data
if len(sys.argv) < 2:
    print("ERROR: No input bag file")
    exit(-1)
bag_data = bagreader(sys.argv[1], tmp=True)
 
#### Dataframes building and data preprocessing ####
ref_traj_df = pd.read_csv(bag_data.message_by_topic('/reference_trajectory'))
ref_traj_df = ref_traj_df[['Time', 'data_0', 'data_1']]
ref_traj_df = ref_traj_df.rename(columns={'data_0':'xref', 'data_1':'yref'})

odometry_df = pd.read_csv(bag_data.message_by_topic('/vesc/odom'))
odometry_df = odometry_df[['Time', 'pose.pose.position.x', 'pose.pose.position.y']]
odometry_df = odometry_df.rename(columns={'pose.pose.position.x':'x', 'pose.pose.position.y':'y'})

# odometry messages are published at 20Hz while trajectory setpoints are published at 100Hz,
# thus we merge the two dataframes with a left join (where odometry is the left df) on Time  

aligned_df = pd.merge_asof(odometry_df, ref_traj_df, on='Time', direction='nearest')

#### Compute RMSE and ISE ####
rmse = RMSE(aligned_df)
ise = ISE(aligned_df)

#### Plotting ####
#create figure with 2x3 grid
fig = plt.figure(sys.argv[1] + " Analisys")
gs = fig.add_gridspec(2,3)

#Big plot of the trajectory
ax_big = fig.add_subplot(gs[:,:-1])
Plot_2D_trajectory(ax_big, aligned_df)

#Little textbox with rmse and ise
text = f"RMSE = {rmse:.4f}\nISE = {ise:.4f}" 
fig.text(x=0.03, y=0.05,s=text , fontsize = 18 , bbox = dict(facecolor = 'lightskyblue', alpha = 0.5))

#Plots of the single dimensions
dimensions = ['x', 'y']
for i in range(2):
    ax = fig.add_subplot(gs[i,2])
    Plot_dim_wrt_time(ax, aligned_df, dimensions[i])

#Finishing touches
figManager = plt.get_current_fig_manager()
figManager.window.showMaximized() #display maximized
plt.show()
