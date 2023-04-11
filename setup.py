'''
Terminal 1
mono MissionPlanner.exe

Terminal 2
cd DroneSim-Installer/ardupilot/Tools/autotest
. fg_quad_view.sh

Terminal 3
sudo python sim_vehicle.py --console --map -v ArduCopter 
(or)
sudo python sim_vehicle.py -L KFSO -v ArduCopter
sim-vehiclde.py -L KSFO

Terminal 4
sudo python drone1.py


export PATH=$PATH:$HOME/path/to/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
'''