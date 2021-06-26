#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# import pymap3d as pm
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
    
colnames=['V_ref', 'V_veh', 'V_in', 't']
p = pd.read_csv('/home/gigacha2/catkin_ws/src/master_node/jongjeeo/06047.csv', names=colnames, header=None)


V_ref = p.loc[:,'V_ref'].tolist()
V_veh = p.loc[:,'V_veh'].tolist()
V_in = p.loc[:,'V_in'].tolist()
t = p.loc[:,'t'].tolist()

# plt.title('speed_ld = 5 / steering_ld = 4 / V_ref_max = 20km/h')
plt.title('P50 I5 D10 / V_ref=15km/h/ jong applied')
plt.plot(t, V_ref,c="red",label='V_ref')
plt.plot(t, V_veh,c='blue',label='V_veh')
plt.plot(t, V_in,c='yellow',label='V_in')
plt.xlabel('time')
plt.ylabel('velocity')
plt.legend(loc='upper right')

plt.show()

