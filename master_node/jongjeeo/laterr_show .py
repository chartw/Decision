#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# import pymap3d as pm
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
    
colnames=['lat_err', 't']
p = pd.read_csv('/home/wego/Desktop/deok/control-main_pointcloud220/lat_50.csv', names=colnames, header=None)


lat_err = p.loc[:,'lat_err'].tolist()
t = p.loc[:,'t'].tolist()

plt.title('Kp = 50 / Kp_ld = 0.03 / Kc = 0.8')
plt.plot(t, lat_err,c='red',label='lat_err')
# plt.axis([0,40, -5, 5])
plt.xlabel('time')
plt.ylabel('lat_err')
plt.legend(loc='upper right')
plt.grid(True, which = 'both', alpha = 0.5)
plt.show()

