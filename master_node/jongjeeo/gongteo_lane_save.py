#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pymap3d as pm
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
		


def cubic(name, x,y):

    save_data = list(zip(x, y))
    # print(save_data)

    save_df = pd.DataFrame(save_data)
    save_df.to_csv('%s.csv'%name, index=False, header = False)

    #plt.scatter(cx, cy)
    
    #plt.show()


    
# cubic("0010", output)




