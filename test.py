from this import d
import pandas as pd
import numpy as np

t = [[1,2,3],[2,3,4]]
print(t.size())
t2 = [[1,2,3],[2,3,4],[2,3,4]]
s = pd.DataFrame()
s2 = pd.DataFrame(t, columns=['u1', 'x1', 'y1'])
s2 = pd.concat([s2,s],axis=1)
print (s2)
