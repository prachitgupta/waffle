import math

WAYPOINTS=[]

interval=9
radius=3

#circle
    
# for i in range(0,361,interval):
#     x=radius*math.cos(math.radians(i))
#     y=radius*math.sin(math.radians(i))
#     WAYPOINTS.append([x,y])

#lemniscate of Bernoulli (infinity symbol)

for i in range(0,361,interval):
    x=(radius*math.sqrt(2)*math.cos(math.radians(i)))/(math.pow(math.sin(math.radians(i)),2)+1)
    y=(radius*math.sqrt(2)*math.cos(math.radians(i))*math.sin(math.radians(i)))/(math.pow(math.sin(math.radians(i)),2)+1)
    WAYPOINTS.append([x,y])
print(WAYPOINTS)
