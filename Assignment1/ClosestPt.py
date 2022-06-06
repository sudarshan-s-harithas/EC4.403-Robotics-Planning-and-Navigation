import numpy as np
import mpl_toolkits.mplot3d.axes3d as Axes3D

import matplotlib.pyplot as plt

xlen = []
ylen = []
zlen = [] 
RectCenters = []

def TransformQpt(  Qpt ):
        
        TransformedQpt = [] 
        
        for Cent in RectCenters:
            
            diff = np.asarray(Qpt) - np.asarray(Cent)
            TransformedQpt.append(diff )
        
        return TransformedQpt

def TransformQpt2( Qpt, Center ):
	diff = np.asarray(Qpt) - np.asarray(Center)
	return diff



def RectangularSDF(  Qpt, Xlength, Ylength , Zlength ):
        
        R = np.asarray( [  Xlength, Ylength , Zlength ])
        
        d =  np.linalg.norm( np.maximum( np.abs(Qpt) - R , np.zeros(3) ) )
        
        return d


def DetectCollision(   Qpt ):
        
        numRects = len( xlen )
        distance = []
        
        for i in range(numRects  ):
            Qpt_transformed =  TransformQpt2(Qpt ,RectCenters[i] )
            
            d= RectangularSDF( Qpt_transformed ,xlen[i] , ylen[i] , zlen[i]  )
            print(d)
            print(Qpt  )
            print( RectCenters[i]  )
            print("--------------")
            distance.append( d )
        
        colsestDistance = np.amin(distance  )


        return colsestDistance

def cuboid_data(box):
        l = box[3] - box[0]
        w = box[4] - box[1]
        h = box[5] - box[2]
        x = [[0, l, l, 0, 0],
             [0, l, l, 0, 0],
             [0, l, l, 0, 0],
             [0, l, l, 0, 0]]
        y = [[0, 0, w, w, 0],
             [0, 0, w, w, 0],
             [0, 0, 0, 0, 0],
             [w, w, w, w, w]]
        z = [[0, 0, 0, 0, 0],
             [h, h, h, h, h],
             [0, 0, h, h, 0],
             [0, 0, h, h, 0]]
        return box[0] + np.array(x), box[1] + np.array(y), box[2] + np.array(z)



def test_env():
        # 3D boxes   lx, ly, lz, hx, hy, hz
        obstacles = [[-5, 25, 0, 20, 35, 60],
                     [30, 25, 0, 55, 35, 100],
                     [45, 35, 0, 55, 60, 60],
                     [45, 75, 0, 55, 85, 100],
                     [-5, 65, 0, 30, 70, 100],
                     [70, 50, 0, 80, 80, 100]]
        
        # create map with obstacles
        fig = plt.figure()
        ax = Axes3D.Axes3D(fig, auto_add_to_figure=False)
        fig.add_axes(ax)
        cnt =0 
        for box in obstacles:
            X, Y, Z = cuboid_data(box)
            xcent  = (box[0] + box[3])/2
            ycent = (box[1] + box[4])/2
            zcent = (box[2] + box[5])/2
            
            
            x_len = (box[3] - box[0])/2
            y_len = (box[4] - box[1])/2
            z_len = (box[5] - box[2])/2
            
            xlen.append( x_len )
            ylen.append( y_len )
            zlen.append( z_len )
            
            RectCenters.append( [xcent, ycent, zcent] )
            plt.xlabel("x axis")
            plt.ylabel("y axis")
            ax.plot_surface(X, Y, Z, rstride=1, cstride=1,color=(0.1, 0.15, 0.3, 0.2),zorder = 1)
            

            
        plt.show()


test_env()



for R in RectCenters:

	 DetectCollision( R ) 
	 print("********************")



# for 