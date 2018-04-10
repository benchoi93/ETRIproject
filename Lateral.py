

N =  Nold          #veh
LC_right = np.zeros((l,m))             #veh
LC_left  = np.zeros((l,m))             #veh

LC_right[1,3]=1
LC_right[2,4]=1
LC_right[1,2]=1

LC_left[3,2]=1
LC_left[2,3]=1
LC_left[2,5]=1

def Lateral(N , maxN, LC_right , LC_left):
    
    Y_right = np.zeros((l,m))
    Y_right += -LC_right
    Y_right[1:] +=  LC_right[:-1,:]

    Y_left = np.zeros((l,m))
    Y_left += -LC_left
    Y_left[:-1] += LC_left[1:,:]
    
    Nnew = N + Y_right + Y_left
    
    maxN
    
    
