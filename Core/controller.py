import kinematics as kn

class Controllers:
    pass

if __name__=="__main__":
    thetas = kn.initKinematics() #radians
    print(thetas)
    kn.plotKinematics()
    