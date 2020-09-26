from modes import AbstractMode
from servos import Servos

class ModeStandby(AbstractMode):

    def __init__(self,servos):
        AbstractMode.__init__(self,servos)

    def init(self):
        self.servos.angle(1,50)
        # [self.servos.angle(v,50) for v in range(0,1)]

    def update(self):
        pass


if __name__ == "__main__":
    try:
        servos=Servos()
        ms = ModeStandby(servos)
        ms.init()
    except Exception as e:
        print(e)
    pass