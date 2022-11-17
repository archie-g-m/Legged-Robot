import math

class SerialIK:

    def __init__(self, L1, L2, L3):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

    def getIK(self, x, y, z):

        w = self.getDist(x, y)
        a = w - self.L1
        b = self.getDist(a,z)
        psi = math.atan(-z/a)

        print(w)
        print(a)
        print(b)
        print(psi)

        theta1 = math.degrees(math.atan(y/x))
        theta2 = math.degrees(self.getLOC(self.L2, b, self.L3) - psi)
        theta3 = 180 - math.degrees(self.getLOC(self.L2, self.L3, b))

        return [theta1, theta2, theta3]

    def getDist(self, a, b):
        return math.sqrt(a**2 + b**2)

    def getLOC(self, a, b, c):
        expr = (a**2 + b**2 - c**2)/(2*a*b)
        return math.acos(expr)


a = SerialIK(1, 2, 3)

print(a.getIK(3, 0, -1))

