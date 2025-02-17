class Vector:

    def __init__(self, x, y, z):
        self.data = [x,y,z]

    def x(self):
        return self.data[0]
    
    def y(self):
        return self.data[1]
    
    def z(self):
        return self.data[2]