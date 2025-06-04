from core.shared_imports import mp

class Map():
    def __init__(self):
        self.resoltion = 100
        
        self.map = [["" for _ in range(self.resoltion)] for _ in range(self.resoltion)]