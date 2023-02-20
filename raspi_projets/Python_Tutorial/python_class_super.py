# Using Super - the child class will inherit all the method and properties from its
# Parents

class iptype:
    def __init__ (self, ip, id):
        self.ip_type = ip
        self.ip_id = id
        
    def showme(self):
        print(self.ip_type, self.ip_id)
        
class customize_generated(iptype):
    def __init__(self, ip, id):
        super().__init__(ip, id)
        self.customized = 3422
        #iptype.__init__(self, ip, id)
        
x=customize_generated("XSPRAM", 213)
print(x.customized)