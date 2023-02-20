class iptype:
     def __init__(self, ip, id):
         self.ip_type = ip
         self.ip_id = id
         
     def showme(self):
         print(self.ip_type, self.ip_id)
        
class customize_generated(iptype):
    def __init__(self, ip, id):
        iptype.__init__(self, ip, id)
        
x = customize_generated ("XSPRAM", 2321)
x.showme()