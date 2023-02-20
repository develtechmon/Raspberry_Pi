class IP_name:
    def __init__(self, ip, id):
        self.ipname = ip
        self.ipid = id
        
    def show_data(self):
        print(self.ipname, self.ipid)
        
x = IP_name("XSPRAM", 214)
x.show_data()