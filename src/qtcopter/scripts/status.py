#!/usr/bin/env python      
import Tkinter as tk       

class Application(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        #self.master = master
        #self.pack(fill=tk.BOTH, expand=1)
        self.grid(sticky=tk.E+tk.W)
        self.createWidgets()
        self.direction_window()
    def direction_window(self):
        self.directions = tk.Toplevel(self.master)
        self.app = DirectionsWindow(self.directions)
        
    def createWidgets(self):
        top = self.winfo_toplevel()
        top.rowconfigure(0, weight=1)
        top.columnconfigure(0, weight=1)
        #self.rowconfigure(0, weight=1)
        # second row gets wider
        self.columnconfigure(1, weight=1)
        
        self.quitButton = tk.Button(self, text='Quit', command=self.close_windows)
        self.quitButton.grid(row=0, column=0, sticky=tk.N+tk.S+tk.E+tk.W)

        self.Status = tk.Label(self, text="some status", bg="red")
        ##self.Status.pack(fill=tk.BOTH, side=tk.LEFT)
        self.Status.grid(row=0, column=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.Distance = Distance(self)
        self.Distance.grid(row=1, columnspan=2)
        
    def setMissionStatus(self, status):
        self.mission_status = status
    def close_windows(self):
        self.master.destroy()

class Distance(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.createStuff()
        self.grid()
    def createStuff(self):
        self.x = tk.Label(self, text="dx=10m")
        self.x.grid()
        
        self.y = tk.Label(self, text="dy=10m")
        self.y.grid()
        
        self.z = tk.Label(self, text="dz=10m")
        self.z.grid()
class DirectionsWindow(tk.Frame):
    def __init__(self, master):
        self.master = master
        tk.Frame.__init__(self, master)
        self.frame = tk.Frame(self.master)
        self.quitButton = tk.Button(self.frame, text = 'Quit', width = 25, command = self.close_windows)
        self.quitButton.pack()
        self.frame.pack()
    def close_windows(self):
        self.master.destroy()

app = Application()
app.master.title('Sample application')
app.mainloop()

