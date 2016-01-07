#!/usr/bin/env python      
import Tkinter as tk       

class Application(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        #self.pack(fill=tk.BOTH, expand=1)
        self.grid(sticky=tk.E+tk.W)
        self.createWidgets()
        #self.direction_window()
    def direction_window(self):
        self.directions = tk.Toplevel(self.master)
        self.app = DirectionsWindow(self.directions)
        self.app.pack()
    def createWidgets(self):
        # make main window resizable
        top = self.winfo_toplevel()
        top.rowconfigure(0, weight=1)
        top.columnconfigure(0, weight=1)
        #self.rowconfigure(0, weight=1)
        # third column gets wider
        self.columnconfigure(2, weight=1)
        self.rowconfigure(1, weight=1)
        
        self.quitButton = tk.Button(self, text='Quit', command=self.close_windows)
        #self.quitButton.grid(row=0, column=0, sticky=tk.N+tk.S+tk.E+tk.W)
        self.quitButton.grid(row=0, column=0, sticky=tk.N+tk.S+tk.E+tk.W)

        self.Distance = Distance(self)
        self.Distance.grid(row=0, column=1)
        
        self.Status = tk.Label(self, text="some status", bg="red")
        ##self.Status.pack(fill=tk.BOTH, side=tk.LEFT)
        self.Status.grid(row=0, column=2, sticky=tk.N+tk.S+tk.E+tk.W)

        self.directions = DirectionsWindow(self)
        self.directions.grid(row=1, columnspan=3)
    def setMissionStatus(self, status):
        self.mission_status = status
    def close_windows(self):
        self.master.destroy()

class Distance(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.createStuff()
        self.grid()
    def createStuff(self):
        self.x = tk.Label(self, text="dx=10m")
        self.x.grid(row=0, column=0)
        
        self.y = tk.Label(self, text="dy=10m")
        self.y.grid(row=0, column=1)
        
        self.z = tk.Label(self, text="dz=10m")
        self.z.grid(row=0, column=2)
class DirectionsWindow(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        
        self.canvas = tk.Canvas(self)
        self.canvas.pack(fill="both", expand="1")
        self.canvas.create_rectangle(50, 25, 150, 75, fill="bisque", tags="r1")
        self.canvas.create_line(0,0, 50, 25, arrow="last", tags="to_r1")
        self.canvas.bind("<B1-Motion>", self.move_box)
        self.canvas.bind("<ButtonPress-1>", self.start_move)
    def move_box(self, event):
        deltax = event.x - self.x
        deltay = event.y - self.y
        self.canvas.move("r1", deltax, deltay)
        coords = self.canvas.coords("to_r1")
        coords[2] += deltax
        coords[3] += deltay
        self.canvas.coords("to_r1", *coords)
        self.x = event.x
        self.y = event.y

    def start_move(self, event):
        self.x = event.x
        self.y = event.y

app = Application()
app.master.title('Sample application')
app.mainloop()

