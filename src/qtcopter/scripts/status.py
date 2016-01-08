#!/usr/bin/env python      
import Tkinter as tk       
import sys

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

    def set_tf(self, x, y, z):
        self.Distance.set_xyz(x, y, z) 
    def callback_tf(self, msg):
        is_good = lambda t: t.header.frame_id=='downward_cam_optical_frame' and\
                            t.child_frame_id=='waypoint'
        transforms = filter(is_good, msg.transforms)
        if not transforms:
            return
        tr = transforms[0]
        trl = tr.transform.translation
        x, y, z = trl.x, trl.y, trl.z
        self.set_tf(x, y, z)
        #print x, y, z

class Distance(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.createStuff()
        self.grid()
    def createStuff(self):
        self.x = tk.StringVar()
        self.xl = tk.Label(self, textvariable=self.x)
        self.xl.grid(row=0, column=0)
        
        self.y = tk.StringVar()
        self.yl = tk.Label(self, textvariable=self.y)
        self.yl.grid(row=0, column=1)
        
        self.z = tk.StringVar()
        self.zl = tk.Label(self, textvariable=self.z)
        self.zl.grid(row=0, column=2)

        self.set_xyz(0, 0, 0)
    def set_xyz(self, x, y, z):
        self.x.set('%.5f' % x)
        self.y.set('%.5f' % y)
        self.z.set('%.5f' % z)
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

# create app
app = Application()
app.master.title('Status monitor')
ros = True
if len(sys.argv) > 1:
    if sys.argv[1] == '--noros':
        ros=False
if ros:
    import rospy
    # create ros node for listening
    rospy.init_node('listener', anonymous=True)
    # subscribe to all the needed topics

    # camera image?
    # /image
    # tf ?


    from tf.msg import tfMessage
    tf_sub = rospy.Subscriber("/tf", tfMessage, app.callback_tf)
    # status?
    # height?
app.mainloop()
if ros:
    tf_sub.unregister()
    '''
    # we quit after the window is closed
    rospy.spin()
    '''
print 'bye!'
