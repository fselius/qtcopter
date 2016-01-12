#!/usr/bin/env python      
import Tkinter as tk       
import tkFont 
import random
import sys
import time

from PIL import Image
from PIL import ImageTk as itk
import numpy as np

class Application(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.big_font = tkFont.Font(family="Courier", size=32)
        #self.pack(fill=tk.BOTH, expand=1)
        self.grid(sticky=tk.N+tk.S+tk.E+tk.W)
        self.createWidgets()

        # it would be nice if app closed on ros quit, but this doesn't work
        #self.after(500, self.check_ros)
    def check_ros(self):
        if rospy.is_shutdown():
            self.close_windows()
        else:
            self.after(500, self.check_ros)
    def createWidgets(self):
        # make main window resizable
        top = self.winfo_toplevel()
        self.top = top
        top.rowconfigure(0, weight=1)
        top.columnconfigure(0, weight=1)

        # (deprecated) third column gets wider
        #self.columnconfigure(2, weight=1)
        #self.rowconfigure(1, weight=1)
        
        self.topbar = TopBar(self, font=self.big_font)
        self.topbar.grid(row=0, sticky=tk.N+tk.S+tk.E+tk.W, columnspan=2)

        # directions on the right, size ??
        self.directions = DirectionsWindow(self)
        self.directions.grid(row=1, column=1, sticky=tk.N+tk.S+tk.E+tk.W)
        # video stream on the left, height = 960/2, width = 1280/4
        self.image = ImageView(self)
        self.image.grid(row=1, column=0)
        top.update()
    def close_windows(self):
        self.master.destroy()

    def set_xyz(self, x, y, z):
        self.topbar.set_xyz(x, y, z)
        self.directions.set_arrow(x, y)
    def callback_tf(self, msg):
        is_good = lambda t: t.header.frame_id=='downward_cam_optical_frame' and\
                            t.child_frame_id=='waypoint'
        transforms = filter(is_good, msg.transforms)
        if not transforms:
            return
        tr = transforms[0]
        trl = tr.transform.translation
        x, y, z = trl.x, trl.y, trl.z
        x = -x
        self.set_xyz(x, y, z)

    def set_status(self, msg):
        self.topbar.set_status(msg)
    def callback_statemachine(self, msg):
        self.set_status(msg.data)
    def callback_image(self, msg):
        width, height, data = msg.width, msg.height, msg.data
        img = np.fromstring(msg.data, dtype=np.uint8).reshape(height, width, 3)
        self.image.set_img(img)
    def callback_debug_image(self, msg):
        width, height, data = msg.width, msg.height, msg.data
        img = np.fromstring(msg.data, dtype=np.uint8).reshape(height, width, 3)
        self.image.set_debug_img(img)


class TopBar(tk.Frame):
    # hold quit button, xyz, and status
    def __init__(self, master, font):
        tk.Frame.__init__(self, master)
        self.font = font
        # quit button | distance to target | status text
        self.quitButton = tk.Button(self, text='Quit', command=self.master.close_windows, font=self.font)
        self.quitButton.grid(row=0, column=0, sticky=tk.N+tk.S+tk.E+tk.W)

        self.Distance = Distance(self, font=self.font)
        self.Distance.grid(row=0, column=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.status = tk.StringVar()
        self.statusl = tk.Label(self, textvariable=self.status, font=self.font, bg="red")
        self.statusl.grid(row=0, column=2, sticky=tk.N+tk.S+tk.E+tk.W)
        self.grid()
    def set_status(self, msg):
        self.status.set(msg)
    def set_xyz(self, x, y, z):
        self.Distance.set_xyz(x, y, z)

class Distance(tk.Frame):
    def __init__(self, master, font):
        tk.Frame.__init__(self, master)
        #self.font = tkFont.Font(family="Fixedsys", size=32)
        self.font = font
        self.createStuff()
        self.grid()
    def createStuff(self):
        self.x = tk.StringVar()
        self.xl = tk.Label(self, textvariable=self.x, font=self.font)
        self.xl.grid(row=0, column=0)
        
        self.y = tk.StringVar()
        self.yl = tk.Label(self, textvariable=self.y, font=self.font)
        self.yl.grid(row=0, column=1)
        
        self.z = tk.StringVar()
        self.zl = tk.Label(self, textvariable=self.z, font=self.font)
        self.zl.grid(row=0, column=2)

        self.set_xyz(0, 0, 0)
    def set_xyz(self, x, y, z):
        self.x.set('%+.2f' % x)
        self.y.set('%+.2f' % y)
        self.z.set('%+.2f' % z)
class DirectionsWindow(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        
        self.canvas = tk.Canvas(self, width=1280/2, height=960/2)
        self.canvas.pack(fill="both", expand="1")
        self.set_arrow(100,200)
    def set_arrow(self, x1, y1):
        C = self.canvas
        x0 = None
        y0 = None 
        arrow_width = None
        width  = x0 if x0 != None else int(C.cget("width"))/2
        height = y0 if y0 != None else int(C.cget("height"))/2
        x1 *= width*2
        y1 *= height*2
        if arrow_width == None:
            arrow_width = min(int(C.cget("width")), int(C.cget("height")))/15

        if (abs(x1) > int(C.cget("width"))/2):
            ratio = 1.0*int(C.cget("width"))/(2*abs(x1))
            x1 *= ratio
            y1 *= ratio
        if (abs(y1) > int(C.cget("height"))/2):
            ratio = 1.0*int(C.cget("height"))/(2*abs(y1))
            x1 *= ratio
            y1 *= ratio
            
        x1_fixed = x1 + width
        y1_fixed = height - y1
        #if x1_fixed < 0:
        #    x1_fixed = 0
        #if y1_fixed < 0:
        #    y1_fixed = 0
        #if abs(x1_fixed) > int(C.cget("width")):
        #    ratio = 1.0*int(C.cget("width"))/abs(x1_fixed)
        #    x1_fixed *= ratio
        #    y1_fixed *= ratio
        #if abs(y1_fixed) > int(C.cget("height")):
        #    ratio = 1.0*int(C.cget("height"))/abs(y1_fixed)
        #    x1_fixed *= ratio
        #    y1_fixed *= ratio

        C.delete("all")
        arrow_id = C.create_line(width, height, x1_fixed, y1_fixed, arrow=tk.LAST, width = arrow_width)
        C.pack()

class ImageView(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.canvas1 = tk.Canvas(self, width=1280/4, height=960/4)
        self.canvas1.grid(row=0)
        self.canvas2 = tk.Canvas(self, width=1280/4, height=960/4)
        self.canvas2.grid(row=1)
    def set_img(self, img):
        img2 = img.copy()
        img2[:,:,0] = img[:,:,2]
        img2[:,:,1] = img[:,:,1]
        img2[:,:,2] = img[:,:,0]
        
        img = img2
        im = Image.fromarray(img)
        im = im.resize((1280/4, 960/4))
        pi = itk.PhotoImage(im)
        self.pi1 = pi
        self.canvas1.delete('all')
        self.canvas1.create_image(1280/4/2, 960/4/2, image=self.pi1)
        #self.canvas1.grid()
    def set_debug_img(self, img):
        img2 = img.copy()
        img2[:,:,0] = img[:,:,2]
        img2[:,:,1] = img[:,:,1]
        img2[:,:,2] = img[:,:,0]
        
        img = img2
        im = Image.fromarray(img, 'RGB')
        im = im.resize((1280/4, 960/4), Image.ANTIALIAS)
        pi = itk.PhotoImage(im)
        self.pi2 = pi
        self.canvas2.delete('all')
        self.canvas2.create_image(1280/4/2, 960/4/2, image=self.pi2)
        #self.canvas2.grid()
        

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
    from sensor_msgs.msg import Image as Image_msg
    image_sub = rospy.Subscriber("/image", Image_msg, app.callback_image)
    debug_image_sub = rospy.Subscriber("/debug_image", Image_msg, app.callback_debug_image)

    # tf
    from tf.msg import tfMessage
    tf_sub = rospy.Subscriber("/tf", tfMessage, app.callback_tf)
    # status?
    from std_msgs.msg import String
    statemachine_sub = rospy.Subscriber("/statemachine", String,
                                        app.callback_statemachine)

    # close app on shutdown :)
    # doesn't work :/
    #rospy.on_shutdown(app.close_windows)

    # height?
else:
    for i in xrange(50):
        x = random.uniform(-2,2)
        y = random.uniform(-2,2)
        z = random.uniform(-2,2)
        app.directions.set_arrow(x,y)
        app.set_xyz(x, y, z)
        app.top.update()
        time.sleep(1)
app.mainloop()
if ros:
    image_sub.unregister()
    debug_image_sub.unregister()
    tf_sub.unregister()
    statemachine_sub.unregister()
    '''
    # we quit after the window is closed
    rospy.spin()
    '''
print 'bye!'
