import Tkinter
import tkMessageBox

#def set_arrow(C = Tkinter.Canvas(Tkinter.Tk(), bg="white", height = 500, width = 500),
def set_arrow(C,  x0 = None,y0 = None,x1 = 300,y1= 300,old_arrow_id = None, arrow_width = None):
    width  = x0 if x0 != None else int(C.cget("width"))/2
    height = y0 if y0 != None else int(C.cget("height"))/2
    if arrow_width == None:
        arrow_width = min(int(C.cget("width")), int(C.cget("height")))/25
    x1_fixed = x1 + width
    y1_fixed = height - y1
    if x1_fixed < 0:
        x1_fixed = 0
    if y1_fixed < 0:
        y1_fixed = 0
    if x1_fixed > int(C.cget("width")):
        x1_fixed = int(C.cget("width"))
    if y1_fixed > int(C.cget("height")):
        y1_fixed = int(C.cget("height"))

    if old_arrow_id:
        C.delete(old_arrow_id)
    arrow_id = C.create_line(width, height, x1_fixed, y1_fixed, arrow=Tkinter.LAST, width = arrow_width)
    return arrow_id

if __name__ == "__main__":
    top = Tkinter.Tk()
    C = Tkinter.Canvas(top, bg="white", height = 800, width = 800)
    
    set_arrow(C,x1 = 200, y1 = -100)
    C.pack()
    top.mainloop()
