import Tkinter
import tkMessageBox

#def set_arrow(C = Tkinter.Canvas(Tkinter.Tk(), bg="white", height = 500, width = 500),
def set_arrow(C,  x0 = None,y0 = None,x1 = 300,y1= 300,old_arrow_id = None):
    width  = x0 if x0 != None else int(C.cget("width"))/2
    height = y0 if y0 != None else int(C.cget("height"))/2
    if old_arrow_id:
        C.delete(old_arrow_id)
    arrow_id = C.create_line(width, height, x1, y1, arrow=Tkinter.LAST)
    return arrow_id

if __name__ == "__main__":
    top = Tkinter.Tk()
    C = Tkinter.Canvas(top, bg="white", height = 800, width = 800)
    
    set_arrow(C,x1 = 700, y1 = 650)
    C.pack()
    top.mainloop()
