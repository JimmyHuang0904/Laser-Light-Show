from Tkinter import *
import math

scale = 5

#       | 58     /
# -58   |    58 /  z = 200mm
# ------|------/
#       |
#       | -58
span = 116
z = 200.0

canvas_width = span*scale
canvas_height = span*scale

# Boolean for first read
i = True

# Count number of reads
count = 0

x1 = 0
x2 = 0
y1 = 0
y2 = 0


def paint(event):
    global x1, x2, y1, y2, i, count
    if i is True:
        i = False
        x1, y1 = (event.x - 1), (event.y - 1)
        x2, y2 = (event.x + 1), (event.y + 1)
    else:
        w.create_line(x1 + 1, y1 + 1, event.x, event.y)
        x1, y1 = (event.x - 1), (event.y - 1)
        x2, y2 = (event.x + 1), (event.y + 1)
        # w.create_oval(x1, y1, x2, y2, width=0, fill='black')

    f = open("x.h", "a")
    # f.write(str(event.x/scale - span/2.0) + "\n, ") # THIS GIVES THE CORRECT X VALUE
    f.write(str(math.atan((event.x/scale - span/2.0)/z)*180/math.pi*2.0/1.8) + "\n, ") # THIS GIVES X ANGLE
    f.close()

    f = open("y.h", "a")
    # f.write(str(-(event.y/scale - span/2.0)) + "\n, ") # THIS GIVES THE CORRECT Y VALUE
    f.write(str(math.atan(-(event.y/scale - span/2.0)/z)*180/math.pi*2.0/1.8) + "\n, ") # THIS GIVES Y ANGLE
    f.close()

    count += 1
    # f.write("[" + str(event.x/scale) + ", " + str(event.y/scale) + "\n")


def erase_last_line(filename):
    readfile = open(filename)

    lines = readfile.readlines()

    readfile.close()
    w = open(filename, 'w')

    w.writelines([item for item in lines[:-1]])

    w.close()


master = Tk()
master.title("Laser Trace")
w = Canvas(master,
           width=canvas_width,
           height=canvas_height)
w.pack(expand=YES, fill=BOTH)
w.bind("<B1-Motion>", paint)

message = Label(master, text="Press and Drag the mouse to draw")
message.pack(side=BOTTOM)

# Clear files
f = open("x.h", "w+")
f.close()
f = open("y.h", "w+")
f.close()

# populate x.h and y.h
mainloop()

# Prepare setpoint file
path = "C:\Users\jimmy_000\Documents\MATLAB\Laser-Light-Show\Arduino Code\SineWaveBoth\setpoint_gen.h"
f = open( path, "w+")

erase_last_line("x.h")
erase_last_line("y.h")

tempfile1 = open("x.h", "r")
tempfile2 = open("y.h", "r")
tempfiles = [tempfile1, tempfile2]

f.write("#ifndef setpoint_h\n"
        "#define setpoint_h\n\n"
        "#define SIZE_OF_ARR " + str(count) +
        "\n\nconst float setpoint_x[SIZE_OF_ARR] = {")

for tempfile in tempfiles:
    for line in tempfile:
        f.write(line)
    f.write("};\n\nconst float setpoint_y[SIZE_OF_ARR] ={")

f.close()

erase_last_line(path)

f = open(path, "a+")

f.write("#endif\n")
f.close
