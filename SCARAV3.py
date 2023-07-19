# The code below is importing the necessary libraries for the program to run.
from tkinter import *
from tkinter import messagebox
import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3

# Creating a GUI window with a title and an icon.
gui = Tk()
gui.title("FK & IK SCARA_V3")
gui.iconbitmap('Robokin.ico')
gui.resizable(False,False)

def reset():
    """
    It clears all the text boxes.
    """
    a1_E.delete(0, END)
    a2_E.delete(0, END)
    a3_E.delete(0, END)
    a4_E.delete(0, END)
    a5_E.delete(0, END)
    a6_E.delete(0, END)
    a7_E.delete(0, END)
    d1_E.delete(0, END)
    t2_E.delete(0, END)
    t3_E.delete(0, END)
    t4_E.delete(0, END)
    t5_E.delete(0, END)
    t6_E.delete(0, END)
    X_E.delete(0, END)
    Y_E.delete(0, END)
    Z_E.delete(0, END)

def f_k():
    """
    It calculates the forward kinematics of the robot.
    """
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100
    a4 = float(a4_E.get())/100
    a5 = float(a5_E.get())/100
    a6 = float(a6_E.get())/100
    a7 = float(a7_E.get())/100

    d1 = float(d1_E.get())/100 #50cm for testing
    T2 = float(t2_E.get())
    T3 = float(t3_E.get())
    T4 = float(t4_E.get())
    T5 = float(t5_E.get())
    T6 = float(t6_E.get())

    T2 = (T2/180.0)*np.pi # Theta 2 in radians #60 deg for testing
    T3 = (T3/180.0)*np.pi # Theta 3 in radians #-30 deg for testing
    T4 = (T4/180.0)*np.pi # Theta 4 in radians #90 deg for testing
    T5 = (T5/180.0)*np.pi # Theta 5 in radians #45 deg for testing
    T6 = (T6/180.0)*np.pi # Theta 6 in radians #-45 deg for testing

    PT = [[(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a1+d1],
          [T2,(0.0/180.0)*np.pi,a2,0],
          [T3,(0.0/180.0)*np.pi,a4,a3],
          [T4,(90.0/180.0)*np.pi,0,-a5],
          [T5,(90.0/180.0)*np.pi,0,0],
          [T6,(0.0/180.0)*np.pi,0,a6+a7]]

    i = 0
    H0_1 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

    i = 1
    H1_2 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

    i = 2
    H2_3 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

    i = 3
    H3_4 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

    i = 4
    H4_5 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

    i = 5
    H5_6 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

    # Position/Translation Joints
    H0_1 = np.matrix(H0_1) 

    H1_2 = np.matrix(H1_2) 

    H2_3 = np.matrix(H2_3)

    # Orientation/Rotation Joints
    H3_4 = np.matrix(H3_4) 
    H4_5 = np.matrix(H4_5)
    H5_6 = np.matrix(H5_6)  

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)
    H0_4 = np.dot(H0_3,H3_4)
    H0_5 = np.dot(H0_4,H4_5)
    H0_6 = np.dot(H0_5,H5_6)

    X0_6 = H0_6[0,3]
    X_E.delete(0,END)
    X_E.insert(0,np.around(X0_6*100,3))
    Y0_6 =H0_6[1,3]
    Y_E.delete(0,END)
    Y_E.insert(0,np.around(Y0_6*100,3))
    Z0_6 =H0_6[2,3]
    Z_E.delete(0,END)
    Z_E.insert(0,np.around(Z0_6*100,3))

    # Create Links
    SCARA_V3 = DHRobot([
            PrismaticDH(0,0,(0/180)*np.pi,a1,qlim=[0,(30/100)]),
            RevoluteDH(0,a2,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
            RevoluteDH(a3,0,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
            PrismaticDH(0,a4,0,0,qlim=[0,0]),
            RevoluteDH(-a5,0,(90/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
            RevoluteDH(0,0,(90/180)*np.pi,0,qlim=[(0/180)*np.pi,(90/180)*np.pi]),
            RevoluteDH(a6+a7,0,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),

        ], name='SCARA_V3')

    # q Paths
    q0 = np.array([0,0,0,0,0,0,0])
    q1 = np.array([d1,T2,T3,0,T4,T5,T6])

    # Trajectory commands
    traj1 = rtb.jtraj(q0,q1,50)

    #plot scale
    x1 = -0.8
    x2 = 0.8
    y1 = -0.8
    y2 = 0.8
    z1 = -0.8
    z2 = 0.8     

    # Plot commands
    SCARA_V3.plot(traj1.q,limits=[x1,x2,y1,y2,z1,z2],block=True) 

def i_k():
    """
    The function takes the values from the entry boxes and uses them to calculate the inverse kinematics
    of the robot
    """
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100
    a4 = float(a4_E.get())/100
    a5 = float(a5_E.get())/100
    a6 = float(a6_E.get())/100
    a7 = float(a7_E.get())/100

    # Create Links
    SCARA_V3 = DHRobot([
            PrismaticDH(0,0,(0/180)*np.pi,a1,qlim=[0,(30/100)]),
            RevoluteDH(0,a2,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
            RevoluteDH(a3,0,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
            PrismaticDH(0,a4,0,0,qlim=[0,0]),
            RevoluteDH(-a5,0,(90/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
            RevoluteDH(0,0,(90/180)*np.pi,0,qlim=[(0/180)*np.pi,(90/180)*np.pi]),
            RevoluteDH(a6+a7,0,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),

        ], name='SCARA_V3')
    xe = float(X_E.get())/100
    ye = float(Y_E.get())/100
    ze = float(Z_E.get())/100

    T = SE3(xe,ye,ze) * SE3.OA([0,0,-1],[-1,0,0])
    IKine = SCARA_V3.ikine_LM(T)
    
    De1 = IKine.q[0]
    Te2 = IKine.q[1]
    Te3 = IKine.q[2]
    OS = IKine.q[3]
    Te4 = IKine.q[4]
    Te5 = IKine.q[5]
    Te6 = IKine.q[6]
    
    d1_E.delete(0,END)
    d1_E.insert(0,np.around(De1*100,3))

    t2_E.delete(0,END)
    t2_E.insert(0,np.around(Te2*180/np.pi,3))

    t3_E.delete(0,END)
    t3_E.insert(0,np.around(Te3*180/np.pi,3))

    ofs_E.delete(0,END)
    ofs_E.insert(0,np.around(OS*100,3))

    t4_E.delete(0,END)
    t4_E.insert(0,np.around(Te4*180/np.pi,3))

    t5_E.delete(0,END)
    t5_E.insert(0,np.around(Te5*180/np.pi,3))

    t6_E.delete(0,END)
    t6_E.insert(0,np.around(Te6*180/np.pi,3))

    # q Paths
    q0 = np.array([0,0,0,0,0,0,0])

    q1 = np.array([De1,Te2,Te3,OS,Te4,Te5,Te6])

    # Trajectory commands
    traj1 = rtb.jtraj(q0,q1,50)

    #plot scale
    x1 = -0.8
    x2 = 0.8
    y1 = -0.8
    y2 = 0.8
    z1 = -0.8
    z2 = 0.8   

    # Plot commands
    SCARA_V3.plot(traj1.q,limits=[x1,x2,y1,y2,z1,z2],block=True)

def jacob():
    """
    It takes the values of the parameters from the entry boxes and uses them to calculate the Jacobian
    matrix of the robot.
    """
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())
    a4 = float(a4_E.get())
    a5 = float(a5_E.get())
    a6 = float(a6_E.get())
    a7 = float(a7_E.get())

    # Create Links
    SCARA_V3 = DHRobot([
            PrismaticDH(0,0,(0/180)*np.pi,a1,qlim=[0,(30)]),
            RevoluteDH(0,a2,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
            RevoluteDH(a3,0,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
            PrismaticDH(0,a4,0,0,qlim=[0,0]),
            RevoluteDH(-a5,0,(90/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
            RevoluteDH(0,0,(90/180)*np.pi,0,qlim=[(0/180)*np.pi,(90/180)*np.pi]),
            RevoluteDH(a6+a7,0,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),

        ], name='SCARA_V3')

    dd1 = float(d1_E.get())
    Tt2 = float(t2_E.get())
    Tt3 = float(t3_E.get())
    Tt4 = float(t4_E.get())
    Tt5 = float(t5_E.get())
    Tt6 = float(t6_E.get())

    Tt2 = (Tt2/180.0)*np.pi # Theta 2 in radians
    Tt3 = (Tt3/180.0)*np.pi # Theta 3 in radians
    Tt4 = (Tt4/180.0)*np.pi # Theta 4 in radians
    Tt5 = (Tt5/180.0)*np.pi # Theta 5 in radians
    Tt6 = (Tt6/180.0)*np.pi # Theta 6 in radians

    # Jacobian Formulas
    J = SCARA_V3.jacob0([dd1,Tt2,Tt3,0,Tt4,Tt5,Tt6])
    J = np.matrix(np.around(J,3))

    JS1 = J[0:,0:3]
    JS2 = J[0:,4:]
    JS = np.concatenate((JS1,JS2),1)

    Det_J = np.linalg.det(JS)
    if 0.0001 >= Det_J > -0.0001:
        messagebox.showerror("The Jacobian Matrix in Non-Invertible!")
        messagebox.iconbitmap('Robokin.ico')
        J1.destroy 

    Inv_J = np.linalg.inv(JS)
    Trans_J = np.transpose(JS)

    Det_J = np.around(Det_J,3)
    Inv_J = np.matrix(np.around(Inv_J,3))
    Trans_J = np.matrix(np.around(Trans_J,3))
    
    # Jacobian Window
    J1 = Toplevel()
    J1.title("Jacobian Matrix")
    J1.iconbitmap('Robokin.ico')
    J1.resizable(False,False)
    
    # JM Frame
    JF1 = LabelFrame(J1,text="Jacobian Matrix",font=(5))
    JF1.grid(row=0,column=0)
    JaM = Label(JF1,text=J,font=(5))
    JaM.grid(row=0,column=0)

    # Det Frame
    JF2 = LabelFrame(J1,text="Determinant of J",font=(5))
    JF2.grid(row=1,column=0)
    JDt = Label(JF2,text=Det_J,font=(5))
    JDt.grid(row=0,column=0)

    # Inv Frame
    JF3 = LabelFrame(J1,text="Inverse of J",font=(5))
    JF3.grid(row=2,column=0)
    Jin = Label(JF3,text=Inv_J,font=(5))
    Jin.grid(row=0,column=0)

    # Trans Frame
    JF4 = LabelFrame(J1,text="Transpose of J",font=(5))
    JF4.grid(row=3,column=0)
    Jtr = Label(JF4,text=Trans_J,font=(5))
    Jtr.grid(row=0,column=0)

    Clbn = Button(J1,text="OK",command=J1.destroy,bg="red",font=(24))
    Clbn.grid(row=4,column=0)

# Link Lengths and Joint Variables Frame
FI = LabelFrame(gui,text="Link Lengths and Joint Variables",font=(5))
FI.grid(row=0,column=0)

# The code below is creating a label frame and 
# then creating labels and entries for the link lengths.
#Link lengths
a1 = Label(FI,text=("a1 = "),font=(10))
a1_E = Entry(FI,width=5,font=(10))
cm1 = Label(FI,text=("cm"),font=(10))

a2 = Label(FI,text=("a2 = "),font=(10))
a2_E = Entry(FI,width=5,font=(10))
cm2 = Label(FI,text=("cm"),font=(10))

a3 = Label(FI,text=("a3 = "),font=(10))
a3_E = Entry(FI,width=5,font=(10))
cm3 = Label(FI,text=("cm"),font=(10))

a4 = Label(FI,text=("a4 = "),font=(10))
a4_E = Entry(FI,width=5,font=(10))
cm4 = Label(FI,text=("cm"),font=(10))

a5 = Label(FI,text=("a5 = "),font=(10))
a5_E = Entry(FI,width=5,font=(10))
cm5 = Label(FI,text=("cm"),font=(10))

a6 = Label(FI,text=("a6 = "),font=(10))
a6_E = Entry(FI,width=5,font=(10))
cm6 = Label(FI,text=("cm"),font=(10))

a7 = Label(FI,text=("a7 = "),font=(10))
a7_E = Entry(FI,width=5,font=(10))
cm7 = Label(FI,text=("cm"),font=(10))

a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)

a4.grid(row=3,column=0)
a4_E.grid(row=3,column=1)
cm4.grid(row=3,column=2)

a5.grid(row=4,column=0)
a5_E.grid(row=4,column=1)
cm5.grid(row=4,column=2)

a6.grid(row=5,column=0)
a6_E.grid(row=5,column=1)
cm6.grid(row=5,column=2)

a7.grid(row=6,column=0)
a7_E.grid(row=6,column=1)
cm7.grid(row=6,column=2)

# The code below is creating a label frame and 
# then creating labels and entries for the Joint Variables.
# Joint Variables
d1 = Label(FI,text=("d1 = "),font=(10))
d1_E = Entry(FI,width=5,font=(10))
cm7 = Label(FI,text=("cm"),font=(10))

t2 = Label(FI,text=("t2 = "),font=(10))
t2_E = Entry(FI,width=5,font=(10))
deg2 = Label(FI,text=("deg"),font=(10))

t3 = Label(FI,text=("t3 = "),font=(10))
t3_E = Entry(FI,width=5,font=(10))
deg3 = Label(FI,text=("deg"),font=(10))

ofs = Label(FI,text=("ofs = "),font=(10),fg="red") #offset
ofs_E = Entry(FI,width=5,font=(10),fg="red")
cmO = Label(FI,text=("cm"),font=(10),fg="red")

t4 = Label(FI,text=("t4 = "),font=(10))
t4_E = Entry(FI,width=5,font=(10))
deg4 = Label(FI,text=("deg"),font=(10))

t5 = Label(FI,text=("t5 = "),font=(10))
t5_E = Entry(FI,width=5,font=(10))
deg5 = Label(FI,text=("deg"),font=(10))

t6 = Label(FI,text=("t6 = "),font=(10))
t6_E = Entry(FI,width=5,font=(10))
deg6 = Label(FI,text=("deg"),font=(10))

d1.grid(row=0,column=3)
d1_E.grid(row=0,column=4)
cm7.grid(row=0,column=5)

t2.grid(row=1,column=3)
t2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

t3.grid(row=2,column=3)
t3_E.grid(row=2,column=4)
deg3.grid(row=2,column=5)

ofs.grid(row=3,column=3) 
ofs_E.grid(row=3,column=4)
cmO.grid(row=3,column=5)

t4.grid(row=4,column=3)
t4_E.grid(row=4,column=4)
deg4.grid(row=4,column=5)

t5.grid(row=5,column=3)
t5_E.grid(row=5,column=4)
deg5.grid(row=5,column=5)

t6.grid(row=6,column=3)
t6_E.grid(row=6,column=4)
deg6.grid(row=6,column=5)

# The code below is creating a frame and buttons.
# Buttons Frame
BF = LabelFrame(gui,text="Forward and Inverse Kinematics",font=(5))
BF.grid(row=1,column=0)

# Buttons
rst = Button(BF,text="RESET",font=(10),bg="green",fg="white",command=reset)
FK = Button(BF,text="↓ Forward",font=(10),bg="blue",fg="white",command=f_k)
IK = Button(BF,text="Inverse ↑",font=(10),bg="red",fg="black",command=i_k)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)
IK.grid(row=0,column=2)

# The code below is creating a label frame, which is a frame that contains labels. The label frame is
# called PV, and it is placed in the gui window. The label frame is given the title "Position
# Vectors", and the font is set to 5. The label frame is placed in the second row and the first
# column.

# Position Vectors Frame
PV = LabelFrame(gui,text="Position Vectors",font=(5))
PV.grid(row=2,column=0)

# Position Vectors
X = Label(PV,text=("X = "),font=(10))
X_E = Entry(PV,width=8,font=(10))
cm8 = Label(PV,text=("cm"),font=(10))

Y = Label(PV,text=("Y = "),font=(10))
Y_E = Entry(PV,width=8,font=(10))
cm9 = Label(PV,text=("cm"),font=(10))

Z = Label(PV,text=("Z = "),font=(10))
Z_E = Entry(PV,width=8,font=(10))
cm10 = Label(PV,text=("cm"),font=(10))

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm8.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm9.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm10.grid(row=2,column=2)

# The code below is creating a button that will call the function jacob when clicked.
# Jacobian Matrix Frame
JM = LabelFrame(gui,text="Jacobian Matrices",font=(10))
JM.grid(row=3,column=0)

# Jacobian Matrices
JMX = Button(JM,text="Jacobian Matrix (J)",font=(10),bg="purple",fg="white",width=15,command=jacob)
JMX.grid(row=0,column=0)

gui.mainloop()