#!/user/bin/python
# -* - coding:UTF-8 -*-

#插入Abaqus模块
import random
from abaqus import *
from abaqusConstants import *
from caeModules import *

p = mdb.models['Model-1'].Part(name='Part-1', dimensionality=THREE_D, type=DEFORMABLE_BODY)

#空间连线建立框架
p.WirePolyLine(points=(((50,50,50),(50,50,-50)), ((50,50,-50),(50,-50,-50)), ((50,-50,-50),(50,-50,50)), ((50,-50,50),(50,50,50)), 
    ((50,50,50),(-50,50,50)), ((-50,50,50),(-50,50,-50)), ((-50,50,-50),(-50,-50,-50)), ((-50,-50,-50),(50,-50,-50)), 
	((-50,-50,-50),(-50,-50,50)),((-50,-50,50), (-50,50,50)), ((-50,-50,50),(50,-50,50)),((-50,50,-50),(50,50,-50))),  meshable=ON)


#生成第一根纤维
#生成第一个点
plane_list=[1,2,3,4]
Num = 0 
while Num < 10000:                                         #生成满足要求长度的纤维最大迭代次数
	pointplane1,pointplane2=random.sample(plane_list, 2) 
	if pointplane1==1:
		X1=random.randint(-49,49)
		Y1=random.randint(-49,49)
		Z1=random.randint(-49,49)
	elif pointplane1==2:
		X1=random.randint(-49,49)
		Y1=random.randint(-49,49)
		Z1=random.randint(-49,49)
	elif pointplane1==3:
		X1=random.randint(-49,49)
		Y1=random.randint(-49,49)
		Z1=random.randint(-49,49)
	elif pointplane1==4:
		X1=random.randint(-49,49)
		Y1=random.randint(-49,49)
		Z1=random.randint(-49,49)
	#生成第二个点
	if pointplane2==1:
		X2=random.randint(-49,49)
		Y2=random.randint(-49,49)
		Z2=random.randint(-49,49)
	elif pointplane2==2:
		X2=random.randint(-49,49)
		Y2=random.randint(-49,49)
		Z2=random.randint(-49,49)
	elif pointplane2==3:
		X2=random.randint(-49,49)
		Y2=random.randint(-49,49)
		Z2=random.randint(-49,49)
	elif pointplane2==4:
		X2=random.randint(-49,49)
		Y2=random.randint(-49,49)
		Z2=random.randint(-49,49)		
	L1 = ((X1-X2)**2+(Y1-Y2)**2+(Z1-Z2)**2)**0.5
	if 12.5<= L1 <= 13.5:                                   #规定生成的纤维长度
		break
	else:
		continue
	Num += 1

#采用生成的点连线
p.WireSpline(points=((X1,Y1,Z1), (X2,Y2,Z2)),  meshable=ON, smoothClosedSpline=ON)

#扫描成第一根纤维
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=100)       #定义模型的草图s
s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(0.0, 0.15))         #确定扫描半径  
e = p.edges
edges = e.findAt(((X1,Y1,Z1), ),((X2,Y2,Z2), ))       #通过findAt()命令来选取相应的体、面、线或点
p.SolidSweep(path=edges, sketchUpEdge=e[5], sketchOrientation=TOP, profile=s)


#建立以纤维端点坐标为元素的列表
point_1=(X1,Y1,Z1,X2,Y2,Z2)
fiberList=[point_1]
	
def PLdistance (x1,y1,z1,x2,y2,z2,x3,y3,z3):			#定义一个计算点到线段最短间距的函数
	t=1.0*((x1-x3)*(x1-x2)+(y1-y3)*(y1-y2)+(z1-z3)*(z1-z2))/((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)
	if 0<=t<=1:											#垂足在线段上
		xt, yt, zt=x1+t*(x2-x1), y1+t*(y2-y1), z1+t*(z2-z1)
		d=((x3-xt)**2+(y3-yt)**2+(z3-zt)**2)**0.5
	elif t>1:
		d=((x3-x2)**2+(y3-y2)**2+(z3-z2)**2)**0.5
	else:
		d=((x3-x1)**2+(y3-y1)**2+(z3-z1)**2)**0.5
	return d

def distance (x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4):		#定义一个计算线段到线段最短间距的函数
	a1=(x2-x1)**2+(y2-y1)**2+(z2-z1)**2
	b1=(x3-x4)*(x2-x1)+(y3-y4)*(y2-y1)+(z3-z4)*(z2-z1)	#这里a2=b1，将a2用b1代替
	b2=(x4-x3)**2+(y4-y3)**2+(z4-z3)**2
	if b1*b1!=a1*b2:                                    #线段不平行
		c1=(x1-x2)*(x1-x3)+(y1-y2)*(y1-y3)+(z1-z2)*(z1-z3)
		c2=(x1-x3)*(x4-x3)+(y1-y3)*(y4-y3)+(z1-z3)*(z4-z3)
		t=1.0*(b1*c1-a1*c2)/(b1*b1-a1*b2)
		s=1.0*(b2*c1-b1*c2)/(a1*b2-b1*b1)
		if 0<=t<=1 and 0<=s<=1:							#垂足在两条线段上
			xt,yt,zt=x3+t*(x4-x3), y3+t*(y4-y3), z3+t*(z4-z3)
			xs,ys,zs=x1+s*(x2-x1), y1+s*(y2-y1), z1+s*(z2-z1)
			D=((xt-xs)**2+(yt-ys)**2+(zt-zs)**2)**0.5
		else:
			d1=PLdistance(x1,y1,z1,x2,y2,z2,x3,y3,z3)	#计算四个点分别到两条直线间的距离
			d2=PLdistance(x1,y1,z1,x2,y2,z2,x4,y4,z4)
			d3=PLdistance(x3,y3,z3,x4,y4,z4,x1,y1,z1)
			d4=PLdistance(x3,y3,z3,x4,y4,z4,x2,y2,z2)
			D=min(d1,d2,d3,d4)							#最短距离即距离最小值
	else:												#线段平行
		d1=PLdistance(x1,y1,z1,x2,y2,z2,x3,y3,z3)
		d2=PLdistance(x1,y1,z1,x2,y2,z2,x4,y4,z4)
		D=min(d1,d2)
	return D

#定义一个循环函数
def createfiber ():
#生成第二根纤维
#随机生成两个点（同上），采用while 循环，直至得到理想结果才继续
	while True:
		N = 0
		while N < 10000:
			pointplane3,pointplane4=random.sample(plane_list, 2) 
			if pointplane3==1:
				X3=random.randint(-49,49)
				Y3=random.randint(-49,49)
				Z3=random.randint(-49,49)
			elif pointplane3==2:
				X3=random.randint(-49,49)
				Y3=random.randint(-49,49)
				Z3=random.randint(-49,49)
			elif pointplane3==3:
				X3=random.randint(-49,49)
				Y3=random.randint(-49,49)
				Z3=random.randint(-49,49)
			elif pointplane3==4:
				X3=random.randint(-49,49)
				Y3=random.randint(-49,49)
				Z3=random.randint(-49,49)
			if pointplane4==1:
				X4=random.randint(-49,49)
				Y4=random.randint(-49,49)
				Z4=random.randint(-49,49)
			elif pointplane4==2:
				X4=random.randint(-49,49)
				Y4=random.randint(-49,49)
				Z4=random.randint(-49,49)
			elif pointplane4==3:
				X4=random.randint(-49,49)
				Y4=random.randint(-49,49)
				Z4=random.randint(-49,49)
			elif pointplane4==4:
				X4=random.randint(-49,49)
				Y4=random.randint(-49,49)
				Z4=random.randint(-49,49)
			L2 = ((X3-X4)**2+(Y3-Y4)**2+(Z3-Z4)**2)**0.5
			if 12.5<= L2 <= 13.5:                                   #规定生成第二根纤维的长度
				break
			else:
				continue
		N += 1
			
		for i in fiberList:								
			Dis=distance(X3,Y3,Z3, X4,Y4,Z4,*i)
			if Dis<= 1.0:                                           #规定生成两根纤维最小距离
				break
			continue
		else:
			break
		
	point_2=(X3,Y3,Z3,X4,Y4,Z4)
	#将第二条直线的点加入列表
	fiberList.append(point_2)
	#采用生成的点连线
	p.WireSpline(points=((X3,Y3,Z3), (X4,Y4,Z4)),  meshable=ON, 
	smoothClosedSpline=ON)
	#扫描成第二根纤维
	s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=100)
	s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(0.0, 0.15))         #确定扫描半径  
	e = p.edges
	Path = e.findAt(((X3,Y3,Z3),),((X4,Y4,Z4), ))
	p.SolidSweep(path=Path, sketchUpEdge=e[5], sketchOrientation=RIGHT, profile=s)


#开始循环
for i in range(1,50):                                         #生成纤维的数量
	createfiber()

#调整图像适合屏幕显示
session.viewports['Viewport: 1'].setValues(displayedObject=p)
session.viewports['Viewport: 1'].view.fitView()
