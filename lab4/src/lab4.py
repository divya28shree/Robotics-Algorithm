#!/usr/bin/env python
import rosbag
import numpy
from tf.transformations import euler_from_quaternion
import os
 
beta = 10
alpha = 45

pos = numpy.zeros((35, 35, 4))
fileDir = os.path.dirname(os.path.realpath('__file__'))
def move():
	
	#print "hiiii"	
	pos[12-1,28-1,3-1] = 1 #initialize the first robot position
	obstruclePos = numpy.array([[125, 525],[125, 325],[125, 125],[425, 125],[425, 325],[425, 525]])
	filename = "grid.bag"
	bag = rosbag.Bag(filename,'r')
	for topic, msg, time_stamp in bag.read_messages(topics=['Movements', 'Observations']):
			#print "IN LOOP"
			if topic == 'Movements':
				motionModel(msg)
				#tryy(msg, pos)
				#break
				#numpy.savetxt('files'+str(ww)+'.txt', cur_arraypos, delimiter=',', fmt='%s')
				#print 'HH'
			else: 			
				obserModel(msg,obstruclePos)
				#break
	bag.close()
	
def tryy(msg, pos):
	rot1 = numpy.degrees((euler_from_quaternion([msg.rotation1.x,msg.rotation1.y,msg.rotation1.z,msg.rotation1.w]))[2])
	rot2 = numpy.degrees((euler_from_quaternion([msg.rotation2.x,msg.rotation2.y,msg.rotation2.z,msg.rotation2.w]))[2])
	trans = msg.translation*100 # all calculations in cm
	final_p = 0
	rot1Diff, transDiff, rot2Diff= requiredControl(1, 1, 1, 1, 1, 1)
	p1 = gaussian(trans_tmp,trans,10)
	p2 = gaussian(rot1_tmp, rot1, alpha)
	p3 = gaussian(rot2_tmp, rot2, alpha)
	print 'sssssssssssssssssssssssssssssssssssssssssssssssssss'
	print rot1Diff, transDiff, rot2Diff,p1 , p2, p3,trans, rot1, rot2
	p = trans_p*rot1_p*rot2_p
	print p
	

def motionModel(msg):
	global pos
	#numpy.savetxt('fileArr1.txt', pos, delimiter=',', fmt='%s')
	rot1 = numpy.degrees((euler_from_quaternion([msg.rotation1.x,msg.rotation1.y,msg.rotation1.z,msg.rotation1.w]))[2])
	rot2 = numpy.degrees((euler_from_quaternion([msg.rotation2.x,msg.rotation2.y,msg.rotation2.z,msg.rotation2.w]))[2])
	trans = msg.translation*100 # all calculations in cm
	summ=0
	pos1= pos
	for i in numpy.arange(pos.shape[0]):
		for j in numpy.arange(pos.shape[1]):
			for k in numpy.arange(pos.shape[2]):
				if pos[i, j, k] < 0.1:
					continue
				for i1 in numpy.arange(pos.shape[0]):
					for j1 in numpy.arange(pos.shape[1]):
						for k1 in numpy.arange(pos.shape[2]):
							rot1Diff, transDiff, rot2Diff = requiredControl(i,j,k,i1,j1,k1)
							p1 = gaussian(transDiff,trans,beta)
							p2 = gaussian(rot1Diff, rot1, alpha)
							p3 = gaussian(rot2Diff, rot2, alpha)
							p = p1*p2*p3
							pos[i1, j1, k1] = pos [i1, j1, k1] + (pos1[i, j, k]* p)
							summ=summ+(pos1[i, j, k]* p)
	pos = pos/summ 					
				
	
	

def requiredControl(i,j,k,i1,j1,k1):
	x,y,a = getPosition(i,j,k)
	x1, y1, a1 = getPosition(i1,j1,k1)
	transDiff = numpy.sqrt((x1-x)**2+ (y1-y)**2)
	rot1Diff = numpy.degrees(numpy.arctan2(y-y1, x - x1))-a
	rot2Diff = a1-a-rot1Diff
	return rot1Diff, transDiff, rot2Diff


def gaussian(val,m,sd):
	#print 'wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww'
	#print val, m, sd
	v1 = numpy.sqrt((2.0*numpy.pi)*sd)
	#print v1
	e = numpy.power(numpy.e,-1*(((val-m)**2.0)/(2.0*sd**2)))
	#print e
	if v1 == 0:
		return 0
	else:
		gaussianValue = (1.0/v1)*e
		return gaussianValue
	


def getPosition(i,j,k):
	x=i*20;
	y=j*20;
	a= (k*90)-135
	return x, y, a
	


def obserModel(msg,obstruclePos):
	global pos
	pos1 = pos
	w = numpy.degrees((euler_from_quaternion([msg.bearing.x, msg.bearing.y, msg.bearing.z, msg.bearing.w]))[2])
	dist = msg.range*100
	#print 'dist:', dist
	tag = msg.tagNum
	summ =0
	#numpy.savetxt('fil.txt', pos, delimiter=',', fmt='%s')
	for i in numpy.arange(pos.shape[0]):
		for j in numpy.arange(pos.shape[1]):
			for k in numpy.arange(pos.shape[2]):
				rotT, transT = calculateTagObs(i, j, k, tag, obstruclePos)
				#print 'trs', transT
				p1 = gaussian(rotT, w, alpha)
				p2 = gaussian(transT,dist,  beta)
				#print transT, dist, beta
				pos[i, j, k] = pos1[i, j, k] * p1 * p2
				#print p1, p2
				summ += pos[i, j, k] * p1 * p2
	pos = pos / summ #normalize the array
	maximum =numpy.amax(pos)
	#print numpy.amax(pos) # maximum prob
	q = numpy.where(pos==maximum)
	#print q[0].size
	#print q[0].data
	#print q[0]/10
	#qx = q[0] / 100
	#print(getPosition(q[0],q[1],q[2])/10)
	#print(getPosition(11,27,2))
	#print numpy.where(pos==max)
	x, y, a = getPosition(q[0],q[1],q[2])
	x=x*1.00
	x=x/100
	y=y*1.0
	y=y/100
	print x
	#qx=x/100
	#qy=y/100
	#print qx
	filename = "trajectory.txt"
	f = open(filename, "a")
	f.write('X:'+str(x)+'   Y:'+str(y))
	f.write("\n")
	
def calculateTagObs(i,j,k,tag,obstruclePos):
	x, y, r = getPosition(i,j,k)
	a = numpy.degrees(numpy.arctan2(obstruclePos[tag,1]-y, obstruclePos[tag,0] -x))	
	t = numpy.sqrt((x - obstruclePos[tag,0]) ** 2 + (y - obstruclePos[tag,1]) ** 2)
	r_t = a - r
	if r_t > 180:
		r_t -=180
	elif r_t <180:
		r_t +=180
	return r_t, t

if __name__=="__main__":

	move()
