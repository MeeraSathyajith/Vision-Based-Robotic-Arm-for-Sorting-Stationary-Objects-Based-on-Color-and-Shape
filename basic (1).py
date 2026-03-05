from zmqRemoteApi import RemoteAPIClient
import math
import time
import cv2
import numpy as np

print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

sim.setArrayParameter(sim.arrayparam_gravity,[0,0,-9.81])

DOF = 4

executedMovId = 'notReady'
targetArm = '/Dobot'
objHandle = sim.getObject(targetArm)

stringSignalName = targetArm + '_executedMovId'
scriptHandle = sim.getScript(sim.scripttype_childscript,objHandle)

# ------------------------------------------------
# WAIT FUNCTION
# ------------------------------------------------

def waitForMovementExecuted(id_):
    global executedMovId
    while executedMovId != id_:
        executedMovId = sim.getStringSignal(stringSignalName)

# ------------------------------------------------
# MOVE FUNCTION
# ------------------------------------------------

def norm_diff(a,b):
    norm=0
    for i in range(len(a)):
        norm+=(a[i]-b[i])**2
    return math.sqrt(norm)

def moveToSet(motorHandles,target,enable):

    vel=20
    accel=40
    jerk=80

    movementData={
        'motorHandles':motorHandles,
        'maxVel':[vel*math.pi/180]*4,
        'maxAccel':[accel*math.pi/180]*4,
        'maxJerk':[jerk*math.pi/180]*4,
        'targetConf':[x*180/math.pi for x in target],
        'enable':enable
    }

    sim.callScriptFunction(
        'remoteApi_movementDataFunction',
        scriptHandle,
        movementData
    )

    while True:
        currentConf=list(map(lambda h:sim.getJointPosition(h),motorHandles))
        diff=norm_diff(currentConf,target)

        if diff<0.01:
            break

# ------------------------------------------------
# VISION FUNCTIONS
# ------------------------------------------------

def getVisionImage():

    img,res = sim.getVisionSensorImg(visionSensor)

    img = np.frombuffer(img,dtype=np.uint8)
    img = img.reshape(res[1],res[0],3)

    img = cv2.flip(img,0)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    return img

def detectShapeColor(frame):

    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    lower_red1=np.array([0,120,70])
    upper_red1=np.array([10,255,255])

    lower_red2=np.array([170,120,70])
    upper_red2=np.array([180,255,255])

    lower_blue=np.array([95,120,70])
    upper_blue=np.array([130,255,255])

    mask_red=cv2.inRange(hsv,lower_red1,upper_red1)+cv2.inRange(hsv,lower_red2,upper_red2)
    mask_blue=cv2.inRange(hsv,lower_blue,upper_blue)

    for mask,color_name in [(mask_red,"Red"),(mask_blue,"Blue")]:

        mask=cv2.GaussianBlur(mask,(5,5),0)

        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:

            area=cv2.contourArea(cnt)
            if area<1200:
                continue

            perimeter=cv2.arcLength(cnt,True)
            approx=cv2.approxPolyDP(cnt,0.02*perimeter,True)

            shape="Unknown"

            if len(approx)==4:
                x,y,w,h=cv2.boundingRect(approx)
                ratio=w/float(h)

                if 0.8<=ratio<=1.2:
                    shape="Square"
                else:
                    shape="Rectangle"

            else:
                circularity=4*math.pi*area/(perimeter*perimeter)

                if circularity>0.65:
                    shape="Circle"

            print("Detected:",color_name,shape)

            return color_name,shape

    return None,None

# ------------------------------------------------
# START SIMULATION
# ------------------------------------------------

sim.startSimulation()
waitForMovementExecuted('ready')

motorHandles=[]
for i in range(1,DOF+1):
    motorHandles.append(sim.getObject('./motor'+str(i)))

proximitySensor=sim.getObject('/Proximity_sensor')
conveyor=sim.getObject('/conveyor')
visionSensor=sim.getObject('/Vision_sensor')

# ------------------------------------------------
# IK SETUP
# ------------------------------------------------

class IK:
    sim=None
    ik=None

def setIK(tip,target):

    simTip=sim.getObject(tip)
    simTarget=sim.getObject(target)

    simJoints=[]

    for i in range(1,DOF+1):
        simJoints.append(sim.getObject('/Dobot/motor'+str(i)))

    simdata={'simTip':simTip,'simTarget':simTarget,'simJoints':simJoints}

    ikdata={'ikEnv':-1,'ikTarget':-1,'ikBase':-1,'ikJoints':[],'ikGroup':-1}

    ikdata['ikEnv'],ikdata['ikTarget'],ikdata['ikBase'],ikdata['ikJoints'],ikdata['ikGroup']=\
        sim.callScriptFunction('remoteApi_setIK',scriptHandle,simdata)

    ik=IK()
    ik.sim=simdata
    ik.ik=ikdata

    return ik

ikUp=setIK('/Dobot/suctionCup/connection','/ComparentCuboidUp')
ikPick=setIK('/Dobot/suctionCup/connection','/ComparentCuboid')

ikRedUp=setIK('/Dobot/suctionCup/connection','/Bin_Red_Up')
ikRed=setIK('/Dobot/suctionCup/connection','/Bin_Red')

ikBlueUp=setIK('/Dobot/suctionCup/connection','/Bin_Blue_Up')
ikBlue=setIK('/Dobot/suctionCup/connection','/Bin_Blue')

# ------------------------------------------------
# MAIN LOOP
# ------------------------------------------------

while True:

    sim.writeCustomTableData(conveyor,'__ctrl__',{'vel':0.02})

    print("Waiting for object...")

    detected=False

    while not detected:

        result,distance,point,objDetected,normal = sim.readProximitySensor(proximitySensor)

        if result==1:

            sim.writeCustomTableData(conveyor,'__ctrl__',{'vel':0})

            time.sleep(0.4)

            frame=getVisionImage()

            color,shape=detectShapeColor(frame)

            objPos=sim.getObjectPosition(objDetected,-1)

            sim.setObjectPosition(sim.getObject('/ComparentCuboid'),-1,objPos)

            upPos=[objPos[0],objPos[1],objPos[2]+0.10]

            sim.setObjectPosition(sim.getObject('/ComparentCuboidUp'),-1,upPos)

            detected=True

        time.sleep(0.005)

    # ---------------- PICK ----------------

    ikans=sim.callScriptFunction('remoteApi_solveIK',scriptHandle,ikUp.ik,ikUp.sim)
    moveToSet(motorHandles,ikans,False)

    ikans=sim.callScriptFunction('remoteApi_solveIK',scriptHandle,ikPick.ik,ikPick.sim)
    moveToSet(motorHandles,ikans,True)

    time.sleep(1)

    ikans=sim.callScriptFunction('remoteApi_solveIK',scriptHandle,ikUp.ik,ikUp.sim)
    moveToSet(motorHandles,ikans,True)

    # ---------------- PLACE ----------------

    if color=="Red":

        ikans=sim.callScriptFunction('remoteApi_solveIK',scriptHandle,ikRedUp.ik,ikRedUp.sim)
        moveToSet(motorHandles,ikans,True)

        ikans=sim.callScriptFunction('remoteApi_solveIK',scriptHandle,ikRed.ik,ikRed.sim)
        moveToSet(motorHandles,ikans,True)

    elif color=="Blue":

        ikans=sim.callScriptFunction('remoteApi_solveIK',scriptHandle,ikBlueUp.ik,ikBlueUp.sim)
        moveToSet(motorHandles,ikans,True)

        ikans=sim.callScriptFunction('remoteApi_solveIK',scriptHandle,ikBlue.ik,ikBlue.sim)
        moveToSet(motorHandles,ikans,True)

    time.sleep(0.5)

    moveToSet(motorHandles,ikans,False)

    time.sleep(0.5)

    # lift after drop
    if color=="Red":
        ikans=sim.callScriptFunction('remoteApi_solveIK',scriptHandle,ikRedUp.ik,ikRedUp.sim)
    else:
        ikans=sim.callScriptFunction('remoteApi_solveIK',scriptHandle,ikBlueUp.ik,ikBlueUp.sim)

    moveToSet(motorHandles,ikans,False)

    time.sleep(0.5)