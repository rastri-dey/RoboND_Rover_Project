import numpy as np


def decision_step(Rover):
	
	# Calculate number of X-pixels in front of Rover
	NumPixFront=0
	for i in range(len(Rover.y_pixel)):
		if (Rover.y_pixel[i]==0):
			NumPixFront=NumPixFront+1
	
	count=0
	if NumPixFront>0:		
		X_Pix_Front=np.zeros(NumPixFront)
		for i in range(len(Rover.y_pixel)):
			if (Rover.y_pixel[i]==0):
				X_Pix_Front[count]=Rover.x_pixel[i]
				count=count+1
		MaxX_InFront=np.max(X_Pix_Front)
	else:
		MaxX_InFront=0
	
	# Calculate number of LHS and RHS pixel count in Rover frame
	LeftCount = 0
	RightCount = 0
	for i in range(len(Rover.y_pixel)):
		if Rover.y_pixel[i]>=0:
			LeftCount=LeftCount+1
		else:
			RightCount=RightCount+1

	if (np.absolute(Rover.vel)>1):
		Rover.LeftSteerCount=0
		Rover.RightSteerCount=0
        
	if (Rover.vel>1):
		Rover.Reverse_Throttle_Count = 0

	LeftAngles=np.zeros(LeftCount)
	RightAngles=np.zeros(RightCount)
	CountL=0
	CountR=0
	for i in range(len(Rover.nav_angles)):
		if Rover.nav_angles[i]>=0:
			LeftAngles[CountL]=Rover.nav_angles[i] * 180/np.pi
			CountL=CountL+1
		else:
			RightAngles[CountR]=Rover.nav_angles[i]	* 180/np.pi
			CountR=CountR+1
	
	## Mapping of Ground Truth 	
	if Rover.count==1:
		for i in range(len(Rover.Dist)):
			Rover.Dist[i]= np.sqrt((Rover.pos[0]-Rover.CPX[i])**2 + (Rover.pos[1]-Rover.CPY[i])**2)
		Rover.MinDist = np.min(Rover.Dist)
		for i in range(len(Rover.Dist)):
			if (Rover.Dist[i]==Rover.MinDist):
				Rover.MinDistId = i

		if Rover.MinDistId<=2:
			Rover.SetFlag = Rover.MinDistId
		else:
			Rover.SetFlag = Rover.MinDistId + 1
		
	
	if (Rover.SetFlag<3):
		
		TgtPx = Rover.CPX[Rover.SetFlag]
		TgtPy = Rover.CPY[Rover.SetFlag]
		TgtDis = np.sqrt((TgtPx - Rover.pos[0])**2 + (TgtPy - Rover.pos[1])**2)
	elif (Rover.SetFlag==3):
		
		TgtPx = Rover.CPX[0]
		TgtPy = Rover.CPY[0]
		TgtDis = np.sqrt((TgtPx - Rover.pos[0])**2 + (TgtPy - Rover.pos[1])**2)
	else:
		
		TgtPx = Rover.CPX[Rover.SetFlag+1]
		TgtPy = Rover.CPY[Rover.SetFlag+1]
		TgtDis = np.sqrt((TgtPx - Rover.pos[0])**2 + (TgtPy - Rover.pos[1])**2)
	
	X=TgtPx-Rover.pos[0]
	Y=TgtPy-Rover.pos[1]
	
	if Y<0:
		TgtAng = (np.arctan(X/Y)* 180/np.pi) +180
	else:
		TgtAng = (np.arctan(X/Y)* 180/np.pi)
		
	if (TgtAng)>=0:
		YawReq = TgtAng - Rover.yaw
	else:
		YawReq = TgtAng + 360 - Rover.yaw
	
	# Calculation of Steer Required to cover unmapped area
	Kp = -0.1							#Proprtional Gain
	SteerReq = YawReq*Kp

	
	if ((Rover.SFCount>=1000) & (TgtDis<=3)) | (TgtDis>3):
		Rover.SFCount=0
	elif (Rover.SFCount==1)  & (TgtDis<=3):
		Rover.SetFlag= Rover.SetFlag+1
		Rover.SFCount=Rover.SFCount+1
	elif (Rover.SFCount<1000) & (TgtDis<=3):
		Rover.SFCount=Rover.SFCount+1
	
	
	##Lateral Control
	if Rover.vel>0.2:
		
		if (LeftCount-RightCount)>20:
			Rover.steer=6
			if (Rover.throttle < 0.2):
				Rover.throttle = 0.2
				Rover.brake = 0
		elif (LeftCount-RightCount)<=20:
			Rover.steer=-6
			if Rover.throttle < 0.2:
				Rover.throttle = 0.2
				Rover.brake = 0

		if (TgtDis<=3) & (Rover.ReversCount < 10):
			Rover.steer = -15
			Rover.throttle = 0
			Rover.brake = 0
			Rover.ReversCount = Rover.ReversCount+1
		elif(TgtDis>4):
			Rover.ReversCount = 0
				
	elif Rover.vel<=0.2:
		Rover.brake = 0
		if Rover.RightSteerCount<200:
			Rover.steer=10
			
			Rover.RightSteerCount=Rover.RightSteerCount+1
			if (len(Rover.x_pixel)>50) & (MaxX_InFront>40):
				Rover.throttle=0.4
			else:
				Rover.throttle=0
		elif Rover.LeftSteerCount<200 & Rover.RightSteerCount>200:
			Rover.steer = -15
			
			Rover.LeftSteerCount=Rover.LeftSteerCount+1
			if (len(Rover.x_pixel)>50) & (MaxX_InFront>40):
				Rover.throttle=0.4
			else:
				Rover.throttle=0
		elif Rover.RightSteerCount>=200:
			Rover.steer=0
			Rover.throttle=-1

	
    ##Reverse Turn
	if Rover.throttle<0:
		Rover.Reverse_Throttle_Count=Rover.Reverse_Throttle_Count+1

	if (Rover.vel<=0.2) & (Rover.throttle<0) & (Rover.Reverse_Throttle_Count>30):
		Rover.throttle = 0.4
	
	
	##Longitudinal Control
	if (Rover.vel>0.2) & (len(Rover.x_pixel)>50):
		
		if MaxX_InFront>50:
			Rover.throttle=0.35
			Rover.brake=0
			
			if ((np.absolute(np.clip(np.mean(Rover.nav_angles * 180/np.pi), -10, 15)))<10):
				Rover.steer=0
			else: 
				Rover.steer=np.clip((np.mean(Rover.nav_angles * 180/np.pi)/2), -10, 15)
			
		elif MaxX_InFront>40:
			Rover.throttle=0.2
			Rover.brake=0
			if ((np.absolute(np.clip(np.mean(Rover.nav_angles * 180/np.pi), -10, 15)))<10):
				Rover.steer=0
			else: 
				Rover.steer=np.clip((np.mean(Rover.nav_angles * 180/np.pi)/2), -10, 15)
			
		elif MaxX_InFront>30:
			Rover.throttle=0
			Rover.brake=0
			if ((np.absolute(np.clip(np.mean(Rover.nav_angles * 180/np.pi), -10, 15)))<10):
				Rover.steer=0
			else: 
				Rover.steer=np.clip(np.mean(Rover.nav_angles * 180/np.pi), -10, 15)
			
		elif MaxX_InFront>20:
			Rover.throttle=0
			Rover.brake=10
			Rover.steer=0
		elif MaxX_InFront<=20: 
			Rover.throttle=0
			Rover.brake=20
			Rover.steer=0
		
		if (np.absolute(np.mean(Rover.nav_angles * 180/np.pi))>30):
			
			if (RightCount > (LeftCount+20)):		
				Rover.steer=np.clip(np.mean(RightAngles * 180/np.pi), -15, 15)
			else:
				Rover.steer=np.clip(np.mean(LeftAngles * 180/np.pi), -15, 15)
		
			
	##Initialization
	if (Rover.count<=200) & (Rover.vel<=0.2):
		Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
		Rover.throttle = 0.4
		
    
    # If in a state where want to pickup a rock send pickup command
	if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
		Rover.send_pickup = True
    
	for i in range(len(Rover.Dist)):
		Rover.DistIn[i]=Rover.Dist[i]
	
	return Rover