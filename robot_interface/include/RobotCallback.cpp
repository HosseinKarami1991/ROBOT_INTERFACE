
#include "RobotCallback.hpp"

robotCallback::robotCallback(int nosphere,int nocylinder,int noplane,int nocone,int nounknown):NoSphere(nosphere),NoCylinder(nocylinder),NoPlane(noplane),NoCone(nocone),NoUnknown(nounknown){

//	RANSAC parameter Initialization
	NoObjects=NoSphere+NoCylinder+NoPlane;
	NoObstacles=NoSphere+NoCylinder;
	NoCorrectRecognizedObject=0;


EstimatedNoSphere=0;
EstimatedNoCylinder=0;
EstimatedNoPlane=0;
EstimatedNoCone=0;
EstimatedNoUnknown=0;
num_joint=7;

	objectFeatureBox=new float* [NoSphere+NoCylinder];
	objectFeatureBall=new float* [NoSphere+NoCylinder];
	goalID=100;

	for (int i=0;i<NoSphere+NoCylinder;i++){
		objectFeatureBox[i]=new float [6]; 	//! 6: 1-3: center ; 4-6: size
		objectFeatureBall[i]=new float [4];	//! center: x,y,z,radius
	}

	obstacleSafetyFactor=1.0;//1.5, 2.5
	obstacleSafetyFactorX=1.1;
	obstacleSafetyFactorY=3.5;
	obstacleSafetyFactorZ=2.0;

	for (int i=0;i<NoSphere+NoCylinder;i++)
		for (int j=0;j<6;j++)
			objectFeatureBox[i][j]=0;

	for (int i=0;i<NoSphere+NoCylinder;i++)
		for (int j=0;j<4;j++)
			objectFeatureBall[i][j]=0;
//	Path planner initialization:
	for(int i=0;i<6;i++) {
		regionGoal[0][i]=0;
		regionGoal[1][i]=0;
		regionGoal[2][i]=0;
		regionOperating[i]=0;
		initPosLeftArm[i]=0;
		initPosRightArm[i]=0;
		initPos[0][i]=0.0;
		initPos[1][i]=0.0;
		initPos[2][i]=0.0;
		tTo1_arr[i]=0.0;
		tTo2_arr[i]=0.0;
		ReachingPoint[0][i]=0.0;
		ReachingPoint[1][i]=0.0;
		ReachingPoint[2][i]=0.0;
		wTo_BiMan[i]=0.0;
		}

	for (int i=0;i<7;i++){
		init_q_[0][i]=0.0;
		init_q_[1][i]=0.0;
	}


//****************************
//	0:
//	cartPos right 1.5 0.0 3.1 0.75 -0.384 0.0
//	1:
//	cartPos right 1.5 0.0 3.1 0.75 -0.384 -0.13
//	2:
//	cartPos right 1.5 0.0 3.1 0.75 -0.384 -0.05
//	3:
//	cartPos left 0.0 -1.5 1.5 0.75 0.08 0.195
//	4:
//	cartPos right 3.092 -1.484 1.605 0.75 -0.04 0.21
//	5:
//	cartPos left 0.0 -1.5 1.5 0.75 0.01 0.195
//	6:
//	cartPos right 3.092 -1.484 1.605 0.7 -0.15 0.21
//	7:
//	cartPos left -1.5  0.00  -3.116 0.75 0.15 0.2
//	8:
//	cartPos left -1.5  0.00  -3.116 0.75 0.25 -0.11
//	----------------
//	initPos:
//	9:
//	jointPos right 0.240 -1.156 0.900 1.979 -0.407 0.911 2.135
//	cartPos right 1.568  -0.005  3.112  0.570  -0.185  0.091
//	cartPos right 1.568  -0.005  3.112  0.5  -0.43  -0.02
//	10:
//	jointPos left -0.111 -1.107 -1.067 1.913 0.472 1.018 -2.241
//	cartPos left -1.5  0.0  -3.116  0.578  0.191  0.118
//	cartPos left -1.5  0.0  -3.116  0.5  0.43  -0.02
//  --------------
	//	11: biman wTg
	//	0.0 0.0 0.0 0.7 -0.02 0.23
//	12: biman wTo
//	0.0 0.0 0.0 0.75 -0.02 0.20

	ReachingPointsVector[0][0]=1.5;		ReachingPointsVector[0][1]=0.0;		ReachingPointsVector[0][2]=3.1;
	ReachingPointsVector[0][3]=0.75;	ReachingPointsVector[0][4]=-0.384;	ReachingPointsVector[0][5]=0.0+0.15;

	ReachingPointsVector[1][0]=1.5;		ReachingPointsVector[1][1]=0.0;		ReachingPointsVector[1][2]=3.1;
	ReachingPointsVector[1][3]=0.75;	ReachingPointsVector[1][4]=-0.384;	ReachingPointsVector[1][5]=-0.13+0.15;

	ReachingPointsVector[2][0]=1.5;		ReachingPointsVector[2][1]=0.0;		ReachingPointsVector[2][2]=3.1;
	ReachingPointsVector[2][3]=0.75;	ReachingPointsVector[2][4]=-0.384;	ReachingPointsVector[2][5]=-0.05+0.15;

	ReachingPointsVector[3][0]=0.0;		ReachingPointsVector[3][1]=-1.5;	ReachingPointsVector[3][2]=1.5;
	ReachingPointsVector[3][3]=0.75;	ReachingPointsVector[3][4]=0.08;	ReachingPointsVector[3][5]=0.195+0.15;

	ReachingPointsVector[4][0]=3.092;	ReachingPointsVector[4][1]=-1.484;	ReachingPointsVector[4][2]= 1.605;
	ReachingPointsVector[4][3]=0.75;	ReachingPointsVector[4][4]=-0.04;	ReachingPointsVector[4][5]=0.21+0.15;

	ReachingPointsVector[5][0]=0.0;		ReachingPointsVector[5][1]=-1.5;	ReachingPointsVector[5][2]= 1.5;
	ReachingPointsVector[5][3]=0.75;	ReachingPointsVector[5][4]=0.01;	ReachingPointsVector[5][5]=0.195+0.15;

	ReachingPointsVector[6][0]= 3.09;	ReachingPointsVector[6][1]=-1.48;	ReachingPointsVector[6][2]= 1.605;
	ReachingPointsVector[6][3]=0.7;		ReachingPointsVector[6][4]=-0.15;	ReachingPointsVector[6][5]=0.21+0.15;

	ReachingPointsVector[7][0]=-1.5;	ReachingPointsVector[7][1]=0.00;	ReachingPointsVector[7][2]=-3.116;
	ReachingPointsVector[7][3]=0.75;	ReachingPointsVector[7][4]=0.15;		ReachingPointsVector[7][5]=0.20+0.15;

	ReachingPointsVector[8][0]=-1.5;	ReachingPointsVector[8][1]=0.0;		ReachingPointsVector[8][2]= 3.116;
	ReachingPointsVector[8][3]=0.75;	ReachingPointsVector[8][4]=0.25;	ReachingPointsVector[8][5]=-0.11+0.15;

	ReachingPointsVector[9][0]=1.568;	ReachingPointsVector[9][1]=-0.005;	ReachingPointsVector[9][2]= 3.112;
	ReachingPointsVector[9][3]=0.6;		ReachingPointsVector[9][4]=-0.20;	ReachingPointsVector[9][5]=-0.02+0.15;

	ReachingPointsVector[10][0]=-1.5;	ReachingPointsVector[10][1]=0.0;	ReachingPointsVector[10][2]= -3.116;
	ReachingPointsVector[10][3]=0.6;	ReachingPointsVector[10][4]=0.20;	ReachingPointsVector[10][5]=-0.02+0.15;

	ReachingPointsVector[11][0]=0.0;	ReachingPointsVector[11][1]=0.0;	ReachingPointsVector[11][2]= 0.0;
	ReachingPointsVector[11][3]=0.70;	ReachingPointsVector[11][4]= -0.02;	ReachingPointsVector[11][5]=0.23+0.15;

	ReachingPointsVector[12][0]=0.0;	ReachingPointsVector[12][1]=0.0;	ReachingPointsVector[12][2]= 0.0;
	ReachingPointsVector[12][3]=0.75;	ReachingPointsVector[12][4]= -0.02;	ReachingPointsVector[12][5]=0.20+0.15;


//*****************************

	regionOperating[0]=0.45;	regionOperating[1]=-0.01;		regionOperating[2]=0.045;	//! center
	regionOperating[3]=0.36;	regionOperating[4]=0.86+0.4;		regionOperating[5]=0.31;		//! size

	perception_regionOperating[0]=0.45;	perception_regionOperating[1]=-0.01;	perception_regionOperating[2]=0.045;	//! center
	perception_regionOperating[3]=0.36;	perception_regionOperating[4]=0.86;		perception_regionOperating[5]=0.31;		//! size



	regionGoal[0][0]=0.65;	regionGoal[0][1]=0.15;	regionGoal[0][2]= 0.19;	//! center
	regionGoal[0][3]=0.03;	regionGoal[0][4]=0.03;	regionGoal[0][5]=0.03;			//! size

	regionGoal[1][0]=0.78;	regionGoal[1][1]=-0.20;	regionGoal[1][2]= -0.19;	//! center
	regionGoal[1][3]=0.03;	regionGoal[1][4]=0.03;	regionGoal[1][5]=0.03;			//! size

	regionGoal[2][0]=0.84;	regionGoal[2][1]=-0.15;	regionGoal[2][2]= 0.24;//0.27	//! center
	regionGoal[2][3]=0.03;	regionGoal[2][4]=0.03;	regionGoal[2][5]=0.03;			//! size

	orientationGoal[0][0]=3.14; orientationGoal[0][1]=0.0; orientationGoal[0][2]=-3.14; // left:     yaw, pitch,roll
	orientationGoal[1][0]=3.14; orientationGoal[1][1]=0.0; orientationGoal[1][2]=-3.14; // right:    yaw, pitch,roll
	orientationGoal[2][0]=0.0; orientationGoal[2][1]=-0.65; orientationGoal[2][2]=0.0; // Bimanual: yaw, pitch,roll

//	controller parameters initialization:
	control_ack_flag[0]			=true;
	control_ack_flag[1]			=true;
	control_ack_flag[2]			=true;
	hri_control_goal_flag[0]	=true;
	hri_control_goal_flag[1]	=true;
	hri_control_goal_flag[2]	=true;
	rob_goal_reach_flag[0]		=true;
	rob_goal_reach_flag[1]		=true;
	rob_goal_reach_flag[2]		=true;
	obj_call_back_flag			=true;

	control_goal_reach_ack[0][0]=false;control_goal_reach_ack[1][0]=false;control_goal_reach_ack[2][0]=false;
	control_goal_reach_ack[0][1]=false;control_goal_reach_ack[1][1]=false;control_goal_reach_ack[2][1]=false;
	control_goal_reach_ack[0][2]=false;control_goal_reach_ack[1][2]=false;control_goal_reach_ack[2][2]=false;
	control_goal_reach_ack[0][3]=false;control_goal_reach_ack[1][3]=false;control_goal_reach_ack[2][3]=false;
	control_goal_reach_ack[0][4]=false;control_goal_reach_ack[1][4]=false;control_goal_reach_ack[2][4]=false;

	controlCmndSent_goalNotReached[0][0]=false;controlCmndSent_goalNotReached[1][0]=false;controlCmndSent_goalNotReached[2][0]=false;
	controlCmndSent_goalNotReached[0][1]=false;controlCmndSent_goalNotReached[1][1]=false;controlCmndSent_goalNotReached[2][1]=false;
	controlCmndSent_goalNotReached[0][2]=false;controlCmndSent_goalNotReached[1][2]=false;controlCmndSent_goalNotReached[2][2]=false;
	controlCmndSent_goalNotReached[0][3]=false;controlCmndSent_goalNotReached[1][3]=false;controlCmndSent_goalNotReached[2][3]=false;
	controlCmndSent_goalNotReached[0][4]=false;controlCmndSent_goalNotReached[1][4]=false;controlCmndSent_goalNotReached[2][4]=false;

	readArmPathFlag[0]=false;readArmPathFlag[1]=false;readArmPathFlag[2]=false;

//	ransacFlag=false;
	pathPlanningFlag=false;
	NopathPlanningFlag=false;
	simulationFlag=false;
	controllerFlag= false;
	directPathAllocationFalg=false;
	NO_ArmState=3;
	NO_ctrlCmdType=5;

	robotCommandType=Ecmnd::e_grasp;
	pointReachNumber=0;
	armsStateFlag_NotIdle[0]=false;armsStateFlag_NotIdle[1]=false;armsStateFlag_NotIdle[2]=false;
	armstateFlag_doAction[0]=false;armstateFlag_doAction[1]=false;armstateFlag_doAction[2]=false;
//	sub_shapes			=nh.subscribe("ransac_segmentation/trackedShapes",10, &robotCallback::CallBackShapes, this);
	sub_LeftArmwTt		=nh.subscribe("wTt_leftArm"	,10,&robotCallback::CallBackArmwTtLeft, 	this);
	sub_RightArmwTt		=nh.subscribe("wTt_rightArm",10,&robotCallback::CallBackArmwTtRight, 	this);
	sub_LeftArmQ		=nh.subscribe("Q_leftArm"	,10,&robotCallback::CallBackArm_Q_Left,		this);
	sub_RightArmQ		=nh.subscribe("Q_rightArm"	,10,&robotCallback::CallBackArm_Q_Right,	this);

	sub_robotCtrlAck	=nh.subscribe("robot_control_ack",80, &robotCallback::SubscribeControlAck, this);
	sub_arrivingCmnd	=nh.subscribe("robot_command",10, &robotCallback::arrivingCommands, this);

	pub_ctrl_task_param		=nh.advertise<controlCommnad_msgs::controlTaskParam>("ctrl_tasks_param",80);
	pub_hri_robot_ack		=nh.advertise<std_msgs::String>("robot_ack",100);
	knowledgeBase_client	=nh.serviceClient<knowledge_msgs::knowledgeSRV>("knowledgeService");
	publishControlCommand	=nh.advertise<controlCommnad_msgs::control>("robot_control_command",80);
	publishKBCommand		=nh.advertise<std_msgs::String>("robot_KB_command",80);


	simRobot_client = nh.serviceClient<simRobot_msgs::simulateRobotSRV>("robotSimulator_service");
	sub_arrivingSimulationCommand=nh.subscribe("simulation_command",10, &robotCallback::arrivingSimulationCommand, this);
	pub_simulationResponse=nh.advertise<robot_interface_msgs::SimulationResponseMsg>("simulation_response",80);

	waiting_time=30.0;
	SetAgentsList();
}
robotCallback::~robotCallback(){
	for(int i=0;i<NoSphere+NoCylinder;i++){
		delete [] objectFeatureBox[i];
		delete [] objectFeatureBall[i];
	}
	delete [] objectFeatureBox;
}

void robotCallback::CallBackArmwTtLeft(const geometry_msgs::Accel& msg){
	// output of interface is with this oreder: [yaw pitch roll x y z]
	initPos[0][0]=msg.angular.x;   	//yaw
	initPos[0][1]=msg.angular.y;	//pitch
	initPos[0][2]=msg.angular.z;	//roll
	initPos[0][3]=msg.linear.x;		//x
	initPos[0][4]=msg.linear.y;	 	//y
	initPos[0][5]=msg.linear.z;		//z
}
void robotCallback::CallBackArmwTtRight(const geometry_msgs::Accel& msg){
	// output of interface is with this oreder: [yaw pitch roll x y z]
	initPos[1][0]=msg.angular.x;
	initPos[1][1]=msg.angular.y;
	initPos[1][2]=msg.angular.z;
	initPos[1][3]=msg.linear.x;
	initPos[1][4]=msg.linear.y;
	initPos[1][5]=msg.linear.z;
}
void robotCallback::CallBackArm_Q_Left(const std_msgs::Float64MultiArray& msg){
	for (int i=0;i<7;i++)
		init_q_[0][i]=msg.data[i];
}

void robotCallback::CallBackArm_Q_Right(const std_msgs::Float64MultiArray& msg){
	for (int i=0;i<7;i++)
		init_q_[1][i]=msg.data[i];
}



void robotCallback::pathInitialPoint(const int arm_state_no){
if (arm_state_no==0){


}
if (arm_state_no==1){


}
if (arm_state_no==2){


}



}
/*
void robotCallback::CallBackShapes(const TrackedShapes& outShapes){
	TrackedShape::Ptr outShape ( new TrackedShape);
	int obj_counter=0; //! No of objects that ransac recognize and is not unknown

	NoCorrectRecognizedObject=0;
	if (obj_call_back_flag==true){
		EstimatedNoSphere=0;
		EstimatedNoCylinder=0;
		EstimatedNoPlane=0;
		EstimatedNoCone=0;
		EstimatedNoUnknown=0;

		objectsVector.clear();
		bool flag_isObjectInWS=true;


		for (int i=0;i<outShapes.tracked_shapes.size();i++){
			//		cout<<outShapes.tracked_shapes[i].shape_tag<<endl;

			// if the recognized objects are in the working space add to object list otherwise do not add.
			flag_isObjectInWS=true;
			if(	outShapes.tracked_shapes[i].x_pc_centroid> (perception_regionOperating[0]+perception_regionOperating[3]/2.0) ||
				outShapes.tracked_shapes[i].x_pc_centroid< (perception_regionOperating[0]-perception_regionOperating[3]/2.0) ||

				outShapes.tracked_shapes[i].y_pc_centroid> (perception_regionOperating[1]+perception_regionOperating[4]/2.0) ||
				outShapes.tracked_shapes[i].y_pc_centroid< (perception_regionOperating[1]-perception_regionOperating[4]/2.0) ||

				outShapes.tracked_shapes[i].z_pc_centroid> (perception_regionOperating[2]+perception_regionOperating[5]/2.0) ||
				outShapes.tracked_shapes[i].z_pc_centroid< (perception_regionOperating[2]-perception_regionOperating[5]/2.0) ){

				flag_isObjectInWS=false;
			}
			;
			if(flag_isObjectInWS==true){

				if (outShapes.tracked_shapes[i].shape_tag=="sphere"){
					int objID=outShapes.tracked_shapes[i].object_id;
					TrackedShape tracked_Shape=outShapes.tracked_shapes[i];
					objectsVector.emplace_back(make_shared <pittObjects::Sphere>(objID,tracked_Shape));
					EstimatedNoSphere++;
					//			objectsVector2.push_back(new pittObjects::Sphere(objID,tracked_Shape)); // as am example keep here this one.
					float vecBall6[6],vecBox6[6];
					if (!objectsVector.empty()){
						objectsVector.back()->BoundingBox(vecBox6);
						objectsVector.back()->BoundingBall(vecBall6);
						objectsVector.back()->FrameSet();

					}
					//
					//			boundBoxSphere(outShapes.tracked_shapes[i],obj_counter);
					//			boundBallSphere(outShapes.tracked_shapes[i],obj_counter);
					//			goalCenterSet(outShapes.tracked_shapes[i],obj_counter);
					//			NoCorrectRecognizedObject++;
				}
				//		if (outShapes.tracked_shapes[i].shape_tag=="cylinder")
				//			boundBoxCylinder(outShapes.tracked_shapes[i],obj_counter);

				if (outShapes.tracked_shapes[i].shape_tag=="plane"){
					int objID=outShapes.tracked_shapes[i].object_id;
					TrackedShape tracked_Shape=outShapes.tracked_shapes[i];
					objectsVector.emplace_back(make_shared <pittObjects::Plane>(objID,tracked_Shape));
					EstimatedNoPlane++;
					//			planeFrameSet(outShapes.tracked_shapes[i],100);
					//			NoCorrectRecognizedObject++;
				}

				if (outShapes.tracked_shapes[i].shape_tag=="cylinder"){
					int objID=outShapes.tracked_shapes[i].object_id;
					TrackedShape tracked_Shape=outShapes.tracked_shapes[i];
					objectsVector.emplace_back(make_shared <pittObjects::Cylinder>(objID,tracked_Shape));
					EstimatedNoCylinder++;
				}

				if (outShapes.tracked_shapes[i].shape_tag=="cone"){
					int objID=outShapes.tracked_shapes[i].object_id;
					TrackedShape tracked_Shape=outShapes.tracked_shapes[i];
					objectsVector.emplace_back(make_shared <pittObjects::Cone>(objID,tracked_Shape));
					EstimatedNoCone++;
				}

				if (outShapes.tracked_shapes[i].shape_tag=="unknown"){
					int objID=outShapes.tracked_shapes[i].object_id;
					TrackedShape tracked_Shape=outShapes.tracked_shapes[i];
					objectsVector.emplace_back(make_shared <pittObjects::Unknown>(objID,tracked_Shape));
					EstimatedNoUnknown++;
				}

				if (outShapes.tracked_shapes[i].shape_tag=="sphere")
					obj_counter++;
			}
		}
		if (NoSphere==EstimatedNoSphere && NoPlane==EstimatedNoPlane && NoCylinder==EstimatedNoCylinder
				&& NoCone==EstimatedNoCone && NoUnknown==EstimatedNoUnknown){
			cout<<"*** Scene Recognition Is Correct ***"<<endl;
			cout<<"objectsVector.size: "<<objectsVector.size()<<endl;
			NoObstacles=objectsVector.size();

			//		cout<<"Normal Vector"<<endl;
			//		for (int i=0;i<objectsVector2.size();i++){
			//			cout<<objectsVector2[i]<<endl;
			//			objectsVector2[i]->Print();
			//		}
			//		cout<<"=================================="<<"Shared_ptr Vector"<<endl;
			float graspPose[6],pathPlanningPose[6];

			for (int i=0;i<objectsVector.size();i++){
				cout<<"*****"<<objectsVector[i]<<"*****"<<endl;
				objectsVector[i]->Print();
				objectsVector[i]->FrameSet();
				objectsVector[i]->GraspingPosition(graspPose,pathPlanningPose,"top");

			}
			obj_call_back_flag=false;
		}
		else{
			cout<<">>>>>> Scene Recognition Is 'NOT' Correct <<<<<<"<<endl;
			cout<<"Normal Vector"<<endl;
			//		cout<<"objectsVector2.size(): "<<objectsVector2.size()<<endl;
			//		for(vector<pittObjects::Objects *>::iterator it = objectsVector2.begin(); it != objectsVector2.end(); ++it) {
			//		  delete (*it);
			//		}
			//		objectsVector2.clear();
			//		cout<<"objectsVector2.size(): "<<objectsVector2.size()<<endl;
			//
			//		cout<<"=================================="<<"Shared_ptr Vector"<<endl;
			cout<<"objectsVector2.size(): "<<objectsVector.size()<<endl;
			objectsVector.clear();
			cout<<"objectsVector.size(): "<<objectsVector.size()<<endl;
		}

	}
	//	cout<<"===================="<<endl;
}*/
void robotCallback::goalCenterSet(const TrackedShape& outShape,const int index){ //for grasping objects
	if (outShape.coefficients[3]<0.05){
//		goalID=index;
//	regionGoal[0]=outShape.x_est_centroid;
//	regionGoal[1]=outShape.y_est_centroid;
//	regionGoal[2]=outShape.z_est_centroid;
	}
}
void robotCallback::planeFrameSet(const TrackedShape& outShape, const int index){
//	Zp=Zw,
	cout<<"planeFrameSet"<<endl;
	cout<<outShape.x_pc_centroid<<" "<<outShape.y_pc_centroid<<" "<<outShape.z_pc_centroid<<endl;
	cout<<outShape.coefficients[0]<<" "<<outShape.coefficients[1]<<" "<<outShape.coefficients[2]<<" "<<outShape.coefficients[3]<<" "<<endl;

	goalID=index;
	float teta;
	float normalVec[3];
	float worldvector[3];
	float planeSize[2];
	normalVec[0]=outShape.coefficients[0];//a
	normalVec[1]=outShape.coefficients[1];//b
	normalVec[2]=outShape.coefficients[2];//c
	worldvector[0]=0.0;
	worldvector[1]=0.0;
	worldvector[2]=1.0;

	float cos_teta=normalVec[0]/(sqrt(normalVec[0]*normalVec[0]+normalVec[1]*normalVec[1]+normalVec[2]*normalVec[2]));
	if (normalVec[1]>0.0)
		teta=acos(cos_teta);
	else
		teta=-1.0*acos(cos_teta);

	initPos[2][0]=teta;//+ 3.1415;   	//yaw
	initPos[2][1]=0.0;	//pitch
	initPos[2][2]=0.0;	//roll
	initPos[2][3]=outShape.x_pc_centroid;		//x
	initPos[2][4]=outShape.y_pc_centroid;	 	//y
	initPos[2][5]=outShape.z_pc_centroid-0.02;		//z

	planeSize[0]=0.2;//cm in Yp direction
	planeSize[1]=0.1;//cm in Yp direction
	cout<<"teta: "<<teta<<endl;
}

void robotCallback::biManualControlCmndParameters(void){
/*
	CMAT::TransfMatrix wTt1,wTt2,wTo ;
	CMAT::Vect6 r_tool, l_tool, objPose;
	CMAT::TransfMatrix tTo1, tTo2, wTg,tTo2_2;

	for (int i = 0; i < 6; ++i) {
		l_tool(i+1) = initPos[0][i];
		r_tool(i+1) = initPos[1][i];
		objPose(i+1) = initPos[2][i];
	}

	wTt1 = l_tool.Vect2TmatrixEsa();
	wTt2 = r_tool.Vect2TmatrixEsa();
	wTo = objPose.Vect2TmatrixEsa();
//	wTo.SetRotMatrix(CMAT::Matrix::Eye(3));

	tTo1 = wTt1.Inverse() * wTo;
	tTo2 = wTt2.Inverse() * wTo;
	tTo2_2=wTt2.Inverse() *wTt1*tTo1 ;

	tTo1.Tmatrix2VectEsa().CopyTo(tTo1_arr);
	tTo2.Tmatrix2VectEsa().CopyTo(tTo2_arr);

	l_tool.Transpose().PrintMtx("l_tool");
	r_tool.Transpose().PrintMtx("r_tool");

	wTt1.PrintMtx("wTt1");
	wTt2.PrintMtx("wTt2");
	wTo.PrintMtx("wTo");
	tTo1.PrintMtx("tTo1");
	tTo2.PrintMtx("tTo2");
	tTo2_2.PrintMtx("tTo2_2");
	*/
}

void robotCallback::publishControlTasksParam(void){
	ControlTaskParameters.obstFrames.resize(1);
	ControlTaskParameters.obstHeights.resize(1);
	cout<<"publishControlTasksParam"<<endl;

//	for (int i=0;i<NoObstacles;i++){
//		if (objectFeatureBall[i][1]>=objectFeatureBall[0][1] && objectFeatureBall[i][1]>=objectFeatureBall[1][1] ){
		ControlTaskParameters.obstFrames[0].cartesianPosition[0]=0.0;
		ControlTaskParameters.obstFrames[0].cartesianPosition[1]=1.0;
		ControlTaskParameters.obstFrames[0].cartesianPosition[2]=0.0;
		ControlTaskParameters.obstFrames[0].cartesianPosition[3]=0.65;//0.65
		ControlTaskParameters.obstFrames[0].cartesianPosition[4]=0.0;
		ControlTaskParameters.obstFrames[0].cartesianPosition[5]=0.0;
		ControlTaskParameters.obstHeights[0]=0.05;

		cout<<"objectFeatureBall[i][0]: "<<objectFeatureBall[0][0]<<endl;
		cout<<"cartesianPosition[3]: "<<ControlTaskParameters.obstFrames[0].cartesianPosition[3]<<endl;
//		ControlTaskParameters.obstHeights.push_back()
//	}
//}
	pub_ctrl_task_param.publish(ControlTaskParameters);
	ROS_INFO("Publishing Control Task Parameters...");
//	PublishRobotAck(2,0);
};

void robotCallback::boundBoxSphere( const TrackedShape& outShape,const int index){
	cout<<"boundBoxSphere"<<endl;
	cout<<outShape.x_est_centroid<<" "<<outShape.y_est_centroid<<" "<<outShape.z_est_centroid<<endl;
	cout<<outShape.coefficients[0]<<" "<<outShape.coefficients[1]<<" "<<outShape.coefficients[2]<<" "<<outShape.coefficients[3]<<" "<<endl;

	objectFeatureBox[index][0]=outShape.x_est_centroid; //! center_x
	objectFeatureBox[index][1]=outShape.y_est_centroid;//! center_y
	objectFeatureBox[index][2]=outShape.z_est_centroid;//! center_z
	objectFeatureBox[index][3]=outShape.coefficients[3]*2.0*obstacleSafetyFactorX;//! size_x,
	objectFeatureBox[index][4]=outShape.coefficients[3]*2.0*obstacleSafetyFactorY;//! size_y
	objectFeatureBox[index][5]=outShape.coefficients[3]*2.0*obstacleSafetyFactorZ;//! size_z
//! outShape.coefficients[3] is the radius of the sphere, for the path planner we need the size of the box
}

void robotCallback::boundBoxCylinder( const TrackedShape& outShape,const int index){
	cout<<"boundBoxCylinder"<<endl;
	cout<<outShape.x_est_centroid<<" "<<outShape.y_est_centroid<<" "<<outShape.z_est_centroid<<endl;
	cout<<outShape.coefficients[0]<<" "<<outShape.coefficients[1]<<" "<<outShape.coefficients[2]<<endl;
	cout<<outShape.coefficients[3]<<" "<<outShape.coefficients[4]<<" "<<outShape.coefficients[5]<<" "<<endl;
	cout<<outShape.coefficients[6]<<endl;
//  assumption (simplification): we consider that the main axis of the cylinder is in parallel to z-axis (later also consider x,y direction);

	objectFeatureBox[index][0]=outShape.x_est_centroid; //! center_x
	objectFeatureBox[index][1]=outShape.y_est_centroid;//! center_y
	objectFeatureBox[index][2]=outShape.z_est_centroid;//! center_z
//	objectFeatureBox[index][3]=outShape.coefficients[3]*2.0;//! size_x,
//	objectFeatureBox[index][4]=outShape.coefficients[3]*2.0;//! size_x
//	objectFeatureBox[index][5]=outShape.coefficients[3]*2.0;//! size_x
//! outShape.coefficients[3] is the radius of the sphere, for the path planner we need the size of the box

}
void robotCallback::boundBallSphere(const TrackedShape& outShape,const int index){
	cout<<"boundBallSphere"<<endl;
	objectFeatureBall[index][0]=outShape.x_est_centroid-0.2; //! center_x
	objectFeatureBall[index][1]=outShape.y_est_centroid+0.2;//! center_y
	objectFeatureBall[index][2]=outShape.z_est_centroid-outShape.coefficients[3];//! center_z
	objectFeatureBall[index][3]=outShape.coefficients[3]*2.0*obstacleSafetyFactor;//! size_x,

}
void robotCallback::boundBallCylinder(const TrackedShape& outShape,const int index){



}

void robotCallback::SubscribeControlAck(const controlCommnad_msgs::controlGoalReachAck& msg) {

	ROS_INFO("I heard Control Ack: arm:%d, cmndType: %d",msg.armState,msg.ctrlCmndTypeAck);

	int arm_index=msg.armState;
	int ctrlcmndType=msg.ctrlCmndTypeAck;
	if(arm_index<NO_ArmState && ctrlcmndType<NO_ctrlCmdType)
	{
		control_goal_reach_ack[arm_index][ctrlcmndType]=true;
		agents_list[arm_index].isBusy=false;
		agents_list[arm_index].isActionSuccessfullyDone=true;

		if(agents_list[arm_index].emergencyFlag==false)
			PublishRobotAck(agents_list[arm_index]);
		else
			agents_list[arm_index].emergencyFlag=false;

	}
	else
		cout<<"robotCallback::ControlAckCallBack==> Error in size"<<endl;


}

void robotCallback::arrivingSimulationCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::arrivingSimulationCommand"))<<endl;
	// arrive the simulation command here
	// base on the arriving command call different functions
	cout<<msg.ActionName<<" ";
	for(int i=0;i<msg.ResponsibleAgents.size();i++)
		cout<<msg.ResponsibleAgents[i]<<" ";
	for(int i=0;i<msg.ColleagueAgents.size();i++)
		cout<<msg.ColleagueAgents[i]<<" ";
	for(int i=0;i<msg.ActionParametersName.size();i++)
		cout<<msg.ActionParametersName[i]<<" ";
	cout<<endl;
	cout<<"Arm joint values:"<<endl;
	for(int i=0;i<msg.ArmsJoint.size();i++)
	{
		for(int j=0;j<7;j++)
			cout<<msg.ArmsJoint[i].values[j]<<" ";
		cout<<endl;
	}

	string tempActionName=msg.ActionName;
	if(tempActionName=="Grasp"|| tempActionName=="UnGrasp")
		SimulateGraspingCommand(msg);
	else if(tempActionName=="Stop")
		SimulateStoppingCommand(msg);
	else if(tempActionName=="HoldOn")
		SimulateHoldingCommand(msg);
	else if(tempActionName=="Approach")
		SimulateApproachingCommand(msg);
	else if(tempActionName=="UpdateJointValues")
		SimulateUpdateJointValues(msg);
	else if(tempActionName=="Transport")
		SimulateTransportingCommand(msg);
	else if(tempActionName=="Rest")
		SimulateRestingcommand(msg);
	else if(tempActionName=="Screw")
		SimulateScrewingCommand(msg);
	else if(tempActionName=="Unscrew")
		SimulateUnscrewingCommand(msg);
	else if(tempActionName=="Reduce")
		SimulateKB_ReductionCommand(msg);
	else
	{
		cout<<"The arriving msg name is wrong: "<<tempActionName <<endl;
		exit(1);
	}
}

void robotCallback::SimulateUpdateJointValues(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateUpdateJointValues"))<<endl;
	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=true;
	tempResponseMsg.time=0.0;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;

	robot_interface_msgs::Joints tempJoint;
	for (int i=0;i<msg.ResponsibleAgents.size();i++)
	{
		if(msg.ResponsibleAgents[i]=="LeftArm")
		{
			for(int i=0;i<7;i++)
				tempJoint.values.push_back(init_q_[0][i]);
			tempResponseMsg.ArmsJoint.push_back(tempJoint);
		}
		else if(msg.ResponsibleAgents[i]=="RightArm")
		{
			for(int i=0;i<7;i++)
				tempJoint.values.push_back(init_q_[1][i]);
			tempResponseMsg.ArmsJoint.push_back(tempJoint);
		}
		else
		{
			cout<<"Error in arriving Msg, agent name: "<<msg.ResponsibleAgents[i]<<endl;
		}
	}

	pub_simulationResponse.publish(tempResponseMsg);
}

void robotCallback::SimulateGraspingCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateGraspingCommand"))<<endl;
	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=1;
	tempResponseMsg.time=0.8;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;

	robot_interface_msgs::Joints tempJoint;
	for(int i=0;i<7;i++)
	tempJoint.values.push_back(msg.ArmsJoint[0].values[i]);
	tempResponseMsg.ArmsJoint.push_back(tempJoint);

	tempJoint.values.clear();
	for(int i=0;i<7;i++)
	tempJoint.values.push_back(msg.ArmsJoint[1].values[i]);
	tempResponseMsg.ArmsJoint.push_back(tempJoint);

	pub_simulationResponse.publish(tempResponseMsg);

};
void robotCallback::SimulateHoldingCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateHoldingCommand"))<<endl;
	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=true;
	tempResponseMsg.time=0.01;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;

	robot_interface_msgs::Joints tempJoint;
	for(int i=0;i<7;i++)
	tempJoint.values.push_back(msg.ArmsJoint[0].values[i]);
	tempResponseMsg.ArmsJoint.push_back(tempJoint);

	tempJoint.values.clear();
	for(int i=0;i<7;i++)
	tempJoint.values.push_back(msg.ArmsJoint[1].values[i]);
	tempResponseMsg.ArmsJoint.push_back(tempJoint);

	pub_simulationResponse.publish(tempResponseMsg);

};
void robotCallback::SimulateStoppingCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateStoppingCommand"))<<endl;
	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=true;
	tempResponseMsg.time=0.01;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;

	robot_interface_msgs::Joints tempJoint;
	for(int i=0;i<7;i++)
	tempJoint.values.push_back(msg.ArmsJoint[0].values[i]);
	tempResponseMsg.ArmsJoint.push_back(tempJoint);

	tempJoint.values.clear();
	for(int i=0;i<7;i++)
	tempJoint.values.push_back(msg.ArmsJoint[1].values[i]);
	tempResponseMsg.ArmsJoint.push_back(tempJoint);

	pub_simulationResponse.publish(tempResponseMsg);

};

void robotCallback::SimulateRestingcommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateRestingcommand"))<<endl;
	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=true;
	tempResponseMsg.time=5.0;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	vector<string> ResponsibleAgents;

	for(int i=0;i<msg.ResponsibleAgents.size();i++)
		ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);

	//! call the knowledge base
	vector<float> goalPose; int goalSize;
	knowledge_msgs::knowledgeSRV knowledge_msg;

	if(ResponsibleAgents.size()==1)
	{
		if(ResponsibleAgents[0]=="RightArm")
			knowledge_msg.request.reqType="Point_RestingRight";
		else if(ResponsibleAgents[0]=="LeftArm")
			knowledge_msg.request.reqType="Point_RestingLeft";
		else
		{
			cout<<"Error in name of coming responsible agent "<<endl;
			exit(1);
		}
	}
	else
	{
		cout<<"Error in Resting Command Responsible Agent size"<<endl;
		exit(1);
	}


	knowledge_msg.request.Name="";
	knowledge_msg.request.requestInfo="Pose";

	if(knowledgeBase_client.call(knowledge_msg)){
		goalSize=knowledge_msg.response.pose.size();
		if(goalSize==7)
		{}
		else
		{
			cout<<"Error in number of knowledge base size"<<endl;
			exit(1);
		}
		for (int i=0;i<goalSize;i++){
			goalPose.push_back(knowledge_msg.response.pose[i]);
		}
	}


	robot_interface_msgs::Joints tempJoint;
	if(ResponsibleAgents[0]=="RightArm")
	{
		for(int i=0;i<7;i++)
			tempJoint.values.push_back(msg.ArmsJoint[0].values[i]);
		tempResponseMsg.ArmsJoint.push_back(tempJoint);
		tempJoint.values.clear();
		for(int i=0;i<7;i++)
		tempJoint.values.push_back(goalPose[i]);
		tempResponseMsg.ArmsJoint.push_back(tempJoint);
	}
	else// "LeftArm"
	{
		for(int i=0;i<7;i++)
			tempJoint.values.push_back(goalPose[i]);
		tempResponseMsg.ArmsJoint.push_back(tempJoint);

		tempJoint.values.clear();

		for(int i=0;i<7;i++)
			tempJoint.values.push_back(msg.ArmsJoint[1].values[i]);
		tempResponseMsg.ArmsJoint.push_back(tempJoint);
	}

	pub_simulationResponse.publish(tempResponseMsg);

};


void robotCallback::SimulateScrewingCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateScrewingCommand"))<<endl;
	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=true;
	tempResponseMsg.time=5.0;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	vector<string> ResponsibleAgents;

	for(int i=0;i<msg.ResponsibleAgents.size();i++)
		ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);

	//! call the knowledge base
	vector<float> goalPose; int goalSize;
	knowledge_msgs::knowledgeSRV knowledge_msg;

	if(ResponsibleAgents.size()==1)
	{
		if(ResponsibleAgents[0]=="RightArm" || ResponsibleAgents[0]=="LeftArm")
		{}
		else
		{
			cout<<"Error in name of coming responsible agent "<<endl;
			exit(1);
		}
	}
	else
	{
		cout<<"Error in Resting Command Responsible Agent size"<<endl;
		exit(1);
	}


	robot_interface_msgs::Joints tempJoint;
	if(ResponsibleAgents[0]=="RightArm")
	{
		for(int i=0;i<7;i++)
			tempJoint.values.push_back(msg.ArmsJoint[0].values[i]);
		tempResponseMsg.ArmsJoint.push_back(tempJoint);
		tempJoint.values.clear();
		for(int i=0;i<7;i++)
		tempJoint.values.push_back(msg.ArmsJoint[1].values[i]);
		tempJoint.values[6]=3.0;
		tempResponseMsg.ArmsJoint.push_back(tempJoint);
	}
	else// "LeftArm"
	{
		for(int i=0;i<7;i++)
			tempJoint.values.push_back(msg.ArmsJoint[0].values[i]);
		tempJoint.values[6]=3.0;
		tempResponseMsg.ArmsJoint.push_back(tempJoint);

		tempJoint.values.clear();
		for(int i=0;i<7;i++)
			tempJoint.values.push_back(msg.ArmsJoint[1].values[i]);
		tempResponseMsg.ArmsJoint.push_back(tempJoint);
	}

	pub_simulationResponse.publish(tempResponseMsg);
};
void  robotCallback::SimulateUnscrewingCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateScrewingCommand"))<<endl;
	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=true;
	tempResponseMsg.time=5.0;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	vector<string> ResponsibleAgents;

	for(int i=0;i<msg.ResponsibleAgents.size();i++)
		ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);

	//! call the knowledge base
	vector<float> goalPose; int goalSize;
	knowledge_msgs::knowledgeSRV knowledge_msg;

	if(ResponsibleAgents.size()==1)
	{
		if(ResponsibleAgents[0]=="RightArm" || ResponsibleAgents[0]=="LeftArm")
		{}
		else
		{
			cout<<"Error in name of coming responsible agent "<<endl;
			exit(1);
		}
	}
	else
	{
		cout<<"Error in Resting Command Responsible Agent size"<<endl;
		exit(1);
	}


	robot_interface_msgs::Joints tempJoint;
	if(ResponsibleAgents[0]=="RightArm")
	{
		for(int i=0;i<7;i++)
			tempJoint.values.push_back(msg.ArmsJoint[0].values[i]);
		tempResponseMsg.ArmsJoint.push_back(tempJoint);
		tempJoint.values.clear();
		for(int i=0;i<7;i++)
		tempJoint.values.push_back(msg.ArmsJoint[1].values[i]);
		tempJoint.values[6]=-3.0;
		tempResponseMsg.ArmsJoint.push_back(tempJoint);
	}
	else// "LeftArm"
	{
		for(int i=0;i<7;i++)
			tempJoint.values.push_back(msg.ArmsJoint[0].values[i]);
		tempJoint.values[6]=-3.0;
		tempResponseMsg.ArmsJoint.push_back(tempJoint);

		tempJoint.values.clear();
		for(int i=0;i<7;i++)
			tempJoint.values.push_back(msg.ArmsJoint[1].values[i]);
		tempResponseMsg.ArmsJoint.push_back(tempJoint);
	}

	pub_simulationResponse.publish(tempResponseMsg);
};


void robotCallback::SimulateKB_ReductionCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateKB_ReductionCommand"))<<endl;
	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=true;
	tempResponseMsg.time=0.0;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;

	robot_interface_msgs::Joints tempJoint;
	for(int i=0;i<7;i++)
		tempJoint.values.push_back(msg.ArmsJoint[0].values[i]);
	tempResponseMsg.ArmsJoint.push_back(tempJoint);
	tempJoint.values.clear();
	for(int i=0;i<7;i++)
		tempJoint.values.push_back(msg.ArmsJoint[1].values[i]);
	tempResponseMsg.ArmsJoint.push_back(tempJoint);

	pub_simulationResponse.publish(tempResponseMsg);

};

void robotCallback::SimulateApproachingCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateApproachingCommand"))<<endl;
	if(msg.ResponsibleAgents.size()==1)
		SimulateApproachingCommandSingleArm(msg);
	else if (msg.ResponsibleAgents.size()==2)
		SimulateApproachingCommandJointArms(msg);
	else
		cout<<"Error in agent number"<<endl;
};

void robotCallback::SimulateApproachingCommandSingleArm(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateApproachingCommandSingleArm"))<<endl;

	// check the input msg and call the knowledge base for information we need for simulation
	//call the simulation service, wait for response and publish the result to the controller
	vector<string> msgAction;
	int agentNumber;

	if(msg.ResponsibleAgents.size()==1)
	{
		if(msg.ResponsibleAgents[0]=="LeftArm")
			agentNumber=0;
		else if(msg.ResponsibleAgents[0]=="RightArm")
			agentNumber=1;
		else
			cout<<"Error in agent name: "<<msg.ResponsibleAgents[0]<<endl;
	}
	else
	{	cout<<"The agent sizes is not defined in this function!"<<endl;
		return;
	}
	if(msg.ActionParametersName.size()>1 || msg.ActionParameterInfo.size()>1)
	{
		cout<<"Error in single approaching command, the parameter of the actions is more than one: "<<msg.ActionParametersName.size()<<msg.ActionParameterInfo.size()<<endl;
	}

	string actionParameters= msg.ActionParametersName[0]; //Approach [-]: point5, Object1-graspingPose1, Object4-XPose3
	vector<string> actionParametersVec;
	boost::split(actionParametersVec,actionParameters, boost::is_any_of("-"));// point5 [ Approach_point5] Cylinder1-graspingPose1 [ Approach_Cylinder1-graspingPose1]

	vector<float> goalPose;
	int goalSize;


	//! call the knowledge base
	knowledge_msgs::knowledgeSRV knowledge_msg;

	knowledge_msg.request.reqType=actionParameters;
//	if(actionParametersVec.size()>1)
//		knowledge_msg.request.Name=actionParametersVec[1];
//	else
	knowledge_msg.request.Name="";

	knowledge_msg.request.requestInfo=msg.ActionParameterInfo[0];// Pose, ...

	if(knowledgeBase_client.call(knowledge_msg))
	{
		goalSize=knowledge_msg.response.pose.size();

		for (int i=0;i<goalSize;i++)
		{
			goalPose.push_back(knowledge_msg.response.pose[i]);
		}
	}

	cout<<actionParameters<<" pose: ";
	for (int i=0;i<goalPose.size();i++)
		cout<<goalPose[i]<<" ";
	cout<<endl;

	vector<float> initialJointPose, finalJointPose;
	finalJointPose.resize(num_joint,0.0);
	double actionTime=0;
	bool simulationResult;
	for(int i=0;i<msg.ArmsJoint[agentNumber].values.size();i++)
		initialJointPose.push_back(msg.ArmsJoint[agentNumber].values[i]);

	SimulateServiceApproachSingleArm(agentNumber, initialJointPose , goalPose, simulationResult, actionTime, finalJointPose);

	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=simulationResult;
	tempResponseMsg.time=actionTime;//sec
	tempResponseMsg.ActionName=msg.ActionName;

	for(int i=0;i<msg.ResponsibleAgents.size();i++)
		tempResponseMsg.ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);
	for(int i=0;i<msg.ActionParameterInfo.size();i++)
		tempResponseMsg.ActionParameterInfo.push_back(msg.ActionParameterInfo[i]);
	for(int i=0;i<msg.ActionParametersName.size();i++)
		tempResponseMsg.ActionParametersName.push_back(msg.ActionParametersName[i]);
	for(int i=0;i<msg.ColleagueAgents.size();i++)
		tempResponseMsg.ColleagueAgents.push_back(msg.ColleagueAgents[i]);




	robot_interface_msgs::Joints leftArmJoint,rightArmJoint;
	for(int i=0;i<7;i++)
	{
		if(agentNumber==0)
		{
			leftArmJoint.values.push_back(finalJointPose[i]);
			rightArmJoint.values.push_back(msg.ArmsJoint[1].values[i]);
		}
		else //==1
		{
			leftArmJoint.values.push_back(msg.ArmsJoint[0].values[i]);
			rightArmJoint.values.push_back(finalJointPose[i]);

		}
	}
	tempResponseMsg.ArmsJoint.push_back(leftArmJoint);
	tempResponseMsg.ArmsJoint.push_back(rightArmJoint);

	pub_simulationResponse.publish(tempResponseMsg);
//	exit(1);
};





void robotCallback::SimulateApproachingCommandJointArms(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateApproachingCommandJointArms"))<<endl;
	cout<<"Not implemented yet, and probably not necessary totally"<<endl;


};
void robotCallback::SimulateTransportingCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateTransportingCommand"))<<endl;
	if(msg.ResponsibleAgents.size()==1)
		SimulateTransportingCommandSingleArm(msg);
	else if (msg.ResponsibleAgents.size()==2)
		SimulateTransportingCommandJointArms(msg);
	else
		cout<<"Error in agent number"<<endl;

};
void robotCallback::SimulateTransportingCommandSingleArm(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateTransportingCommandSingleArm"))<<endl;


	//! parse the input command
	vector<string> parameter1,parameter2,parameter1Info,parameter2Info;
	// Transport wTo wTg
	boost::split(parameter1,msg.ActionParametersName[0], boost::is_any_of("-"));// wTo: Point3, Cylinder2-ConnectionFrame
	boost::split(parameter2,msg.ActionParametersName[1], boost::is_any_of("-"));// wTg: Point4, Plate-Plate1-connectionFrame
	int agentNumber;

	if(msg.ResponsibleAgents.size()==1)
	{
		if(msg.ResponsibleAgents[0]=="LeftArm")
			agentNumber=0;
		else if(msg.ResponsibleAgents[0]=="RightArm")
			agentNumber=1;
		else
			cout<<"Error in agent name: "<<msg.ResponsibleAgents[0]<<endl;
	}
	else
	{
		cout<<"The agent sizes is not defined in this function!"<<endl;
		return;
	}


	vector<float> wTo,wTg;
	int vectorSize;

	//! call the knowledge base
	knowledge_msgs::knowledgeSRV knowledge_msg;

	knowledge_msg.request.reqType=msg.ActionParametersName[0];
//	if(parameter1.size()>1)
//		knowledge_msg.request.Name=parameter1[1];
//	else
	knowledge_msg.request.Name="";

	knowledge_msg.request.requestInfo=msg.ActionParameterInfo[0]; // objectPose

	if(knowledgeBase_client.call(knowledge_msg)){
		vectorSize=knowledge_msg.response.pose.size();
		if(vectorSize!=6)
		{
			cout<<"Error in number of knowledge base point size for the joint action"<<endl;
			exit(1);
		}

		for (int i=0;i<6;i++)
			wTo.push_back(knowledge_msg.response.pose[i]);
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	cout<<"wTo: ";
	for (int i=0;i<6;i++){
			cout<<wTo[i]<<" ";
		}
	cout<<endl;

	knowledge_msg.request.reqType=msg.ActionParametersName[1];

//	if(parameter2.size()>1)
//		knowledge_msg.request.Name=parameter2[1];
//	else
	knowledge_msg.request.Name="";

	knowledge_msg.request.requestInfo=msg.ActionParameterInfo[1]; // objectPose

	if(knowledgeBase_client.call(knowledge_msg)){
		vectorSize=knowledge_msg.response.pose.size();
		if(vectorSize!=6)
		{
			cout<<"Error in number of knowledge base point size for the joint action"<<endl;
			exit(1);
		}
		for (int i=0;i<6;i++)
			wTg.push_back(knowledge_msg.response.pose[i]);
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	cout<<"wTg: ";
	for (int i=0;i<6;i++)
			cout<<wTg[i]<<" ";
	cout<<endl;

	//! path Planning
	if(pathPlanningFlag==true)
	{

	}
	else
	{

	}


//	vector<vector<float>> initialJointPose;
	vector<float> initialJointPose, finalJointPose;
	vector<float> leftArmJoints,rightArmJoints;
	for(int i=0;i<num_joint;i++)
	{
		leftArmJoints.push_back(msg.ArmsJoint[0].values[i]);
		rightArmJoints.push_back(msg.ArmsJoint[1].values[i]);
	}

	if (agentNumber==0)
		initialJointPose=leftArmJoints;
	else if(agentNumber==1)
		initialJointPose=rightArmJoints;
	else
		cout<<"Error in Arm Index"<<endl;
	//	initialJointPose.push_back(leftArmJoints);
//	initialJointPose.push_back(rightArmJoints);

	cout<<"Arm: "<<agentNumber<<", initial q: ";
	for(int i=0;i<initialJointPose.size();i++)
		cout<<initialJointPose[i]<<" ";
	cout<<endl;

	bool simulationResult;
	double  actionTime;

		finalJointPose.resize(num_joint,0.0);

	SimulateServiceTransportSingleArms(agentNumber,initialJointPose,wTo ,wTg, simulationResult, actionTime, finalJointPose);

	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=simulationResult;
	tempResponseMsg.time=actionTime;//sec
	tempResponseMsg.ActionName=msg.ActionName;

	for(int i=0;i<msg.ResponsibleAgents.size();i++)
		tempResponseMsg.ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);
	for(int i=0;i<msg.ActionParameterInfo.size();i++)
		tempResponseMsg.ActionParameterInfo.push_back(msg.ActionParameterInfo[i]);
	for(int i=0;i<msg.ActionParametersName.size();i++)
		tempResponseMsg.ActionParametersName.push_back(msg.ActionParametersName[i]);
	for(int i=0;i<msg.ColleagueAgents.size();i++)
		tempResponseMsg.ColleagueAgents.push_back(msg.ColleagueAgents[i]);


	robot_interface_msgs::Joints leftArmJoint,rightArmJoint;
	if(agentNumber==0)
	{
		for(int i=0;i<7;i++)
		{
			leftArmJoint.values.push_back(finalJointPose[i]);
			rightArmJoint.values.push_back(rightArmJoints[i]);
		}

	}
	else if(agentNumber==1)
	{
		for(int i=0;i<7;i++)
		{
			leftArmJoint.values.push_back(leftArmJoints[i]);
			rightArmJoint.values.push_back(finalJointPose[i]);
		}
	}
	else
	{
		cout<<"Error in arm index: "<<agentNumber<<endl;
	}
	tempResponseMsg.ArmsJoint.push_back(leftArmJoint);
	tempResponseMsg.ArmsJoint.push_back(rightArmJoint);

	pub_simulationResponse.publish(tempResponseMsg);

};

void robotCallback::SimulateTransportingCommandJointArms(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("robotCallback::SimulateTransportingCommandJointArms"))<<endl;
	/* 1- Parse the assigned action
	   2- get from the knowledge base the necessary info
	   3- base on the flag: find the path for the robot end effector
	   4- base on the flag: simulate the robot behavior based on the path
	   5- base on the flag: give command to the controller

	 */



	//! parse the input command
	vector<string> parameter1,parameter2,parameter1Info,parameter2Info;
	// Transport wTo wTg
	boost::split(parameter1,msg.ActionParametersName[0], boost::is_any_of("-"));// wTo: Point3, Cylinder2-ConnectionFrame
	boost::split(parameter2,msg.ActionParametersName[1], boost::is_any_of("-"));// wTg: Point4, Plate1_connectionFrame


	vector<float> wTo,wTg;
	vector<int> armIndex;
	armIndex.push_back(0);
	armIndex.push_back(1);
	int vectorSize;

	//! call the knowledge base
	knowledge_msgs::knowledgeSRV knowledge_msg;

	knowledge_msg.request.reqType=msg.ActionParametersName[0];
//	if(parameter1.size()>1)
//		knowledge_msg.request.Name=parameter1[1];
//	else
	knowledge_msg.request.Name="";

	knowledge_msg.request.requestInfo=msg.ActionParameterInfo[0]; // objectPose

	if(knowledgeBase_client.call(knowledge_msg)){

		vectorSize=knowledge_msg.response.pose.size();

		if(vectorSize!=6)
		{
			cout<<"Error in number of knowledge base point size for the joint action"<<endl;
			exit(1);
		}
		for (int i=0;i<6;i++)
			wTo.push_back(knowledge_msg.response.pose[i]);
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	cout<<"wTo: ";
	for (int i=0;i<6;i++){
			cout<<wTo[i]<<" ";
		}
	cout<<endl;


	knowledge_msg.request.reqType=msg.ActionParametersName[1];

//	if(parameter2.size()>1)
//		knowledge_msg.request.Name=parameter2[1];
//	else
	knowledge_msg.request.Name="";

	knowledge_msg.request.requestInfo=msg.ActionParameterInfo[1]; // objectPose

	if(knowledgeBase_client.call(knowledge_msg)){

		vectorSize=knowledge_msg.response.pose.size();
		if(vectorSize!=6)
		{
			cout<<"Error in number of knowledge base point size for the joint action"<<endl;
			exit(1);
		}
		for (int i=0;i<6;i++)
			wTg.push_back(knowledge_msg.response.pose[i]);
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	cout<<"wTg: ";
	for (int i=0;i<6;i++)
			cout<<wTg[i]<<" ";
	cout<<endl;

	//! path Planning
	if(pathPlanningFlag==true)
	{

	}
	else
	{

	}

	vector<vector<float>> initialJointPose;
	vector<float> leftArmJoints,rightArmJoints;
	for(int i=0;i<num_joint;i++)
	{
		leftArmJoints.push_back(msg.ArmsJoint[0].values[i]);
		rightArmJoints.push_back(msg.ArmsJoint[1].values[i]);

	}
	initialJointPose.push_back(leftArmJoints);
	initialJointPose.push_back(rightArmJoints);

	cout<<"initial q (left): ";
	for(int i=0;i<initialJointPose[0].size();i++)
		cout<<initialJointPose[0][i]<<" ";
	cout<<endl;

	cout<<"initial q (right): ";
	for(int i=0;i<initialJointPose[1].size();i++)
		cout<<initialJointPose[1][i]<<" ";
	cout<<endl;

	bool simulationResult;
	double  actionTime;
	vector<vector<float>> finalJointPose;

	SimulateServiceTransportJointArms(armIndex,initialJointPose,wTo ,wTg, simulationResult, actionTime, finalJointPose);

	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=simulationResult;
	tempResponseMsg.time=actionTime;//sec
	tempResponseMsg.ActionName=msg.ActionName;

	for(int i=0;i<msg.ResponsibleAgents.size();i++)
		tempResponseMsg.ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);
	for(int i=0;i<msg.ActionParameterInfo.size();i++)
		tempResponseMsg.ActionParameterInfo.push_back(msg.ActionParameterInfo[i]);
	for(int i=0;i<msg.ActionParametersName.size();i++)
		tempResponseMsg.ActionParametersName.push_back(msg.ActionParametersName[i]);
	for(int i=0;i<msg.ColleagueAgents.size();i++)
		tempResponseMsg.ColleagueAgents.push_back(msg.ColleagueAgents[i]);


	robot_interface_msgs::Joints leftArmJoint,rightArmJoint;
	for(int i=0;i<7;i++)
	{
		leftArmJoint.values.push_back(finalJointPose[0][i]);
		rightArmJoint.values.push_back(finalJointPose[1][i]);
	}
	tempResponseMsg.ArmsJoint.push_back(leftArmJoint);
	tempResponseMsg.ArmsJoint.push_back(rightArmJoint);

	pub_simulationResponse.publish(tempResponseMsg);
};

void robotCallback::SimulateServiceApproachSingleArm(int armIndex,vector<float> initialJointPose, vector<float> goalPose,  bool &simulationResult, double &actionTime, vector<float> &finalJointPose ){
	cout<<BOLD(FBLU("robotCallback::SimulateRobotSingleArm"))<<endl;

	simRobot_msgs::simulateRobotSRV simRobot_srv;
	simRobot_msgs::transformation simRobot_pose;

//	cout<<"2001"<<endl;
	simRobot_srv.request.simRobot.Activation=1;
	simRobot_srv.request.simRobot.sim_single_arm.armIndex=armIndex;
	simRobot_srv.request.simRobot.sim_single_arm.NoGoals=1;

	for (int j=0;j<goalPose.size();j++)
	{
		simRobot_pose.cartesianPosition[j]=goalPose[j];
	}
//	cout<<"2002"<<endl;

	simRobot_srv.request.simRobot.sim_single_arm.cartGoal.push_back(simRobot_pose);
	for (int i=0;i<num_joint;i++)
		simRobot_srv.request.simRobot.sim_single_arm.jointsInit.jointPosition[i]=initialJointPose[i];
//	cout<<"2003"<<endl;
	if (simRobot_client.call(simRobot_srv))
	{
//		cout<<"2004"<<endl;
		simulationResult=(bool)simRobot_srv.response.simResponse;
		// add the action Time here,
		// add the final joint Pose here
//		cout<<"2005"<<endl;
		if (armIndex==0)
			for(int i=0;i<num_joint;i++)
				finalJointPose[i]=simRobot_srv.response.jointsfinal_arm1[i];
		else if(armIndex==1)
			for(int i=0;i<num_joint;i++)
				finalJointPose[i]=simRobot_srv.response.jointsfinal_arm2[i];
		else
			cout<<"Error in arm Index"<<endl;

//		cout<<"2006"<<endl;

		actionTime=(double)simRobot_srv.response.time;

		if (simulationResult)
			cout<<"**** Simulation Shoes Robot Can Follow the Given Path ****"<<endl;
		else
			cout<<"**** Simulation Shoes Robot Can NOT Follow the Given Path ****"<<endl;

	}

//	return simulationResult;
};

void robotCallback::SimulateServiceTransportSingleArms(int armIndex, vector<float> initialJointPose, vector<float> wTo ,vector<float> wTg, bool &simulationResult, double &actionTime, vector<float> &finalJointPose ){
	cout<<BOLD(FBLU("robotCallback::SimulateServiceTransportSingleArms"))<<endl;

	//	bool simulationResult;
		simRobot_msgs::simulateRobotSRV simRobot_srv;
		simRobot_msgs::transformation simRobot_pose;

		if(armIndex>1)
			cout<<"the arm indices is not correct"<<endl;

		simRobot_srv.request.simRobot.Activation=3;
		simRobot_srv.request.simRobot.sim_single_arm_transport.arm_Index=armIndex;
		simRobot_srv.request.simRobot.sim_single_arm_transport.NoGoals=1;


		for (int j=0;j<num_joint;j++)
			simRobot_srv.request.simRobot.sim_single_arm_transport.jointsInit_arm.jointPosition[j]=initialJointPose[j];

		for (int j=0;j<wTg.size();j++)
			simRobot_pose.cartesianPosition[j]=wTg[j];

		simRobot_srv.request.simRobot.sim_single_arm_transport.wTg.push_back(simRobot_pose);

		for (int j=0;j<wTo.size();j++)
			simRobot_srv.request.simRobot.sim_single_arm_transport.wTo.cartesianPosition[j]=wTo[j];


		if (simRobot_client.call(simRobot_srv))
		{
			simulationResult=simRobot_srv.response.simResponse;
			actionTime=simRobot_srv.response.time;
			if (armIndex==0)
				for(int i=0;i<num_joint;i++)
					finalJointPose[i]=simRobot_srv.response.jointsfinal_arm1[i];
			else if(armIndex==1)
				for(int i=0;i<num_joint;i++)
					finalJointPose[i]=simRobot_srv.response.jointsfinal_arm2[i];
			else
				cout<<"Error in arm Index"<<endl;

			if (simulationResult)
				cout<<"**** Simulation Shoes Robot Can Follow the Given Path ****"<<endl;
			else
				cout<<"**** Simulation Shoes Robot Can NOT Follow the Given Path ****"<<endl;
		}

};

void robotCallback::SimulateServiceTransportJointArms(vector<int> armIndex, vector<vector<float>> initialJointPose, vector<float> wTo ,vector<float> wTg, bool &simulationResult, double &actionTime, vector<vector<float>> &finalJointPose ){
	cout<<BOLD(FBLU("robotCallback::SimulateServiceTransportJointArms"))<<endl;

//	bool simulationResult;
	simRobot_msgs::simulateRobotSRV simRobot_srv;
	simRobot_msgs::transformation simRobot_pose;

	if(armIndex.size()!=2)
		cout<<"the arm indices is not correct"<<endl;

	simRobot_srv.request.simRobot.Activation=2;
	simRobot_srv.request.simRobot.sim_biman_arm.arm1_Index=armIndex[0];
	simRobot_srv.request.simRobot.sim_biman_arm.arm2_Index=armIndex[1];
	simRobot_srv.request.simRobot.sim_biman_arm.NoGoals=1;


	for (int j=0;j<num_joint;j++)
		simRobot_srv.request.simRobot.sim_biman_arm.jointsInit_arm1.jointPosition[j]=initialJointPose[armIndex[0]][j];
	for (int j=0;j<num_joint;j++)
		simRobot_srv.request.simRobot.sim_biman_arm.jointsInit_arm2.jointPosition[j]=initialJointPose[armIndex[1]][j];


	for (int j=0;j<wTg.size();j++)
		simRobot_pose.cartesianPosition[j]=wTg[j];

	simRobot_srv.request.simRobot.sim_biman_arm.wTg.push_back(simRobot_pose);

	for (int j=0;j<wTo.size();j++)
		simRobot_srv.request.simRobot.sim_biman_arm.wTo.cartesianPosition[j]=wTo[j];


	if (simRobot_client.call(simRobot_srv))
	{
		simulationResult=simRobot_srv.response.simResponse;
		actionTime=simRobot_srv.response.time;
		vector<float> leftArmJoints,rightArmJoints;
		for(int i=0;i<num_joint;i++)
		{
			leftArmJoints.push_back(simRobot_srv.response.jointsfinal_arm1[i]);
			rightArmJoints.push_back(simRobot_srv.response.jointsfinal_arm2[i]);
		}
		finalJointPose.push_back(leftArmJoints);
		finalJointPose.push_back(rightArmJoints);

		if (simulationResult)
			cout<<"**** Simulation Shoes Robot Can Follow the Given Path ****"<<endl;
		else
			cout<<"**** Simulation Shoes Robot Can NOT Follow the Given Path ****"<<endl;
	}
};

void robotCallback::arrivingCommands(const std_msgs::String::ConstPtr& input1){
	// MSG: "[action] [agents who should perform] [collaborators]"
	// Example:  Approach-Point-11 LeftArm+RightArm Human
	cout<<BOLD(FBLU("robotCallback::arrivingCommands"))<<endl;

	string input=input1-> data.c_str();
	ROS_INFO("Arrived robot command: %s",input.c_str());
	int agentNumber;
	vector<string> msg, msgAction,msgAgents,msgColleagues;
	string cmndType, reachingPoint ;

//	cout<<"101"<<endl;
	boost::split(msg, input, boost::is_any_of(" "));
//	cout<<"101-1"<<endl;
	boost::split(msgAction, msg[0], boost::is_any_of("_"));
//	cout<<"101-2"<<endl;

	boost::split(msgAgents, msg[1], boost::is_any_of("+"));
//	cout<<"101-3"<<endl;

	if(msg.size()==3)
	boost::split(msgColleagues, msg[2], boost::is_any_of("+"));
	else if(msg.size()>3)
		cout<<"Error in arriving msg size: "<<msg.size()<<input<<endl;

	//! first find which agents should perform the action and assign it.
//	cout<<"102"<<endl;
	if(msgAgents.size()==1)
	{
		if(msgAgents[0]=="LeftArm")
		{
			agentNumber=0;
		}
		else if(msgAgents[0]=="RightArm")
		{
			agentNumber=1;
		}
		else
		{
			cout<<"The agents definition is not correct: "<<msgAgents.size()<<", "<<msg[1]<<endl;
		}
	}
	else if(msgAgents.size()==2)
	{
		if((msgAgents[0]=="LeftArm" && msgAgents[1]=="RightArm") ||(msgAgents[1]=="LeftArm" && msgAgents[0]=="RightArm"))
		{
			agentNumber=2;
		}
		else
		{
			cout<<"The agents definition is not correct: "<<msgAgents.size()<<", "<<msg[1]<<endl;
		}
	}
	else
	{
		cout<<"The agents size is not correct: "<<msgAgents.size()<<", "<<msg[1]<<endl;
	}
//	cout<<"103: "<<agentNumber<<agents_list.size()<<endl;

	//***********************************************************
	// fill the related agent with the action
	if(agents_list[agentNumber].isBusy==false || msg[0]=="Stop")
	{
//		cout<<"103-1"<<endl;

		agents_list[agentNumber].isBusy=true;
//		cout<<"103-2"<<endl;
		agents_list[agentNumber].lastAssignedAction=msg[0];
//		cout<<"103-3"<<endl;
		agents_list[agentNumber].microSec_StartingTime=duration_cast< microseconds >(system_clock::now().time_since_epoch());
//		cout<<"103-4"<<endl;
		agents_list[agentNumber].collaborators=msgColleagues;

	}
	else
	{
		cout<<"The agent you assigned is busy now and it can not perform a new action "<<endl;
		// publish false to the planner for the arrived action
	}
//	cout<<"104"<<endl;

	//******************************************************
	// find the correct function for each action

	if(msgAction[0]=="Approach")
	{
		SendApproachingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="Transport")
	{
		SendTransportingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="Rest")
	{
		SendRestingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="Screw")
	{
		SendScrewingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="Unscrew")
	{
		SendUnscrewingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="Grasp" || msgAction[0]=="UnGrasp")
	{
		SendGraspingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="Stop")
	{
		SendStoppingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="HoldOn")
	{
		SendHoldingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="Reduce")
	{
		SendKB_ReductionCommand(agents_list[agentNumber]);
	}
	else
	{
		cout<<"Error in arriving msg action:"<<msgAction[0]<<endl;
	}



}


void robotCallback::PublishRobotAck(agents_tasks& agent){
	cout<<BOLD(FBLU("robotCallback::PublishRobotAck"))<<endl;

	std_msgs::String ackMsg;
	cout<<"Last assigned action: "<<agent.lastAssignedAction<<endl;
	ackMsg.data=agent.lastAssignedAction+" "; // Appraoch_Point2, Grasp, Transport_Cylinder2-GraspingPose1_Point7

	for(int i=0;i<agent.agents.size();i++){
		ackMsg.data+=agent.agents[i];
		if(i<agent.agents.size()-1)
			ackMsg.data=ackMsg.data+"+";
	}
	if(agent.isActionSuccessfullyDone==true)
		ackMsg.data+=" true";
	else
		ackMsg.data+=" false";

	if (agent.emergencyFlag==false)
		pub_hri_robot_ack.publish(ackMsg);
	else
	{
		cout<<"The agent emergency flag is true, therefore we do not give ack to the planner"<<endl;
		agent.Print();
	}
}

void robotCallback::SetAgentsList(){
	agents_tasks agent1;
	agent1.agents.push_back("LeftArm");
	agent1.agentsNumber=0;
	agent1.collaborators.clear();
	agents_list.push_back(agent1);

	agents_tasks agent2;
	agent2.agents.push_back("RightArm");
	agent2.agentsNumber=1;
	agent2.collaborators.clear();
	agents_list.push_back(agent2);

	agents_tasks agent3;
	agent3.agents.push_back("LeftArm");
	agent3.agents.push_back("RightArm");
	agent3.agentsNumber=2;
	agent3.collaborators.clear();
	agents_list.push_back(agent3);

};


void robotCallback::SendGraspingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("robotCallback::sendGraspingCommand"))<<endl;

	agent.Print();

	control_msg.Activation=2;
	control_msg.GripArm.arm=agent.agentsNumber;

	if(agent.lastAssignedAction=="Grasp")
		control_msg.GripArm.value=0;
	else if(agent.lastAssignedAction=="UnGrasp")
		control_msg.GripArm.value=1;
	else
		cout<<"Error in assigned action: "<<agent.lastAssignedAction<<endl;
	publishControlCommand.publish(control_msg);

};
void robotCallback::SendHoldingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("robotCallback::sendHoldingCommand"))<<endl;

	control_msg.Activation=4;
	control_msg.holdModeArm.arm=agent.agentsNumber;
	control_msg.holdModeArm.holdingmode=1; // hold on=1, hold off=0
	publishControlCommand.publish(control_msg);

};
void robotCallback::SendStoppingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("robotCallback::sendStoppingCommand"))<<endl;

	agent.Print();

	control_msg.Activation=3;
	control_msg.stopArm.arm=agent.agentsNumber;
	publishControlCommand.publish(control_msg);

};

void robotCallback::SendApproachingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("robotCallback::SendApproachingCommand"))<<endl;

	if(agent.agentsNumber==0 || agent.agentsNumber==1)
		SendApproachingCommandSingleArm(agent);
	else if (agent.agentsNumber==2)
	{
		//SendTransportingCommandJointArms(agent);
	}
	else
		cout<<"Error in agent number"<<endl;
};
void robotCallback::SendTransportingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("robotCallback::SendTransportingCommand"))<<endl;

	if(agent.agentsNumber==0 || agent.agentsNumber==1)
		SendTransportingCommandSingleArms(agent);
	else if (agent.agentsNumber==2)
		SendTransportingCommandJointArms(agent);
	else
		cout<<"Error in agent number"<<endl;
};

void robotCallback::SendRestingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("robotCallback::SendRestingCommand"))<<endl;
	/* 1- Parse the assigned action
	   2- get from the knowledge base the necessary info
	   3- base on the flag: find the path for the robot end effector
	   4- base on the flag: simulate the robot behavior based on the path
	   5- base on the flag: give command to the controller
	 */

	agent.Print();

	//! parse the input command
	vector<string> msgAction, msgParameters;
	vector<float> goalPose;
	int goalSize;
	if(agent.agentsNumber==0)
		msgParameters.push_back("Point_RestingLeft");
	else if(agent.agentsNumber==1)
		msgParameters.push_back("Point_RestingRight");
	else
		cout<<"Error in agent number"<<endl;

	//! call the knowledge base
	knowledge_msgs::knowledgeSRV knowledge_msg;

	knowledge_msg.request.reqType=msgParameters[0];
	if(msgParameters.size()>1)
		knowledge_msg.request.Name=msgParameters[1];
	else
		knowledge_msg.request.Name="";
	knowledge_msg.request.requestInfo="Pose";

	if(knowledgeBase_client.call(knowledge_msg)){

		goalSize=knowledge_msg.response.pose.size();

		if(goalSize==6)
			control_msg.oneArm.armCmndType="cartPos";
		else if(goalSize==7)
			control_msg.oneArm.armCmndType="jointPos";
		else
		{
			cout<<"Error in number of knowledge base size"<<endl;
			exit(1);
		}

		for (int i=0;i<goalSize;i++){
			goalPose.push_back(knowledge_msg.response.pose[i]);
		}

		simulationFlag=true;
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	cout<<"goalPose: ";
	for (int i=0;i<goalSize;i++)
	{
			cout<<goalPose[i]<<" ";
	}
	cout<<endl;

		control_msg.Activation=0;
		control_msg.oneArm.armIndex=agent.agentsNumber;
		control_msg.oneArm.armCmndType="jointPos";
		for(int i=0;i<goalSize;i++)
		control_msg.oneArm.cartGoal.jointPosition[i]=goalPose[i];
		publishControlCommand.publish(control_msg);
};



void robotCallback::SendScrewingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("robotCallback::SendScrewingCommand"))<<endl;

	agent.Print();

	//! parse the input command
	vector<string> msgAction, msgParameters;
	vector<float> goalPose;
	int goalSize;
	if(agent.agentsNumber==0)
		msgParameters.push_back("LeftArm_q");
	else if(agent.agentsNumber==1)
		msgParameters.push_back("RightArm_q");
	else
		cout<<"Error in agent number"<<endl;

	//! call the knowledge base
	knowledge_msgs::knowledgeSRV knowledge_msg;

	knowledge_msg.request.reqType=msgParameters[0];
	if(msgParameters.size()>1)
		knowledge_msg.request.Name=msgParameters[1];
	else
		knowledge_msg.request.Name="";
	knowledge_msg.request.requestInfo="Pose";

	if(knowledgeBase_client.call(knowledge_msg)){

		goalSize=knowledge_msg.response.pose.size();

		if(goalSize==6)
			control_msg.oneArm.armCmndType="cartPos";
		else if(goalSize==7)
			control_msg.oneArm.armCmndType="jointPos";
		else
		{
			cout<<"Error in number of knowledge base size"<<endl;
			exit(1);
		}

		for (int i=0;i<goalSize;i++){
			goalPose.push_back(knowledge_msg.response.pose[i]);
		}
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
		exit(1);
	}
	goalPose[6]=3.0;
	cout<<"goalPose: ";
	for (int i=0;i<goalSize;i++)
	{
			cout<<goalPose[i]<<" ";
	}
	cout<<endl;

		control_msg.Activation=0;
		control_msg.oneArm.armIndex=agent.agentsNumber;
		control_msg.oneArm.armCmndType="jointPos";
		for(int i=0;i<goalSize;i++)
		control_msg.oneArm.cartGoal.jointPosition[i]=goalPose[i];
		publishControlCommand.publish(control_msg);

};
void robotCallback::SendUnscrewingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("robotCallback::SendUnscrewingCommand"))<<endl;

	agent.Print();

	//! parse the input command
	vector<string> msgAction, msgParameters;
	vector<float> goalPose;
	int goalSize;
	if(agent.agentsNumber==0)
		msgParameters.push_back("LeftArm_q");
	else if(agent.agentsNumber==1)
		msgParameters.push_back("RightArm_q");
	else
		cout<<"Error in agent number"<<endl;

	//! call the knowledge base
	knowledge_msgs::knowledgeSRV knowledge_msg;

	knowledge_msg.request.reqType=msgParameters[0];
	if(msgParameters.size()>1)
		knowledge_msg.request.Name=msgParameters[1];
	else
		knowledge_msg.request.Name="";
	knowledge_msg.request.requestInfo="Pose";

	if(knowledgeBase_client.call(knowledge_msg)){

		goalSize=knowledge_msg.response.pose.size();

		if(goalSize==6)
			control_msg.oneArm.armCmndType="cartPos";
		else if(goalSize==7)
			control_msg.oneArm.armCmndType="jointPos";
		else
		{
			cout<<"Error in number of knowledge base size"<<endl;
			exit(1);
		}

		for (int i=0;i<goalSize;i++){
			goalPose.push_back(knowledge_msg.response.pose[i]);
		}
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
		exit(1);
	}
	goalPose[6]=-3.0;
	cout<<"goalPose: ";
	for (int i=0;i<goalSize;i++)
	{
			cout<<goalPose[i]<<" ";
	}
	cout<<endl;

		control_msg.Activation=0;
		control_msg.oneArm.armIndex=agent.agentsNumber;
		control_msg.oneArm.armCmndType="jointPos";
		for(int i=0;i<goalSize;i++)
		control_msg.oneArm.cartGoal.jointPosition[i]=goalPose[i];
		publishControlCommand.publish(control_msg);


};

void robotCallback::SendKB_ReductionCommand(agents_tasks& agent){
	std_msgs::String KBmsg;
	KBmsg.data=agent.lastAssignedAction+ " "+to_string(agent.agentsNumber);
	publishKBCommand.publish(KBmsg); // Reduce_WS 1 Reduce_cylinder 0 ...
};



void robotCallback::SendApproachingCommandSingleArm(agents_tasks& agent){
	cout<<BOLD(FBLU("robotCallback::SendApproachingCommandSingleArm"))<<endl;
	/* 1- Parse the assigned action
	   2- get from the knowledge base the necessary info
	   3- base on the flag: find the path for the robot end effector
	   4- base on the flag: simulate the robot behavior based on the path
	   5- base on the flag: give command to the controller

	 */
	agent.Print();
	bool simulationResult;

	//! parse the input command
	vector<string> msgAction, msgParameters;
	boost::split(msgAction,agent.lastAssignedAction, boost::is_any_of("_"));// Approach_Point5, Approach_Cylinder1-graspingPose2
	boost::split(msgParameters, msgAction[1], boost::is_any_of("-"));// Point5, Cylinder1-graspingPose2
	vector<float> goalPose;
	int goalSize;

	//! call the knowledge base
	knowledge_msgs::knowledgeSRV knowledge_msg;

	knowledge_msg.request.reqType=msgAction[1];
//	if(msgParameters.size()>1)
//		knowledge_msg.request.Name=msgParameters[1];
//	else
	knowledge_msg.request.Name="";
	knowledge_msg.request.requestInfo="Pose";

	if(knowledgeBase_client.call(knowledge_msg)){

		goalSize=knowledge_msg.response.pose.size();

		if(goalSize==6)
			control_msg.oneArm.armCmndType="cartPos";
		else if(goalSize==7)
			control_msg.oneArm.armCmndType="jointPos";
		else
			cout<<"Error in number of knowledge base size"<<endl;

		for (int i=0;i<goalSize;i++){
			goalPose.push_back(knowledge_msg.response.pose[i]);
		}

		simulationFlag=true;
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	cout<<"goalPose: ";
	for (int i=0;i<goalSize;i++){
			cout<<goalPose[i]<<" ";
		}
	cout<<endl;

	//! path Planning
	if(pathPlanningFlag==true)
	{

	}
	else
	{


	}

	//! simulation
//	if(simulationFlag==true)
//	{
//
//		//simulationResult=SimulateRobotSingleArm(agent.agentsNumber, goalPose);
//		if(simulationResult==true)
//			controllerFlag=true;
//		else
//		{
//			cout<<"Simulation shows the robot can not perform the action; do you Want to Continue? ";
//			bool input_flag;
//			cin>>input_flag;
//			if(input_flag==true)
//				controllerFlag=true;
//			else
//			{
//				controllerFlag=false;
//				// here publish the result to the planner
//				agent.isActionSuccessfullyDone=false;
//				agent.isBusy=false;
//				PublishRobotAck(agent);
//			}
//		}
//
//	}

	//! Controller
//	if(controllerFlag==true)
//	{
		control_msg.Activation=0;
		control_msg.oneArm.armIndex=agent.agentsNumber;
		control_msg.oneArm.armCmndType="cartPos";
		for(int i=0;i<goalSize;i++)
		control_msg.oneArm.cartGoal.cartesianPosition[i]=goalPose[i];
		publishControlCommand.publish(control_msg);
//	}
};

void robotCallback::SendTransportingCommandSingleArms(agents_tasks& agent){

	cout<<BOLD(FBLU("robotCallback::SendTransportingCommandSingleArms"))<<endl;
	/* 1- Parse the assigned action
	   2- get from the knowledge base the necessary info
	   3- base on the flag: find the path for the robot end effector
	   4- base on the flag: simulate the robot behavior based on the path
	   5- base on the flag: give command to the controller

	 */
	agent.Print();
	bool simulationResult;

	//! parse the input command
	vector<string> msgAction, msgParameters1, msgParameters2;
	boost::split( msgAction, agent.lastAssignedAction, boost::is_any_of("_"));// Transport_point5_Point3, Transport_Cylinder1-graspingPose1_Plane1-graspingPose1,
	boost::split( msgParameters1, msgAction[1], boost::is_any_of("-"));//
	boost::split( msgParameters2, msgAction[2], boost::is_any_of("-"));//
	vector<float> wTo,wTg;
	vector<int> armIndex;
	armIndex.push_back(0);
	armIndex.push_back(1);
	int vectorSize;

	//! call the knowledge base
	knowledge_msgs::knowledgeSRV knowledge_msg1,knowledge_msg2;

//	cout<<msgParameters[0]<<" "<<msgParameters[1]<<endl;
	knowledge_msg1.request.reqType=msgAction[1];
	knowledge_msg2.request.reqType=msgAction[2];

	knowledge_msg1.request.Name="";
	knowledge_msg2.request.Name="";
	knowledge_msg1.request.requestInfo="Pose";
	knowledge_msg2.request.requestInfo="Pose";

	if(knowledgeBase_client.call(knowledge_msg1))
	{
		vectorSize=knowledge_msg1.response.pose.size();
		if(vectorSize!=6)
		{
			cout<<"Error in number of knowledge base point size for the joint action"<<endl;
			exit(1);
		}
		for (int i=0;i<6;i++)
			wTo.push_back(knowledge_msg1.response.pose[i]);
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	if(knowledgeBase_client.call(knowledge_msg2))
	{

		vectorSize=knowledge_msg2.response.pose.size();

		if(vectorSize!=6)
		{
			cout<<"Error in number of knowledge base point size for the joint action"<<endl;
			exit(1);
		}
		for (int i=0;i<6;i++)
			wTg.push_back(knowledge_msg2.response.pose[i]);
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	cout<<"wTo: ";
	for (int i=0;i<6;i++)
		cout<<wTo[i]<<" ";
	cout<<endl;

	cout<<"wTg: ";
	for (int i=0;i<6;i++)
			cout<<wTg[i]<<" ";
	cout<<endl;

	//! path Planning
	if(pathPlanningFlag==true)
	{}
	else
	{}
	control_msg.Activation=5;
	control_msg.oneArmTransport.arm1=armIndex[0];
	for(int i=0;i<6;i++)
	{
		control_msg.oneArmTransport.wTo.cartesianPosition[i]=wTo[i];
		control_msg.oneArmTransport.wTg.cartesianPosition[i]=wTg[i];
	}
	publishControlCommand.publish(control_msg);
};

void robotCallback::SendTransportingCommandJointArms(agents_tasks& agent){
	cout<<BOLD(FBLU("robotCallback::SendTransportingCommandJointArms"))<<endl;
	/* 1- Parse the assigned action
	   2- get from the knowledge base the necessary info
	   3- base on the flag: find the path for the robot end effector
	   4- base on the flag: simulate the robot behavior based on the path
	   5- base on the flag: give command to the controller

	 */
	agent.Print();
	bool simulationResult;

	//! parse the input command
	vector<string> msgAction, msgParameters1, msgParameters2;
	boost::split( msgAction, agent.lastAssignedAction, boost::is_any_of("_"));// Transport_point5_Point3, Transport_Cylinder1-graspingPose1_Plane1-graspingPose1,
	boost::split( msgParameters1, msgAction[1], boost::is_any_of("-"));//
	boost::split( msgParameters2, msgAction[2], boost::is_any_of("-"));//
	vector<float> wTo,wTg;
	vector<int> armIndex;
	armIndex.push_back(0);
	armIndex.push_back(1);
	int vectorSize;

	//! call the knowledge base
	knowledge_msgs::knowledgeSRV knowledge_msg1,knowledge_msg2;

//	cout<<msgParameters[0]<<" "<<msgParameters[1]<<endl;
	knowledge_msg1.request.reqType=msgAction[1];
	knowledge_msg2.request.reqType=msgAction[2];

//	if(msgParameters1.size()>1)
//		knowledge_msg1.request.Name=msgParameters1[1];
//	else
	knowledge_msg1.request.Name="";

//	if(msgParameters2.size()>1)
//		knowledge_msg2.request.Name=msgParameters2[1];
//	else
	knowledge_msg2.request.Name="";


//	knowledge_msg1.request.Name=msgParameters[1];

	knowledge_msg1.request.requestInfo="Pose";
	knowledge_msg2.request.requestInfo="Pose";

	if(knowledgeBase_client.call(knowledge_msg1))
	{

		vectorSize=knowledge_msg1.response.pose.size();

		if(vectorSize!=6)
		{
			cout<<"Error in number of knowledge base point size for the joint action"<<endl;
			exit(1);
		}

		for (int i=0;i<6;i++)
			wTo.push_back(knowledge_msg1.response.pose[i]);

//		simulationFlag=true;
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	if(knowledgeBase_client.call(knowledge_msg2))
	{

		vectorSize=knowledge_msg2.response.pose.size();

		if(vectorSize!=6)
		{
			cout<<"Error in number of knowledge base point size for the joint action"<<endl;
			exit(1);
		}
		for (int i=0;i<6;i++)
			wTg.push_back(knowledge_msg2.response.pose[i]);

//		simulationFlag=true;
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	cout<<"wTo: ";
	for (int i=0;i<6;i++){
			cout<<wTo[i]<<" ";
		}
	cout<<endl;

	cout<<"wTg: ";
	for (int i=0;i<6;i++){
			cout<<wTg[i]<<" ";
		}
	cout<<endl;

	//! path Planning
	if(pathPlanningFlag==true)
	{

	}
	else
	{


	}


	//! simulation
//	if(simulationFlag==true)
//	{
//
////		simulationResult=SimulateRobotJointArms(armIndex,wTo ,wTg);
////		simulationResult=true;
//
//		if(simulationResult==true)
//			controllerFlag=true;
//		else
//		{
//			controllerFlag=false;
//			// here publish the result to the planner
//			agent.isActionSuccessfullyDone=false;
//			agent.isBusy=false;
//			PublishRobotAck(agent);
//		}
//
//	}


	//! Controller
//	if(controllerFlag==true)
//	{
		control_msg.Activation=1;
		control_msg.bimanualArm.arm1=armIndex[0];
		control_msg.bimanualArm.arm2=armIndex[1];
		for(int i=0;i<6;i++)
		{
			control_msg.bimanualArm.wTo.cartesianPosition[i]=wTo[i];
			control_msg.bimanualArm.wTg.cartesianPosition[i]=wTg[i];
		}
		publishControlCommand.publish(control_msg);
//	}

};





void robotCallback::FailureCheck(void){


	microseconds microSec_CurrentTime=duration_cast< microseconds >(system_clock::now().time_since_epoch());

	for(int i=0;i<agents_list.size();i++)
	{
		if(agents_list[i].isBusy==true)
		{
			if( (microSec_CurrentTime.count()-agents_list[i].microSec_StartingTime.count())/1000000.0 > waiting_time ) //value of time in micro seconds
			{
				cout<<"robotCallback::FailureCheck"<<endl;
				cout<<"waiting time from last command: "<<(microSec_CurrentTime.count()-agents_list[i].microSec_StartingTime.count())/1000000.0<<endl;
				agents_list[i].isBusy=false;
				agents_list[i].isActionSuccessfullyDone=false;
				PublishRobotAck(agents_list[i]);
				// also stop the robot for the current action
				StopRobotEmergency(agents_list[i]);

			}
		}

	}

};

void robotCallback::StopRobotEmergency(agents_tasks& agent){
	cout<<BOLD(FBLU("robotCallback::StopRobotEmergency"))<<endl;
	vector<string> msgColleagues;

	agent.isBusy=true;
	agent.lastAssignedAction="Stop";
	agent.microSec_StartingTime=duration_cast< microseconds >(system_clock::now().time_since_epoch());
	agent.collaborators=msgColleagues;
	agent.emergencyFlag=true;
	SendStoppingCommand(agent);

};


