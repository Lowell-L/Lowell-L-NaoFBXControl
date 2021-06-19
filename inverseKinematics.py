import FbxCommon
from fbx import *
import re
from numpy import *
import sys
from Vector3 import Vector3
import math
from writeXAR2 import *


'''########################以下数据结构用于逆运动学模仿运动###########################'''

# 存储关节的全局坐标的列表
Spine_position				= []
Spine1_position 			= []
Spine2_position 			= []
LeftShoulder_position		= []
LeftArm_position			= []
LeftForeArm_position		= []
LeftHand_position			= []
RightShoulder_position 		= []
RightArm_position			= []
RightForeArm_position		= []
RightHand_position			= []
Hips_position				= []
LeftUpLeg_position			= []
LeftLeg_position			= []
LeftFoot_position			= []
LeftFoot_End_position		= []
RightUpLeg_position			= []
RightLeg_position			= []
RightFoot_position			= []
RightFoot_End_position		= []


# 存储端部关节的 Lcl Rotation
Head_Rotation				= []
Neck_Rotation				= []
LeftHand_Rotation			= []
RightHand_Rotation			= []
LeftFoot_Rotation			= []
RightFoot_Rotation			= []


# 存储 Nao 机器人最终关节角度的列表
# 左臂
LShoulderRoll_Angle			= []
LShoulderPitch_Angle		= []
LElbowRoll_Angle			= []
LElbowYaw_Angle				= []
# 右臂
RShoulderRoll_Angle			= []
RShoulderPitch_Angle		= []
RElbowRoll_Angle			= []
RElbowYaw_Angle				= []
# 髋部
LHipYawPitch_Angle			= []
RHipYawPitch_Angle			= []
# 左腿
LHipPitch_Angle				= []
LHipRoll_Angle				= []
LKneePitch_Angle			= []
# 右腿
RHipPitch_Angle				= []
RHipRoll_Angle				= []
RKneePitch_Angle			= []
# 端部关节
HeadPitch_Angle				= []
HeadYaw_Angle				= []
LWristYaw_Angle				= []
RWristYaw_Angle				= []
LAnklePitch_Angle			= []
LAnkleRoll_Angle			= []
RAnklePitch_Angle			= []
RAnkleRoll_Angle			= []
'''########################以上数据结构用于逆运动学模仿运动###########################'''


'''######################以下数据结构用于正运动学计算模仿相似度########################'''

# 存储 FBX 文件中人体模型的关节在 Spine 的局部坐标系下的 LclTranslation
# 用于计算模仿的相似度
RightArmLclTranslationForSpine_HumanFBX 	= []
RightForeArmLclTranslationForSpine_HumanFBX	= []
RightHandLclTranslationForSpine_HumanFBX	= []


# 存储 Nao 机器人的关节在 Torso 的局部坐标系下的 LclTranslation
# 用于计算模仿的相似度
RWristLclTranslationForTorso_Nao	= []
RElbowLclTranslationForTorso_Nao	= []
RShoulderLclTranslationForTorso_Nao	= []


# 存储 FBX 文件中的关节向量
# 用于计算模仿的相似度
SpineToRightArm_HumanFBX			= []
RightArmToRightForeArm_HumanFBX		= []
RightForeArmToRightHand_HumanFBX	= []


# 存储 Nao 机器人各关节在 Torso 的局部坐标系下的关节向量
# 用于计算模仿的相似度
TorsoToRShoulder_Nao	= []
RShoulderToRElbow_Nao	= []
RElbowToRWrist_Nao		= []


# 相似度结果
TorsoToRShoulder_Similarity 	= []
RShoulderToRElbow_Similarity 	= []
RElbowToRWrist_Similarity		= []

'''######################以上数据结构用于正运动学计算模仿相似度########################'''



'''########################以下函数用于正运动学计算模仿相似度#########################'''

def vectorConversion(vector):
	'''将 FbxVector4 类型的向量转换为 Vector3类型，并将其单位化'''
	component = re.search(
		r'fbx.FbxVector4\(([-]?[0-9]+\.[0-9]+), ([-]?[0-9]+\.[0-9]+), ([-]?[0-9]+\.[0-9]+), [-]?[0-9]+\.[0-9]+\)',
		str(vector), re.M | re.I)
	if component:
		x = float(component.group(1))
		y = float(component.group(2))
		z = float(component.group(3))
		vector = Vector3(x, y, z)
		# 向量单位化
		vector = vector.unitization()
		return vector
	else:
		print("Vector conversion failed!")



def positiveKinematics_Nao(frameNumber):
	'''Nao机器人正运动学计算：
			根据 Nao 的肢体长度，在 MotionBuilder 内建立 Nao 机器人的 FBX 模型
			输入每时刻下 Nao 机器人各关节的旋转角度，通过 FBX 的函数获取 Nao 机器人各关节在正运动学计算后的局部坐标
			以此计算出 Nao 机器人各关节的向量，标准化后替代为此部分肢体的姿态。
			同样的方法应用于人体的 FBX 模型，得到人体对应部分肢体的姿态，
			最终以此定量计算评价模仿的相似性
	'''

	# 打开 Nao 的模型文件
	fileName = "nao.fbx"
	(lSdkManager, lScene) = FbxCommon.InitializeSdkObjects()
	if FbxCommon.LoadScene(lSdkManager, lScene, fileName):
		Torso_node = lScene.GetRootNode()
		RShoulder_node = Torso_node.FindChild("RShoulder")
		RElbow_node	= Torso_node.FindChild("RElbow")
		RWrist_node = Torso_node.FindChild("RWrist")

		# TODO：注意旋转顺序 解决！
		# 设置 Nao 机器人关节的旋转顺序
		# Shoulder：ShoulderPitch(X轴)先旋转 ShoulderRoll(Y轴) 后旋转
		RShoulder_node.SetRotationOrder(0, 0) # 0 代表旋转顺序为 XYZ
		# Elbow：ElbowYaw(Z轴)先旋转 ElbowRoll(Y轴) 后旋转
		RElbow_node.SetRotationOrder(0, 5) # 5 代表旋转顺序为 ZYX

		# for 循环范围 帧：[0, frameNumber]
		for i in range(0, frameNumber + 1):
			# 设置旋转角度
			# TODO: 注意角度映射 正确！
			RShoulder_node.LclRotation.Set(FbxDouble3(RShoulderPitch_Angle[i], RShoulderRoll_Angle[i], 0))
			RElbow_node.LclRotation.Set(FbxDouble3(0, RElbowRoll_Angle[i], RElbowYaw_Angle[i]))

			# 获取坐标
			lTimeIncrement = FbxTime()
			lTimeIncrement.SetTime(0, 0, 0, 0, 0, lScene.GetGlobalSettings().GetTimeMode())
			# 获取 RShoulder RElbow RWrist 的局部坐标
			RShoulderLclTranslation = lScene.GetAnimationEvaluator().GetNodeLocalTranslation(RShoulder_node, FbxTime(lTimeIncrement.Get()))
			RElbowLclTranslation = lScene.GetAnimationEvaluator().GetNodeLocalTranslation(RElbow_node, FbxTime(lTimeIncrement.Get()))
			RWristTranslation = lScene.GetAnimationEvaluator().GetNodeLocalTranslation(RWrist_node, FbxTime(lTimeIncrement.Get()))
			# 获取 RElbow RShoulder Torso 的齐次转换矩阵
			RElbowLclTransform = lScene.GetAnimationEvaluator().GetNodeLocalTransform(RElbow_node, FbxTime(lTimeIncrement.Get()))
			RShoulderLclTransform = lScene.GetAnimationEvaluator().GetNodeLocalTransform(RShoulder_node, FbxTime(lTimeIncrement.Get()))
			TorsoLclTransform = lScene.GetAnimationEvaluator().GetNodeLocalTransform(Torso_node, FbxTime(lTimeIncrement.Get()))

			# 计算各结点在 Torso 的局部坐标系下的局部坐标
			RShoulderLclTranslationForTorso_Nao.append(TorsoLclTransform.MultT(RShoulderLclTranslation))
			RElbowLclTranslationForTorso_Nao.append(TorsoLclTransform.MultT(RShoulderLclTransform.MultT(RElbowLclTranslation)))
			RWristLclTranslationForTorso_Nao.append(TorsoLclTransform.MultT(RShoulderLclTransform.MultT(RElbowLclTransform.MultT(RWristTranslation))))

			# 计算各结点在 Torso 的局部坐标系下的关节向量并单位化
			TorsoToRShoulder_Nao.append(
				vectorConversion(RShoulderLclTranslationForTorso_Nao[i]))
			RShoulderToRElbow_Nao.append(
				vectorConversion(RElbowLclTranslationForTorso_Nao[i] - RShoulderLclTranslationForTorso_Nao[i]))
			RElbowToRWrist_Nao.append(
				vectorConversion(RWristLclTranslationForTorso_Nao[i] - RElbowLclTranslationForTorso_Nao[i]))

		print("Nao 机器人正运动学计算完成！")
	else:
		print("Nao 机器人正运动学计算失败！")
		sys.exit(1)



def positiveKinematics_HumanFbx(filename, frameNumber):
	'''获取 FBX 文件中人体各关节结点在局部坐标系下的坐标，用于定量计算模仿的相似度'''
	(lSdkManager, lScene) = FbxCommon.InitializeSdkObjects()
	if FbxCommon.LoadScene(lSdkManager, lScene, filename):
		# 获取身体树状结构的根节点
		root_node = lScene.GetRootNode()
		# 获取 Spine 的关节结点，Spine的作用等同于 Nao 机器人的 Torso
		Spine_node = root_node.FindChild("mixamorig:Spine2")
		# 右臂关节结点
		RightShoulder_node = root_node.FindChild("mixamorig:RightShoulder")
		RightArm_node = root_node.FindChild("mixamorig:RightArm")
		RightForeArm_node = root_node.FindChild("mixamorig:RightForeArm")
		RightHand_node = root_node.FindChild("mixamorig:RightHand")

		# for 循环范围 帧：[0, frameNumber]
		for i in range(0, frameNumber + 1):
			# 获取坐标
			lTimeIncrement = FbxTime()
			lTimeIncrement.SetTime(0, 0, 0, i, 0, lScene.GetGlobalSettings().GetTimeMode())

			# 右臂局部计算
			# 获取 RightArm RightForeArm RightHand 的局部坐标
			RightArmLclTranslation = lScene.GetAnimationEvaluator().GetNodeLocalTranslation(RightArm_node, FbxTime(lTimeIncrement.Get()))
			RightForeArmLclTranslation = lScene.GetAnimationEvaluator().GetNodeLocalTranslation(RightForeArm_node, FbxTime(lTimeIncrement.Get()))
			RightHandLclTranslation = lScene.GetAnimationEvaluator().GetNodeLocalTranslation(RightHand_node, FbxTime(lTimeIncrement.Get()))

			# 获取 Spine RightArm RightForeArm 的齐次转换矩阵
			SpineLclTransform = lScene.GetAnimationEvaluator().GetNodeLocalTransform(Spine_node, FbxTime(lTimeIncrement.Get()))
			RightShoulderLclTransform = lScene.GetAnimationEvaluator().GetNodeLocalTransform(RightShoulder_node, FbxTime(lTimeIncrement.Get()))
			RightArmLclTransform = lScene.GetAnimationEvaluator().GetNodeLocalTransform(RightArm_node, FbxTime(lTimeIncrement.Get()))
			RightForeArmLclTransform = lScene.GetAnimationEvaluator().GetNodeLocalTransform(RightForeArm_node, FbxTime(lTimeIncrement.Get()))

			# 计算各结点在 Spine 的局部坐标系下的局部坐标
			RightArmLclTranslationForSpine_HumanFBX.append(SpineLclTransform.MultT(RightShoulderLclTransform.MultT(RightArmLclTranslation)))
			RightForeArmLclTranslationForSpine_HumanFBX.append(SpineLclTransform.MultT(RightShoulderLclTransform.MultT(RightArmLclTransform.MultT(RightForeArmLclTranslation))))
			RightHandLclTranslationForSpine_HumanFBX.append(SpineLclTransform.MultT(RightShoulderLclTransform.MultT(RightArmLclTransform.MultT(RightForeArmLclTransform.MultT(RightHandLclTranslation)))))

			# 计算各结点在 Spine 的局部坐标系下的关节向量并单位化
			SpineToRightArm_HumanFBX.append(
				vectorConversion(RightArmLclTranslationForSpine_HumanFBX[i]))
			RightArmToRightForeArm_HumanFBX.append(
				vectorConversion(RightForeArmLclTranslationForSpine_HumanFBX[i] - RightArmLclTranslationForSpine_HumanFBX[i]))
			RightForeArmToRightHand_HumanFBX.append(
				vectorConversion(RightHandLclTranslationForSpine_HumanFBX[i] - RightForeArmLclTranslationForSpine_HumanFBX[i]))

		print("人体模型 FBX 文件正运动学计算完成！")
	else:
		print("人体模型 FBX 文件正运动学计算失败！")
		sys.exit(1)



def similarity(frameNumber):
	'''根据 Nao 机器人和人体模型的正运动学计算数据，定量计算模仿的相似度'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		TorsoToRShoulder_Similarity.append(arccos(TorsoToRShoulder_Nao[i] * SpineToRightArm_HumanFBX[i]))
		RShoulderToRElbow_Similarity.append(arccos(RShoulderToRElbow_Nao[i] * RightArmToRightForeArm_HumanFBX[i]))
		RElbowToRWrist_Similarity.append(arccos(RElbowToRWrist_Nao[i] * RightForeArmToRightHand_HumanFBX[i]))
	# print('TorsoToRShoulder : average = ' + str(average(TorsoToRShoulder_Similarity))+ ' variance = ' + str(var(TorsoToRShoulder_Similarity)))
	print('RShoulderToRElbow : average = ' + str(average(RShoulderToRElbow_Similarity)) + ' variance = ' + str(var(RShoulderToRElbow_Similarity)))
	print('RElbowToRWrist : average = ' + str(average(RElbowToRWrist_Similarity)) + ' variance = ' + str(var(RElbowToRWrist_Similarity)))


'''########################以上函数用于正运动学计算模仿相似度#########################'''



'''##########################以下函数用于逆运动学模仿运动############################'''
def calculateRotation(humanNode, rotation, lScene, frameNumber):
	'''得到单个身体结点的局部旋转坐标：根据 humanNode 从 fbx 文件中获取此结点所有的局部旋转信息并存入相应的 rotation 列表中'''
	lTimeIncrement = FbxTime()

	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		lTimeIncrement.SetTime(0, 0, 0, i, 0, lScene.GetGlobalSettings().GetTimeMode())
		rotationTemp = lScene.GetAnimationEvaluator().GetNodeLocalRotation(humanNode, FbxTime(lTimeIncrement.Get()))
		# 返回值类型为 FbxVector4
		component = re.search(
			r'fbx.FbxVector4\(([-]?[0-9]+\.[0-9]+), ([-]?[0-9]+\.[0-9]+), ([-]?[0-9]+\.[0-9]+), [-]?[0-9]+\.[0-9]+\)',
			str(rotationTemp), re.M | re.I)
		if component:
			x = float(component.group(1))
			y = float(component.group(2))
			z = float(component.group(3))
			rotation.append(Vector3(x, y, z))



def calculatePosition(humanNode, position, lScene, frameNumber):
	'''得到单个身体结点的全局坐标：根据 humanNode 从 fbx 文件中获取此结点所有的全局位置信息并存入相应的 position 列表中'''
	lTimeIncrement = FbxTime()

	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		lTimeIncrement.SetTime(0, 0, 0, i, 0, lScene.GetGlobalSettings().GetTimeMode())
		positionTemp = lScene.GetAnimationEvaluator().GetNodeGlobalTransform(humanNode, FbxTime(lTimeIncrement.Get()))
						# 返回值类型为 FbxAMatrix
		globalTransform = positionTemp.GetT()
						# 返回值类型为 FbxVector4
		component = re.search(r'fbx.FbxVector4\(([-]?[0-9]+\.[0-9]+), ([-]?[0-9]+\.[0-9]+), ([-]?[0-9]+\.[0-9]+), [-]?[0-9]+\.[0-9]+\)', str(globalTransform), re.M | re.I)
		if component:
			x = float(component.group(1))
			y = float(component.group(2))
			z = float(component.group(3))
			position.append(Vector3(x, y, z))



def getGlobalTransformAndFrameNum(filename):
	'''获取所有关节结点的全局坐标以及 FBX 文件的帧数'''
	(lSdkManager, lScene) = FbxCommon.InitializeSdkObjects()
	if FbxCommon.LoadScene(lSdkManager, lScene, filename):
		# 计算帧数
		AnimationStack = lScene.GetCurrentAnimationStack()
		startTime = AnimationStack.GetLocalTimeSpan().GetStart()
		stopTime = AnimationStack.GetLocalTimeSpan().GetStop()
		timeSpan = stopTime - startTime
		frameRate = FbxTime().GetFrameRate(lScene.GetGlobalSettings().GetTimeMode())
		frameNumber = timeSpan.GetSecondDouble() * frameRate
		frameNumber = int(frameNumber)
		print("总帧数 = " + str(frameNumber))

		# 获取全局坐标
		# 获取身体树状结构的根节点
		root_node = lScene.GetRootNode()
		# print(root_node.GetChild(0).GetChildCount())

		# 适用于 www.mixamo.com 网站的 FBX 文件
		# Head_node Neck_node LeftFoot RightFoot 获取 LclRotation
		Head_node			= root_node.FindChild("mixamorig:Head")
		calculateRotation(Head_node, Head_Rotation, lScene, frameNumber)

		Neck_node			= root_node.FindChild("mixamorig:Neck")
		calculateRotation(Neck_node, Neck_Rotation, lScene, frameNumber)

		LeftHand_node		= root_node.FindChild("mixamorig:LeftHand")
		calculateRotation(LeftHand_node, LeftHand_Rotation, lScene, frameNumber)

		RightHand_node		= root_node.FindChild("mixamorig:RightHand")
		calculateRotation(RightHand_node, RightHand_Rotation, lScene, frameNumber)

		LeftFoot_node		= root_node.FindChild("mixamorig:LeftFoot")
		calculateRotation(LeftFoot_node, LeftFoot_Rotation, lScene, frameNumber)

		RightFoot_node		= root_node.FindChild("mixamorig:RightFoot")
		calculateRotation(RightFoot_node, RightFoot_Rotation, lScene, frameNumber)

		# 其余获取GblTranslation
		Spine_node		 	= root_node.FindChild("mixamorig:Spine")
		calculatePosition(Spine_node, Spine_position, lScene, frameNumber)

		Spine1_node		 = root_node.FindChild("mixamorig:Spine1")
		calculatePosition(Spine1_node, Spine1_position, lScene, frameNumber)

		Spine2_node		 = root_node.FindChild("mixamorig:Spine2")
		calculatePosition(Spine2_node, Spine2_position, lScene, frameNumber)

		LeftShoulder_node = root_node.FindChild("mixamorig:LeftShoulder")
		calculatePosition(LeftShoulder_node, LeftShoulder_position, lScene, frameNumber)

		RightShoulder_node = root_node.FindChild("mixamorig:RightShoulder")
		calculatePosition(RightShoulder_node, RightShoulder_position, lScene, frameNumber)

		LeftArm_node = root_node.FindChild("mixamorig:LeftArm")
		calculatePosition(LeftArm_node, LeftArm_position, lScene, frameNumber)

		RightArm_node = root_node.FindChild("mixamorig:RightArm")
		calculatePosition(RightArm_node, RightArm_position, lScene, frameNumber)

		LeftForeArm_node = root_node.FindChild("mixamorig:LeftForeArm")
		calculatePosition(LeftForeArm_node, LeftForeArm_position, lScene, frameNumber)

		RightForeArm_node = root_node.FindChild("mixamorig:RightForeArm")
		calculatePosition(RightForeArm_node, RightForeArm_position, lScene, frameNumber)

		LeftHand_node = root_node.FindChild("mixamorig:LeftHand")
		calculatePosition(LeftHand_node, LeftHand_position, lScene, frameNumber)

		RightHand_node = root_node.FindChild("mixamorig:RightHand")
		calculatePosition(RightHand_node, RightHand_position, lScene, frameNumber)

		Hips_node = root_node.FindChild("mixamorig:Hips")
		calculatePosition(Hips_node, Hips_position, lScene, frameNumber)

		LeftUpLeg_node = root_node.FindChild("mixamorig:LeftUpLeg")
		calculatePosition(LeftUpLeg_node, LeftUpLeg_position, lScene, frameNumber)

		RightUpLeg_node = root_node.FindChild("mixamorig:RightUpLeg")
		calculatePosition(RightUpLeg_node, RightUpLeg_position, lScene, frameNumber)

		LeftLeg_node = root_node.FindChild("mixamorig:LeftLeg")
		calculatePosition(LeftLeg_node, LeftLeg_position, lScene, frameNumber)

		RightLeg_node = root_node.FindChild("mixamorig:RightLeg")
		calculatePosition(RightLeg_node, RightLeg_position, lScene, frameNumber)

		LeftFoot_node = root_node.FindChild("mixamorig:LeftFoot")
		calculatePosition(LeftFoot_node, LeftFoot_position, lScene, frameNumber)

		RightFoot_node = root_node.FindChild("mixamorig:RightFoot")
		calculatePosition(RightFoot_node, RightFoot_position, lScene, frameNumber)

		'''
		# 适用于学长给的 FBX 文件
		Spine_node		 = root_node.FindChild("Spine")
		calculatePosition(Spine_node, Spine_position, lScene, frameNumber)

		Spine1_node		 = root_node.FindChild("Spine1")
		calculatePosition(Spine1_node, Spine1_position, lScene, frameNumber)

		Spine2_node		 = root_node.FindChild("Spine2")
		calculatePosition(Spine2_node, Spine2_position, lScene, frameNumber)

		LeftShoulder_node = root_node.FindChild("LeftShoulder")
		calculatePosition(LeftShoulder_node, LeftShoulder_position, lScene, frameNumber)

		RightShoulder_node = root_node.FindChild("RightShoulder")
		calculatePosition(RightShoulder_node, RightShoulder_position, lScene, frameNumber)

		LeftArm_node = root_node.FindChild("LeftArm")
		calculatePosition(LeftArm_node, LeftArm_position, lScene, frameNumber)

		RightArm_node = root_node.FindChild("RightArm")
		calculatePosition(RightArm_node, RightArm_position, lScene, frameNumber)

		LeftForeArm_node = root_node.FindChild("LeftForeArm")
		calculatePosition(LeftForeArm_node, LeftForeArm_position, lScene, frameNumber)

		RightForeArm_node = root_node.FindChild("RightForeArm")
		calculatePosition(RightForeArm_node, RightForeArm_position, lScene, frameNumber)

		LeftHand_node = root_node.FindChild("LeftHand")
		calculatePosition(LeftHand_node, LeftHand_position, lScene, frameNumber)

		RightHand_node = root_node.FindChild("RightHand")
		calculatePosition(RightHand_node, RightHand_position, lScene, frameNumber)

		Hips_node = root_node.FindChild("Hips")
		calculatePosition(Hips_node, Hips_position, lScene, frameNumber)

		LeftUpLeg_node = root_node.FindChild("LeftUpLeg")
		calculatePosition(LeftUpLeg_node, LeftUpLeg_position, lScene, frameNumber)

		RightUpLeg_node = root_node.FindChild("RightUpLeg")
		calculatePosition(RightUpLeg_node, RightUpLeg_position, lScene, frameNumber)

		LeftLeg_node = root_node.FindChild("LeftLeg")
		calculatePosition(LeftLeg_node, LeftLeg_position, lScene, frameNumber)

		RightLeg_node = root_node.FindChild("RightLeg")
		calculatePosition(RightLeg_node, RightLeg_position, lScene, frameNumber)

		LeftFoot_node = root_node.FindChild("LeftFoot")
		calculatePosition(LeftFoot_node, LeftFoot_position, lScene, frameNumber)

		RightFoot_node = root_node.FindChild("RightFoot")
		calculatePosition(RightFoot_node, RightFoot_position, lScene, frameNumber)
		
		LeftFoot_End_node = root_node.FindChild("LeftFoot_End")
		calculatePosition(LeftFoot_End_node, LeftFoot_End_position, lScene, frameNumber)
		
		RightFoot_End_node = root_node.FindChild("RightFoot_End")
		calculatePosition(RightFoot_End_node, RightFoot_End_position, lScene, frameNumber)
		'''

		# 返回 FBX 文件的总帧数
		return frameNumber
	else :
		print("从二进制 FBX 文件中获取全局坐标时，载入 FBX 文件失败！")
		sys.exit(1)



def calculateLShoulderRoll(frameNumber):
	'''计算 LShoulderRoll 的角度——向量与向量的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		vector1 = LeftForeArm_position[i] - LeftArm_position[i]
		vector2 = RightArm_position[i] - LeftArm_position[i]
		angle = (vector1 * vector2) / (vector1.length() * vector2.length())
		angle = math.degrees(math.acos(angle))

		# 角度映射 正确 1
		angle = angle - 90
		if angle > 76 :
			angle = 76
		elif angle < -18 :
			angle = -18
		LShoulderRoll_Angle.append(angle)



def calculateRShoulderRoll(frameNumber):
	'''计算 RShoulderRoll 的角度——向量与向量的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		vector1 = RightForeArm_position[i] - RightArm_position[i]
		vector2 = LeftArm_position[i] - RightArm_position[i]
		angle = (vector1 * vector2) / (vector1.length() * vector2.length())
		angle = math.degrees(math.acos(angle))

		# 角度映射 正确 1
		angle = 90 - angle
		if angle > 18 :
			angle = 18
		elif angle < -76 :
			angle = -76
		RShoulderRoll_Angle.append(angle)



def calculateLShoulderPitch(frameNumber):
	'''计算 LShoulderPitch 关节的旋转角度——面与面的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		# 求第一个平面的法向量
		vector1 = LeftArm_position[i] - LeftForeArm_position[i]
		vector2 = RightArm_position[i] - LeftArm_position[i]
		normalVector1 = vector1 ** vector2
		vector3 = LeftArm_position[i] - Spine_position[i]
		vector4 = vector2
		normalVector2 = vector3 ** vector4
		angle = (normalVector1 * normalVector2) / (normalVector1.length() * normalVector2.length())
		angle = math.degrees(math.acos(angle))

		'''
		Nao规定：手臂平举为 0°，向上举为 0° ~ -119.5°，向下举为 0° ~ 119.5°
		角度映射：
		1. 如果面EFI法向量向前向下（手臂向后向下），则平面夹角为 0° ~ 29.5°（另一个方向的夹角），对应 Nao 为 90°  ~  119.5°
		2. 如果面EFI法向量向前向上（手臂向前向下），则平面夹角为 0° ~ 90°，对应 Nao 为 90°  ~  0°
		3. 如果面EFI法向量向后向上（手臂向前向上），则平面夹角为 90° ~ 180°，对应 Nao 为 0° ~ -90°
		4. 如果面EFI法向量向后向下（手臂向后向下），则平面法向量夹角为 180° ~ 150.5°，对应 Nao为 -90° ~  -119.5°
		面EFI法向量向前代表其Z坐标与面CEI法向量同向
		面EFI法向量向上代表其Y坐标大于0
		'''
		if normalVector1.z * normalVector2.z > 0:
			# 向前
			if normalVector1.y < 0:
				# 向下
				angle = 90 + angle
			else:
				# 向上
				angle = 90 - angle
		else:
			# 向后
			if normalVector1.y > 0:
				# 向上
				angle = 90 - angle
			else:
				# 向下
				angle = angle - 270

		if angle > 119.5 :
			angle = 119.5
		elif angle < -119.5 :
			angle = -119.5
		LShoulderPitch_Angle.append(angle)



def calculateRShoulderPitch(frameNumber):
	'''计算 RShoulderPitch 关节的旋转角度——面与面的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		# 求第一个平面的法向量
		vector1 = RightArm_position[i] - LeftArm_position[i]
		vector2 = RightForeArm_position[i] - RightArm_position[i]
		normalVector1 = vector1 ** vector2
		vector3 = LeftArm_position[i] - Spine_position[i]
		vector4 = vector1
		normalVector2 = vector3 ** vector4
		angle = (normalVector1 * normalVector2) / (normalVector1.length() * normalVector2.length())
		angle = math.degrees(math.acos(angle))

		'''
		Nao规定：手臂平举为 0°，向上举为 0° ~ -119.5°，向下举为 0° ~ 119.5°
		角度映射：
		1. 如果面JEI法向量向前向下（手臂向后向下），则平面夹角为 0° ~ 29.5°（另一个方向的夹角），对应 Nao 为 90°  ~  119.5°
		2. 如果面JEI法向量向前向上（手臂向前向下），则平面夹角为 0° ~ 90°，对应 Nao 为 90°  ~  0°
		3. 如果面JEI法向量向后向上（手臂向前向上），则平面夹角为 90° ~ 180°，对应 Nao 为 0° ~ -90°
		4. 如果面JEI法向量向后向下（手臂向后向下），则平面法向量夹角为 180° ~ 150.5°，对应 Nao为 -90° ~  -119.5°
		面JEI法向量向前代表其Z坐标与面CEI法向量同向
		面JEI法向量向上代表其Y坐标大于0
		'''
		if normalVector1.z * normalVector2.z > 0:
			# 向前
			if normalVector1.y < 0:
				# 向下
				angle = 90 + angle
			else:
				# 向上
				angle = 90 - angle
		else:
			# 向后
			if normalVector1.y > 0:
				# 向上
				angle = 90 - angle
			else:
				# 向下
				angle = angle - 270

		if angle > 119.5 :
			angle = 119.5
		elif angle < -119.5 :
			angle = -119.5
		RShoulderPitch_Angle.append(angle)



def calculateLElbowRoll(frameNumber):
	'''计算 LElbowRoll 关节的旋转角度——向量与向量的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		vector1 = LeftForeArm_position[i] - LeftArm_position[i]
		vector2 = LeftHand_position[i] - LeftForeArm_position[i]
		angle = (vector1 * vector2) / (vector1.length() * vector2.length())
		angle = math.degrees(math.acos(angle))

		# 角度映射
		angle = -angle
		if angle > -2 :
			angle = -2
		elif angle < -88.5 :
			angle = -88.5
		LElbowRoll_Angle.append(angle)



def calculateRElbowRoll(frameNumber):
	'''计算 RElbowRoll 关节的旋转角度——向量与向量的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		vector1 = RightForeArm_position[i] - RightArm_position[i]
		vector2 = RightHand_position[i] - RightForeArm_position[i]
		angle = (vector1 * vector2) / (vector1.length() * vector2.length())
		angle = math.degrees(math.acos(angle))

		# 角度映射 正确 1
		if angle > 88.5 :
			angle = 88.5
		elif angle < 2 :
			angle = 2
		RElbowRoll_Angle.append(angle)



def calculateLElbowYaw(frameNumber):
	'''计算 LElbowYaw 关节的旋转角度——面与面的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		vector1 = LeftForeArm_position[i] - LeftHand_position[i]
		vector2 = LeftArm_position[i] - LeftForeArm_position[i]
		normalVector1 = vector1 ** vector2
		vector3 = vector2
		vector4 = RightArm_position[i] - LeftArm_position[i]
		normalVector2 = vector3 ** vector4
		angle = (normalVector1 * normalVector2) / (normalVector1.length() * normalVector2.length())
		angle = math.degrees(math.acos(angle))	# 得到的是两平面法向量的夹角

		'''
		角度映射：
		Nao规定：双臂前平举，手心向下为 0°，手心向外旋转至朝上为 0° ~ 119.5°，手心向内旋转至朝上为 0° ~ -119.5°
		以平面EFG法向量的朝向为划分依据。
		1. 如果EFG法向量朝向——从RShoulder指向LShoulder，则手心向内旋转；0° ~ -119.5°
		2. 如果EFG法向量朝向——从LShoulder指向RShoulder，则手心向外旋转：0° ~  119.5°
		'''
		LeftToRightVector = RightShoulder_position[i] - LeftShoulder_position[i]
		if normalVector1.x * LeftToRightVector.x < 0:
			angle = -angle

		if angle > 119.5 :
			angle = 119.5
		elif angle < -119.5 :
			angle = -119.5
		LElbowYaw_Angle.append(angle)



def calculateRElbowYaw(frameNumber):
	'''计算 RElbowYaw 关节的旋转角度——面与面的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		vector1 = RightForeArm_position[i] - RightArm_position[i]
		vector2 = RightHand_position[i] - RightForeArm_position[i]
		normalVector1 = vector1 ** vector2
		vector3 = RightArm_position[i] - LeftArm_position[i]
		vector4 = vector1
		normalVector2 = vector3 ** vector4
		angle = (normalVector1 * normalVector2) / (normalVector1.length() * normalVector2.length())
		angle = math.degrees(math.acos(angle))

		'''
		角度映射：
		Nao规定：双臂前平举，手心向下为 0°，手心向外旋转至朝上为 0° ~ -119.5°，手心向内旋转至朝上为 0° ~ 119.5°
		以平面IJK法向量的朝向为划分依据。
		1. 如果IJK法向量朝向——从RShoulder指向LShoulder，则手心向外旋转；0° ~ -119.5°
		2. 如果IJK法向量朝向——从LShoulder指向RShoulder，则手心向内旋转：0° ~  119.5°
		'''
		LeftToRightVector = RightShoulder_position[i] - LeftShoulder_position[i]
		if normalVector1.x * LeftToRightVector.x < 0:
			angle = -angle

		if angle > 119.5 :
			angle = 119.5
		elif angle < -119.5 :
			angle = -119.5
		RElbowYaw_Angle.append(angle)



def calculateLHipRoll(frameNumber):
	'''计算 LHipRoll 关节的旋转角度——向量与向量的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		vector1 = LeftLeg_position[i] - LeftUpLeg_position[i]
		vector2 = RightUpLeg_position[i] - LeftUpLeg_position[i]
		angle = (vector1 * vector2) / (vector1.length() * vector2.length())
		angle = math.degrees(math.acos(angle))

		# 角度映射
		angle = angle - 90

		if angle > 21.74 :
			angle = 21.74
		elif angle < -45.29 :
			angle = -45.29
		LHipRoll_Angle.append(angle)



def calculateRHipRoll(frameNumber):
	'''计算 RHipRoll 关节的旋转角度——向量与向量的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		vector1 = RightLeg_position[i] - RightUpLeg_position[i]
		vector2 = LeftUpLeg_position[i] - RightUpLeg_position[i]
		angle = (vector1 * vector2) / (vector1.length() * vector2.length())
		angle = math.degrees(math.acos(angle))

		# 角度映射
		angle = 90 - angle

		if angle > 21.74 :
			angle = 21.74
		elif angle < -45.29 :
			angle = -45.29
		RHipRoll_Angle.append(angle)



def calculateLHipPitch(frameNumber):
	'''计算 LHipPitch 关节的旋转角度——向量与平面的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		vector1 = LeftLeg_position[i] - LeftUpLeg_position[i]
		vector2 = RightUpLeg_position[i] - Spine_position[i]
		vector3 = LeftUpLeg_position[i] - RightUpLeg_position[i]
		normalVector1 = vector2 ** vector3
		angle = (vector1 * normalVector1) / (vector1.length() * normalVector1.length())
		angle = math.degrees(math.acos(angle))

		'''
		角度映射
		根据向量 MN 与面 CMQ 的法向量夹角为划分依据。
		1. 当夹角小于 90° 时，说明大腿向前迈，故角度 0° ~ -88°
		2. 当夹角大于 90° 时，说明大腿向后迈，故角度 0° ~ 27.73°
		'''
		if angle < 90:
			angle = angle - 90
		else:
			angle = angle - 90

		if angle > 27.73 :
			angle = 27.73
		elif angle < -88 :
			angle = -88
		LHipPitch_Angle.append(angle)



def calculateRHipPitch(frameNumber):
	'''计算 RHipPitch 关节的旋转角度——向量与平面的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		vector1 = RightLeg_position[i] - RightUpLeg_position[i]
		vector2 = RightUpLeg_position[i] - Spine_position[i]
		vector3 = LeftUpLeg_position[i] - RightUpLeg_position[i]
		normalVector1 = vector2 ** vector3
		angle = (vector1 * normalVector1) / (vector1.length() * normalVector1.length())
		angle = math.degrees(math.acos(angle))

		'''
		角度映射
		根据向量 MN 与面 CMQ 的法向量夹角为划分依据。
		1. 当夹角小于 90° 时，说明大腿向前迈，故角度 0° ~ -88°
		2. 当夹角大于 90° 时，说明大腿向后迈，故角度 0° ~ 27.73°
		'''
		if angle < 90:
			angle = angle - 90
		else:
			angle = angle - 90
		# angle = -angle

		if angle > 27.73 :
			angle = 27.73
		elif angle < -88 :
			angle = -88
		RHipPitch_Angle.append(angle)



def calculateLKneePitch(frameNumber):
	'''计算 LKneePitch 关节的旋转角度——向量与向量的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		vector1 = LeftLeg_position[i] - LeftUpLeg_position[i]
		vector2 = LeftFoot_position[i] - LeftLeg_position[i]
		angle = (vector1 * vector2) / (vector1.length() * vector2.length())
		angle = math.degrees(math.acos(angle))

		'''
		角度映射：
		Nao规定：当大腿与小腿在同一竖直线上时，定义为0°，小腿往前抬时，角度 0° ~ -5.29°，小腿往后弯时，角度 0° ~ 121.04°
		1. 当NO向量的Z坐标与法向量的Z坐标同号（即小腿往前抬）时，角度 0° ~ -5.29°
		2. 当NO向量的Z坐标与法向量的Z坐标异号（即小腿往后弯）时，角度 0° ~ 121.04°
		'''
		vector3 = RightUpLeg_position[i] - Spine_position[i]
		vector4 = LeftUpLeg_position[i] - RightUpLeg_position[i]
		normalVector = vector3 ** vector4
		if vector2.z * normalVector.z > 0:
			angle = -angle

		if angle > 121.04 :
			angle = 121.04
		elif angle < -5.29 :
			angle = -5.29

		LKneePitch_Angle.append(angle)



def calculateRKneePitch(frameNumber):
	'''计算 RKneePitch 关节的旋转角度——向量与向量的夹角'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		vector1 = RightLeg_position[i] - RightUpLeg_position[i]
		vector2 = RightFoot_position[i] - RightLeg_position[i]
		angle = (vector1 * vector2) / (vector1.length() * vector2.length())
		angle = math.degrees(math.acos(angle))

		'''
		角度映射：
		Nao规定：当大腿与小腿在同一竖直线上时，定义为0°，小腿往前抬时，角度 0° ~ -5.29°，小腿往后弯时，角度 0° ~ 121.04°
		1. 当RS向量的Z坐标与法向量的Z坐标同号（即小腿往前抬）时，角度 0° ~ -5.29°
		2. 当RS向量的Z坐标与法向量的Z坐标异号（即小腿往后弯）时，角度 0° ~ 121.04°
		'''
		vector3 = RightUpLeg_position[i] - Spine_position[i]
		vector4 = LeftUpLeg_position[i] - RightUpLeg_position[i]
		normalVector = vector3 ** vector4
		if vector2.z * normalVector.z > 0:
			angle = - angle

		if angle > 121.04 :
			angle = 121.04
		elif angle < -5.29 :
			angle = -5.29
		RKneePitch_Angle.append(angle)



def calculateHeadPitch(frameNumber):
	'''计算 HeadPitch 关节的旋转角度——直接使用 FBX 文件的 LclRotation'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		angle = Head_Rotation[i].x + Neck_Rotation[i].x
		angle = -angle

		if angle > 29.5 :
			angle = 29.5
		elif angle < -38.5 :
			angle = -38.5
		HeadPitch_Angle.append(angle)



def calculateHeadYaw(frameNumber):
	'''计算 HeadYaw 关节的旋转角度——直接使用 FBX 文件的 LclRotation'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		angle = Head_Rotation[i].y + Neck_Rotation[i].y

		if angle > 119.5 :
			angle = 119.5
		elif angle < -119.5 :
			angle = -119.5
		HeadYaw_Angle.append(angle)



def calculateLWristYaw(frameNumber):
	'''计算 LWristYaw 关节的旋转角度——直接使用 FBX 文件的 LclRotation'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		angle = LeftHand_Rotation[i].x

		if angle > 104.5 :
			angle = 104.5
		elif angle < -104.5 :
			angle = -104.5
		LWristYaw_Angle.append(angle)



def calculateRWristYaw(frameNumber):
	'''计算 RWristYaw 关节的旋转角度——直接使用 FBX 文件的 LclRotation'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		angle = -RightHand_Rotation[i].x

		if angle > 104.5 :
			angle = 104.5
		elif angle < -104.5 :
			angle = -104.5
		RWristYaw_Angle.append(angle)



def calculateLAnklePitch(frameNumber):
	'''计算 LAnklePitch 关节的旋转角度——直接使用 FBX 文件的 LclRotation'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		angle = LeftFoot_Rotation[i].x

		if angle > 53.4 :
			angle = 53.4
		elif angle < -67.97 :
			angle = -67.97
		LAnklePitch_Angle.append(angle)



def calculateLAnkleRoll(frameNumber):
	'''计算 LAnkleRoll 关节的旋转角度——直接使用 FBX 文件的 LclRotation'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		angle = LeftFoot_Rotation[i].z

		if angle > 53.4 :
			angle = 53.4
		elif angle < -67.97 :
			angle = -67.97
		LAnkleRoll_Angle.append(angle)



def calculateRAnklePitch(frameNumber):
	'''计算 RAnklePitch 关节的旋转角度——直接使用 FBX 文件的 LclRotation'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		angle = RightFoot_Rotation[i].x

		if angle > 53.4 :
			angle = 53.4
		elif angle < -67.97 :
			angle = -67.97
		RAnklePitch_Angle.append(angle)



def calculateRAnkleRoll(frameNumber):
	'''计算 RAnkleRoll 关节的旋转角度——直接使用 FBX 文件的 LclRotation'''
	# for 循环范围 帧：[0, frameNumber]
	for i in range(0, frameNumber + 1):
		angle = RightFoot_Rotation[i].z

		if angle > 53.4 :
			angle = 53.4
		elif angle < -67.97 :
			angle = -67.97
		RAnkleRoll_Angle.append(angle)



def calculateAllAngle(frameNumber):
	# 计算上肢关节的角度
	calculateLShoulderRoll(frameNumber)
	calculateRShoulderRoll(frameNumber)
	calculateLShoulderPitch(frameNumber)
	calculateRShoulderPitch(frameNumber)
	calculateLElbowRoll(frameNumber)
	calculateRElbowRoll(frameNumber)
	calculateLElbowYaw(frameNumber)
	calculateRElbowYaw(frameNumber)

	# 计算下肢关节的角度
	calculateLHipRoll(frameNumber)
	calculateRHipRoll(frameNumber)
	calculateLHipPitch(frameNumber)
	calculateRHipPitch(frameNumber)
	calculateLKneePitch(frameNumber)
	calculateRKneePitch(frameNumber)

	# 计算端部关节的角度
	calculateHeadPitch(frameNumber)
	calculateHeadYaw(frameNumber)
	calculateLWristYaw(frameNumber)
	calculateRWristYaw(frameNumber)
	calculateLAnklePitch(frameNumber)
	calculateLAnkleRoll(frameNumber)
	calculateRAnklePitch(frameNumber)
	calculateRAnkleRoll(frameNumber)

'''##########################以上函数用于逆运动学模仿运动############################'''



'''###########################以下函数用于生成 XAR 文件#############################'''

def writeXARBody(XARFile):
	'''把每个关节对应的数据均写入 xar 文件中'''
	# 上肢
	writeXARNode("LShoulderRoll", LShoulderRoll_Angle, XARFile)
	writeXARNode("LShoulderPitch", LShoulderPitch_Angle, XARFile)
	writeXARNode("LElbowRoll", LElbowRoll_Angle, XARFile)
	writeXARNode("LElbowYaw", LElbowYaw_Angle, XARFile)
	writeXARNode("RShoulderRoll", RShoulderRoll_Angle, XARFile)
	writeXARNode("RShoulderPitch", RShoulderPitch_Angle, XARFile)
	writeXARNode("RElbowYaw", RElbowYaw_Angle, XARFile)
	writeXARNode("RElbowRoll", RElbowRoll_Angle, XARFile)

	# 下肢
	writeXARNode("LHipPitch", LHipPitch_Angle, XARFile)
	writeXARNode("RHipPitch", RHipPitch_Angle, XARFile)
	writeXARNode("LHipRoll", LHipRoll_Angle, XARFile)
	writeXARNode("RHipRoll", RHipRoll_Angle, XARFile)
	writeXARNode("LKneePitch", LKneePitch_Angle, XARFile)
	writeXARNode("RKneePitch", RKneePitch_Angle, XARFile)

	# 端部关节
	writeXARNode("HeadPitch", HeadPitch_Angle, XARFile)
	writeXARNode("HeadYaw", HeadYaw_Angle, XARFile)
	writeXARNode("LWristYaw", LWristYaw_Angle, XARFile)
	writeXARNode("RWristYaw", RWristYaw_Angle, XARFile)
	writeXARNode("LAnklePitch", LAnklePitch_Angle, XARFile)
	writeXARNode("LAnkleRoll", LAnkleRoll_Angle, XARFile)
	writeXARNode("RAnklePitch", RAnklePitch_Angle, XARFile)
	writeXARNode("RAnkleRoll", RAnkleRoll_Angle, XARFile)

'''###########################以上函数用于生成 XAR 文件#############################'''

