import sys
import easygui
from writeXAR2 import *							# 写 XAR 文件
from inverseKinematics import *



def main():
	# 选择 fbx 文件并打开
	fileName = easygui.fileopenbox()
	if fileName:
		print(fileName.split('\\')[-1])
		frameNumber = getGlobalTransformAndFrameNum(fileName)
		# 逆运动学计算
		calculateAllAngle(frameNumber)
	else:
		print("FBX File not selected!")
		sys.exit(1)

	# 将角度写入 xar 文件中
	XARFile = open(r"behavior.xar", "w")
	if XARFile:
		print("Start writing XAR file!")
		writeXARHead(XARFile, frameNumber)
		writeXARBody(XARFile)
		writeXARFoot(XARFile)
		XARFile.close()
		print("XAR file write completed!")
	else:
		print("Failed to create XAR file!")
		sys.exit(1)

	# 对此次模拟的相似度进行评估
	print("\n对此次模拟进行评估：")
	positiveKinematics_HumanFbx(fileName, frameNumber)
	positiveKinematics_Nao(frameNumber)
	similarity(frameNumber)



if __name__ == '__main__':
	# 导入 fbx 和 FbxCommon 第三方模块
	try:
		import FbxCommon
		from fbx import *
	except ImportError:
		print("Error: module FbxCommon and/or fbx failed to import.")
		print(
			"Copy the files located in the compatible sub-folder lib/python<version> into your python interpreter site-packages folder.")
		sys.exit(1)
	else:
		print("Import FbxCommon and fbx successfully!")
		main()