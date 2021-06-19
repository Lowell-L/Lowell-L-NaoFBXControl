def writeXARHead(XARFile, frameNumber):
	'''写 xar 文件的头'''
	# TODO： <Timeline enable="1" fps="25" start_frame="1" end_frame="-1" size="67"> 需要更改的地方: fps
	XARFile.write('''<?xml version="1.0" encoding="UTF-8" ?>
	<ChoregrapheProject xmlns="http://www.aldebaran-robotics.com/schema/choregraphe/project.xsd" xar_version="3">
		<Box name="root" id="-1" localization="8" tooltip="Root box of Choregraphe&apos;s behavior. Highest level possible." x="0" y="0">
			<bitmap>media/images/box/root.png</bitmap>
			<script language="4">
				<content>
					<![CDATA[]]>
				</content>
			</script>
			<Input name="onLoad" type="1" type_size="1" nature="0" inner="1" tooltip="Signal sent when diagram is loaded." id="1" />
			<Input name="onStart" type="1" type_size="1" nature="2" inner="0" tooltip="Box behavior starts when a signal is received on this input." id="2" />
			<Input name="onStop" type="1" type_size="1" nature="3" inner="0" tooltip="Box behavior stops when a signal is received on this input." id="3" />
			<Output name="onStopped" type="1" type_size="1" nature="1" inner="0" tooltip="Signal sent when box behavior is finished." id="4" />
			<Timeline enable="0">
				<BehaviorLayer name="behavior_layer1">
					<BehaviorKeyframe name="keyframe1" index="1">
						<Diagram scale="100">
							<Box name="Timeline" id="2" localization="8" tooltip="This box is empty (contains a single motion layer with no motor position&#x0A;defined in it) and should be used to create any animation you would like." x="667" y="113">
								<bitmap>media/images/box/movement/move.png</bitmap>
								<script language="4">
									<content>
										<![CDATA[]]>
									</content>
								</script>
								<Input name="onLoad" type="1" type_size="1" nature="0" inner="1" tooltip="Signal sent when diagram is loaded." id="1" />
								<Input name="onStart" type="1" type_size="1" nature="2" inner="0" tooltip="Box behavior starts when a signal is received on this input." id="2" />
								<Input name="onStop" type="1" type_size="1" nature="3" inner="0" tooltip="Box behavior stops when a signal is received on this input." id="3" />
								<Output name="onStopped" type="1" type_size="1" nature="1" inner="0" tooltip="Signal sent when box behavior is finished." id="4" />
								<Timeline enable="1" fps="30" start_frame="1" end_frame="-1" size="''' + str(frameNumber) + '''">
									<ActuatorList model="">
	''')
	print("XAR:" + str(frameNumber))



def writeXARNode(naoNodeName, angleList, XARFile):
	'''根据传入的 Nao 机器人关节名称向 xar 文件中写入此单个结点对应的关键帧数据'''
	firstLine = '                                    <ActuatorCurve name="value" actuator="' + naoNodeName + '" recordable="1" mute="0" unit="0">\n'
	XARFile.write(firstLine)

	frame = 1
	# 循环 angleList 判断角度是否处于范围内 把角度写入 XAR 每一帧中
	for angle in angleList :
		XARFile.write('                                        <Key frame="' + str(frame) + '" value="' + str(angle) + '" />\n')
		frame += 1  # 留出一帧的间隔 留给 Choregraphe 优化的余地
		# 后期可以视情况加大关键帧间隔，需要更改 fps
	# 按行读取，逗号分隔
	XARFile.write('''                                    </ActuatorCurve>\n''')



def writeXARFoot(XARFile):
	XARFile.write('''                                </ActuatorList>
								</Timeline>
							</Box>
							<Link inputowner="2" indexofinput="2" outputowner="0" indexofoutput="2" />
							<Link inputowner="0" indexofinput="4" outputowner="2" indexofoutput="4" />
						</Diagram>
					</BehaviorKeyframe>
				</BehaviorLayer>
			</Timeline>
		</Box>
	</ChoregrapheProject>''')
