import math


class Vector3:
	def __init__(self, x = 0, y = 0, z = 0):
		'''构造函数'''
		self.x = x
		self.y = y
		self.z = z

	def __add__(self, obj):
		'''重载 + 作为加号'''
		return Vector3(self.x + obj.x, self.y + obj.y, self.z + obj.z)

	def __sub__(self, obj):
		'''重载 - 作为减号'''
		return Vector3(self.x - obj.x, self.y - obj.y, self.z - obj.z)

	def __mul__(self, obj):
		'''重载 * 作为点乘'''
		return self.x * obj.x + self.y * obj.y + self.z * obj.z

	def __pow__(self, obj):
		'''重载 ** 作为叉乘'''
		return Vector3(self.y * obj.z - obj.y * self.z, self.z * obj.x - self.x * obj.z, self.x * obj.y - obj.x * self.y)

	def __str__(self):
		'''供print打印的字符串'''
		return '(' + str(self.x) + ',' + str(self.y) + ',' + str(self.z) + ')'

	def unitization(self):
		'''向量单位化'''
		norm = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
		return Vector3(self.x/norm, self.y/norm, self.z/norm)

	def length(self):
		return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
