#create the environment by importing necessary libraries
import numpy as np
import math
from collections import defaultdict
from heapq import *
import cv2


#create the class Node 
class (object):

	#initialize the variables for start, goalm clearance, radius and position
	def __init__(self, start, goal, position , clearance , radius ):
		self.start = start
		self.goal = goal
		self.position = position
		self.clearance = clearance
		self.radius = radius
		self.cost = 0
		self.y = 0
		self.x = 0
		self.numRows = 250
		self.numCols = 400

	# define the position for point robot	
	def __eq__(self, other):
		return self.position == other.position

	# check if the clearance for the point robot is valid
	def validClearance(self, currRow, currCol):
		sum = self.clearance + self.radius
		return (currRow >= (1 + sum) and currRow <= (250 - sum) and currCol >= (1 + sum) and currCol <= (400 - sum))

	#check the obstacle space and plot it 
	def obstacle(self, row, col):
		sum = self.clearance + self.radius
		sqrt_rc = 1.4142 * sum

		# check circle
		dist1 = ((row - 150) * (row - 150) + (col - 225) * (col - 225)) - ((25 + sum) * (25 + sum)) 

		#check egg
		dist2 = ((((row - 100) * (row - 100)) / ((20 + sum) * (20 + sum))) + (((col - 150) * (col - 150)) / ((40 + sum) * (40 + sum)))) - 1
        
		#polygon
		(x1, y1) = (120 - (2.62 * sum), 20 - (1.205 * sum))
		(x2, y2) = (150 - sqrt_rc, 50)
		(x3, y3) = (185 + sum, 25 - (sum * 0.9247))
		first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
		second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
		third = ((col - y3) * (x1 - x3)) - ((y1 - y3) * (row - x3))
		dist3 = 1
		if(first <= 0 and second <= 0 and third <= 0):
			dist3 = 0
		(x1, y1) = (150 - sqrt_rc, 50)
		(x2, y2) = (185 + sum, 25 - (sum * 0.9247))
		(x3, y3) = (185 + sum, 75 + (sum * 0.5148))
		first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
		second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
		third = ((col - y3) * (x1 - x3)) - ((y1 - y3) * (row - x3))
		dist4 = 1
		if(first >= 0 and second >= 0 and third >= 0):
			dist4 = 0


		(x1, y1) = (10 - sqrt_rc, 225)
		(x2, y2) = (25, 250 - sqrt_rc)
		(x3, y3) = (40 + sqrt_rc, 225)
		(x4, y4) = (25, 250 + sqrt_rc)
		first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
		second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
		third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
		fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
		dist5 = 1
		dist6 = 1
		if(first >= 0 and second >= 0 and third >= 0 and fourth >= 0):
			dist5 = 0
			dist6 = 0
        
        # check square
		(x1, y1) = (150 - sqrt_rc, 50)
		(x2, y2) = (120 - sqrt_rc, 75)
		(x3, y3) = (150, 100 + sqrt_rc)
		(x4, y4) = (185 + sum, 75 + (sum * 0.5148))
		first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
		second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
		third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
		fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
		dist7 = 1
		dist8 = 1
		if(first <= 0 and second <= 0 and third <= 0 and fourth <= 0):
			dist7 = 0
			dist8 = 0
        
        # check rod
		first = ((col - 95) * (8.66 + sqrt_rc)) - ((5 + sqrt_rc) * (row - 30 + sqrt_rc))
		second = ((col - 95) * (37.5 + sqrt_rc)) - ((-64.95 - sqrt_rc) * (row - 30 + sqrt_rc))
		third = ((col - 30.05 + sqrt_rc) * (8.65 + sqrt_rc)) - ((5.45 + sqrt_rc) * (row - 67.5))
		fourth = ((col - 35.5) * (-37.49 - sqrt_rc)) - ((64.5 + sqrt_rc) * (row - 76.15 - sqrt_rc))
		dist9 = 1
		dist10 = 1
		if(first <= 0 and second >= 0 and third >= 0 and fourth >= 0):
			dist9 = 0
			dist10 = 0

		if(dist1 <= 0 or dist2 <= 0 or dist3 == 0 or dist4 == 0 or dist5 == 0 or dist6 == 0 or dist7 == 0 or dist8 == 0 or dist9 == 0 or dist10 == 0):
			return True
		return False

	# action to move up and check the cost between the new node and existing node and update the cost after comparison
	def moveUp(self, currRow, currCol):
		if(self.validClearance(currRow - 1, currCol) and self.obstacle(currRow - 1, currCol) == False):
			self.x = 0
			self.y = 1
			self.cost = 1
			return True
		else:
			return False
	# action to move Down and check the cost between the new node and existing node and update the cost after comparison
	def moveDown(self, currRow, currCol):
		if(self.validClearance(currRow, currCol - 1) and self.obstacle(currRow, currCol - 1) == False):
			self.x = 0 
			self.y = -1
			self.cost = 1
			return True
		else:
			return False

    # action to move right and check the cost between the new node and existing node and update the cost after comparison
	def moveRight(self, currRow, currCol):
		if(self.validClearance(currRow+1, currCol) and self.obstacle(currRow +1, currCol) == False):
			self.x = 1 
			self.y = 0
			self.cost = 1
			return True
		else:
			return False

	# action to move left and check the cost between the new node and existing node and update the cost after comparison
	def moveLeft(self, currRow, currCol):
		if(self.validClearance(currRow - 1, currCol) and self.obstacle(currRow - 1, currCol) == False):
			self.x = -1 
			self.y = 0
			self.cost = 1
			return True
		return False

	# action to move up right  and check the cost between the new node and existing node and update the cost after comparison	
	def moveUpRight(self, currRow, currCol):
		if(self.validClearance(currRow+1, currCol+1) and self.obstacle(currRow +1, currCol + 1) == False):
			self.x = 1
			self.y = 1
			self.cost = np.sqrt(2)
			return True
		else:
			return False
    # action to move down right  and check the cost between the new node and existing node and update the cost after comparison
	def moveDownRight(self, currRow, currCol):
		if(self.validClearance(currRow+1, currCol-1) and self.obstacle(currRow + 1, currCol - 1) == False):
			self.x = 1 
			self.y = -1
			self.cost = np.sqrt(2)
			return True
		else:
			return False
	
	# action to move up left  and check the cost between the new node and existing node and update the cost after comparison
	def moveUpLeft(self, currRow, currCol):
		if(self.validClearance(currRow - 1, currCol+1) and self.obstacle(currRow - 1, currCol + 1) == False):
			self.x = -1 
			self.y = 1
			self.cost = np.sqrt(2)
			return True
		else:
			return False
    # action to move down left  and check the cost between the new node and existing node and update the cost after comparison
	def moveDownLeft(self, currRow, currCol):
		if(self.validClearance(currRow - 1, currCol - 1) and self.obstacle(currRow - 1, currCol - 1) == False):
			self.x = -1 
			self.y = -1
			self.cost = np.sqrt(2)
			return True
		else:
			return False

	

	
    #check if the goal is reached 
	def goalReached(self,currRow, currCol):
		if(currRow== self.goal[0] and currCol == self.goal[1]):
			print("Goal Reached")
			return True
		else:
			return False

	#main dijkastra algorithm
	def Dij(self):

		#define the cost map, visited nodes and path directories
		costMap = {}
		visited_nodes = {}
		path = {}

		#check the costmap , visited nodes and updated it
		for row in np.arange(1, self.numRows + 1, 1):
			for col in np.arange(1, self.numCols + 1, 1): 
				costMap[(row, col)] = float('inf')
				visited_nodes[(row, col)] = False
				path[(row, col)] = -1

		#define the list for explored nodes and queue		
		explored_nodes = []
		queue = []

		# use heappush to push the nodes in the queue
		heappush(queue, (0, self.start))
		costMap[self.start] = 0

		#if queue is greater than zero then check current node find the minimal cost and update the node with minimal cost 
		while(len(queue)) > 0:
			heapify(queue)
			_, currNode = heappop(queue)
			visited_nodes[currNode] = True
			explored_nodes.append(currNode)
			

			if(self.goalReached(currNode[0], currNode[1]) == True):
				break

			if(self.moveUp(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.moveUpRight(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.moveRight(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.moveDownRight(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.moveDown(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.moveDownLeft(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.moveLeft(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.moveUpLeft(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

		#define the check list and goal nodes
		check = []
		goalx = self.goal[0]
		goaly = self.goal[1]

		#check the cost map until the goal node and append it 
		for a in np.arange(goalx - 1, goalx + 1, 1):
			for b in np.arange(goaly - 1, goaly + 1, 1):
				check.append(costMap[a,b])

		# if path exists print path 
		ans = float('inf')
		for c in range(len(check)):
			if(check[c] != ans):
				print("Path Exists")

			NoPath = 1
			break
		if (NoPath == 0):
			print("Path does not exist")
			return (explored_nodes, [], costMap[goalx,goaly])

		print(costMap[goalx, goaly], "answer")
		result = (goalx, goaly)

		#backtrack the explored nodes and show the path
		backtrack = []
		node = result
		while(path[node] != -1):
			backtrack.append(node)
			node = path[node]
		backtrack.append(self.start)
		backtrack = list(reversed(backtrack))

		print(backtrack)
		return (explored_nodes, backtrack, costMap[goalx, goaly])





	# visualization	
	def animation(self, explored_nodes, backtrack, path):
		fourcc = cv2.VideoWriter_fourcc(*'XVID')
		out = cv2.VideoWriter(str(path), fourcc, 20.0, (self.numCols, self.numRows))
		image = np.zeros((self.numRows, self.numCols, 3), dtype=np.uint8)
		count = 0
		for state in explored_nodes:
			image[int(self.numRows - state[0]), int(state[1] - 1)] = (255, 150, 0)
			if(count%75 == 0):
				out.write(image)
			count = count + 1
			cv2.imshow('explored_nodes', image)
		count = 0
		for row in range(1, self.numRows + 1):
			for col in range(1, self.numCols + 1):
				if(image[int(self.numRows - row), int(col - 1), 0] == 0 and image[int(self.numRows - row), int(col - 1), 1] == 0 and image[int(self.numRows - row), int(col - 1), 2] == 0):
					if(self.validClearance(row, col) and self.obstacle(row, col) == False):
						image[int(self.numRows - row), int(col - 1)] = (154, 250, 0)
						if(count%75 == 0):
							out.write(image)
						count = count + 1
            
		if(len(backtrack) > 0):
			for state in backtrack:
				image[int(self.numRows - state[0]), int(state[1] - 1)] = (0, 0, 255)
				out.write(image)
				cv2.imshow('result', image)
				cv2.waitKey(5)
                
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		out.release()



    	
