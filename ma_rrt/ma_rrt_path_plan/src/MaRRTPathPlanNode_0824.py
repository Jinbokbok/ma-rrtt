#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RRT Path Planning with multiple remote goals.

author: Maxim Yastremsky(@MaxMagazin)
based on the work of AtsushiSakai(@Atsushi_twi)

"""
import rospy
import csv

import ma_rrt

from vehicle_msgs.msg import TrackCone, Track, Command, Waypoint, WaypointsArray

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from cam_lidar_calib.msg import sf_Info  ##센서퓨전

from scipy.spatial import Delaunay

# Matrix/Array library
import numpy as np
import time, math

class MaRRTPathPlanNode:
    # All variables, placed here are static

    def __init__(self):
        # Get all parameters from launch file
        self.shouldPublishWaypoints = rospy.get_param('~publishWaypoints', True)
        self.shouldPublishPredefined = rospy.get_param('~publishPredefined', False)

        if rospy.has_param('~path'):
            self.path = rospy.get_param('~path')

        if rospy.has_param('~filename'):
            self.filename = rospy.get_param('~filename')

        if rospy.has_param('~odom_topic'):
            self.odometry_topic = rospy.get_param('~odom_topic')
        else:
            self.odometry_topic = "/odometry"

        if rospy.has_param('~world_frame'):
            self.world_frame = rospy.get_param('~world_frame')
        else:
            self.world_frame = "velodyne"

        waypointsFrequency = rospy.get_param('~desiredWaypointsFrequency', 5)
        self.waypointsPublishInterval = 1.0 / waypointsFrequency
        self.lastPublishWaypointsTime = 0

        # All Subs and pubs
        rospy.Subscriber("/track", Track, self.mapCallback)
        rospy.Subscriber('/cam_lidar_calib/sf_info', sf_Info, self.sfCallback)
        #rospy.Subscriber(self.odometry_topic, Odometry, self.odometryCallback)
        #rospy.subscriber("/imu/data_raw", Imu, self.yawCallback)
        #rospy.Subscriber("/dvsim/cmd", Command, self.carSensorsCallback)

        # Create publishers
        self.waypointsPub = rospy.Publisher("/waypoints", WaypointsArray, queue_size=0)
        self.newwaypointsPub = rospy.Publisher("/newwaypoints", WaypointsArray, queue_size=5)

        # visuals
        self.treeVisualPub = rospy.Publisher("/visual/tree_marker_array", MarkerArray, queue_size=0)
        self.bestBranchVisualPub = rospy.Publisher("/visual/best_tree_branch", Marker, queue_size=1)
        self.filteredBranchVisualPub = rospy.Publisher("/visual/filtered_tree_branch", Marker, queue_size=1)
        self.delaunayLinesVisualPub = rospy.Publisher("/visual/delaunay_lines", Marker, queue_size=1)
        self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)

        self.carPosX = 0.0
        self.carPosY = 0.0
        self.carPosYaw = 0.0
        
        self.fusedDataByClass = {}

        self.map = []
        self.savedWaypoints = []
        self.preliminaryLoopClosure = False
        self.loopClosure = False

        self.rrt = None

        # if self.shouldPublishPredefined:
        #     with open(self.path + self.filename) as csv_file:
        #         csv_reader = csv.reader(csv_file, delimiter=',')

        #         for row in csv_reader:
        #             self.savedWaypoints.append((float(row[0]), float(row[1])))

        #         # print("predefinedWaypoints:", self.savedWaypoints)
        #     self.preliminaryLoopClosure = True
        #     self.loopClosure = True

        self.filteredBestBranch = []
        self.discardAmount = 0

        print("MaRRTPathPlanNode Constructor has been called")

    def __del__(self):
        print('MaRRTPathPlanNode: Destructor called.')

    def odometryCallback(self, odometry):
        # rospy.loginfo("odometryCallback")

        # start = time.time()

        self.carPosX = odometry.pose.pose.position.x
        self.carPosY = odometry.pose.pose.position.y
        #print "Estimated processing odometry callback: {0} ms".format((time.time() - start)*1000)

    def yawCallback(self, yaw):
        self.carPosYaw = yaw.orientation.z

    def carSensorsCallback(self, command):
        # rospy.loginfo("carSensorsCallback")

        # start = time.time()
        # The ackermann angle [rad]
        self.steerAngle = math.degrees((command.theta_l + command.theta_r) / 2.0)

        #print "Estimated processing map callback: {0} ms".format((time.time() - start)*1000);

    def mapCallback(self, track):
        self.map = track.cones

    def sfCallback(self, msg):
        self.sf_pub_id = msg.pub_id
        self.sf_y = msg.sf_y    
        self.sf_x = msg.sf_x
        print(self.sf_x)
        print(self.sf_y)

        ##대경 수정
        if self.sf_pub_id not in self.fusedDataByClass:
            self.fusedDataByClass[self.sf.pub_id] = []

        cone_data = {
            "x": self.sf_x,
            "y": self.sf_y
        }
        
        self.fusedDataByClass[self.sf_pub_id].append(cone_data)

    def sampleTree(self):
        # sampleTreeStartTime = time.time()
        if self.loopClosure and len(self.savedWaypoints) > 0:
            # print("Publish savedWaypoints/predifined waypoints, return")
            self.publishWaypoints()
            return

        # print("------- sampleTree() -------");

        if not self.fusedDataByClass:
            # print "sampleTree(): map is still empty, return"
            return

        # print "map size: {0}".format(len(self.map.cones));

        frontConesDist = 8 # 12
        frontCones1,frontCones2 = self.getFrontConeObstacles(self.fusedDataByClass, frontConesDist)  ##대경 수정
        # frontCones = [] # empty for tests

        # print "frontCones size: {0}".format(len(frontCones));
        # print(frontCones)

        coneObstacleSize = 0.5 #높이 68cm, 밑판 37*37(cm2)
        coneObstacleList = []
        rrtConeTargets = []
        coneTargetsDistRatio = 0.5
        combinedGroup = frontCones1 + frontCones2
        for cone in  combinedGroup :
            coneObstacleList.append((cone["x"], cone["y"], coneObstacleSize))

            coneDist = self.dist(self.carPosX, self.carPosY,cone["x"], cone["y"])

            if coneDist > frontConesDist * coneTargetsDistRatio:
                rrtConeTargets.append((cone["x"], cone["y"], coneObstacleSize))

        # Set Initial parameters
        start = [self.carPosX, self.carPosY, self.carPosYaw]
        iterationNumber = 1000 #알고리즘이 최적 경로를 찾기 위해 트리를 확장하는 최대 반복 횟수
        planDistance = 12 #RRT 알고리즘이 확장하는 단계별 거리
        expandDistance = 0.5 #1.0 #트리 확장 시 한 번에 이동하는 거리
        expandAngle = 20 #트리 확장 시 한 번에 회전하는 각도

        # rrt planning
        # planningStartTime = time.time()
        rrt = ma_rrt.RRT(start, planDistance, obstacleList=coneObstacleList, expandDis=expandDistance, turnAngle=expandAngle, maxIter=iterationNumber, rrtTargets = rrtConeTargets)
        nodeList, leafNodes = rrt.Planning()
        # print "rrt.Planning(): {0} ms".format((time.time() - planningStartTime) * 1000);

        # print "nodeList size: {0}".format(len(nodeList))
        # print "leafNodes size: {0}".format(len(leafNodes))

        self.publishTreeVisual(nodeList, leafNodes)

        frontConesBiggerDist = 6 # 15 #탐색 거리 앞쪽으로 6만큼 안에 있는 장애물을 탐색
        largerGroupFrontCones1, largerGroupFrontCones2 = self.getFrontConeObstacles(self.fusedDataByClass, frontConesBiggerDist) #이때 6보다 먼곳에 있는 장애물들을 저장
        combinedLargerGroup = largerGroupFrontCones1 + largerGroupFrontCones2 

        ########## STEP2,3) Selecting BestBranch and Filtering #########
        # findBesttBranchStartTime = time.time()
        bestBranch = self.findBestBranch(leafNodes, nodeList, largerGroupFrontCones1+largerGroupFrontCones2, coneObstacleSize, expandDistance, planDistance)
        # print "find best branch time: {0} ms".format((time.time() - findBesttBranchStartTime) * 1000);

        # print "best branch", bestBranch

        if bestBranch:
            filteredBestBranch = self.getFilteredBestBranch(bestBranch) #bestBrach에서 일부 노드를 필터링
            # print "filteredBestBranch", filteredBestBranch

            if filteredBestBranch:
                # Delaunay
                # delaunayStartTime = time.time()
                delaunayEdges = self.getDelaunayEdges(frontCones1,frontCones2) #들로네 삼각분할 이용
                # print "delaunay time: {0} ms".format((time.time() - delaunayStartTime) * 1000);

                self.publishDelaunayEdgesVisual(delaunayEdges)

                # findWaypointsStartTime = time.time()

                #새로운 웨이포인트 생성
                newWaypoints = [] 

                if delaunayEdges:
                    # print "len(delaunayEdges):", len(delaunayEdges)
                    # print delaunayEdges

                    newWaypoints = self.getWaypointsFromEdges(filteredBestBranch, delaunayEdges)
                # else:
                #     print "newWaypoints from filteredBestBranch", newWaypoints
                #     newWaypoints = [(node.x, node.y) for node in filteredBestBranch]

                # print "find waypoints time: {0} ms".format((time.time() - findWaypointsStartTime) * 1000)

                if newWaypoints:
                    # print "newWaypoints:", waypoints

                    # mergeWaypointsStartTime = time.time()
                    self.mergeWaypoints(newWaypoints) #웨이포인트 병합 : 중복을 제거해서 경로 단순화
                    # print "merge waypoints time: {0} ms".format((time.time() - mergeWaypointsStartTime) * 1000);

                self.publishWaypoints(newWaypoints)

        # print "whole map callback: {0} ms".format((time.time() - sampleTreeStartTime)*1000);


    ###기존 waypoint랑 newwaypoint병합###
    def mergeWaypoints(self, newWaypoints): 
        # print "mergeWaypoints:", "len(saved):", len(self.savedWaypoints), "len(new):", len(newWaypoints)
        if not newWaypoints:
            return

        maxDistToSaveWaypoints = 2.0 #웨이포인트들간의 최대 거리
        maxWaypointAmountToSave = 2 #저장할 최대 waypoint 개수
        waypointsDistTollerance = 1000 # 웨이포인트들 간의 거리 허용치 / 만약 두 웨이포인트 사이의 거리가 이 값보다 작다면 해당 웨이포인트는 중복으로 간주되어 제거

        # check preliminary loopClosure : 저장된 웨이포인트 중에서 가장 첫 번째 웨이포인트와 새로운 웨이포인트 사이의 거리를 비교하여 일정 거리 이내에 있는지 확인
        if len(self.savedWaypoints) > 15:
            firstSavedWaypoint = self.savedWaypoints[0]

            for waypoint in reversed(newWaypoints):
                distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypoint[0], waypoint[1])
                if distDiff < waypointsDistTollerance:
                    self.preliminaryLoopClosure = False
                    #print ("preliminaryLoopClosure = True")
                    break

        # print "savedWaypoints before:", self.savedWaypoints
        # print "newWaypoints:", newWaypoints

        newSavedPoints = []

        for i in range(len(newWaypoints)):
            waypointCandidate = newWaypoints[i]

            carWaypointDist = self.dist(self.carPosX, self.carPosY, waypointCandidate[0], waypointCandidate[1])
            # print "check candidate:", waypointCandidate, "with dist:", carWaypointDist

            if i >= maxWaypointAmountToSave or carWaypointDist > maxDistToSaveWaypoints:
                # print "condition to break:", i, i >= maxWaypointAmountToSave,  "or", (carWaypointDist > maxDistToSaveWaypoints)
                break
            else:
                for savedWaypoint in reversed(self.savedWaypoints):
                    waypointsDistDiff = self.dist(savedWaypoint[0], savedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                    if waypointsDistDiff < waypointsDistTollerance:
                        self.savedWaypoints.remove(savedWaypoint) #remove similar
                        # print "remove this point:", savedWaypoint, "with diff:", waypointsDistDiff
                        break

                if (self.preliminaryLoopClosure):# Loop closure 체크 : 일정거리 내에 있을 경우 반복문 종료
                    distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                    if distDiff < waypointsDistTollerance:
                        self.loopClosure = False
                        print ("loopClosure = True")
                        break

                # print "add this point:", waypointCandidate
                self.savedWaypoints.append(waypointCandidate)
                newSavedPoints.append(waypointCandidate)

        if newSavedPoints: # make self.savedWaypoints and newWaypoints having no intersection
            for point in newSavedPoints:
                newWaypoints.remove(point)

        # print "savedWaypoints after:", self.savedWaypoints
        # print "newWaypoints after:", newWaypoints

    ### waypoint생성-using branch and Delaunay ###
    def getWaypointsFromEdges(self, filteredBranch, delaunayEdges): #fitered된 노드들이랑 들로네 점들의 교차 확인
        if not delaunayEdges:
            return

        waypoints = [] 
        for i in range (len(filteredBranch) - 1): 
            node1 = filteredBranch[i]
            node2 = filteredBranch[i+1]
            a1 = np.array([node1.x, node1.y])
            a2 = np.array([node2.x, node2.y])

            # print "node1:", node1
            # print "node2:", node2

            maxAcceptedEdgeLength = 7 #경로 생성 시 허용되는 최대 에지(선분) 길이 / 이 값보다 들로네 그래프의 에지가 더 길면 웨이포인트 생성 안됨
            maxEdgePartsRatio = 3 #최대 에지(선분) 길이 비율 / 에지의 비율이 이보다 크면 웨포 생성 안됨

            intersectedEdges = []
            for edge in delaunayEdges:
                # print "edge:", edge

                b1 = np.array([edge.x1, edge.y1])
                b2 = np.array([edge.x2, edge.y2])

                if self.getLineSegmentIntersection(a1, a2, b1, b2): #(node1, node2)에 대해 해당 선분과 delaunayEdges의 에지들을 교차하는지 확인
                    if edge.length() < maxAcceptedEdgeLength: 
                        edge.intersection = self.getLineIntersection(a1, a2, b1, b2)

                        edgePartsRatio = edge.getPartsLengthRatio()
                        # print "edgePartsRatio:", edgePartsRatio

                        if edgePartsRatio < maxEdgePartsRatio:
                            intersectedEdges.append(edge)

            if intersectedEdges:
                # print "len(intersectedEdges):", len(intersectedEdges)
                # print "intersectedEdges:", intersectedEdges

                if len(intersectedEdges) == 1: #교차하는 에지들의 개수를 확인 / 이게 1이면 리스트에 있는 유일한 에지를 가져옴
                    edge = intersectedEdges[0] 

                    # print "edge middle:", edge.getMiddlePoint()
                    waypoints.append(edge.getMiddlePoint()) #중간지점에 웨포 추가
                else:
                    # print "initial:", intersectedEdges  
                    intersectedEdges.sort(key=lambda edge: self.dist(node1.x, node1.y, edge.intersection[0], edge.intersection[1], shouldSqrt = False))
                    # print "sorted:", intersectedEdges   #리스트를 node1과의 거리를 기준으로 정렬

                    for edge in intersectedEdges:
                        waypoints.append(edge.getMiddlePoint())

        return waypoints

    ### 선분 생성 - 들로네 삼각화 ###
    def getDelaunayEdges(self, frontCones1, frontCones2):
        combinedCones =  + frontCones2
        if len(combinedCones) < 4 or frontCones1 < 1 or frontCones2 < 1: # no sense to calculate delaunay /cone의 개수가 4개 미만이면 삼각분할 의미 없어서 수행안함
            return

        conePoints = np.zeros((len(combinedCones), 2)) #배열생성 (행 개수, 열 개수) ->2차원
 
        for i in range(len(combinedCones)):
            cone = combinedCones[i]
            conePoints[i] = ([cone["x"], cone["y"]])

        # print conePoints
        tri = Delaunay(conePoints)
        # print "len(tri.simplices):", len(tri.simplifrontCones1ces)

        delaunayEdges = []
        for simp in tri.simplices:
            # print simp

            for i in range(3):
                j = i + 1
                if j == 3: #j는 i의 다음 정점
                    j = 0 #만약 j가 3이라면, 다음 정점은 삼각형의 첫 번째 정점이 되어야 하므로 j를 0으로 설정
                edge = Edge(conePoints[simp[i]][0], conePoints[simp[i]][1], conePoints[simp[j]][0], conePoints[simp[j]][1])

                if edge not in delaunayEdges: #중복확인 / 중복하지 않는 것만 추가
                    delaunayEdges.append(edge)

        return delaunayEdges

    # 단순 거리 계산
    def dist(self, x1, y1, x2, y2, shouldSqrt = True): #shouldSqrt : 거리를 제곱근으로 변환할지 말지 결정
        distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
        return math.sqrt(distSq) if shouldSqrt else distSq

    ### waypoint 발행 ###
    def publishWaypoints(self, newWaypoints = None):
        if (time.time() - self.lastPublishWaypointsTime) < self.waypointsPublishInterval:
            return
        # self.lastPublishWaypointsTime: 이전 발행 시간으로부터 일정 시간이 경과했는지 확인
        # 일정시간이 경과하지 않았으면 메서드 종료


        # print "publishWaypoints(): start"
        waypointsArray = WaypointsArray()
        newwaypointsArray = WaypointsArray()
        waypointsArray.header.frame_id = self.world_frame
        waypointsArray.header.stamp = rospy.Time.now()
        newwaypointsArray.header.frame_id = self.world_frame
        newwaypointsArray.header.stamp = rospy.Time.now()



        # if not self.savedWaypoints and newWaypoints:
        #     firstWaypoint = newWaypoints[0]
        #
        #     auxWaypointMaxDist = 2
        #
        #     # auxilary waypoint to start
        #     if self.dist(self.carPosX, self.carPosY, firstWaypoint[0], firstWaypoint[1]) > auxWaypointMaxDist:
        #         waypointsArray.waypoints.append(Waypoint(0, self.carPosX, self.carPosY))
        #         print "add aux point with car pos"

        for i in range(len(self.savedWaypoints)):
            waypoint = self.savedWaypoints[i]
            waypointId = len(waypointsArray.waypoints)
            w = Waypoint(waypoint[0], waypoint[1], waypointId)
            waypointsArray.waypoints.append(w)

        if newWaypoints is not None:
            for i in range(len(newWaypoints)):
                waypoint = newWaypoints[i]
                waypointId = len(waypointsArray.waypoints)
                w = Waypoint(waypoint[0], waypoint[1], waypointId)
                waypointsArray.waypoints.append(w)
                newwaypointsArray.waypoints.append(w)
                # print "added from newWaypoints:", waypointId, waypoint[0], waypoint[1]

        if self.shouldPublishWaypoints:
            # print "publish ros waypoints:", waypointsArray.waypoints
            self.waypointsPub.publish(waypointsArray)
            self.newwaypointsPub.publish(newwaypointsArray)
            self.lastPublishWaypointsTime = time.time()

            self.publishWaypointsVisuals(newWaypoints)

            # print "publishWaypoints(): len(waypointsArray.waypoints):", len(waypointsArray.waypoints)
            # print "------"

    ### waypoint 시각화 ###
    def publishWaypointsVisuals(self, newWaypoints = None):

        markerArray = MarkerArray()

        savedWaypointsMarker = Marker()
        savedWaypointsMarker.header.frame_id = self.world_frame
        savedWaypointsMarker.header.stamp = rospy.Time.now()
        savedWaypointsMarker.lifetime = rospy.Duration(1)
        savedWaypointsMarker.ns = "saved-publishWaypointsVisuals"
        savedWaypointsMarker.id = 1

        savedWaypointsMarker.type = savedWaypointsMarker.SPHERE_LIST
        savedWaypointsMarker.action = savedWaypointsMarker.ADD
        savedWaypointsMarker.pose.orientation.w = 1
        savedWaypointsMarker.scale.x = 0.15
        savedWaypointsMarker.scale.y = 0.15
        savedWaypointsMarker.scale.z = 0.15

        savedWaypointsMarker.color.a = 1.0
        savedWaypointsMarker.color.b = 1.0

        for waypoint in self.savedWaypoints:
            p = Point(waypoint[0], waypoint[1], 0.0)
            savedWaypointsMarker.points.append(p)

        markerArray.markers.append(savedWaypointsMarker)

        if newWaypoints is not None:
            newWaypointsMarker = Marker()
            newWaypointsMarker.header.frame_id = self.world_frame
            newWaypointsMarker.header.stamp = rospy.Time.now()
            newWaypointsMarker.lifetime = rospy.Duration(1)
            newWaypointsMarker.ns = "new-publishWaypointsVisuals"
            newWaypointsMarker.id = 2

            newWaypointsMarker.type = newWaypointsMarker.SPHERE_LIST
            newWaypointsMarker.action = newWaypointsMarker.ADD
            newWaypointsMarker.pose.orientation.w = 1
            newWaypointsMarker.scale.x = 0.3
            newWaypointsMarker.scale.y = 0.3
            newWaypointsMarker.scale.z = 0.3

            newWaypointsMarker.color.a = 0.65
            newWaypointsMarker.color.b = 1.0

            for waypoint in newWaypoints:
                p = Point(waypoint[0], waypoint[1], 0.0)
                newWaypointsMarker.points.append(p)

            markerArray.markers.append(newWaypointsMarker)

        self.waypointsVisualPub.publish(markerArray)

    ### 두 개의 선분이 교차하는 지점을 계산 ###
    def getLineIntersection(self, a1, a2, b1, b2): 
        """
        Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
        a1: [x, y] a point on the first line
        a2: [x, y] another point on the first line
        b1: [x, y] a point on the second line
        b2: [x, y] another point on the second line
        https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
        """
        s = np.vstack([a1,a2,b1,b2])        # s for stacked
        h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
        l1 = np.cross(h[0], h[1])           # get first line
        l2 = np.cross(h[2], h[3])           # get second line
        x, y, z = np.cross(l1, l2)          # point of intersection
        if z == 0:                          # lines are parallel
            return (float('inf'), float('inf'))
        return (x/z, y/z)

    ### 두 개의 선분이 교차하는지를 확인 ###
    def getLineSegmentIntersection(self, a1, a2, b1, b2): 
        # https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
        # Return true if line segments a1a2 and b1b2 intersect / 반환하면 True, 하지 않으면 False
        # return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
        return self.ccw(a1,b1,b2) != self.ccw(a2,b1,b2) and self.ccw(a1,a2,b1) != self.ccw(a1,a2,b2)
        # 반시계방향으로 세점이 정렬되어 있는지를 확인하는 것 / 다른 방향으로 정렬되어 있어야 교차함
    
    def ccw(self, A, B, C):
        # if three points are listed in a counterclockwise order.
        # return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    ### filtering된 가지 얻기 ###
    ### bestBranch의 각 노드와 filteredBestBranch의 해당 노드 사이의 거리를 계산하고, ###
    ### 거리의 변화율을 확인하여 필터링을 수행 ###
    def getFilteredBestBranch(self, bestBranch):
        if not bestBranch: # filtered branch 찾기 전에 bestbranch 인지 아닌지 먼저 확인해주는
            return

        everyPointDistChangeLimit = 2.0 # 각 노드의 거리 변화 제한값 / 해당 노드 사이의 값이 이보다 커지면 필터링이 수행되고 해당 가지가 건너뛰어짐
        newPointFilter = 0.2 # 새로운 노드를 filteredBestBranch에 추가할 때 이전 값과 새로운 값 사이의 가중 평균을 계산하는 데 사용되는 필터 가중치 / 0과 1 사이 값 / 1에 가까울수록 영향 커짐
        maxDiscardAmountForReset = 2 # 필터링 과정에서 건너뛴 가지(거리 변화가 큰 가지)의 최대 허용 횟수

        if not self.filteredBestBranch:
            self.filteredBestBranch = list(bestBranch) #초기 상태에서 filteredBestBranch가 아직 설정되지 않았을 때, 최초의 bestBranch 값을 filteredBestBranch로 설정하기 위함
        else:
            changeRate = 0
            shouldDiscard = False #가지를 버릴지 말지 결정하는 변수 / false니까 초기는 안버리는 걸로
            for i in range(len(bestBranch)):
                node = bestBranch[i]
                filteredNode = self.filteredBestBranch[i]

                dist = math.sqrt((node.x - filteredNode.x) ** 2 + (node.y - filteredNode.y) ** 2)
                if dist > everyPointDistChangeLimit: # changed too much, skip this branch
                    shouldDiscard = True #너무 커져버리면 가지 버림(=건너뜀)
                    self.discardAmount += 1 #버린 횟수 +1
                    # print "above DistChangeLimit:, shouldDiscard!,", "discAmount:", self.discardAmount

                    if self.discardAmount >= maxDiscardAmountForReset:
                        self.discardAmount = 0 #안버림
                        self.filteredBestBranch = list(bestBranch)
                        # print "broke maxDiscardAmountForReset:, Reset!"
                    break

                changeRate += (everyPointDistChangeLimit - dist) #변화거리 / 변화율이 작을수록 안정성이 더 커짐
            # print "branch changeRate: {0}".format(changeRate);

            if not shouldDiscard: #버리지 않았을 떄의 가지 업데이트
            #     return
            # else:
                for i in range(len(bestBranch)):
                    self.filteredBestBranch[i].x = self.filteredBestBranch[i].x * (1 - newPointFilter) + newPointFilter * bestBranch[i].x
                    self.filteredBestBranch[i].y = self.filteredBestBranch[i].y * (1 - newPointFilter) + newPointFilter * bestBranch[i].y

                self.discardAmount = 0
                # print "reset discardAmount, ", "discAmount:", self.discardAmount

        self.publishFilteredBranchVisual()
        return list(self.filteredBestBranch) # return copy

    ### 들로네 에지 시각화 ###
    def publishDelaunayEdgesVisual(self, edges):
        if not edges: # 선분 아니면 수행 안함
            return

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(1)
        marker.ns = "publishDelaunayLinesVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05 #선분 굵기

        marker.pose.orientation.w = 1

        marker.color.a = 0.5 #투명도(알파값)
        marker.color.r = 1.0 #빨강(0-1범위 값)
        marker.color.b = 1.0 #파랑

        for edge in edges:
            # print edge

            p1 = Point(edge.x1, edge.y1, 0)
            p2 = Point(edge.x2, edge.y2, 0)

            marker.points.append(p1)
            marker.points.append(p2)

        self.delaunayLinesVisualPub.publish(marker)

    ### bestBranch 찾기 ###
    def findBestBranch(self, leafNodes, nodeList, combinedLargerGroup, coneObstacleSize, expandDistance, planDistance):
        if not leafNodes:
            return

        coneDistLimit = 4.0 #원뿔 밑면 크기 제한 
        coneDistanceLimitSq = coneDistLimit * coneDistLimit;

        bothSidesImproveFactor = 3 #양쪽 방향으로 가지를 확장할 떄의 개선 요소/확장시에는 양쪽 방향으로 동시에 진행됨/이게 클수록 가지가 더 많이 확장됨
        minAcceptableBranchRating = 80 # fits good fsg18 / 가지의 최소 허용 등급 / 값이 클수록 높은 품질

        leafRatings = []
        for leaf in leafNodes: #leaf는 가지 끝 부분을 말함
            branchRating = 0 #가지의 품질을 나타내는 지표
            node = leaf
            # print " ===== calculating leaf node {0} ====== ".format(leaf)
            while node.parent is not None: #노드의 부모가 존재하는 한 계속 실행되는 반복문
                nodeRating = 0
                # print "---- check node {0}".format(node)

                leftCones = []
                rightCones = []

                for cone in combinedLargerGroup:
                    coneDistSq = ((cone["x"] - node.x) ** 2 + (cone["y"] - node.y) ** 2)

                    if coneDistSq < coneDistanceLimitSq:
                        actualDist = math.sqrt(coneDistSq)

                        if actualDist < coneObstacleSize:
                            # node can be really close to a cone, cause we have new cones in this comparison, so skip these ones
                            continue

                        nodeRating += (coneDistLimit - actualDist) #nodeRating에 coneDisLimit-actualDsit를 더함
                        # print "found close cone({1},{2}), rating: {0}".format(nodeRating, cone["x"], cone["y"])

                        if self.isLeftCone == True: #현재 가지와 원뿔이 왼쪽인지 오른쪽인지 판단
                            leftCones.append(cone)
                        elif self.isLeftCone == False:
                            rightCones.append(cone)

                if ((len(leftCones) == 0 and len(rightCones)) > 0 or (len(leftCones) > 0 and len(rightCones) == 0)):
                    # print "cones are only from one side, penalize rating"
                    nodeRating /= bothSidesImproveFactor #원뿔이 한쪽방향으로만 있으니까 나눠줌/가지 확장이 한 방향으로 치우쳐지는 걸 제한함

                if (len(leftCones) > 0 and len(rightCones) > 0):
                    # print "cones are from both sides, improve rating"
                    nodeRating *= bothSidesImproveFactor

                # print "node.cost: {0}, node.rating: {1}".format(node.cost, nodeRating)

                # make conversion: (expandDistance to planDistance) -> (1 to 2)
                nodeFactor = (node.cost - expandDistance)/(planDistance - expandDistance) + 1
                # print "nodeFactor: {0}".format(nodeFactor)

                branchRating += nodeRating * nodeFactor
                # branchRating += nodeRating
                # print "current branch rating: {0}".format(branchRating)
                node = nodeList[node.parent]

            leafRatings.append(branchRating)
            # print "leaf node {0}, rating: {1}".format(leaf, branchRating)

        # print leafRatings
        maxRating = max(leafRatings)
        maxRatingInd = leafRatings.index(maxRating)

        node = leafNodes[maxRatingInd]
        # print "!!maxRating leaf node {0}, rating: {1}".format(node, maxRating)

        if maxRating < minAcceptableBranchRating:
            return

        self.publishBestBranchVisual(nodeList, node)

        reverseBranch = []
        reverseBranch.append(node)
        while node.parent is not None:
            node = nodeList[node.parent]
            reverseBranch.append(node)

        directBranch = []
        for n in reversed(reverseBranch):
            directBranch.append(n)
            # print n

        return directBranch

    def isLeftCone(self, cone_data, frontConeList2):
        if cone_data in frontConeList2:
            return True  # 항상 왼쪽 콘(left cone)으로 판단
        else:
            return False
        
        # //((b.X - a.X)*(cone["y"] - a.Y) - (b.Y - a.Y)*(cone["x"] - a.X)) > 0;
        # frontConeList2.append(cone_data)
        # return ((node.x - parentNode.x) * (cone["y"] - parentNode.y) - (node.y - parentNode.y) * (cone["x"] - parentNode.x)) > 0;

    def publishBestBranchVisual(self, nodeList, leafNode):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(0.2)
        marker.ns = "publishBestBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1

        marker.color.a = 0.7
        marker.color.r = 1.0

        node = leafNode

        parentNodeInd = node.parent
        while parentNodeInd is not None:
            parentNode = nodeList[parentNodeInd]
            p = Point(node.x, node.y, 0)
            marker.points.append(p)

            p = Point(parentNode.x, parentNode.y, 0)
            marker.points.append(p)

            parentNodeInd = node.parent
            node = parentNode

        self.bestBranchVisualPub.publish(marker)

    def publishFilteredBranchVisual(self):

        if not self.filteredBestBranch:
            return

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(0.2)
        marker.ns = "publisshFilteredBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1

        marker.color.a = 0.7
        marker.color.b = 1.0

        for i in range(len(self.filteredBestBranch)):
            node = self.filteredBestBranch[i]
            p = Point(node.x, node.y, 0)
            if i != 0:
                marker.points.append(p)

            if i != len(self.filteredBestBranch) - 1:
                marker.points.append(p)

        self.filteredBranchVisualPub.publish(marker)

    def publishTreeVisual(self, nodeList, leafNodes):

        if not nodeList and not leafNodes:
            return

        markerArray = MarkerArray()

        # tree lines marker
        treeMarker = Marker()
        treeMarker.header.frame_id = self.world_frame
        treeMarker.header.stamp = rospy.Time.now()
        treeMarker.ns = "rrt"

        treeMarker.type = treeMarker.LINE_LIST
        treeMarker.action = treeMarker.ADD
        treeMarker.scale.x = 0.03

        treeMarker.pose.orientation.w = 1

        treeMarker.color.a = 0.7
        treeMarker.color.g = 0.7

        treeMarker.lifetime = rospy.Duration(0.2)

        for node in nodeList:
            if node.parent is not None:
                p = Point(node.x, node.y, 0)
                treeMarker.points.append(p)

                p = Point(nodeList[node.parent].x, nodeList[node.parent].y, 0)
                treeMarker.points.append(p)

        markerArray.markers.append(treeMarker)

        # leaves nodes marker
        leavesMarker = Marker()
        leavesMarker.header.frame_id = self.world_frame
        leavesMarker.header.stamp = rospy.Time.now()
        leavesMarker.lifetime = rospy.Duration(0.2)
        leavesMarker.ns = "rrt-leaves"

        leavesMarker.type = leavesMarker.SPHERE_LIST
        leavesMarker.action = leavesMarker.ADD
        leavesMarker.pose.orientation.w = 1
        leavesMarker.scale.x = 0.15
        leavesMarker.scale.y = 0.15
        leavesMarker.scale.z = 0.15

        leavesMarker.color.a = 0.5
        leavesMarker.color.b = 0.1

        for node in leafNodes:
            p = Point(node.x, node.y, 0)
            leavesMarker.points.append(p)

        markerArray.markers.append(leavesMarker)

        # publis marker array
        self.treeVisualPub.publish(markerArray)

    #앞에 있는 장애물 위치 찾기
    def getFrontConeObstacles(self,fusedDataByClass, frontDist):
        if not fusedDataByClass:
            return []

        headingVector = self.getHeadingVector()
        # print("headingVector:", headingVector)

        headingVectorOrt = [-headingVector[1], headingVector[0]] #heading vector를 시계방향으로 90도 돌린 / 앞쪽의 장애물을 확인하는 데 사용
        # print("headingVectorOrt:", headingVectorOrt)

        behindDist = 1.0 #뒤쪽으로의 거리=얼만큼 왔는지 알수 있는
        carPosBehindPoint = [self.carPosX - behindDist * headingVector[0], self.carPosY - behindDist * headingVector[1]]

        # print "carPos:", [self.carPosX, self.carPosY]
        # print "carPosBehindPoint:", carPosBehindPoint

        frontDistSq = frontDist ** 2

        ##대경수정
        frontConeList1 = []  ##파란색 콘
        frontConeList2 = []  ##노란색 콘

        #파란색 콘(클래스 1) 데이터 처리
        for cone_data in self.fusedDataByClass.get(1, []):
            cone_x = cone_data["x"]
            cone_y = cone_data["y"]

            if (headingVectorOrt[0] * (cone_y - carPosBehindPoint[1]) - headingVectorOrt[1] * (cone_x - carPosBehindPoint[0])) < 0:
                if ((cone_x) ** 2 + (cone_y) ** 2) < frontDistSq:
                    frontConeList1.append(cone_data)


        #노란색 콘(클래스 2) 데이터 처리
        for cone_data in self.fusedDataByClass.get(2, []):
            cone_x = cone_data["x"]
            cone_y = cone_data["y"]
            
            if (headingVectorOrt[0] * (cone_y - carPosBehindPoint[1]) - headingVectorOrt[1] * (cone_x - carPosBehindPoint[0])) < 0:
                if ((cone_x - self.carPosX) ** 2 + (cone_y - self.carPosY) ** 2) < frontDistSq:
                    frontConeList2.append(cone_data)

        return frontConeList1, frontConeList2


    # 진행방향 찾기 : car 회전행렬을 이용해서 진행 방향 벡터 headingVector를 구함
    def getHeadingVector(self): 
        headingVector = [1.0, 0]
        carRotMat = np.array([[math.cos(self.carPosYaw), -math.sin(self.carPosYaw)], [math.sin(self.carPosYaw), math.cos(self.carPosYaw)]])
        #NEU에 맞춰서 수정할 필요 있음!
        headingVector = np.dot(carRotMat, headingVector)
        return headingVector

    # 주어진 반경 내에 있는 장애물(cone)을 찾아 리스트로 반환
    def getConesInRadius(self, fusedDataByClass, x, y, radius):
        coneList = []
        radiusSq = radius * radius
        for cone in fusedDataByClass:
            if ((cone["x"] - x) ** 2 + (cone["y"] - y) ** 2) < radiusSq:
                coneList.append(cone)
        return coneList

class Edge():
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.intersection = None

    def getMiddlePoint(self):
        return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

    def length(self):
        return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

    def getPartsLengthRatio(self):
        import math

        part1Length = math.sqrt((self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
        part2Length = math.sqrt((self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)

    def __eq__(self, other):
        return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
             or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

    def __str__(self):
        return "(" + str(round(self.x1, 2)) + "," + str(round(self.y1,2)) + "),(" + str(round(self.x2, 2)) + "," + str(round(self.y2,2)) + ")"

    def __repr__(self):
        return str(self)

if __name__ == '__main__':

    a1 = np.array([0, 0])
    a2 = np.array([5, 0])
    b1 = np.array([0, 5])
    b2 = np.array([5, 0])

    maNode = MaRRTPathPlanNode()

    if maNode.getLineSegmentIntersection(a1, a2, b1, b2):
        print ("intersected")
    else:
        print ("not intersected")
