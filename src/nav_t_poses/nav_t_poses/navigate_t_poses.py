#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from rclpy.time import Time
import csv
from copy import deepcopy
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
import rclpy
import time
from time import sleep




def main():
    rclpy.init()
    print("entro nel main")
    

    positions = []  # Vector to store Odometry messages
    with open('/home/marco/4d_thesis/src/saving_pose/poses_data.csv', 'r') as file:
        csv_reader = csv.reader(file)
    
        for row in csv_reader:
            try:
                x = float(row[0])
                y = float(row[1])
                qx = float(row[2])
                qy = float(row[3])
                qz = float(row[4])
                qw = float(row[5])
                positions.append((x, y, qx, qy, qz, qw))  # Store coordinates 
            except (IndexError, ValueError):
                print("Skipping invalid row: {row}")
    
    print("finita trasformazione vettore")
    

    navigator = BasicNavigator()
    

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()
    

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')
    
    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    #global_costmap = navigator.getGlobalCostmap()
    #local_costmap = navigator.getLocalCostmap()
    
    print("incomincio route")
  
    goal_poses= []
    
    
    for pt in positions:
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        inspection_pose.pose.orientation.z = pt[2]
        inspection_pose.pose.orientation.w = pt[3]
        inspection_pose.pose.orientation.z = pt[4]
        inspection_pose.pose.orientation.w = pt[5]
        goal_poses.append(inspection_pose)

    initial_pose=goal_poses[0]
    check_poses=[]

    for pt in positions:
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        inspection_pose.pose.orientation.z = pt[2]
        inspection_pose.pose.orientation.w = pt[3]
        inspection_pose.pose.orientation.z = pt[4]
        inspection_pose.pose.orientation.w = pt[5]
        check_poses.append(inspection_pose)

    #go through poses
    navigator.goThroughPoses(goal_poses)
    #path = navigator.getPathThroughPoses(goal_poses[0], goal_poses, planner_id='GridBased', use_start=False)
    #navigator.followPath(path)

    i = 0
    j = 0
    count=0
    last_poses=0
    home=0

    done = 0
    last_j_changed=time.time()
    #print(last_j_changed)
    while ((not navigator.isTaskComplete())) :
        if done==0:
            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if (feedback and (i % 5 == 0)):
                print('Estimated distance remaining : ' + '{0:.3f}'.format(feedback.distance_remaining) )
            
            print(j)
            print(len(check_poses))
            
        
            #dx = check_poses[0].pose.position.x  - navigator.getFeedback().current_pose.pose.position.x
            #dy = check_poses[0].pose.position.y - navigator.getFeedback().current_pose.pose.position.y 
            #theta = math.atan2(2*(navigator.getFeedback().current_pose.pose.orientation._x*navigator.getFeedback().current_pose.pose.orientation._y+navigator.getFeedback().current_pose.pose.orientation._w*navigator.getFeedback().current_pose.pose.orientation._z),1-2*(navigator.getFeedback().current_pose.pose.orientation._y*navigator.getFeedback().current_pose.pose.orientation._y+navigator.getFeedback().current_pose.pose.orientation._z*navigator.getFeedback().current_pose.pose.orientation._z)) * 180/math.pi
            #thetaj = math.atan2(2*(check_poses[0].pose.orientation._x*check_poses[0].pose.orientation._y+check_poses[0].pose.orientation._w*check_poses[0].pose.orientation._z),1-2*(check_poses[0].pose.orientation._y*check_poses[0].pose.orientation._y+check_poses[0].pose.orientation._z*check_poses[0].pose.orientation._z)) * 180/math.pi
            #distance = math.sqrt(dx * dx + dy * dy)
            #distance_yaw= thetaj - theta
            
            if ((abs(check_poses[0].pose.position.x  - navigator.getFeedback().current_pose.pose.position.x) < 0.55) and (abs(check_poses[0].pose.position.y - navigator.getFeedback().current_pose.pose.position.y) < 0.55)) :
                #if(distance<0.25):
                #del check_poses[j]
                check_poses.pop(0)
                print('removed number: ',j)
                j =j + 1
                #last_j_changed=navigator.get_clock().now().nanoseconds/1e9
                last_j_changed=time.time()

            if (time.time() - last_j_changed )>= 7:
                print("Robot has been stuck for 7 seconds. Cancelling task.")
                navigator.cancelTask()
                sleep(3.0)
                print('recupero pose e evito ostacolo')
                recupero=1
                wait=0
                while recupero == 1:
                    feedback=navigator.getFeedback()
                    pose_tmp=feedback.current_pose
                    while (abs(check_poses[0].pose.position.x  - pose_tmp.pose.position.x) < 4.5) and (abs(check_poses[0].pose.position.y - pose_tmp.pose.position.y) < 4.5 ) and wait==0 and len(check_poses)>10: 
                        #del check_poses[j]
                        if len(check_poses)>1:
                            check_poses.pop(0)
                            j=j+1
                            last_j_changed=time.time()
                        else : 
                            navigator.cancelTask()
                            done=1
                        #last_j_changed=navigator.get_clock().now().nanoseconds/1e9
                        #last_j_changed=time.time()
                    #navigator.cancelTask()
                    #sleep(1.0)
                    if len(check_poses)<11:
                        navigator.goToPose(check_poses[len(check_poses)-1],'/home/marco/4d_thesis/src/scout_2/config/navigate_to_pose_w_replanning_and_recovery.xml')
                        sleep(3.0)
                        while((not navigator.isTaskComplete())):
                            i=i+1
                            base_feedback = navigator.getFeedback()
                            if base_feedback and i%5==0:
                                print('Estimated distance remaining : ' + '{0:.3f}'.format(base_feedback.distance_remaining))

                            # if Duration.from_msg(base_feedback.navigation_time) > Duration(seconds=40.0):
                            #     navigator.cancelTask()

                        print('risultati ultime pose')
                        result_last= navigator.getResult()
                        if result_last == TaskResult.SUCCEEDED:
                            print('last goal succeded')
                            recupero=0
                            done=1
                        else:
                            navigator.cancelTask()
                            print('ritorno alla base 2')
                            navigator.goToPose(initial_pose,'/home/marco/4d_thesis/src/scout_2/config/navigate_to_pose_w_replanning_and_recovery.xml')
                            sleep(3.0)
                            while ((not navigator.isTaskComplete())):
                                i = i + 1
                                feedback = navigator.getFeedback()
                                if (feedback and (i % 5 == 0)):
                                    print('Estimated distance remaining : ' + '{0:.3f}'.format(feedback.distance_remaining) )
                            
                            result_base= navigator.getResult()
                            if result_base == TaskResult.SUCCEEDED:
                                print('return to base 2 succeded')
                                recupero=0
                                done=1
                                last_poses=1
                            elif result_base == TaskResult.CANCELED:
                                print('return to base 2 canceled')
                                recupero=0
                                done=1
                                last_poses=1
                            elif result_base == TaskResult.FAILED:
                                print('return to base 2 failed')
                                recupero=0
                                done=1
                                last_poses=1

                    else:    
                        print('valore :',j)
                        navigator.goToPose(check_poses[0],'/home/marco/4d_thesis/src/scout_2/config/navigate_to_pose_w_replanning_and_recovery.xml')
                        wait=1
                        while ((not navigator.isTaskComplete())) :
                            i = i + 1
                            new_feedback = navigator.getFeedback()
                            if new_feedback and i % 5 == 0:
                                print('Estimated distance remaining : ' + '{0:.3f}'.format(new_feedback.distance_remaining) )
                            
                            if new_feedback.distance_remaining > 14.0 :  #parameters to be tuned based on the map and the environment
                                home=1
                                navigator.cancelTask()
                            
                        if home == 1:
                            navigator.goToPose(initial_pose)
                            sleep(2.0)
                            while ((not navigator.isTaskComplete())):
                                i = i + 1
                                base2_feedback = navigator.getFeedback()
                                if (base2_feedback and (i % 5 == 0)):
                                    print('Estimated distance remaining : ' + '{0:.3f}'.format(base2_feedback.distance_remaining) )
                            result_base2 = navigator.getResult()
                            if result_base2 == TaskResult.SUCCEEDED:
                                print('return to base 3 succeded')
                                recupero=0
                                done=1
                                last_poses=1
                            elif result_base2 == TaskResult.CANCELED:
                                print('return to base 3 canceled')
                                recupero=0
                                done=1
                                last_poses=1
                            elif result_base2 == TaskResult.FAILED:
                                print('return to base 3 failed')
                                recupero=0
                                done=1
                                last_poses=1
                        if home==0:
                            print('risultati recupero')
                            result_tmp= navigator.getResult()
                            if result_tmp == TaskResult.SUCCEEDED:
                                print('intermediate goal succeded')
                                if len(check_poses)>0:
                                    check_poses.pop(0)
                                    j=j+1
                                    last_j_changed=time.time()
                                    navigator.goThroughPoses(check_poses)
                                    count=0
                                    recupero=0
                                else:
                                    navigator.cancelTask
                                    done=1
                            elif result_tmp == TaskResult.FAILED:
                                print('Goal failed!')
                                if count==0:
                                    recupero=1
                                    wait=0
                                    count=count+1
                                else:
                                    recupero=0
                                    done=1
                            else: #result_tmp == TaskResult.CANCELED:
                                print('goal unreachable')
                                recupero=1
                                wait=0
            if len(check_poses)==0:
                done=1
    if last_poses==0:
        print('pronto a ottenere risultati')
        # Do something depending on the return code
        
        #sleep(3.0)
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            print('ritorno alla base')
            navigator.goToPose(initial_pose,'/home/marco/4d_thesis/src/scout_2/config/navigate_to_pose_w_replanning_and_recovery.xml')
            while (not navigator.isTaskComplete):
                i = i + 1
                feedback = navigator.getFeedback()
                if (feedback and (i % 5 == 0)):
                    print('Estimated distance remaining : ' + '{0:.3f}'.format(feedback.distance_remaining) )
            
            result_base= navigator.getResult()
            if result_base == TaskResult.SUCCEEDED:
                print('return to base succeded')
            elif result_base == TaskResult.CANCELED:
                print('return to base canceled')
                exit(1)
            elif result_base == TaskResult.FAILED:
                print('return to base failed')
        else:
            print('Goal has an invalid return status!')

    #close nav2 Stack
    #navigator.lifecycleShutdown()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
