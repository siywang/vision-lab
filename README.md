# Package Name: vehicle_tracker_prediction_skeleton
- bag files are too large to upload, other files are here now.
- I'm not able to see  map in Rviz by using the given launch file. I comment out the map_server line in launch file, if you have same issue you can try the following. 
- Feel free to edit and make changes 
- don't forget to change the address of waypoints file
---

1. Comment out "Run map_server" related lines in bag_future_pose_estimation.launch

2. open your terminal and go to map folder: `rosrun map_server map_server levine.yaml` 

3. then another terminal ` rosrun vehicle_tracker_prediction_skeleton vehicle_tracker_node`

4.  `roslaunch vehicle_tracker_prediction_skeleton bag_future_pose_estimation.launch `

   - In Rviz, add by topic /pose_predictions 
   
5. In another terminal go to your bag folder: `rosbag play matt-driving-two-cars-2.bag  `

     ---

     <s>TODO: Nodelets...</s>
