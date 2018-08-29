# asarob_demo

launches:
1. unload from box
	´´´ rosservice call /docker_control/undock station_travel_box´´´

2.stop auto brinup 
	´´´ sudo cob-stop´´´
 or 
  stop sceraio
	´´´ rosservice call /behavior/stop_state_machine´´´
	

3. bring drivers run from base pc
	´´´ ssh -X asarob@b1.cob4-7 
	    password: asarob

	roslauch asarob_demo ipa_brinup.launch
	´´´
	than initialize all driver via joysticks


4. Run face camera from head pc
	´´´ ssh -X asarob@h1.cob4-7 
	    password: asarob

	roslaunch asarob_demo cam3d_realsense2_rgbd.launch
	´´´

