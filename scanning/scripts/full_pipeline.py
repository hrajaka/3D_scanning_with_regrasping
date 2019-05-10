#!/home/cc/ee106b/sp19/class/ee106b-aai/virtualenvironment/my_new_app/local/bin/python


# import sys
import os












if __name__ == '__main__':

	## Manually run this in some terminal: roslaunch scanning test_ar.launch ##

	## Record a point cloud for the table ##
	raw_input("Press enter to record empty scene (and Ctrl+C when done)")	
	os.system("rosrun scanning record_one_point_cloud input:=/camera/depth/color/points _prefix:=empty")  


	## Record a point cloud for the first view ##
	raw_input("Press enter to record object scene (and Ctrl+C when done)")
	os.system("rosrun scanning record_one_point_cloud input:=/camera/depth/color/points _prefix:=object0")  

	## Clean the scenes and isolate them ##
	#os.system("rosrun scanning processing_pipeline object0.pcd scene.pcd object0_clean.pcd object0_isolated.pcd")  

	## Generate obj file for the object ##
	os.system("rosrun scanning generate_mesh object0_isolated.pcd object0.obj")  

	'''
	## Find the next best view ##
	os.system("rosrun scanning next_best_view.py")  

	## Generate a grasp, the target pose and execute the motion ##
	#TODO 

	#########

	## Record a point cloud for the second view ##
	raw_input("Press enter to record object scene (and Ctrl+C when done)")
	os.system("rosrun scanning record_one_point_cloud input:=/camera/depth/color/points _prefix:=object1")  

	## Clean the scenes and isolate them ##
	os.system("rosrun scanning processing_pipeline object1.pcd scene.pcd object1_clean.pcd object1_isolated.pcd") 

	## Merge the two point clouds ##
	os.system("rosrun scanning template_alignment object1_isolated.pcd object0_isolated.pcd merged0.pcd") 

	## Generate an obj file of the merged point cloud ##
	os.system("rosrun scanning generate_mesh merged0.pcd merged0.obj")  

	## Smooth the obj file with laplacian filter ##
	os.system("rosrun scanning smooth_obj.py merged0.obj")  

	## Generate a pcd file from the smoothed obj file ##
	os.system("rosrun scanning obj2pcd merged0_smoothed.obj merged0_smoothed.pcd")  

	#########

	## Record a point cloud for the third view ##
	raw_input("Press enter to record object scene (and Ctrl+C when done)")
	os.system("rosrun scanning record_one_point_cloud input:=/camera/depth/color/points _prefix:=object2")  

	## Clean the scenes and isolate them ##
	os.system("rosrun scanning processing_pipeline object2.pcd scene.pcd object2_clean.pcd object2_isolated.pcd") 

	## Merge the two point clouds ##
	os.system("rosrun scanning template_alignment object2_isolated.pcd merged0_smoothed.pcd merged1.pcd") 

	## Generate an obj file of the merged point cloud ##
	os.system("rosrun scanning generate_mesh merged1.pcd merged1.obj")  

	## Smooth the obj file with laplacian filter ##
	os.system("rosrun scanning smooth_obj.py merged1.obj")  

	## Generate a pcd file from the smoothed obj file ##
	os.system("rosrun scanning obj2pcd merged1_smoothed.obj merged1_smoothed.pcd")  
'''
	#########

	
