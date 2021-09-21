import rospy
from gazebo_ros import gazebo_interface
from geometry_msgs.msg import Pose, Quaternion
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
import tf.transformations as tft

_path_xml = "/home/roboworks/gazebo_models/markers/MODEL_NAME/model-1_4.sdf"

initial_pose = Pose()

delta_x = 0.0  #-5.0
delta_y = 0.0 #-7.0

markers = {
    "name": [
		"mark0001",
		"mark0008",
		"mark0014",
		"mark0017",
		"mark0018",
		"mark0019",
		"mark0020",
		"mark0701",
		"mark0702",
		"mark0703",
		"mark0704",
		"mark0705",
		"mark0706",
		"mark0717",
		"mark0718",
		"mark0719",
		"mark0720"
    ],

    "pose": [ 
		[7.650161266326904, 9.201255798339844, 0.5889569282531738, 0.0, 0.0, -0.7068251967430115, 0.7073882818222046],
		[4.111289978027344, 6.041636943817139, 0.6627340078353882, 0.0, 0.0, 0.0, 1.0],
		[4.348664283752441, 8.202133178710938, 0.5325400590896606, 0.0, 0.0, -0.7068251967430115, 0.7073882818222046],
		[9.59250259399414, 9.203836441040039, 0.5492967128753662, 0.0, 0.0, -0.7068251967430115, 0.7073882818222046],
		[10.071708679199219, 7.17535924911499, 0.3255733609199524, 0.0, 0.0, 0.9999997019767761, 0.0007963267271406949],
		[10.074090957641602, 4.463552951812744, 0.308580732345581, 0.0, 0.0, 0.9999997019767761, 0.0007963267271406949],
		[10.073058128356934, 1.7082232236862183, 0.3073870301246643, 0.0, 0.0, 0.9999997019767761, 0.0007963267271406949],
		[10.075018882751465, 0.7320303320884705, 0.6074292421340942, 0.0, 0.0, 0.9999997019767761, 0.0007963267271406949],
		[4.105309963226318, 7.85575008392334, 0.5343385100364685, 0.0, 0.0, 0.0, 1.0],
		[4.103829860687256, 4.704699516296387, 0.6170341730117798, 0.0, 0.0, 0.0, 1.0],
		[9.230583190917969, 0.009543705731630325, 0.6296695232391357, 0.0, 0.0, 0.7068251967430115, 0.7073882818222046],
		[6.500030040740967, 8.793408393859863, 0.5220313429832458, 0.0, 0.0, 0.0, 1.0],
		[6.98948335647583, 9.203631401062012, 0.6033094882965088, 0.0, 0.0, -0.7068251967430115, 0.7073882818222046],
		[4.103429794311523, 1.852953314781189, 0.5134173393249512, 0.0, 0.0, 0.0, 1.0],
		[4.8319878578186035, -5.144747774465941e-05, 0.3233571410179138, 0.0, 0.0, 0.7068251967430115, 0.7073882818222046],
		[5.7395243644714355, 0.00031555796158500016, 0.3246859669685364, 0.0, 0.0, 0.7068251967430115, 0.7073882818222046],
		[8.114075660705566, 0.0016615003114566207, 0.3197292447090149, 0.0, 0.0, 0.7068251967430115, 0.7073882818222046]
    ]
}


if __name__ == '__main__':

	rospy.init_node('spawn_objects')

	#Delete marks
	get_model = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
	delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

	model_info = get_model()
	for model_name in model_info.model_names:
		if model_name.startswith('mark0'):
			#Delete object
			try:
				delete_model(model_name)
				print "Model %s removed successfuly"%model_name

			except rospy.ServiceException, e:
				print "Service call failed: %s"%e

	#Add marks
	for i in range(len(markers["name"])):
		initial_pose.position.x = markers["pose"][i][0]+delta_x
		initial_pose.position.y = markers["pose"][i][1]+delta_y
		initial_pose.position.z = markers["pose"][i][2]

		q = Quaternion(markers["pose"][i][3], markers["pose"][i][4], markers["pose"][i][5], markers["pose"][i][6])
		initial_pose.orientation = q

		name = markers["name"][i]
		path_xml = _path_xml.replace('MODEL_NAME', name)

		with open(path_xml, "r") as f:
		    model_xml = f.read()

		gazebo_name = name
		gazebo_interface.spawn_sdf_model_client(gazebo_name, model_xml, rospy.get_namespace(),initial_pose, "", "/gazebo")
