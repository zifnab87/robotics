//////////////////////////////////////////////////
/////     FORWARD KINEMATICS
//////////////////////////////////////////////////

// CS148: compute and draw robot kinematics (.xform matrix for each link)
// CS148: compute and draw robot heading and lateral vectors for base movement in plane
// matrix_2Darray_to_threejs converts a 2D JavaScript array to a threejs matrix
//   for example: var tempmat = matrix_2Darray_to_threejs(link.xform);
// simpleApplyMatrix transforms a threejs object by a matrix
//   for example: simpleApplyMatrix(link.geom,tempmat);

/*
CS148: reference code has functions for:
robot_forward_kinematics 
traverse_forward_kinematics_link
traverse_forward_kinematics_joint
compute_and_draw_heading
*/


var robot_forward_kinematics = function(){
	robot.origin.xform = generate_identity(4);
	robot.links[robot.base].xform = robot.origin.xform;
	traverse_forward_kinematics_joint(robot.links[robot.base]);
}



var traverse_forward_kinematics_link = function(parent_joint){
	
	var child_link = robot.links[parent_joint.child];
	child_link.xform = parent_joint.xform;
	traverse_forward_kinematics_joint(child_link);
}




var traverse_forward_kinematics_joint = function(parent_link){
	for (var i=0; i<parent_link.children.length; i++){
		var child_joint = robot.joints[parent_link.children[i]];
		
		var mat_trans = matrix_multiply(parent_link.xform,generate_translation_matrix(child_joint.origin.xyz));

		var rot_x = child_joint.origin.rpy[0];
		var rot_y = child_joint.origin.rpy[1];
		var rot_z = child_joint.origin.rpy[2];
		mat_trans = matrix_multiply(mat_trans,generate_rotation_matrix_X(rot_x));
		mat_trans = matrix_multiply(mat_trans,generate_rotation_matrix_Y(rot_y));
		mat_trans = matrix_multiply(mat_trans,generate_rotation_matrix_Z(rot_z));
		

		child_joint.origin.xform = mat_trans;
		child_joint.xform = matrix_multiply(mat_trans,generate_identity(4));
		traverse_forward_kinematics_link(child_joint);

	}
	

}
