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
	/*var coord = generate_identity(4);
	coord[0][3] = robot.origin.xyz[0];
	coord[1][3] = robot.origin.xyz[1];
	coord[2][3] = robot.origin.xyz[2];


	robot.origin.xform =  coord;//robot.origin.xyz;//generate_identity(4);*/
	var rh = generate_translation_matrix(robot.origin.xyz);
	rh = matrix_multiply(rh,generate_rotation_matrix_X(robot.origin.rpy[0]));
	rh = matrix_multiply(rh,generate_rotation_matrix_Y(robot.origin.rpy[1]));
	rh = matrix_multiply(rh,generate_rotation_matrix_Z(robot.origin.rpy[2]));

	robot.origin.xform = rh;

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


function compute_and_draw_heading(){

	if (typeof heading_geom === 'undefined') {
	    var temp_geom = new THREE.CubeGeometry(0.3, 0.3, 0.3);
	    var temp_material = new THREE.MeshBasicMaterial( {color: 0x00ffff} )
	    heading_geom = new THREE.Mesh(temp_geom, temp_material);
	    scene.add(heading_geom);
	}
	if (typeof lateral_geom === 'undefined') {
	    var temp_geom = new THREE.CubeGeometry(0.3, 0.3, 0.3);
	    var temp_material = new THREE.MeshBasicMaterial( {color: 0x008888} )
	    lateral_geom = new THREE.Mesh(temp_geom, temp_material); 
	    scene.add(lateral_geom);
	}

	var heading_local = matrix_init(4,1);
	heading_local[0][0] = 0;
	heading_local[1][0] = 0;
	heading_local[2][0] = 1;
	heading_local[3][0] = 1;
	
	var lateral_local = matrix_init(4,1);
	lateral_local[0][0] = 1;
	lateral_local[1][0] = 0;
	lateral_local[2][0] = 0;
	lateral_local[3][0] = 1;

	robot_heading = matrix_multiply(robot.origin.xform, heading_local);
	var robot_heading_transf = generate_translation_matrix(
		new Array(robot_heading[0][0],robot_heading[1][0],robot_heading[2][0])
	);
	robot_heading_transf = matrix_multiply(robot_heading_transf,generate_rotation_matrix(robot.origin.rpy));

	robot_lateral = matrix_multiply(robot.origin.xform, lateral_local);
	var robot_lateral_transf = generate_translation_matrix(
		new Array(robot_lateral[0][0],robot_lateral[1][0],robot_lateral[2][0])
	);
	robot_lateral_transf = matrix_multiply(robot_lateral_transf,generate_rotation_matrix(robot.origin.rpy));


	var heading_mat = matrix_2Darray_to_threejs(robot_heading_transf);
	var lateral_mat = matrix_2Darray_to_threejs(robot_lateral_transf);

	simpleApplyMatrix(heading_geom,heading_mat);
	simpleApplyMatrix(lateral_geom,lateral_mat);
}