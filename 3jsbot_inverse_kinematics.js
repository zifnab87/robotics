//////////////////////////////////////////////////
/////     INVERSE KINEMATICS 
/////     Resolved-rate IK with geometric jacobian
//////////////////////////////////////////////////

// CS148: generate joint controls to move robot to move robot endeffector to target location

/*
CS148: reference code has functions for:

robot_inverse_kinematics
iterate_inverse_kinematics
*/

function robot_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
    // compute joint angle controls to move location on specified link to Cartesian location

    if (update_ik) {
        iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos);
        endeffector_geom.visible = true;
        target_geom.visible = true;
    }
    else {
        endeffector_geom.visible = false;
        target_geom.visible = false;
    }
    update_ik = false;

}

function iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos){
//endeffector transformation 

	var link_xform = robot.links[robot.joints[endeffector_joint].child].xform;
	var translation = generate_identity(4);
    translation[0][3] = endeffector_local_pos[0][0];
    translation[1][3] = endeffector_local_pos[1][0];
    translation[2][3] = endeffector_local_pos[2][0];

    var transformation = matrix_multiply(link_xform,translation);
    var tempmat = matrix_2Darray_to_threejs(transformation);
    simpleApplyMatrix(endeffector_geom,tempmat);
    //create_jacobian(target_pos, endeffector_joint, endeffector_local_pos);





    //Jacobian calculation


    //endeffetor location
    var o_n = [transformation[0][3], transformation[1][3],transformation[2][3]];

    var joints_stack = new Array();
    var joints_stack2 = new Array();
	var current_joint = robot.joints[endeffector_joint];
	while(current_joint.parent !== "base"){
		joints_stack.push(current_joint);
		joints_stack2.push(current_joint);
		current_joint = robot.joints[robot.links[current_joint.parent].parent]
	}
	joints_stack.push(current_joint);
	joints_stack2.push(current_joint);
	var jacobian = matrix_init(6,joints_stack.length);


	//rotation matrix is not the identity because we could have rotated the robot itself
	var rotation_matrix = generate_identity(4);
	rotation_matrix = matrix_multiply(rotation_matrix,generate_rotation_matrix_X(robot.origin.rpy[0]));
	rotation_matrix = matrix_multiply(rotation_matrix,generate_rotation_matrix_Y(robot.origin.rpy[1]));
	rotation_matrix = matrix_multiply(rotation_matrix,generate_rotation_matrix_Z(robot.origin.rpy[2]));

	var col = 0;

	while(joints_stack.length !== 0){
		current_joint = joints_stack.pop();

		var o_i = [current_joint.xform[0][3], current_joint.xform[1][3], current_joint.xform[2][3]];

		var o_diff_vec = vector_diff(o_n,o_i);


		rotation_matrix = matrix_multiply(rotation_matrix,generate_rotation_matrix_X(current_joint.origin.rpy[0]));
		rotation_matrix = matrix_multiply(rotation_matrix,generate_rotation_matrix_Y(current_joint.origin.rpy[1]));
		rotation_matrix = matrix_multiply(rotation_matrix,generate_rotation_matrix_Z(current_joint.origin.rpy[2]));

		var q = quaternion_from_axisangle(current_joint.axis,current_joint.angle);
		var q_matrix = matrix_transpose(quaternion_to_rotation_matrix(q));

		rotation_matrix = matrix_multiply(rotation_matrix,q_matrix);

		var axis_mat = vec_to_mat([current_joint.axis[0],current_joint.axis[1],current_joint.axis[2],1]);
		var z = matrix_multiply(rotation_matrix,axis_mat);


		var cross_prod = vector_cross(z, o_diff_vec);

		jacobian[0][col] = cross_prod[0];
		jacobian[1][col] = cross_prod[1];
		jacobian[2][col] = cross_prod[2];
		jacobian[3][col] = z[0];
		jacobian[4][col] = z[1];
		jacobian[5][col] = z[2];

		col++;
		//console.log(z);
		


	}



	var diff_x = vector_diff(o_n,ik_target);
	var d_x = [diff_x[0], diff_x[1], diff_x[2], 0, 0, 0]; // zeros for the angular velocities errors 

	var alpha = 0.03;
	var d_theta =  matrix_num_multiply(alpha,matrix_multiply(matrix_transpose(jacobian),vec_to_mat(d_x)));
	var d_theta_vec = mat_to_vec(d_theta);

	var col = 0;
	while(joints_stack2.length !== 0){
		current_joint = joints_stack2.pop();
		current_joint.control += d_theta_vec[col]
		col++;
	}

}