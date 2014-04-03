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


}

