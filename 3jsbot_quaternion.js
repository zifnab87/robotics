//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

quaternion_from_axisangle
quaternion_normalize
quaternion_multiply
quaternion_to_rotation_matrix
*/

var quaternion_multiply = function(q,r){
	var result = new Array(4);
	result[0] = (r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3]);
	result[1] = (r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2]);
	result[2] = (r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1]);
	result[3] = (r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]);
	return result;
}

var quaternion_normalize = function(q){
	var norm = Math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*[2]+q[3]*q[3]);
	//console.log(norm);
	q[0] /= norm;
	q[1] /= norm;
	q[2] /= norm;
	q[3] /= norm;
	return q;
}

var quaternion_from_axisangle = function (axis_vec,angle){
	var q = new Array(4);
	q[0] = Math.cos(angle/2.0);
	q[1] = Math.sin(angle/2.0)*axis_vec[0];
	q[2] = Math.sin(angle/2.0)*axis_vec[1];
	q[3] = Math.sin(angle/2.0)*axis_vec[2];
	return q;
}

var quaternion_to_rotation_matrix = function(q){
	var result = matrix_init(4,4);
	result[0][0] = q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
	result[0][1] = 2*(q[1]*q[2]-q[0]*q[3]);
	result[0][2] = 2*(q[0]*q[2]+q[1]*q[3]);

	result[1][0] = 2*(q[1]*q[2]+q[0]*q[3]);
	result[1][1] = q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];
	result[1][2] = 2*(q[2]*q[3]-q[0]*q[1]);

	result[2][0] = 2*(q[1]*q[3]-q[0]*q[2]);
	result[2][1] = 2*(q[0]*q[1]+q[2]*q[3]);
	result[2][2] = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];

	result[3][3] = 1;

	return result;
}


