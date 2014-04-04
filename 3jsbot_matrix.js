//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////




/*
CS148: reference code has functions for:
matrix_multiply
matrix_transpose
vector_normalize
vector_cross
generate_identity
generate_translation_matrix
generate_rotation_matrix_X
generate_rotation_matrix_Y
generate_rotation_matrix_Z
*/











var matrix_init = function(y,x){
	var mat = new Array(y);
	for (var j=0; j<mat.length; j++){
		mat[j] = new Array(x);
		for (var i=0; i<x; i++){
			mat[j][i] = 0;
		}
	}
	return mat;
}

var matrix_multiply = function(matA,matB) {
	var mat = matrix_init(matA.length,matB[0].length);


	for (var j=0; j<matA.length; j++){ 
		for (var i=0; i<matB[0].length; i++){
			mat[j][i] = 0;
			for (var k=0; k<matA[0].length; k++){
				mat[j][i] += matA[j][k] * matB[k][i];
			}
		}
	}
	return mat;
}

var matrix_num_multiply = function(num,mat){
	var result = matrix_init(mat.length,mat[0].length);
	for (var j=0; j<mat.length; j++){ 
		for (var i=0; i<mat[0].length; i++){
			result[j][i] =  num*mat[j][i];
		}
	}
	return result;
}

var matrix_transpose = function(mat){
	var matT = matrix_init(mat[0].length,mat.length);

	for (var j=0; j<mat.length; j++){
		for (var i=0; i<mat[0].length; i++){
			matT[i][j] = mat[j][i];
		}
	} 
	return matT;
}

//assume row-vector
var vector_normalize = function(vec){
	var norm = vector_norm(vec);
	for (var i=0; i<vec.length; i++){
		vec[i] = vec[i] / norm;
	}
	return vec;
}

var vector_norm = function(vec){ //assume row-vector
	var sum = 0;
	for (var i=0; i<vec.length; i++){
		sum += vec[i]*vec[i];
	}
	return Math.sqrt(sum);

}

var vector_cross = function(vec1, vec2) { 
	var cross = new Array(3);

	cross[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
	cross[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
	cross[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];

	return cross;
}


var generate_identity = function(n){
	var identity = matrix_init(n,n);
	for (var j=0; j<n; j++){
		for (var i=0; i<n; i++){
			if (i===j){
				identity[j][i] = 1;
			}
			else {
				identity[j][i] = 0;
			}
		}
	}
	return identity;

}

var generate_translation_matrix = function(vec){
	var translation = generate_identity(4);
	translation[0][3] = vec[0];
	translation[1][3] = vec[1];
	translation[2][3] = vec[2];
	return translation;
}

var generate_rotation_matrix_X = function(theta){
	var Rx = generate_identity(4);
	Rx[1][1] = Math.cos(theta);
	Rx[1][2] = Math.sin(theta)*(-1.0);
	Rx[2][1] = Math.sin(theta);
	Rx[2][2] = Math.cos(theta);
	return Rx;
}

var generate_rotation_matrix_Y = function(theta){
	var Ry = generate_identity(4);
	Ry[0][0] = Math.cos(theta);
	Ry[0][2] = Math.sin(theta)
	Ry[2][0] = Math.sin(theta)*(-1.0);
	Ry[2][2] = Math.cos(theta);
	return Ry;
}

var generate_rotation_matrix_Z = function(theta){
	var Rz = generate_identity(4);
	Rz[0][0] = Math.cos(theta);
	Rz[0][1] = Math.sin(theta)*(-1.0);
	Rz[1][0] = Math.sin(theta);
	Rz[1][1] = Math.cos(theta);
	return Rz;
}


var generate_rotation_matrix = function(angles_vec){
	var res = generate_rotation_matrix_X(angles_vec[0]);
	res = matrix_multiply(res, generate_rotation_matrix_Y(angles_vec[1]));
	res = matrix_multiply(res,generate_rotation_matrix_Z(angles_vec[2]));
	return res;
}

var vector_diff = function(v1,v2){
	var res = new Array(v1.length);
	for (var i=0; i<v1.length; i++){
		res[i] = v1[i] - v2[i];
	}
	return res;
}

var vec_to_mat = function(vec){ 
	var mat = matrix_init(vec.length,1);
	for (var i=0; i<vec.length; i++){
		mat[i][0] = vec[i];
	}
	return mat;
}

var mat_to_vec = function(mat){ // mat(N,1)
	var vec = new Array(mat.length);
	for (var i=0; i<vec.length; i++){
		vec[i] = mat[i][0];
	}
	return vec;
}




var a = [[1,2],[3,4]];
var b = [[1,2,3],[4,5,6]];
var c = [1,2,3];
var d = [4,5,6];

var rot_x = generate_rotation_matrix_X(1.57);
var rot_y = generate_rotation_matrix_Y(1.57);
var rot_z = generate_rotation_matrix_Z(1.57);
console.log(rot_x);
console.log(rot_y);
console.log(rot_z);


console.log(generate_translation_matrix([3,4,5]));



console.log(matrix_transpose(a));


//console.log(vector_normalize(c))
//console.log(matrix_multiply(a,b));


console.log(vector_cross(c,d));

console.log(generate_identity(3));