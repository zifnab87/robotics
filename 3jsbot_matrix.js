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



var generate_translation_matrix = function(tx,ty,tz){
	var translation = generate_identity(4);
	translation[0][3] += tx;
	translation[1][3] += ty;
	translation[2][3] += tz;
	return translation;
}





var matrix_init = function(y,x){
	var mat = new Array(y);
	for (var j=0; j<mat.length; j++){
		mat[j] = new Array(x);
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

 var a = [[1,2],[3,4]];
 var b = [[1,2,3],[4,5,6]];
 var c = [1,2,3];
 var d = [4,5,6];

console.log(matrix_transpose(a));

//console.log(vector_normalize(c))
//console.log(matrix_multiply(a,b));


console.log(vector_cross(c,d));

console.log(generate_identity(3));