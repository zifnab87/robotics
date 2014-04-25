//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// CS148: 
// implement RRT-Connect by Kuffner and LaValle (2000)
//    paper link: http://msl.cs.uiuc.edu/~lavalle/papers/KufLav00.pdf

// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by robot_collision_test()

/*
CS148: reference code has functions for:

    tree_add_vertex
    tree_add_edge
    random_config
    new_config
    nearest_neighbor
    rrt_extend
    rrt_connect
    find_path
    path_dfs
*/


function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // CS148: add necessary RRT initialization here

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();


    epsilon = 1;
    robot_path_traverse_idx = 0;
    rrt_iterate = true;
    tree1 = tree_init(vertex_init(q_start_config));
    tree2 = tree_init(vertex_init(q_goal_config));
    x_min = robot_boundary[0][0];
    x_max = robot_boundary[1][0];
    y_min = robot_boundary[0][2];
    y_max = robot_boundary[1][2];

    rrt_iter_count = 0;

    console.log("planner initialized");
}


function robot_rrt_planner_iterate() {

    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();


        rrt_planning_iteration();
        if (rrt_iter_count>= 100000){
            rrt_iterate = false;
        }


        // CS148: implement RRT iteration here

    }

    // return path not currently found
    return false;
}


function rrt_connect(q, qnew, tree){

    var state = rrt_extend(q,qnew,tree);
    while (state === "advanced"){
        qnew = vertex_init([0,0,0,0,0,0,0,0,0,0,0]);
        state = rrt_extend(q, qnew,tree);
    }
    return state;
}

function new_config(q, qnear, qnew) {

    if (vertex_distance(q,qnear) <= epsilon) {
        vertex_copy(q,qnew);

    }
    else {
        //console.log(q);
        var diff_vec = vector_diff(q,qnear);
        var normalized_vec = vector_normalize(diff_vec);
        var scaled_vec = vector_mult_scalar(normalized_vec,epsilon);
        var result = vector_add(qnear,scaled_vec);
        qnew.angle = rand_range(0,2*Math.PI);
        for(var i = 0; i<qnew.jointangles.length; i++){
            qnew.jointangles[i] = rand_range(0,2*Math.PI);
        }
        qnew[0] = result[0];
        qnew[1] = result[1];

    }
    console.log("q"+q+"qnear "+qnear+" qnew "+qnew);
    return !robot_collision_test(vertex_converter(qnew).vertex);
}




function add_config_origin_indicator_geom(vertex,q) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);

    vertex.geom = temp_mesh;
    q.geom = temp_mesh;
}


function random_config(){
    var rand_point = [0,0,0,0,0,0,0,0,0,0,0];
    rand_point[4] = rand_range(0,2*Math.PI); //angle;
    rand_point[0] = rand_range(x_min,x_max);
    rand_point[2] = rand_range(y_min,y_max);

    for (var i = 6; i<rand_point.length; i++){
        rand_point[i] = rand_range(0,2*Math.PI);
    }


    return vertex_init(rand_point);
}

function vertex_equality(vertex1, vertex2){
    return vertex1[0] === vertex2[0] && vertex1[1] === vertex2[1];
}

function vertex_distance(vertex1, vertex2){
    return Math.sqrt(Math.pow((vertex1[0]-vertex2[0]),2)+Math.pow((vertex1[1]-vertex2[1]),2));
}

function nearest_neighbor(vertex, tree){
    var min_distance = 1000000;
    var nearest = tree.root;
    for (var i=0; i<tree.vertices.length; i++){
        var tree_vertex = tree.vertices[i];
        var dist = vertex_distance(tree_vertex, vertex);
        if (dist < min_distance){
            min_distance = dist;
            nearest = tree_vertex;
        }
    }
    return nearest;
}

function find_path(tree1,tree2) {
    var start = tree1.root;
    start.visited = true;
    var target = tree2.root;
    var path = new Array();
    path.push(vertex_converter(start));
    var path_found = path_dfs(start,target,path);
    

    robot_path = path;
    for (var i=0; i<path.length; i++){
        robot_path[i].geom.material.color = {r:1,g:0,b:0};
    }


    return path_found;

}

function vertex_init(point){
    var vertex = new Array(2);
    vertex[0] = point[0];
    vertex[1] = point[2];
    vertex.edges = new Array();
    vertex.neighbors = new Array();
    vertex.jointangles = new Array();
    vertex.geom = "";
    vertex.angle = point[4];
    vertex.visited = false;
    for (var i=6; i<point.length; i++){
        vertex.jointangles.push(point[i]);
    }
    return vertex;
}



function tree_init(q){
    var tree = new Object();
    tree.vertices = new Array();
    tree.edges = new Array();
    tree.root = q;
    tree_add_vertex(q, tree);
    //add_config_origin_indicator_geom(vertex_converter(q),q);
    return tree;
}


function tree_add_edge(vertex1, vertex2, tree){
    var edge = new Object();
    edge.vertex1 = vertex1;
    edge.vertex2 = vertex2;
    vertex1.edges.push(edge);
    vertex2.edges.push(edge);
    vertex1.neighbors.push(vertex2);
    vertex2.neighbors.push(vertex1);
    tree.edges.push(edge);
}

function tree_add_vertex(vertex, tree){

    tree.vertices.push(vertex);
    add_config_origin_indicator_geom(vertex_converter(vertex),vertex);
}

function vertex_copy(oldvert,newvert){
    newvert[0] = oldvert[0];
    newvert[1] = oldvert[1];
    newvert.angle = oldvert.angle;
    newvert.edges = oldvert.edges;
    newvert.neighbors = oldvert.neighbors;
    newvert.visited = oldvert.visited;
    newvert.geom = oldvert.geom;
    newvert.jointangles = oldvert.jointangles;

}

function path_dfs(start, end, path) {
    for (var i=0; i<start.neighbors.length; i++){
        next = start.neighbors[i];
        if (next.visited){
            continue;
        }
        next.visited = true;
        path.push(vertex_converter(next));
        if (vertex_equality(next,end)){
            return true;
        }

        if (path_dfs(start.neighbors[i], end, path)){
            return true;
        }
        else {
            path.pop();
        }
    }
    return false;
}





function vertex_converter(oldvert){
    var v = new Object();
    var vertex = [0,0,0,0,0,0];
    vertex[4] = oldvert.angle;
    vertex[0] = oldvert[0];
    vertex[2] = oldvert[1];

    for (var i = 0; i<oldvert.jointangles.length; i++){
        vertex.push(oldvert.jointangles[i]);
    }

    v.vertex = vertex;
    v.geom = oldvert.geom;

    return v;
}


function rrt_planning_iteration(path){
    var qrand = random_config();
    var qnew = vertex_init([0,0,0,0,0,0,0,0,0,0,0]);
    if (rrt_extend(qrand,qnew,tree1) !== "trapped"){
        var qnew2 = vertex_init([0,0,0,0,0,0,0,0,0,0,0]);
        if (rrt_connect(qnew,qnew2,tree2) === "reached"){
            rrt_iterate = false;
            return find_path(tree1,tree2);
        }
    }
    var dummy = tree1;
    tree1 = tree2;
    tree2 = dummy;
    rrt_iter_count++;
    return false;
}


function rrt_extend(q, qnew, tree){ 

    var qnear = nearest_neighbor(q, tree);

    if (new_config(q,qnear,qnew)){
       // console.log("rrt_extend_in1");
        if (vertex_equality(qnew,q)){
            qnew = q; //hack
        }
        tree_add_vertex(qnew,tree);
        tree_add_edge(qnear,qnew,tree);
        if (vertex_equality(qnew,q)){
            return "reached";
        }
        else {
            return "advanced";
        }
    }
    else {
        return "trapped";
    }

}


function rand_range(min, max) { 
    return Math.random() * (max - min) + min; 
}
