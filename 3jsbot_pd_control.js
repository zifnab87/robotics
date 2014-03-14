//////////////////////////////////////////////////
/////     MOTION CONTROL ROUTINES 
//////////////////////////////////////////////////

// CS148: add PD controller here
function robot_pd_control () {
	var cur_date = new Date();
	for (x in robot.joints){
		robot.joints[x].servo.p_desired = cur_date.getSeconds()/60*2*Math.PI;
		robot.joints[x].control += robot.joints[x].servo.k*(robot.joints[x].servo.p_desired - robot.joints[x].angle);
	}

}

