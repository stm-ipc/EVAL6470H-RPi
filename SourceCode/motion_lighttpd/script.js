//JavaScript used to manage button



var shutdown_system = document.getElementById("shutdown_system");

var reload_register = document.getElementById("reload_register");


var motor1_backward = document.getElementById("motor1_backward");
var motor1_forward = document.getElementById("motor1_forward");
var motor1_run_stop = document.getElementById("motor1_run_stop");
var motor1_steppersecond = document.getElementById("motor1_steppersecond");

var motor2_backward = document.getElementById("motor2_backward");
var motor2_forward = document.getElementById("motor2_forward");
var motor2_run_stop = document.getElementById("motor2_run_stop");
var motor2_steppersecond = document.getElementById("motor2_steppersecond");

var motor3_backward = document.getElementById("motor3_backward");
var motor3_forward = document.getElementById("motor3_forward");
var motor3_run_stop = document.getElementById("motor3_run_stop");
var motor3_steppersecond = document.getElementById("motor3_steppersecond");

var motor4_backward = document.getElementById("motor4_backward");
var motor4_forward = document.getElementById("motor4_forward");
var motor4_run_stop = document.getElementById("motor4_run_stop");
var motor4_steppersecond = document.getElementById("motor4_steppersecond");

var motor5_backward = document.getElementById("motor5_backward");
var motor5_forward = document.getElementById("motor5_forward");
var motor5_run_stop = document.getElementById("motor5_run_stop");
var motor5_steppersecond = document.getElementById("motor5_steppersecond");

var motor6_backward = document.getElementById("motor6_backward");
var motor6_forward = document.getElementById("motor6_forward");
var motor6_run_stop = document.getElementById("motor6_run_stop");
var motor6_steppersecond = document.getElementById("motor6_steppersecond");

var motor7_backward = document.getElementById("motor7_backward");
var motor7_forward = document.getElementById("motor7_forward");
var motor7_run_stop = document.getElementById("motor7_run_stop");
var motor7_steppersecond = document.getElementById("motor7_steppersecond");

var motor8_backward = document.getElementById("motor8_backward");
var motor8_forward = document.getElementById("motor8_forward");
var motor8_run_stop = document.getElementById("motor8_run_stop");
var motor8_steppersecond = document.getElementById("motor8_steppersecond");



function send_motor_order( motor, action, nbstep) {
	//this is the http request
	var request = new XMLHttpRequest();

	request.open( "GET" , "motor.php?motor=" + motor + "&action=" + action + "&nbstep=" + nbstep );
	request.send(null);

	//receiving information
	request.onreadystatechange = function () {
		if (request.readyState == 4 && request.status == 200) {
			return (parseInt(request.responseText));
		}
	//test if fail
		else if (request.readyState == 4 && request.status == 500) {
			alert ("server error");
			return ("fail");
		}
	//else 
		else { 

			return ("fail"); 
			}
	}
}




//
//  Reload register parameters
//

reload_register.addEventListener("click", function () {
	send_motor_order( 0, 5, 0); 
});


//
//  Shutdown
//

shutdown_system.addEventListener("click", function () {
	shutdown_system.src = "../images/shutdown_system_selected.png";
	send_motor_order( 0, 99, 0); 

	/* alert ("System is shutting down");  */
});





//
// Motor 1
//
motor1_backward.addEventListener("click", function () {
	if(motor1_backward.alt === "nact") {
		motor1_backward.alt = "act";
		motor1_backward.src = "../images/Bouton1Backward_act.png";
		
		motor1_forward.alt = "nact";
		motor1_forward.src = "../images/Bouton1Forward_nact.png";
		
		motor1_run_stop.alt = "run";
		motor1_run_stop.src = "../images/Bouton1Running.png";
	}
	send_motor_order( 0, 3, -(motor1_steppersecond.value));
});


motor1_forward.addEventListener("click", function () { 
	if(motor1_forward.alt === "nact") {
		motor1_forward.alt = "act";
		motor1_forward.src = "../images/Bouton1Forward_act.png";
		
		motor1_backward.alt = "nact";
		motor1_backward.src = "../images/Bouton1Backward_nact.png";
		
		motor1_run_stop.alt = "run";
		motor1_run_stop.src = "../images/Bouton1Running.png";
	}
	send_motor_order( 0, 3, motor1_steppersecond.value);
});

motor1_run_stop.addEventListener("click", function () {
	if(motor1_run_stop.alt ==="run") {
		motor1_forward.alt = "nact";
		motor1_forward.src = "../images/Bouton1Forward_nact.png";
		
		motor1_backward.alt = "nact";
		motor1_backward.src = "../images/Bouton1Backward_nact.png";
		
		motor1_run_stop.alt = "stop";
		motor1_run_stop.src = "../images/Bouton1Stopped.png";
	}
	send_motor_order( 0, 4, 0);
});




//
// Motor 2
//

motor2_backward.addEventListener("click", function () { 
	if(motor2_backward.alt === "nact") {
		motor2_backward.alt = "act";
		motor2_backward.src = "../images/Bouton2Backward_act.png";
		
		motor2_forward.alt = "nact";
		motor2_forward.src = "../images/Bouton2Forward_nact.png";

		motor2_run_stop.alt = "run";
		motor2_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 1, 3, -(motor2_steppersecond.value));
});


motor2_forward.addEventListener("click", function () { 
	if(motor2_forward.alt === "nact") {
		motor2_forward.alt = "act";
		motor2_forward.src = "../images/Bouton2Forward_act.png";
		
		motor2_backward.alt = "nact";
		motor2_backward.src = "../images/Bouton2Backward_nact.png";

		motor2_run_stop.alt = "run";
		motor2_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 1, 3, motor2_steppersecond.value);	
});


motor2_run_stop.addEventListener("click", function () {
	if(motor2_run_stop.alt ==="run") {
		motor2_forward.alt = "nact";
		motor2_forward.src = "../images/Bouton2Forward_nact.png";
		
		motor2_backward.alt = "nact";
		motor2_backward.src = "../images/Bouton2Backward_nact.png";
		
		motor2_run_stop.alt = "stop";
		motor2_run_stop.src = "../images/Bouton2Stopped.png";
	}
	send_motor_order( 1, 4, 0);
});


//
// Motor 3
//

motor3_backward.addEventListener("click", function () { 
	if(motor3_backward.alt === "nact") {
		motor3_backward.alt = "act";
		motor3_backward.src = "../images/Bouton2Backward_act.png";
		
		motor3_forward.alt = "nact";
		motor3_forward.src = "../images/Bouton2Forward_nact.png";

		motor3_run_stop.alt = "run";
		motor3_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 2, 3, -(motor3_steppersecond.value));
});


motor3_forward.addEventListener("click", function () { 
	if(motor3_forward.alt === "nact") {
		motor3_forward.alt = "act";
		motor3_forward.src = "../images/Bouton2Forward_act.png";
		
		motor3_backward.alt = "nact";
		motor3_backward.src = "../images/Bouton2Backward_nact.png";

		motor3_run_stop.alt = "run";
		motor3_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 2, 3, motor3_steppersecond.value);	
});


motor3_run_stop.addEventListener("click", function () {
	if(motor3_run_stop.alt ==="run") {
		motor3_forward.alt = "nact";
		motor3_forward.src = "../images/Bouton2Forward_nact.png";
		
		motor3_backward.alt = "nact";
		motor3_backward.src = "../images/Bouton2Backward_nact.png";
		
		motor3_run_stop.alt = "stop";
		motor3_run_stop.src = "../images/Bouton2Stopped.png";
	}
	send_motor_order( 2, 4, 0);
});


//
// Motor 4
//

motor4_backward.addEventListener("click", function () { 
	if(motor4_backward.alt === "nact") {
		motor4_backward.alt = "act";
		motor4_backward.src = "../images/Bouton2Backward_act.png";
		
		motor4_forward.alt = "nact";
		motor4_forward.src = "../images/Bouton2Forward_nact.png";

		motor4_run_stop.alt = "run";
		motor4_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 3, 3, -(motor4_steppersecond.value));
});


motor4_forward.addEventListener("click", function () { 
	if(motor4_forward.alt === "nact") {
		motor4_forward.alt = "act";
		motor4_forward.src = "../images/Bouton2Forward_act.png";
		
		motor4_backward.alt = "nact";
		motor4_backward.src = "../images/Bouton2Backward_nact.png";

		motor4_run_stop.alt = "run";
		motor4_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 3, 3, motor4_steppersecond.value);	
});


motor4_run_stop.addEventListener("click", function () {
	if(motor4_run_stop.alt ==="run") {
		motor4_forward.alt = "nact";
		motor4_forward.src = "../images/Bouton2Forward_nact.png";
		
		motor4_backward.alt = "nact";
		motor4_backward.src = "../images/Bouton2Backward_nact.png";
		
		motor4_run_stop.alt = "stop";
		motor4_run_stop.src = "../images/Bouton2Stopped.png";
	}
	send_motor_order( 3, 4, 0);
});



//
// Motor 5
//

motor5_backward.addEventListener("click", function () { 
	if(motor5_backward.alt === "nact") {
		motor5_backward.alt = "act";
		motor5_backward.src = "../images/Bouton2Backward_act.png";
		
		motor5_forward.alt = "nact";
		motor5_forward.src = "../images/Bouton2Forward_nact.png";

		motor5_run_stop.alt = "run";
		motor5_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 4, 3, -(motor5_steppersecond.value));
});


motor5_forward.addEventListener("click", function () { 
	if(motor5_forward.alt === "nact") {
		motor5_forward.alt = "act";
		motor5_forward.src = "../images/Bouton2Forward_act.png";
		
		motor5_backward.alt = "nact";
		motor5_backward.src = "../images/Bouton2Backward_nact.png";

		motor5_run_stop.alt = "run";
		motor5_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 4, 3, motor5_steppersecond.value);	
});


motor5_run_stop.addEventListener("click", function () {
	if(motor5_run_stop.alt ==="run") {
		motor5_forward.alt = "nact";
		motor5_forward.src = "../images/Bouton2Forward_nact.png";
		
		motor5_backward.alt = "nact";
		motor5_backward.src = "../images/Bouton2Backward_nact.png";
		
		motor5_run_stop.alt = "stop";
		motor5_run_stop.src = "../images/Bouton2Stopped.png";
	}
	send_motor_order( 4, 4, 0);
});


//
// Motor 6
//

motor6_backward.addEventListener("click", function () { 
	if(motor6_backward.alt === "nact") {
		motor6_backward.alt = "act";
		motor6_backward.src = "../images/Bouton2Backward_act.png";
		
		motor6_forward.alt = "nact";
		motor6_forward.src = "../images/Bouton2Forward_nact.png";

		motor6_run_stop.alt = "run";
		motor6_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 5, 3, -(motor6_steppersecond.value));
});


motor6_forward.addEventListener("click", function () { 
	if(motor6_forward.alt === "nact") {
		motor6_forward.alt = "act";
		motor6_forward.src = "../images/Bouton2Forward_act.png";
		
		motor6_backward.alt = "nact";
		motor6_backward.src = "../images/Bouton2Backward_nact.png";

		motor6_run_stop.alt = "run";
		motor6_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 5, 3, motor6_steppersecond.value);	
});


motor6_run_stop.addEventListener("click", function () {
	if(motor6_run_stop.alt ==="run") {
		motor6_forward.alt = "nact";
		motor6_forward.src = "../images/Bouton2Forward_nact.png";
		
		motor6_backward.alt = "nact";
		motor6_backward.src = "../images/Bouton2Backward_nact.png";
		
		motor6_run_stop.alt = "stop";
		motor6_run_stop.src = "../images/Bouton2Stopped.png";
	}
	send_motor_order( 5, 4, 0);
});


//
// Motor 7
//

motor7_backward.addEventListener("click", function () { 
	if(motor7_backward.alt === "nact") {
		motor7_backward.alt = "act";
		motor7_backward.src = "../images/Bouton2Backward_act.png";
		
		motor7_forward.alt = "nact";
		motor7_forward.src = "../images/Bouton2Forward_nact.png";

		motor7_run_stop.alt = "run";
		motor7_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 6, 3, -(motor7_steppersecond.value));
});


motor7_forward.addEventListener("click", function () { 
	if(motor7_forward.alt === "nact") {
		motor7_forward.alt = "act";
		motor7_forward.src = "../images/Bouton2Forward_act.png";
		
		motor7_backward.alt = "nact";
		motor7_backward.src = "../images/Bouton2Backward_nact.png";

		motor7_run_stop.alt = "run";
		motor7_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 6, 3, motor7_steppersecond.value);	
});


motor7_run_stop.addEventListener("click", function () {
	if(motor7_run_stop.alt ==="run") {
		motor7_forward.alt = "nact";
		motor7_forward.src = "../images/Bouton2Forward_nact.png";
		
		motor7_backward.alt = "nact";
		motor7_backward.src = "../images/Bouton2Backward_nact.png";
		
		motor7_run_stop.alt = "stop";
		motor7_run_stop.src = "../images/Bouton2Stopped.png";
	}
	send_motor_order( 6, 4, 0);
});


//
// Motor 8
//

motor8_backward.addEventListener("click", function () { 
	if(motor8_backward.alt === "nact") {
		motor8_backward.alt = "act";
		motor8_backward.src = "../images/Bouton2Backward_act.png";
		
		motor8_forward.alt = "nact";
		motor8_forward.src = "../images/Bouton2Forward_nact.png";

		motor8_run_stop.alt = "run";
		motor8_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 7, 3, -(motor8_steppersecond.value));
});


motor8_forward.addEventListener("click", function () { 
	if(motor8_forward.alt === "nact") {
		motor8_forward.alt = "act";
		motor8_forward.src = "../images/Bouton2Forward_act.png";
		
		motor8_backward.alt = "nact";
		motor8_backward.src = "../images/Bouton2Backward_nact.png";

		motor8_run_stop.alt = "run";
		motor8_run_stop.src = "../images/Bouton2Running.png";
	}
	send_motor_order( 7, 3, motor8_steppersecond.value);	
});


motor8_run_stop.addEventListener("click", function () {
	if(motor8_run_stop.alt ==="run") {
		motor8_forward.alt = "nact";
		motor8_forward.src = "../images/Bouton2Forward_nact.png";
		
		motor8_backward.alt = "nact";
		motor8_backward.src = "../images/Bouton2Backward_nact.png";
		
		motor8_run_stop.alt = "stop";
		motor8_run_stop.src = "../images/Bouton2Stopped.png";
	}
	send_motor_order( 7, 4, 0);
});





