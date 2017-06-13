<!-- This page is requested by the JavaScript, it send order to the motor -->
<?php
//Getting and using values
$filename = "/tmp/cmd.txt";


if (!file_exists($filename)) {
	

	if (isset ($_GET["motor"]) && isset($_GET["action"]) && isset($_GET["nbstep"]) ) {
		$motor = strip_tags($_GET["motor"]);
		$action = strip_tags($_GET["action"]);
		$nbstep = strip_tags($_GET["nbstep"]);
		
		//Testing if values are numbers
		if ( (is_numeric($motor)) && (is_numeric($action)) && (is_numeric($nbstep)) && 
		     ($motor >= 0) && ($motor <= 7) && ($action >= 1) ) {
			file_put_contents($filename, $motor . ":" . $action . ":" . $nbstep, null, null);
		}
		else 
		{ 
			echo ("fail"); 
		}
	} //print fail if cannot use values
	else 
	{ 
		echo ("fail"); 
	}
}
?>
