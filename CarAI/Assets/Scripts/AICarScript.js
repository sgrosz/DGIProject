
/* 
	Script based on the AICarScript written by FlatTutorials
	(www.flattutorials.com)
	Extended to fit the needs for this project by Ludvig Jansson, 911223-2872
*/

var centerOfMass : Vector3; // Adjust the center of mass of the car to counter its tendency to tip
var path : Array; // The path the car is following
var pathGroup : Transform; // The original GameObject of the path
var laneNumber : int; //The laneNumber which the car is att this moment
var maxSteer : float = 15.0f; // Constant to add sense of "steering"
var wheelFR : WheelCollider; // | Wheels
var wheelFL : WheelCollider; // | of
var wheelRR : WheelCollider; // | the
var wheelRL : WheelCollider; // v car
var currentPathObject : int; // The current index of the path the car is in
var distFromPath : float = 20; // The proximity threshold when the car changes currentPathObject
var maxTorque : float = 5; // Motor torque to move the car
var currentSpeed : float; // The current speed
var topSpeed : float = 150; // The maximum speed of the car
var decellerationSpeed : float = 60; // This is applied when the car is braking
var brakeSensorLength : float = 25; // How far ahead the brake sensor picks up objects
var sideSensorLength : float = 7; // How far the sie sensors are
var frontSensorStartPoint : float = 3.42; // Where the front sensors are located on the car, is multiplied with the car's forward vector
var frontSensorSideDistance : float = 1.2; // Where the front side sensors are located relative to the front, multiplied with the car's right vector

private var startPos : Vector3; // Used to reset the car when it has reached the end of the road
private var laneFlag : boolean = false;
private var random : int;



/*
	This function is called when the program starts
*/
function Start () {
 	/*
 		Save the start position, set the center of mass and retrieve the path objects 
 	*/
	startPos = transform.position;
	rigidbody.centerOfMass = centerOfMass;
	GetPath();
}

/*
	Retrieve the path objects from the path GameObject
*/
function GetPath(){
	var chosenPath : Transform = pathGroup.GetChild(laneNumber);
	var path_objects : Array = chosenPath.GetComponentsInChildren(Transform);
	path = new Array();
	
	for(var path_obj : Transform in path_objects){
	// Make sure the parent of this object is not added to the array
		if(path_obj != pathGroup){
			path[path.length] = path_obj;
		}
	}
}

/*
	This function is called repeatedly until the program is terminated
*/
function Update () {
	random = Random.Range(1, 1000);
	// Figure out which path object to steer to
	GetSteer();
	//Randomly change lanes
	if(random == 21){
		laneFlag = true;
	}	
	// Move to this path object
	Move();
	// See if the car needs to avoid any other cars and does this of so
	FrontSensors();
}

/* 
	Function for steering
*/
function GetSteer(){
	// Get the direction this car is currently going in
	var steerVector : Vector3 = transform.InverseTransformPoint(Vector3(path[currentPathObject].position.x, transform.position.y, path[currentPathObject].position.z));
	// Find the new steer angle of the front wheels and set it
	var newSteer : float = maxSteer*(steerVector.x / steerVector.magnitude);
	wheelFL.steerAngle = newSteer;
	wheelFR.steerAngle = newSteer;
	
	// Determine if the car should steer towards the next path object
	if(Mathf.Abs(transform.position.z-path[currentPathObject].transform.position.z) <= distFromPath){
		currentPathObject++;
		//If the car should change lane randomly
		if(laneFlag == true && (LeftLaneAvailable() || RightLaneAvailable())){
			if(LeftLaneAvailable() && laneNumber != 0){
				laneNumber -= 1;
			} else if(RightLaneAvailable() && laneNumber != 3){
				laneNumber += 1;
			}
			GetPath();
			laneFlag = false;
		}
		if(currentPathObject == path.length-1){
			transform.position = startPos;
			currentPathObject = 0;
		}
	}
}

/* 
	Function for actually moving the car
*/
function Move(){
	// Calculate the speed the car is currently travelling in
	currentSpeed = 2*(22/7)*wheelRL.radius *wheelRL.rpm * 60/1000;
	currentSpeed = Mathf.Round(currentSpeed);
	// Accellerate if the car hasn't reached max speed and brake if it has
	if(currentSpeed <= topSpeed){
		wheelRR.brakeTorque = 0;
		wheelRL.brakeTorque = 0;
		wheelRL.motorTorque = maxTorque;
		wheelRR.motorTorque = maxTorque;
	} else {
		wheelRL.motorTorque = 0;
		wheelRR.motorTorque = 0;
		wheelRL.brakeTorque = decellerationSpeed;
		wheelRR.brakeTorque = decellerationSpeed;
	}
	
}

/*
	This handles the real car AI. The foundation of this code was provided by the Car AI tutorial, 
	but most of the code was written by Ludvig Jansson.
*/
function FrontSensors(){
	var avoidSensitivity : float = 0; // Reset the avoid sensitivity, which will be used to steer the car
	var pos : Vector3; // This will contain the original position of rays cast from the car
	var hit : RaycastHit; // These three contain information of what the ray hit if it did

	pos = transform.position;
	pos += transform.forward*frontSensorStartPoint;
	var halfRightPos : Vector3 = pos + transform.right*frontSensorSideDistance*2/3;
	var halfLeftPos : Vector3 = pos - transform.right*frontSensorSideDistance*2/3;

	// Fire away the braking sensors
	if(Physics.Raycast(pos, transform.forward, hit, brakeSensorLength)){
		// Middle sensor
		Brake(hit, pos);	
	}  else if(Physics.Raycast(halfRightPos, transform.forward, hit, brakeSensorLength)){
	    // Right sensor
		Brake(hit, pos);
	} else if(Physics.Raycast(halfLeftPos, transform.forward, hit, brakeSensorLength)){
		// Left sensor
		Brake(hit, pos);
	} else {
		// Reset the braking if no obstacle is hit
		wheelRL.brakeTorque = 0;
		wheelRR.brakeTorque = 0;
	}
}

function LeftLaneAvailable(){
	//The position of the car
	pos = transform.position;
	var hit : RaycastHit; // These three contain information of what the ray hit if it did
	
	//The sensors for the front and rear side
	var frontPos : Vector3 = pos + transform.forward;
	var rearPos : Vector3 = pos - transform.forward;
	
	//Adjust the sensors the the left side of the car
	var carLeftFront : Vector3 = frontPos - transform.right;
	var carLeftRear : Vector3 = rearPos - transform.right;
	
	//Set a 45 degree angle for the direction of the rays for the rear and front
	var rear45 : Vector3 = transform.forward + transform.right;
	var front45 : Vector3 = transform.forward - transform.right;
	
	//Checks if a car is to close to the side sensors, if so then the lane is not available
	if(Physics.Raycast(carLeftFront, front45, hit, sideSensorLength)){
		return false;
	}
	
	if(Physics.Raycast(carLeftFront, transform.right, hit, sideSensorLength)){
		return false;
	}
	
	if(Physics.Raycast(carLeftRear, -transform.right, hit, sideSensorLength)){
		return false;
	}
	
	if(Physics.Raycast(carLeftRear, -rear45, hit, sideSensorLength)){
		return false;
	}

	return true;
}
function RightLaneAvailable(){
		//The position of the car
	pos = transform.position;
	var hit : RaycastHit; // These three contain information of what the ray hit if it did
	
	//The sensors for the front and rear side
	var frontPos : Vector3 = pos + transform.forward;
	var rearPos : Vector3 = pos - transform.forward;
	
	//Adjust the sensors the the left side of the car
	var carRightFront : Vector3 = frontPos - transform.right;
	var carRightRear : Vector3 = rearPos - transform.right;
	
	//Set a 45 degree angle for the direction of the rays for the rear and front
	var front45 : Vector3 = transform.forward + transform.right;
	var rear45 : Vector3 = transform.forward - transform.right;

	//Checks if a car is to close to the side sensors, if so then the lane is not available	
	if(Physics.Raycast(carRightFront, front45, hit, sideSensorLength)){
		return false;
	}
	
	if(Physics.Raycast(carRightFront, transform.right, hit, sideSensorLength)){
		return false;
	}
	
	if(Physics.Raycast(carRightRear, transform.right, hit, sideSensorLength)){
		return false;
	}
	
	if(Physics.Raycast(carRightRear, -rear45, hit, sideSensorLength)){
		return false;
	}

	return true;
}

/*
	If the car needs to brake as decided by the Sensors() function, this function handles
	it.
*/
function Brake(hit : RaycastHit, pos : Vector3){
	// Make sure it actually was a car the ray hit
	if(hit.transform.tag == "Car"){		
		wheelRL.brakeTorque = decellerationSpeed;
		wheelRR.brakeTorque = decellerationSpeed;
		if(LeftLaneAvailable() && laneNumber != 0){
			laneNumber -= 1;
			GetPath();
		}else if(RightLaneAvailable() && laneNumber != 3){
			laneNumber += 1; 
			GetPath();
		}
	}
}




