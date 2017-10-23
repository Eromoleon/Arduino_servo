#include <Servo.h>


//
// Constants:

const double  dist = 100;
const double  l1 = 150;
const double  l2 = 300;

//
// Variables:

Servo servoR;
Servo servoL;
const int buttonPin = 2;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status
volatile bool stop = false;

//
// Typedefs:
struct Point{
	double  x;
	double  y;
};

struct Circle{
	double  x;
	double  y;
	double  r;
};

struct ServoAngles{
	double  theta1;
	double  theta2;
};

Point * intersection(Circle c1, Circle c2);
ServoAngles inverse(Point p, bool &outOfRange);

//
// Function headers:

void buttonPressed();
bool turnServo(int servoNum, int duration);
float x, y;

void setup(){
	
	servoR.attach(9);
	servoL.attach(10);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(buttonPin, INPUT);
	attachInterrupt(0,buttonPressed, HIGH); // (Interrupt number, Interrupt_Service_routine, Rising-Falling?)
	Serial.begin(115200);
	while(!Serial){}
	
	Serial.println("Enter position x");
	while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
	x = Serial.parseFloat();  // SLOW!! Needs to be rewritten!
	
	Serial.println("Enter position y");
	while (Serial.available() && Serial.read()); // empty buffer again
	while (!Serial.available());                 // wait for data
	y = Serial.parseFloat();  // SLOW!! Needs to be rewritten!
	Serial.println(x);
	Serial.println(y);
	bool outOfRange = false;
	Point p;
	p.x = x;
	p.y = y;
	ServoAngles angles = inverse(p, outOfRange);
	Serial.println((180/3.14)*angles.theta1);
	Serial.println((180/3.14)*angles.theta2);
	digitalWrite(LED_BUILTIN, HIGH);
	
}

void loop(){
	
	
	
	delay(500);
	/*
	if (stop == false) turnServo(servoR,5);
	if (stop == false) turnServo(servoL,5);
	delay(500);
	if (stop == false) turnServo(servoR,-5);
	if (stop == false) turnServo(servoL,-5);
	delay(500);
	servoR.write(90);
	
	digitalWrite(LED_BUILTIN, LOW);
	delay(500);
	*/
}

//Function definitions:

void buttonPressed(){
	//digitalWrite(LED_BUILTIN, LOW);
	
	stop = true;
}

bool turnServo(Servo &servo, int duration){
	int speed = 20; // a value between 0 and 90
	bool done = false;
	int ticks = 0;
	int direction =1;
	if (duration < 0){
		direction = -1;
		
	}
	ticks = duration*direction;
	while( ticks >= 0 && stop==false ){
		servo.write(90+speed*direction);
		delay(15);
		ticks--;
		
	}
	servo.write(90);
	done = true;
	return done;
}

Point * intersection(Circle c1, Circle c2){
	//double  * output = (double *)malloc(4*sizeof(double ));
	Point * outputPoints = (Point*)malloc(2*sizeof(Point));
	double  x1 = c1.x;
	double  y1 = c1.y;
	double  r1 = c1.r;
	double  x2 = c2.x;
	double  y2 = c2.y;
	double  r2 = c2.r;
	// P1, P2: center points, I1, I2: intersection points
	// L: line connecting the center points
	double  d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
	if (!(abs(r1 - r2) <= d && d <= r1 + r2)) { // no intersection
		outputPoints[0].x = -1;	
		outputPoints[0].y = -1;
		outputPoints[1].x = -1;
		outputPoints[1].y = -1;
		return outputPoints;
		
	 // empty list of results
	}
		 // intersection(s) should exist
	double  a = (r1*r1-r2*r2+d*d)/(2*d); // length of the orthogonal projection of P1_I1 vector to L (as well as P1I2 to L) 
	double  h = sqrt(r1*r1-a*a); // length of the orthogonal complement of P1_I1 vector to L
	double  xc = x1+(a/d)*(x2-x1); // Endpoint x of the orthogonal projection of P1_I1 vector to L
	double  yc = y1+(a/d)*(y2-y1); // Endpoint y of the orthogonal projection of P1_I1 vector to L
	// I1, I2:
	double  xi1 = xc + (h/d)*(y2-y1);
	double  yi1 = yc - (h/d)*(x2-x1);
		
	double  xi2 = xc - (h/d)*(y2-y1);
	double  yi2 = yc + (h/d)*(x2-x1);
	
	if (yi1 > yi2){
		outputPoints[0].x = xi1;	
		outputPoints[0].y = yi1;
		outputPoints[1].x = xi2;
		outputPoints[1].y = yi2;
	}
	else{
		outputPoints[1].x = xi1;	
		outputPoints[1].y = yi1;
		outputPoints[0].x = xi2;
		outputPoints[0].y = yi2;
	}
	
	return outputPoints;
}

ServoAngles inverse(Point p, bool &outOfRange){
	outOfRange = false;
	ServoAngles servoAngles;
	servoAngles.theta1 = 0;
	servoAngles.theta2 = 0;
	Circle cBaseL, cBaseR, cEnd;
	cBaseR.x = (-1)*dist/2;
	cBaseR.y = 0;
	cBaseR.r = l1;
	
	cBaseL.x = dist/2;
	cBaseL.y = 0;
	cBaseL.r = l1;
	
	cEnd.x = p.x;
	cEnd.y = p.y;
	cEnd.r = l2;
	
	Point * intersect_pointsL;
	intersect_pointsL = intersection(cBaseL, cEnd);
	Point LeftArm = intersect_pointsL[0];
	
	
	Point * intersect_pointsR;
	intersect_pointsR = intersection(cBaseR, cEnd);
	Point RightArm = intersect_pointsR[0];
	
	if(RightArm.y == -1||LeftArm.y==-1){
		outOfRange = true;
		servoAngles.theta1 = 0;
		servoAngles.theta2 = 0;
	}
	else{
	servoAngles.theta1 =M_PI/2-atan2(LeftArm.y, LeftArm.x-dist/2);
	servoAngles.theta2 =atan2(RightArm.y, RightArm.x+dist/2)-M_PI/2;
	}	
	free(intersect_pointsL);
	free(intersect_pointsR);
	return servoAngles;
}