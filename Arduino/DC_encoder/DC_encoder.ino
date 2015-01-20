

#include <EEPROMex.h>					https://github.com/thijse/Arduino-Libraries/tree/master/EEPROMEx
#include <AS5045.h>						https://github.com/DashZhang/AS5045
#include <PID_v1.h>						http://playground.arduino.cc/Code/PIDLibrary
#include <TimerOne.h>					https://www.pjrc.com/teensy/td_libs_TimerOne.html
#include <TimerThree.h>					https://www.pjrc.com/teensy/td_libs_TimerOne.html

// CHANGE THESE AS APPROPRIATE
#define CSpin_A   2
#define CLKpin_A  3
#define DOpin_A   4

AS5045 encoder_A (CSpin_A,CLKpin_A, DOpin_A) ;

#define CSpin_B   A0
#define CLKpin_B  A1
#define DOpin_B   A2

AS5045 encoder_B (CSpin_B,CLKpin_B, DOpin_B) ;


// Motor pins
#define motor_A_DIR1 10
#define motor_A_DIR2 9

#define motor_B_DIR1 11
#define motor_B_DIR2 5

#define MAXENCVALUE 4095
#define MINSPEED 6

#define OFFSETANGLE(deg) (((4096/360)*(180+deg)))

struct motorValues{
int SensorValue;
int inputOffet;
int rawEncValue;
int encOffset;
int encValue;
float p_val;	
int rawSensorValue;

double Setpoint, Input, Output;

	} Motor_A, Motor_B;


double consKp_A=0.8, consKi_A=0.01, consKd_A=0.05;
PID motor_A_PID(&(Motor_A.Input), &(Motor_A.Output), &(Motor_A.Setpoint),consKp_A,consKi_A,consKd_A, DIRECT);

double consKp_B=0.8, consKi_B=0.01, consKd_B=0.05;
PID motor_B_PID(&(Motor_B.Input), &(Motor_B.Output), &(Motor_B.Setpoint),consKp_B,consKi_B,consKd_B, DIRECT);



void setup ()
{
  
	pinMode(OUTPUT, motor_A_DIR1);  
	pinMode(OUTPUT, motor_B_DIR2);
 
	pinMode(OUTPUT, motor_B_DIR1);
	pinMode(OUTPUT, motor_B_DIR2);  
  
	pinMode(A4, INPUT);
	pinMode(A5, INPUT);
	
	motor_A_PID.SetMode(AUTOMATIC);
	motor_A_PID.SetOutputLimits(-900,900);

	motor_B_PID.SetMode(AUTOMATIC);
	motor_B_PID.SetOutputLimits(-900,900);
		
	Timer1.initialize(100);
	Timer3.initialize(100);
	
	
	Serial.begin (115200) ;   // NOTE BAUD RATE
	//while (!Serial) {
		//; // wait for serial port to connect. Needed for Leonardo only
	//}
	
	if (!encoder_A.begin ())
		Serial.println ("Error setting up AS5045") ;
		
	if (!encoder_B.begin ())
	Serial.println ("Error setting up AS5045") ;		
		
	Motor_A.SensorValue = 0;
	Motor_A.inputOffet =  0;
	Motor_A.rawEncValue = 0;
	Motor_A.encOffset = EEPROM.readInt(0);
	Motor_A.encValue = 0;
	Motor_A.p_val = 1.5;		

	Motor_B.SensorValue = 0;
	Motor_B.inputOffet =  0;
	Motor_B.rawEncValue = 0;
	Motor_B.encOffset = EEPROM.readInt(2);
	Motor_B.encValue = 0;
	Motor_B.p_val = 1.5;
}



void loop ()
{
	// DEBUG Plot data
	sendPlotData("Encoder_A", Motor_A.rawEncValue);
	sendPlotData("encOffset_A", Motor_A.encOffset);
	sendPlotData("Setpoint_A", Motor_A.Setpoint);
	sendPlotData("Input_A", Motor_A.Input);
	sendPlotData("Output_A", Motor_A.Output);
  
	sendPlotData("Encoder_B", Motor_B.rawEncValue);
	sendPlotData("encOffset_B", Motor_B.encOffset);
	sendPlotData("Setpoint_B", Motor_B.Setpoint);
	sendPlotData("Input_B", Motor_B.Input);
	sendPlotData("Output_B", Motor_B.Output);
    
	// main controll loop
  PIDLoop();
  
  delay(50);
  
  while (Serial.available() > 0) {
    char inCMD = (char)Serial.read();
    
    if(inCMD == 'G')	//stepper 0
    {
     Motor_A.encOffset = 2047-Motor_A.rawEncValue;
	 EEPROM.writeInt(0, Motor_A.encOffset );
    }
    else if(inCMD == 'B')
    {
      Motor_B.encOffset = 2047-Motor_B.rawEncValue;
	  EEPROM.writeInt(2, Motor_B.encOffset );
	  
    }
  } 
}

void PIDLoop()
{
  // Motor A
  // read Input
  Motor_A.rawEncValue = MAXENCVALUE - encoder_A.read();
  Motor_A.encValue = calcOffset(Motor_A.rawEncValue,Motor_A.encOffset,MAXENCVALUE);
  
  // read Setpoint
  Motor_A.rawSensorValue = analogRead(A5);
  Motor_A.SensorValue = map(Motor_A.rawSensorValue , 0, 1023, OFFSETANGLE(-120) , OFFSETANGLE(120) );
  
  
  // Set Input Values for PID
  Motor_A.Setpoint = (double)Motor_A.SensorValue;
  Motor_A.Input = (double) Motor_A.encValue;
  
  // calc PID
  motor_A_PID.Compute();
  
  // set Output Values for PID
  setMotor_A(Motor_A.Output);
  
  
  // Motor B
  // read Input
  Motor_B.rawEncValue = MAXENCVALUE - encoder_B.read();
  Motor_B.encValue = calcOffset(Motor_B.rawEncValue,Motor_B.encOffset,MAXENCVALUE);
   
  // read Setpoint
  Motor_B.rawSensorValue = analogRead(A4);
  Motor_B.SensorValue = map(Motor_B.rawSensorValue , 0, 1023, OFFSETANGLE(-130) , OFFSETANGLE(130) );

  // Set Input Values for PID   
  Motor_B.Setpoint = (double)Motor_B.SensorValue;
  Motor_B.Input = (double)Motor_B.encValue;

  // calc PID   
  motor_B_PID.Compute();

  // set Output Values for PID   
  setMotor_B(Motor_B.Output); 
}


void setMotor_A(double speed)
{	
	// don't run the dc motors on to low speed this will burn the brushes.
	if (speed >MINSPEED)
	{
        //analogWrite(motor_1_DIR1,0);
        //analogWrite(motor_1_DIR2,(int)speed);
		
		Timer1.pwm(motor_A_DIR1, 0);          
		Timer1.pwm(motor_A_DIR2, (int)speed);          		
	}
	else if (speed < -MINSPEED)
	{
        //analogWrite(motor_1_DIR2,0);
		//analogWrite(motor_1_DIR1,(int)(-speed));	

		Timer1.pwm(motor_A_DIR1, (int)(-speed));			
		Timer1.pwm(motor_A_DIR2, 0);
	}
	else
	{
        //analogWrite(motor_1_DIR1,0);
        //analogWrite(motor_1_DIR2,0);	
			
		Timer1.pwm(motor_A_DIR1,0);			
		Timer1.pwm(motor_A_DIR2, 0);

	}
}

void setMotor_B(double speed)
{
	// don't run the dc motors on to low speed this will burn the brushes.
	if (speed >MINSPEED)
	{
		//analogWrite(motor_2_DIR1,0);
		//analogWrite(motor_2_DIR2,(int)speed);
		
		Timer1.pwm(motor_B_DIR1, 0);
		Timer3.pwm(motor_B_DIR2, (int)speed);				
	}
	else if (speed < -MINSPEED)
	{
		//analogWrite(motor_2_DIR2,0);
		//analogWrite(motor_2_DIR1,(int)(-speed));
		
		Timer1.pwm(motor_B_DIR1, (int)(-speed));	
		Timer3.pwm(motor_B_DIR2, 0);			
	}
	else
	{
		//analogWrite(motor_2_DIR1,0);
		//analogWrite(motor_2_DIR2,0);
		
		Timer1.pwm(motor_B_DIR1,0);
		Timer3.pwm(motor_B_DIR2, 0);
	}
}


int calcOffset(int inVal, int Offset, int MaxVal)
{
	inVal += Offset;
  if (inVal > MaxVal)
  {
	  inVal -= MaxVal;
  }
  else if (inVal < 0)
  {
	  inVal += MaxVal;
  }	
  
  return inVal;
}

void sendPlotData(String seriesName, double data)
{
	Serial.print("{");
		Serial.print(seriesName);
		Serial.print(",T,");
		Serial.print(data);
	Serial.println("}");
}