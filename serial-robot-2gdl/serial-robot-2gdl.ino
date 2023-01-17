int art2Pot = A6;
int art2MotorA = 5;
int art2MotorB = 6;

double input, output, setpoint;
 
void setup(){
  pinMode(art2MotorA, OUTPUT);
  pinMode(art2MotorB, OUTPUT);

  setpoint = 300;

  Serial.begin(9600);
}    
 
void loop(){
  if(Serial.available()){
    String msg = Serial.readStringUntil('\n');
    setpoint = msg != "" ? msg.toDouble() : setpoint;
    delay(100);
  }

  input = analogRead(art2Pot);
  output = computePID(input, setpoint);  
  
  output = output > 255 ? 255 : output;   //Output range
  output = output < -255 ? -255 : output;

  if(output > 0 && input < 800){
    analogWrite(art2MotorA, output);
    analogWrite(art2MotorB, 0);
  }
  else if(output < 0 && input > 200){
    analogWrite(art2MotorA, 0);
    analogWrite(art2MotorB, -output);
  }
  delay(10);
  analogWrite(art2MotorA, 0);
  analogWrite(art2MotorB, 0);
}
 
double computePID(double input, double setpoint){  
  static unsigned long  currentTime   = 0; //t_i
  static unsigned long  previousTime  = 0; //t_(i-1)
  double                elapsedTime   = 0; //dt

  double                error         = 0; //e_i  
  static double         lastError     = 0; //e_(i-1)
  static double         accumError    = 0; //sum(e_(i))
  double                rateError     = 0; //de/dt
  double                output        = 0;

  int maxError = 10;
  int minError = -10; 

  double  kp =0.4, 
          kd =300, 
          ki =0.0005;

  int offset = 60;
  
  currentTime = millis();                              
  elapsedTime = (double)(currentTime - previousTime);     
        
  error = setpoint - input; 
  error = error < maxError && error > minError ? 0 : error; //Error range
                    
  accumError += error * elapsedTime;                      
  rateError = (error - lastError) / elapsedTime;         
  
  if(error == 0) output = 0;
  else {
    output = kp*error + kd*rateError + ki*accumError;
    output = output > 0 ? output + offset : output; //Inertial static offset
    output = output < 0 ? output - offset : output;
  }
  
  lastError = error;                                      
  previousTime = currentTime;                     
    
  return output;
}