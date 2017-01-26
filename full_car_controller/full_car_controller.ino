/* This is the code that ran our intelligent ballistic cart */

#include <elapsedMillis.h>

//libraries

#include <PID_v1.h>
#include <math.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#define DATA_LIMIT  1010

//radio declarations

RF24 radio(14, 15);

static const uint8_t analog_pins[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A15, A16, A17, A18, A19, A20};
int pin_triggered;
const byte rxAddr[6] = "00001";
byte addresses[][6] = {"1Node", "2Node"};             // Radio pipe addresses for the 2 nodes to communicate.

elapsedMillis timeElapsed;

//trigger declarations


//motor declarations

int led = 13;
int motorPinF = 19; //Set HIGH to move forward
int motorPinR = 20; //Set HIGH to move backward
int motorPin = 21;
int motor_command;
int motor_command_data;

//variables for encoder measurements
int test_data;
int encoder0PinA = 22;
int encoder0PinB = 23;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;
unsigned long start_time, time;
int previous_time = 0;
unsigned long current_time;
int distance_desired;
int flag = 1;
int launch = 0;
int check_kill_frequency = 20;
int data_sample_frequency = 10;
int global_counter = 0;
bool configured = false;
bool connection_established = false;
char* init_message = "Car on: waiting for response...";
char* experiment_message = "Running experiment";
char* configure_messages_array[] = {"Mode?: ", "Kp?: ", "Ki?: ", "Kd?: ", "Velocity?: ", "Counts to launch?: ", "Counts to ball?:  " , "Launch? (0/1): " };
char* send_data_message = "Send data? (0/1): ";
float configuration[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //mode, kp, ki, kd, velocity, encoder_counts_to_launch, encoder_counts_to_ball, launch?
                                                   // 0     1   2   3   4                   5                    6                    7

//pid declarations
int reading;
bool reading_received = false;
bool send_signal_received = false;
int test_reading;
int testvar = 69;
float testfloat=32.0;
float message;
float message_data;
int rounded_message;
int rounded_message_data;
double encoder_count_desired, input_pos, output_pos;
double velocity_desired, input_vel, output_vel;
bool experiment_done = false;
int experiment_time_limit = 15000; //five-second experiments
//creates matrix for storing data, number of data points
//defined above
float data_matrix[DATA_LIMIT][2];
float transmit_array[1][2];
int experiment_index = 0; //counts number of loops it has ran for
int old_experiment_index = 1; //helps check if its N loops further ahead
int data_index = 0; //index of array to write to
int data_threshold = 300; //how many loops should it wait before writing.

double g = 9.8; //m/s^2
int angle_launch = 0;
int angle_car = 0;
int y_final = 0;
int y_initial = 0;
double launch_position;// =
double air_time = 1;
double car_velocity, now;
double encoder0Pos_last;
double encoder0Pos_switch;
double encoders_traveled;
// PID declarations
double encoder_speed_desired;
double encoder_speed;
//mode 1 test variables, assuming 1m/s car velocity at launch
double encoder_counts_to_launch; //start at 140cm -> launch at 250cm;
double encoder_counts_to_ball; //assuming 1.5s, launch at 250cm, catch at 400;

int track_sensor;

// TRIGGER
double trigger_time;
double run_time;
int triggerPinF = 4;
int triggerPinR = 5;
int triggerPin = 6;
int trigger_input;
double encoder_launch;
int counter = 1;
bool data_mode;


PID position_PID(&input_pos, &output_pos, &encoder_count_desired, 3, 0, .1, DIRECT);
PID velocity_PID(&input_vel, &output_vel, &encoder_speed_desired, 1.3, 0, 0, DIRECT);
//kp_v=1.3==good
void setup() {

  //---------------TRIGGER SETUP--------------//

  pinMode(triggerPin, OUTPUT);
  pinMode(triggerPinR, OUTPUT);
  pinMode(triggerPinF, OUTPUT);


  //---------------MOTOR SETUP----------------//

  pinMode(motorPin, OUTPUT);
  pinMode(motorPinF , OUTPUT);
  pinMode(motorPinR , OUTPUT);

  //----------------ENCODER SETUP---------------//


  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);

  //---------------PID SETUP-------------------//

  //PID setup
  position_PID.SetMode(AUTOMATIC);
  position_PID.SetOutputLimits(-215, 215);
  velocity_PID.SetMode(AUTOMATIC);
  velocity_PID.SetOutputLimits(0, 255);

  //---------------RADIO SETUP-----------------//

  SPI.begin();
  radio.begin();
  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads
  
  radio.setRetries(15, 15);
  radio.openWritingPipe(addresses[1]);        // Both radios listen on the same pipes by default, but opposite addresses
  radio.openReadingPipe(1, addresses[0]);
  radio.stopListening();

  //---------------OTHER SETUP-----------------//


  distance_desired = 150; //distance in cm;
  encoder_speed_desired = 418;

  encoder_count_desired = round(distance_desired * 150 / 35.9); //replace w/desired count
  //mode 1 test variables, assuming 1m/s car velocity at launch
  encoder_counts_to_launch = 460; //start at 140cm -> launch at 250cm;
  encoder_counts_to_ball = 636; //assuming 1.5s, launch at 250cm, catch at 400;
  pinMode(led, OUTPUT); //debug light
  Serial.begin (115200);


  //-------------------------------------------//

}




void loop() {
  delay(1000);
  while (!configured) {
    analogWrite(motorPin, 0);
    configure_experiment();
  }
  run_experiment();
  data_mode=true;
  while (data_mode){
      send_data();
  }



}



//------------------------CONTROLLER MODES----------------------------//

void mode_1(unsigned long time_started) {

  encoder0Pos = 0;
  encoder_counts_to_launch = configuration[5];
  encoder_counts_to_ball = configuration[6];
  encoder_speed_desired = configuration[4] * (150 / 35.9) * 100;
  counter = 1;
  velocity_PID.SetTunings(1.3, 0, 0);

  while (!experiment_done) {

    //Serial.println(encoder_counts_to_launch);
    update_encoder();

    if (check_time(time_started) > experiment_time_limit) {
      analogWrite(motorPin, 0);

      experiment_done = true;
      configured = false;
      Serial.println("done!");
      analogWrite(motorPin, 0);

    }
    store_data(encoder0Pos, time_started);

    //----------------------------------------------------//

    if (encoder0Pos < encoder_counts_to_launch) {
      //velocity control to desired distance;
      velocity_control(encoder_speed_desired);
      Serial.print("velocity: ");
      Serial.println(encoder0Pos);
    }

    else if (encoder0Pos > encoder_counts_to_launch){
    //trigger launch at launch point
      Serial.print("position: ");
      Serial.println(encoder0Pos);


      position_PID.SetTunings(configuration[1], configuration[2], configuration[3]);
      launch_ball();
      //desired_position=track_dist;
      //go to desired distance
      position_control(encoder_counts_to_launch + encoder_counts_to_ball);
    }


    //-----------------------------------------------------------//





  }
  for (int i = 0; i <= DATA_LIMIT; i++)
  {
    /*Serial.print(data_matrix[i][0]);
      Serial.print("---");
      Serial.println(data_matrix[i][1]);*/
  }

}






void mode_2(unsigned long time_started) {
  position_PID.SetTunings(configuration[1], configuration[2], configuration[3]);

  while (!experiment_done) {

    Serial.println(position_PID.GetKd());
    if (check_time(time_started) > experiment_time_limit) {
      analogWrite(motorPin, 0);

      experiment_done = true;
      configured = false;
      Serial.println("done!");
      analogWrite(motorPin, 0);

    }
    store_data(motor_command_data, time_started);
    update_encoder();

    
    position_control(configuration[5]);;

  }





}

void mode_3(unsigned long time_started) {
  //velocity control only
  encoder_speed_desired = configuration[4] * (150 / 35.9) * 100;

  experiment_done = false;
  velocity_PID.SetTunings(configuration[1], configuration[2], configuration[3]);

  while (!experiment_done) {


    if (check_time(time_started) > experiment_time_limit) {
      analogWrite(motorPin, 0);

      experiment_done = true;
      configured = false;
      Serial.println("done!");
      analogWrite(motorPin, 0);

    }
   
    //store_data(1.0,time_started);
    update_encoder();

    double track_dist = 626;
    velocity_control(encoder_speed_desired);

  }

}

void mode_4(unsigned long time_started) {
  counter = 1;
  while (!experiment_done) {


    if (check_time(time_started) > experiment_time_limit) {
      analogWrite(motorPin, 0);

      experiment_done = true;
      configured = false;
      Serial.println("done!");
      analogWrite(motorPin, 0);

    }
    launch_ball();
  }
}







//------------------------CAR INTERNAL FUNCTIONS----------------------------//
void send_data() {
  analogWrite(motorPin, 0);

  configured=true;
  send_signal_received = false;
  /*for (int i=0; i<1010; i++){
    data_matrix[i][0]=i; 
    data_matrix[i][1]=i;
  }*/

  while (send_signal_received == false && data_mode) {
    
    radio.write(send_data_message, strlen(send_data_message));


    //Serial.println(radio.available());
    int index=0;
    if (radio.available() ) {
      Serial.println("hi_car");
      radio.read( &message_data, sizeof(message_data));
      rounded_message_data = (int) message_data;
      if (rounded_message_data == 1) {
        send_signal_received=true;
        configured=true;
        while (index<10) {
     
  
         
          radio.write(&testvar, sizeof(int));
          
          Serial.println(testvar);
          index++;
          delay(10);

         
        }

        while (index<1010) {
     
  
          transmit_array[0][0]=data_matrix[index-10][0];
          transmit_array[0][1]=data_matrix[index-10][1];
          Serial.println(transmit_array[0][1]);

          delay(10);

          radio.write(&transmit_array, sizeof(transmit_array));
          index++;

         
        }
        data_mode=false;
        configured=false;
       

      }
      else if (rounded_message_data == 0){
        send_signal_received=true;
        data_mode=false;
        configured=false;
      }
      Serial.println(message);
      //reading_received = true;

    }

  }


}

void store_data(float data, unsigned long time_started) {
  //stores [data,time] in an array called data_matrix

  if (data_index < DATA_LIMIT) {
    if (experiment_index >= (data_threshold + old_experiment_index)) {

      data_matrix[data_index][0] = data;
      data_matrix[data_index][1] = check_time(time_started);
      old_experiment_index = experiment_index;
      data_index++;

    }

    experiment_index++;
  }
}



void run_experiment() {
  //this function directs the car to run different controllers
  //based on the configuration chosen

  experiment_done = false;
  experiment_index = 0;
  old_experiment_index = 0;
  data_index = 0;
  encoder0Pos = 0;
  current_time = timeElapsed;
  if (configuration[0] == 1.0) {
    mode_1(current_time);
  }
  else if (configuration[0] == 2.0)
  {
    mode_2(current_time);
  }
  else if (configuration[0] == 3.0)
  {
    mode_3(current_time);
  }
  else if (configuration[0] == 4.0) {
    mode_4(current_time);
  }
  //put configuration/run experiment stuff here


}



float check_time(unsigned long time_started) {

  //this function checks to see if the experiment has run for a certain amount of time
  //and returns TRUE if it is. Used to kill the experiment
  float timefloat = (float) timeElapsed - time_started;
  //Serial.println(timefloat);
  //Serial.println(timefloat);
  return timefloat;

}

//-----------------------RADIO FUNCTIONS----------------------------//


void configure_experiment() {

  //this function handles talking to the remote radio and reading
  //responses into a configuration it uses to run responses.

  establish_connection();

  for (int i = 0; i <= 7; i++) {

    while (reading_received == false) {

      if ( radio.write(configure_messages_array[i], strlen(configure_messages_array[i])) ) {


        //Serial.println(radio.available());

        while (radio.available() ) {
          //Serial.println("hi_car");
          radio.read( &message, sizeof(message));
          Serial.println(message);

          reading_received = true;
          configuration[i] = message;

        }
      }
    }
    reading_received = false;

  }

  for (int i = 0; i <= 7; i++) {
    Serial.print("Configuration Entry ");
    Serial.print(i);
    Serial.print(": is ");
    Serial.println(configuration[i]);
  }

  if (configuration[7] == 1.0) {
    configured = true;
  }
}

void establish_connection() {

  // this function makes sure that there's a link between
  // the remote and the car before we can continue

  while (!connection_established) {

    if ( radio.write(init_message, strlen(init_message)) ) {

      while (radio.available() ) {                    // If an ack with payload was received
        radio.read( &message, sizeof(message));
        Serial.print(F("Got response "));
        Serial.println(message);
        rounded_message = (int) message;
        if (rounded_message == 1) {
          connection_established = true;
          Serial.println("true");

        }
      }
    }
  }

}


//-----------------------CAR FUNCTIONS----------------------------//



void update_encoder()
{


  encoder0Pos_last = encoder0Pos; //used for projectile calcs
  n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos++;

    } else {
      encoder0Pos--;
    }
    //Serial.println(encoder0Pos);
  }
  now = millis();
  encoder0PinALast = n;
  input_pos = encoder0Pos;

  //------------PROJECTILE CALCS---------------//
  if (encoder0Pos_last == encoder0Pos) {

  }
  else
  {

    encoder_speed = (encoder0Pos_last - encoder0Pos) / ((previous_time - now) / 1000);
    input_vel = encoder_speed;
    previous_time = millis();

  }


}

//----------------------------------------------------------------//


void trigger_control(int trigger_input) {
  digitalWrite(triggerPinF, HIGH);
  digitalWrite(triggerPinR, LOW);
  analogWrite(triggerPin, trigger_input);
  //Serial.println("triggering");
}


void launch_ball() {
  //Serial.println("LAUNCHING");
  // trigger launch for 0.25 seconds
  if (counter == 1)
  {
    run_time = millis();
    trigger_time = millis();
    trigger_control(255);
    counter = counter + 1;
    //Serial.println("1");

  }
  else if ((trigger_time - run_time) <= (0.25 * 1000))
  {
    trigger_control(255);
    trigger_time = millis();
    //encoder_launch = encoder0Pos;
    //Serial.println("2");

  }
  else {
    trigger_control(0);
    // Serial.println("3");

  }

}



void velocity_control(double desired_velocity)
{

  encoder_speed_desired = desired_velocity;
  //------------PID CALCS---------------//

  velocity_PID.Compute();

  /*Serial.print("encoder_speed: ");
    Serial.print(encoder_speed);
    Serial.print("\t");
    Serial.print("e_s_desired: ");
    Serial.print(encoder_speed_desired);
    Serial.print("\t");*/

  //Serial.println(output_vel);

  //Serial.print("velocity: ");
  //Serial.println(encoder0Pos);
  //Serial.print("\t");

  motor_command = abs(output_vel);
  analogWrite(motorPin, motor_command); //0 to 255 for pwm
  if (output_vel >= 0) {
    digitalWrite(motorPinF, HIGH);
    digitalWrite(motorPinR, LOW);
  }
  else if (output_vel < 0) {
    digitalWrite(motorPinF, LOW);
    digitalWrite(motorPinR, HIGH);
  }
  //digitalWrite(triggerPinF, HIGH);
  //digitalWrite(triggerPinR, LOW);
}




void position_control(double des_position) {
  encoder_count_desired = des_position; //subtract 83 for offset
  position_PID.Compute();
  //Serial.print("position: ");
  //Serial.println(encoder0Pos);
  /*Serial.print("\t");
    Serial.print(encoder_count_desired);
    Serial.print("\t");
    Serial.println(output_pos);*/
  //-215<->215
 
  motor_command = abs(output_pos)+40;
  
  Serial.println(motor_command);
  analogWrite(motorPin, motor_command); //0 to 255 for pwm

  position_PID.Compute();

  if (output_pos > 0) {
    digitalWrite(motorPinF, HIGH);
    digitalWrite(motorPinR, LOW);
    motor_command_data=motor_command;
    
  }
  else if (output_pos < 0) {
    digitalWrite(motorPinF, LOW);
    digitalWrite(motorPinR, HIGH);
    motor_command_data=-motor_command;
  }
}







