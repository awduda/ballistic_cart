#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#define DATA_LIMIT  1010
/***** transmitter analog code ******/

RF24 radio(7, 8);
int reading;
int final_flag_received=0;
int message_received=0;  
char message[100];
char old_message[100];

const byte rxAddr[6] = "00001";
byte addresses[][6] = {"1Node","2Node"};              // Radio pipe addresses for the 2 nodes to communicate.
boolean response_read=false; 
boolean response_complete=false; 
boolean messages_same=false;
float response;
int received=42;
int count=3;
bool ready_to_receive_data=false;
bool data_mode=false;
char send_data_message[100]="Send data? (0/1): ";
float data_matrix[1010][2];
float transmit_array[1][2];
String message_string;
float testfloat;
int testvar;
bool reading_data=false;
byte pipeNo;
float test_response=1.0;
bool incoming=false;
int i=0;
bool data_received=false;
void setup()
{
  Serial.begin(9600);
  Serial.println("on");
  SPI.begin();
  radio.begin();
  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();  
  radio.startListening();
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
}

void loop()
{
  while (!incoming){
  
  read_message_send_ack();  }
  i=0;    
 
  while (data_mode && incoming){
    receive_data();
  }
   


}

  
  


//------------------------------------------//

void read_message_send_ack(){
    //Serial.println(response_complete);
    response_read=false; 
   
    if(radio.available(&pipeNo)){

          response_complete=false;

          
        
          
          radio.read(&message, sizeof(message) );
          
          Serial.println(message);
          if (strlen(message)==1){
            incoming=true;
          }
          String message_string(message);
          if (message_string.substring(0,4)=="Send"){
            data_mode=true;
            incoming=false;
          }
         
       
          
         
          if (response_complete==false){
    
    
    
          if (Serial.available()){
            response=Serial.parseFloat();
            
            radio.writeAckPayload(pipeNo,&response, sizeof(float) );
            response_complete=true;
          
            
            
          }
            
          }
          
          

      }
      
}
void receive_data(){

    //Serial.println("INCOMING");
    if (radio.available()){
      if (i<1010){
      radio.read(&transmit_array,sizeof(transmit_array));
      data_matrix[i][0]=transmit_array[0][0];
      data_matrix[i][1]=transmit_array[0][1];

      //Serial.println(data_matrix[i][1]);
      i++;
      }
    
    else{
      for (int n=0; n<=1010; n++){
        Serial.print(data_matrix[n][0]);
        Serial.print("\t");
        Serial.println(data_matrix[n][1]);
      }
      delay(5000);
      data_received=true;
      incoming=false;
      data_mode=false;
    
    }
    }
   
    
  }
            
  



   









