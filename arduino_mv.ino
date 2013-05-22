
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
//#include <SD.h>

#define DEBUG

SoftwareSerial debugPort(2,3);// used for debugging
//nomal Serial port used for gsm module for fully interrupt driven prog



//File myFile;

int dataPin=6;
int clockPin=7;

//Definition of Row pins 
int R0=14;
int R1=15;
int R2=16;
int R3=17;
int R4=18;
int R5=19;
int R6=8;


#define SCROLL_DELAY 2     /* scroll delay */


void fill_rx_bufer(void);

unsigned char matrix[91][7] PROGMEM=
{
  {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF  },  // Space
  {0xFB,0xFB,0xFB,0xFB,0xFB,0xFF,0xFB  },  // ;!
  {0xF5,0xF5,0xF5,0xFF,0xFF,0xFF,0xFF  },  // "
  {0xF5,0xF5,0xE0,0xF5,0xE0,0xF5,0xF5  },  // #
  {0xFB,0xF0,0xEB,0xF1,0xFA,0xE1,0xFB  },  // $
  {0xE3,0xEA,0xE5,0xFB,0xF4,0xEA,0xF8  },  // %
  {0xF7,0xEB,0xEB,0xF7,0xEA,0xED,0xF2  },  // &
  {0xF9,0xF9,0xFD,0xFB,0xFF,0xFF,0xFF  },  // '
  {0xFD,0xFB,0xF7,0xF7,0xF7,0xFB,0xFD  },  /*/ (  */
  {0xF7,0xFB,0xFD,0xFD,0xFD,0xFB,0xF7  },  // )
  {0xFB,0xEA,0xF1,0xFB,0xF1,0xEA,0xFB  },  // *
  {0xFF,0xFB,0xFB,0xE0,0xFB,0xFB,0xFF  },  // +
  {0xFF,0xFF,0xFF,0xF3,0xF3,0xFB,0xF7  },  // ,
  {0xFF,0xFF,0xFF,0xF1,0xFF,0xFF,0xFF  },  // -
  {0xFF,0xFF,0xFF,0xFF,0xFF,0xF3,0xF3  },  // .
  {0xFF,0xFE,0xFD,0xFB,0xF7,0xEF,0xFF  },  // /
  {0xF1,0xEE,0xEC,0xEA,0xE6,0xEE,0xF1  },  // 0
  {0xFB,0xF3,0xFB,0xFB,0xFB,0xFB,0xF1  },  // 1
  {0xF1,0xEE,0xFE,0xF1,0xEF,0xEF,0xE0  } ,  // 2
  {0xF1,0xEE,0xFE,0xF9,0xFE,0xEE,0xF1  },  // 3
  {0xFD,0xF9,0xF5,0xED,0xE0,0xFD,0xFD  },  // 4
  {0xE0,0xEF,0xE1,0xFE,0xFE,0xFE,0xE1  },  // 5
  {0xF9,0xF7,0xEF,0xE1,0xEE,0xEE,0xF1  },  // 6
  {0xE0,0xFE,0xFD,0xFB,0xF7,0xF7,0xF7  },  // 7
  {0xF1,0xEE,0xEE,0xF1,0xEE,0xEE,0xF1  },  // 8
  {0xF1,0xEE,0xEE,0xF0,0xFE,0xFD,0xF3  },  // 9
  {0xFF,0xF3,0xF3,0xFF,0xF3,0xF3,0xFF  },  // :
  {0xF3,0xFB,0xF3,0xF3,0xFF,0xF3,0xF3  },  // ;
  {0xFD,0xFB,0xF7,0xEF,0xF7,0xFB,0xFD  },  // <
  {0xFF,0xFF,0xF1,0xFF,0xF1,0xFF,0xFF  },  // ;=
  {0xF7,0xFB,0xFD,0xFE,0xFD,0xFB,0xF7  },  // >
  {0xF1,0xEE,0xFE,0xFD,0xFB,0xFF,0xFB  },  // ?
  {0xF1,0xEE,0xFE,0xF2,0xEA,0xEA,0xF1  },  // @
  {0xFB,0xF5,0xEE,0xEE,0xE0,0xEE,0xEE  },  // A
  {0xE1,0xF6,0xF6,0xF1,0xF6,0xF6,0xE1  },  // B
  {0xF1,0xEE,0xEF,0xEF,0xEF,0xEE,0xF1  },  // C
  {0xE1,0xF6,0xF6,0xF6,0xF6,0xF6,0xE1  },  // D
  {0xE0,0xEF,0xEF,0xE3,0xEF,0xEF,0xE0  },  // E
  {0xE0,0xEF,0xEF,0xE3,0xEF,0xEF,0xEF  },  // F
  {0xF1,0xEE,0xEF,0xE8,0xEE,0xEE,0xF1  },  // G
  {0xEE,0xEE,0xEE,0xE0,0xEE,0xEE,0xEE  },  // H
  {0xF1,0xFB,0xFB,0xFB,0xFB,0xFB,0xF1  },  // ;I
  {0xF8,0xFD,0xFD,0xFD,0xFD,0xFD,0xF3  },  // J
  {0xEE,0xED,0xEB,0xE7,0xEB,0xED,0xEE  },  // K
  {0xEF,0xEF,0xEF,0xEF,0xEF,0xEF,0xE0  },  // L
  {0xEE,0xE4,0xEA,0xEA,0xEE,0xEE,0xEE  },  // M
  {0xEE,0xE6,0xEA,0xEC,0xEE,0xEE,0xEE  },  // N
  {0xF1,0xEE,0xEE,0xEE,0xEE,0xEE,0xF1  },  // O
  {0xE1,0xEE,0xEE,0xE1,0xEF,0xEF,0xEF  },  // P
  {0xF1,0xEE,0xEE,0xEE,0xEA,0xED,0xF2  },  // Q
  {0xE1,0xEE,0xEE,0xE1,0xEB,0xED,0xEE  },  // R
  {0xF1,0xEE,0xEF,0xF1,0xFE,0xEE,0xF1  },  // S
  {0xE0,0xFB,0xFB,0xFB,0xFB,0xFB,0xFB  },  // T
  {0xEE,0xEE,0xEE,0xEE,0xEE,0xEE,0xF1  },  // U
  {0xEE,0xEE,0xEE,0xF5,0xF5,0xFB,0xFB  },  // V
  {0xEE,0xEE,0xEE,0xEA,0xEA,0xE4,0xEE  },  // W
  {0xEE,0xEE,0xF5,0xFB,0xF5,0xEE,0xEE  },  // X
  {0xEE,0xEE,0xF5,0xFB,0xFB,0xFB,0xFB  },  // Y
  {0xE0,0xFE,0xFD,0xFB,0xF7,0xEF,0xE0  },  // Z
  {0xF1,0xF7,0xF7,0xF7,0xF7,0xF7,0xF1  },  /* [ */
  {0xFF,0xEF,0xF7,0xFB,0xFD,0xFE,0xFF  },  /* \ */
  {0xF1,0xFD,0xFD,0xFD,0xFD,0xFD,0xF1  },  /* [ */
  {0xFB,0xF5,0xEE,0xFF,0xFF,0xFF,0xFF  },  // ^
  {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xE0  },  // _
  {0xF3,0xF3,0xF7,0xFB,0xFF,0xFF,0xFF  },  // '
  {0xFF,0xFF,0xF1,0xFE,0xF0,0xEE,0xF1  },  // a
  {0xEF,0xEF,0xE9,0xE6,0xEE,0xE6,0xE9  },  // b
  {0xFF,0xFF,0xF8,0xF7,0xF7,0xF7,0xF8  },  // c
  {0xFE,0xFE,0xF2,0xEC,0xEE,0xEC,0xF2  },  // d
  {0xFF,0xFF,0xF1,0xEE,0xE0,0xEF,0xF1  },  // e
  {0xF9,0xF6,0xF7,0xE1,0xF7,0xF7,0xF7  },  // f
  {0xFF,0xFF,0xF0,0xEE,0xF0,0xFE,0xF1  },  // g
  {0xEF,0xEF,0xE9,0xE6,0xEE,0xEE,0xEE  },  // h
  {0xFB,0xFF,0xF3,0xFB,0xFB,0xFB,0xF1  },  // i
  {0xFD,0xFF,0xF9,0xFD,0xFD,0xFD,0xF3  },  // j
  {0xF7,0xF7,0xF6,0xF5,0xF3,0xF5,0xF6  },  // k
  {0xF3,0xFB,0xFB,0xFB,0xFB,0xFB,0xF1  },  // l
  {0xFF,0xFF,0xE5,0xEA,0xEA,0xEA,0xEA  },  // m
  {0xFF,0xFF,0xE9,0xE6,0xEE,0xEE,0xEE  },  // n
  {0xFF,0xFF,0xF1,0xEE,0xEE,0xEE,0xF1  },  // o
  {0xFF,0xFF,0xE1,0xEE,0xE1,0xEF,0xEF  },  // p
  {0xFF,0xFF,0xF0,0xEE,0xF0,0xFE,0xFE  },  // q
  {0xFF,0xFF,0xE9,0xE6,0xEF,0xEF,0xEF  },  // r
  {0xFF,0xFF,0xF0,0xEF,0xF1,0xFE,0xE1  },  // s
  {0xFB,0xFB,0xF0,0xFB,0xFB,0xFB,0xFC  },  // t
  {0xFF,0xFF,0xEE,0xEE,0xEE,0xEC,0xF2  },  // u
  {0xFF,0xFF,0xEE,0xEE,0xEE,0xF5,0xFB  },  // v
  {0xFF,0xFF,0xEE,0xEE,0xEA,0xEA,0xF4  },  // w
  {0xFF,0xFF,0xEE,0xF5,0xFB,0xF5,0xEE  },  // x
  {0xFF,0xFF,0xEE,0xF5,0xFB,0xFB,0xF3  },  // y
  {0xFF,0xFF,0xE0,0xFD,0xFB,0xF7,0xE0  }   // z
}; 

char *integerToString(int );
bool Get_AT_Response(const char *);
void putEchoOff(void);
void setSMSmode(void);  // Set to sms mode 
void clear_buffer(void);
void setCNMImode(void);
void displayMessage(unsigned char msg[]);
void displayMessage2(unsigned char msg[]);
bool has(char *haystack, char *needle);



void initTimer1(void);
void printArray(unsigned char *);

void modu1(void);
void modu2(void);
void modu3(void);
void modu4(void);
void modu5(void);

char display_str[200];// string to display
char rxc_buffer[200];     //primary buffer to which uart data is copied
int rxc_buffer_index;

 	
unsigned char  PROGMEM MSG1[] ="WELCOME TO THE UNIVERSITY OF NAIROBI SCIENCE AND TECHNOLOGY PARK AND THE FABLAB"; 
unsigned char     amAwesome[] ="MUCHIRI WA MBARI DESIGNS: FOR DIRECT ORDERS CALL 0724 540 103";
unsigned char          fox [] ="The quick brown fox jumps over the lazy dog";
unsigned char   msg [160]; 
unsigned char   msg1[160];
unsigned char   msg2[160];
unsigned char   msg3[50];
unsigned char   dig[30];
unsigned char   CH_LIM,OFF_CH;     
unsigned char   tmp;
unsigned int     t, count;



unsigned char modu,divi,blank1_limit,blank2_limit,ch_limit;
unsigned char i,j,k,old_divi,diff; 
int offset_col,t1;
int row,offset_ch;

// flag to enable diplay one message at a time;
int displayNxt;

//Serial event variables
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup()
{
  //DATA AND CLOCK pins
  pinMode(dataPin,OUTPUT);
  pinMode(clockPin,OUTPUT);
  //ROW VARIABLES
  pinMode(R0,OUTPUT);
  pinMode(R1,OUTPUT);
  pinMode(R2,OUTPUT);
  pinMode(R3,OUTPUT);
  pinMode(R4,OUTPUT);
  pinMode(R5,OUTPUT);
  pinMode(R6,OUTPUT);
  
  pinMode(10,OUTPUT);
  
  digitalWrite(R0,LOW);
  digitalWrite(R1,LOW);
  digitalWrite(R2,LOW);
  digitalWrite(R3,LOW);
  digitalWrite(R4,LOW);
  digitalWrite(R5,LOW);
  digitalWrite(R6,LOW);
  
  
  old_divi=0;
  modu=0;
  offset_col=0;
  blank1_limit=17;
  ch_limit=0;
  blank2_limit=0;
  offset_ch=100;
  CH_LIM=0;
  OFF_CH=0;
  k=0;
  
  Serial.begin(9600);// USART_Init();
  debugPort.begin(9600); 
  inputString.reserve(200);
  debugPort.println("DEBUG ON");
  debugPort.println("LED SIGN BY PETER MBARI!");
  
  
//  Serial.print("Initializing SD card...");
//    if (!SD.begin(4)) {
//    Serial.println("initialization failed!");
//    return;
//  }
//  Serial.println("initialization done.");
//  
//  myFile = SD.open("test.txt", FILE_WRITE);
//  
//  // if the file opened okay, write to it:
//  if (myFile) {
//    Serial.print("Writing to test.txt...");
//    myFile.println("testing 1, 2, 3.");
//	// close the file:
//    myFile.close();
//    Serial.println("done.");
//  }
//  else {
//    // if the file didn't open, print an error:
//    Serial.println("error opening test.txt");
//  }
  
  

  // end of initiliasation


  // putEchoOff();
  //setCNMImode();
  //setSMSmode();


  for(t1=(sizeof(MSG1));t1>0;t1--)msg[t1]=pgm_read_byte(&(MSG1[sizeof(MSG1)-t1]));
  for(t1=sizeof(amAwesome);t1>0;t1--)msg1[t1]=(amAwesome[sizeof(amAwesome)-t1]);
  for(t1=sizeof(fox);t1>0;t1--)msg2[t1]=(fox[sizeof(fox)-t1]);

  //displayMessage(msg);


}//----------------end setup

void loop()
{
  displayMessage(msg);
  displayMessage(msg1);
  displayMessage(msg2);
  // displayMessage(msg3);

}//end loop
void displayMessage(unsigned char msg[]){

  displayNxt=0;
  while(displayNxt==0){
    for(k=0;k<SCROLL_DELAY;k++){

      for(row=6;row>-1;row--){
        i=16;
        for(j=0;j<blank1_limit;j++){
          dig[i]=0xff;
          --i;
        }
        t=0;
        for(j=0;j<ch_limit;j++){
          dig[i]=pgm_read_byte(&(matrix[msg[(t+offset_ch)]-0x20][row]));
          --t;
          --i;
        }
        for(j=0;j<blank2_limit;j++){
          dig[i]=0xff;
          --i;
        }
        if     (modu==1) modu1();
        else if(modu==2) modu2();
        else if(modu==3) modu3();
        else if(modu==4) modu4();
        else if(modu==5) modu5();

        for(i=1;i<17;i++){
          t=dig[i];
          for (int x=0;x<6;x++){
            if(x==0){
              if(t & 0x01) digitalWrite(dataPin,LOW); 	//SET DATA TO ZERO   
              else digitalWrite(dataPin,HIGH);     			//SET DATA TO ONE 
              digitalWrite(clockPin,HIGH);         			// SET THE CLOCK
              digitalWrite(clockPin,LOW); 				// CLEAR THE CLOCK
              digitalWrite(clockPin,HIGH); 
            }
            else {
              t = t >>1;
              if(t & 0x01) digitalWrite(dataPin,LOW); 	//SET DATA TO ZERO   
              else digitalWrite(dataPin,HIGH);     			//SET DATA TO ONE 
              digitalWrite(clockPin,HIGH);         			// SET THE CLOCK
              digitalWrite(clockPin,LOW); 				// CLEAR THE CLOCK
              digitalWrite(clockPin,HIGH);				
            }
          }				 
        }
        switch(row){
        case 0:
              digitalWrite(R0,HIGH);
              delay(1);
              digitalWrite(R0,LOW);
              break;
        case 1:
             digitalWrite(R1,HIGH);
             delay(1);
             digitalWrite(R1,LOW);
             break;
             
        case 2:
              digitalWrite(R2,HIGH);
              delay(1);
               digitalWrite(R2,LOW);
              break;
         case 3:
              digitalWrite(R3,HIGH);
              delay(1);
             digitalWrite(R3,LOW);
              break;
         case 4:
              digitalWrite(R4,HIGH);
              delay(1);
            digitalWrite(R4,LOW);
              break;
          case 5:
             digitalWrite(R5,HIGH);
              delay(1);
          digitalWrite(R5,LOW);
              break;
          case 6:
              digitalWrite(R6,HIGH);
              delay(1);
              digitalWrite(R6,LOW);
              break;
         default:
             break;
        }
      }
    }//delay
    if(++offset_col>700){	   
      offset_col=0;
      blank1_limit=17;
      ch_limit=0;
      blank2_limit=0;
      offset_ch=100;
      CH_LIM=0x00;
      OFF_CH=0x00;
      displayNxt=1;

    }
    modu=offset_col%6;
    divi=offset_col/6;
    if(divi!=old_divi)diff=1;
    else diff=0;
    if(blank1_limit!=0)blank1_limit=blank1_limit-diff;
    if(CH_LIM!=0x00){
      if(OFF_CH!=0x00){
        blank2_limit=blank2_limit+diff;
        ch_limit=ch_limit-diff;
        if(ch_limit==255)ch_limit=0;
      }
      offset_ch = offset_ch-diff;
      if(offset_ch>101)offset_ch=0;
    }
    else{
      ch_limit=ch_limit+diff;
    }

    if(offset_ch==16) OFF_CH=0xff;
    if(ch_limit==17)  CH_LIM=0xff;
    old_divi = divi;
  }

}




void modu1(){
  int count;
  for (count=0; count<=18; count++){
    if(count ==0){
      tmp=dig[count];
      dig[count]<<=1;
    }
    else if(count%2!=0){
      t=dig[count];
      dig[count]<<=1;
      dig[count]=(dig[count])|((tmp>>5) & 0x01);
    }
    else if(count%2==0){
      tmp=dig[count];
      dig[count]<<=1;
      dig[count]=(dig[count])|((t>>5)  & 0x01); 
    }

  }


}
void modu2(){
  int count;
  for (count=0; count<=18; count++){
    if(count ==0){
      tmp=dig[count];
      dig[count]<<=2;
    }
    else if(count%2!=0){
      t=dig[count];
      dig[count]<<=2;
      dig[count]=(dig[count])|((tmp>>4) & 0x03);
    }
    else if(count%2==0){
      tmp=dig[count];
      dig[count]<<=2;
      dig[count]=(dig[count])|((t>>4)& 0x03); 
    }

  }

}
void modu3(){

  int count;
  for (count=0; count<=18; count++){
    if(count == 0){
      tmp=dig[count];
      dig[count]<<=3;
    }
    else if(count%2!=0){
      t=dig[count];
      dig[count]<<=3;
      dig[count]=(dig[count])|((tmp>>3) & 0x07);
    }
    else if(count%2==0){
      tmp=dig[count];
      dig[count]<<=3;
      dig[count]=(dig[count])|((t>>3)& 0x07); 
    }

  }

}
void modu4(){
  int count;
  for (count=0; count<=18; count++){
    if(count ==0){
      tmp=dig[count];
      dig[count]<<=4;
    }
    else if(count%2!=0){
      t=dig[count];
      dig[count]<<=4;
      dig[count]=(dig[count])|((tmp>>2) & 0x0f);
    }
    else if(count%2==0){
      tmp=dig[count];
      dig[count]<<=4;
      dig[count]=(dig[count])|((t>>2)& 0x0f); 
    }

  }

}
void modu5(){
  int count;
  for (count=0; count<=18; count++){
    if(count ==0){
      tmp=dig[count];
      dig[count]<<=5;
    }
    else if(count%2!=0){
      t=dig[count];
      dig[count]<<=5;
      dig[count]=(dig[count])|((tmp>>1) & 0x1f);
    }
    else if(count%2==0){
      tmp=dig[count];
      dig[count]<<=5;
      dig[count]=(dig[count])|((t>>1)  & 0x1f); 
    }

  }
}

bool Get_AT_Response(const char * response)
{
  //_delay_ms(2000);
  if (strstr_P(rxc_buffer, response)) //If string is found, would return pointer value so becomes true
      return 1;
  else
    return 0;
}


void putEchoOff(void)  // Sends echo command off
{
  while (1)
  {
    Serial.println("ATE0\r");
    while(!stringComplete);// to implement a timeout to avoid freezing here
    if (Get_AT_Response(PSTR("OK")));
  }

}

void setSMSmode(void)  // Set to sms mode 
{
  while (1)
  {
    Serial.println("AT+CMGF=1\r");
    while(!stringComplete);
    if (Get_AT_Response(PSTR("OK")))break;
  }
}

void setCNMImode(void)  // set sms to be received automatically OR FLASHED TO THE SERIAL PORT ON RECEIVE
{
  while (1)
  {
							// clear the buffer
    Serial.println("AT+CNMI=2,2,0,0,0"); // send command
    while(!stringComplete);						// wait for response
    if (Get_AT_Response(PSTR("OK")))		//check the response
      break;	
  }
}


bool has(char *haystack, char *needle)
{
  int compareOffset = 0;
  //iterate through the zero terminated string heystack
  while(*haystack) {
    if (*haystack == *needle) { //we might have found a match
      compareOffset = 0; //start at the current location in haystack
      //see if the string from that location matches the needle
      while (haystack[compareOffset] == needle[compareOffset]) {
        compareOffset++;
        if (needle[compareOffset]=='\0') {
          //clear_command_buffer();
          //clear_buffer();
          return true; //we have reached the end of needle and everything matched
        }
      }
    }
    haystack++; //increment the location in the string (the pointer)
  }
  //clear_command_buffer();
  //clear_buffer();
  return false; //no match was found
}
void serialEvent() {
  while (Serial.available()) {
    debugPort.println("rx");
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\r') {
      stringComplete = true;
      debugPort.println(inputString);
      inputString="";
      stringComplete = false;     
    } 
  }
}

