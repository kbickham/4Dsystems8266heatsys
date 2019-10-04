#include <WiFiManager.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
 #include <ThingSpeak.h>
#include "genieArduino.h"
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>
#include <HX711_ADC.h>		
			  

char ssid[16] = " ";
char password[16] = " ";

char taremsg[] = "Taring...";
char calibmsg[] = "Place 50g on scale";


#define D0 (16)
#define D1 (5) //scl
#define D2 (4) //sda/
#define D3 (0) //10k pullup
#define D4 (2) //10k pullup built in led                                                                                                                                                                                                                                                                                       
#define D5 (14) //RX
#define D6 (12) //TX
#define D7 (13)
#define D8 (15) //10k pull 



#define FAN2PIN  D0
//    LC CLK      D1
//     LC DATA    D2
#define RESETLINE D4
//  ESP TX TO UNO D5
//  ESP RX TO UNO D6
#define FANPIN   D7  // AIRFLOW MOSFET CONTROL
#define HEATPIN   D8 // HEAT MOSFET CONTROL

struct filter
{ bool filterbegin;
  float yn;
  float yn_1;
  unsigned long count;
};

 int minutecounter,halfmin;
  typedef struct
  {  unsigned long milcount;
  } timer, *ptime;

   typedef struct 
  { volatile bool statusnow;
    volatile bool enable;
  } oride, *poride;

  typedef struct 
  { volatile  bool statusnow;
    oride ORIDE;
  } transferstate, *ptransferstate;

  typedef struct
  { transferstate OUT;
  } heat, *pheat;

  typedef struct
  { transferstate OUT;
    transferstate IN; // HDC.AIR.oride //HDC.AIR.
  } air, *pair;

  typedef struct
  { int oldform,newform;
  }formx, *pformx;

  typedef struct  
  { formx FORM;
  } disp, *pdisp;

  typedef struct  
  {  bool state, mode1;
    bool runningnow,done;
    bool startF[3]= {false,false,false};
    float heatP,cycleT,maxT;
    int maxTadj; 
    float massI,massG,massT = 0.0; 
    unsigned long timerT,start_time,heatonT = 0.0;
    } dryp, *pdryp;

  typedef struct
  {float scale;  
   int heatavg;               
  } sensorsE, *psensorsE;

  typedef struct
  { timer STATE[5];
    int currentstate; 
  } board, *pboard;

  typedef struct
  { board MCU;
    sensorsE SENSORS;    
    air AIR;
    dryp DRYP;
    heat HEAT; 
    disp DISP;
  } SYSstate, *pSYSstate;

struct unodata
{
 float mlx[4];
 float dhtT[3];
 float dhtH[3];
 float bmp[2];

};

float filteredmass,filteredGmass;

float starttemp,starthumid=0.0;
bool channel;
int updatestate;
bool upd,readscale;
float percentM = 0;
int conpress=0;
unsigned long screentimer;     
unsigned long swuartT=0;
unsigned long controlsysT, loadcellT;
int manST=0;
int tempST=0;
int donecount; 
int manSTstate;
bool wifistatus;
int tempSTstate;
 int tapcount;
//keyboard vars
String msg1;
int tempk;
int tempk2;
bool flagk;
char keyvalue1[16]; // array 1 to hold keyboard values for string1
char keyvalue2[16]; // array 2 to hold keyboard values for string2
int counterk1 = 0;
int counterk2 = 0;
bool string_1 = 0;
float calF,tempCal;

float compensate;
float humidripple=10;
float heatripple=20;
float humidMAX=92; // rel humidity ..uh a number
float highesthumid;
float hottestgel; //obvious find maximum function
float tempH;
unsigned long startpE,startpI;
unsigned long pulseE=  5000;
unsigned long pulseI = 5000;
  bool pulseEF,pulseIF = false;
 bool  safeTmass;
unsigned long safeTmassT,safeTreading;


  
unodata UNODATA;
 filter HX,HX2; 
 HX711_ADC LC(D1,D2);					 
 WiFiClient client;

 WiFiManager wifiManager;
SoftEasyTransfer ET;
SoftwareSerial suart (D6,D5,false,68);
Genie genie;
  SYSstate HDC;
 
void setup()

{ 
  ESP.wdtDisable();
  ESP.wdtEnable(WDTO_8S);
  pinMode(HEATPIN,OUTPUT);
  pinMode(FANPIN,OUTPUT);
  pinMode(FAN2PIN,OUTPUT);
  pinMode(RESETLINE, OUTPUT); //display reset 
  
  LC.begin();
  LC.setCalFactor(210.98);
   LC.setSamplesInUse(20);
  LC.start(1500);
   
  Serial.begin(19200);
  suart.begin(19200);
  
  ET.begin(details(UNODATA),&suart);
  genie.Begin(Serial);
    genie.AttachEventHandler(myGenieEventHandler);
     digitalWrite(RESETLINE, 0);  // Reset the Display via D4
      delay(100);
      digitalWrite(RESETLINE, 1);  // unReset the Display via D4
      delay (4500);//let the display start up after the reset (This is important)
      genie.WriteContrast(15); 
      
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 15,heatripple);
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 16,humidripple);
 updatestate=0;
 HDC.MCU.STATE[3].milcount= millis();
 HDC.MCU.STATE[2].milcount=millis();
 controlsysT=millis();
 screentimer=millis();
 swuartT = millis();
};


void loop() 
{
  if (millis()-swuartT >=450)
       {  ET.receiveData();
          interfacestate();
          swuartT = millis();  
       };
  scalerun();
  genie.DoEvents();
      yield();   
      
if (millis() - controlsysT >= 330)
    {   updaterF();
    	if (HDC.DRYP.runningnow ==true)
          dryprog();    
        yield();
        fancontrol();
        heatcontrol();
       controlsysT=millis();
    };
     
   ESP.wdtFeed();
};


void interfacestate()
{
  switch(HDC.DISP.FORM.newform)
       {
             case 0:         genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1,UNODATA.dhtT[0]*10);  
                             genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4,UNODATA.dhtH[0]*10); 
                             genie.WriteObject (GENIE_OBJ_SCOPE, 1,UNODATA.dhtH[0]);
                              yield();
                             genie.WriteObject(GENIE_OBJ_LED_DIGITS, 2,UNODATA.dhtT[1]*10); 
                             genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5,UNODATA.dhtH[1]*10);
                             yield();
                             genie.WriteObject (GENIE_OBJ_SCOPE, 2,UNODATA.dhtH[1]);
                             genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0,UNODATA.dhtT[2]*10); 
                             genie.WriteObject(GENIE_OBJ_LED_DIGITS, 3,UNODATA.dhtH[2]*10); 
                              yield(); 
                             genie.WriteObject (GENIE_OBJ_SCOPE,3,UNODATA.dhtH[3]);                         
                        break;
            case 1:     break;
            case 2:          if (WiFi.status() == WL_CONNECTED)
                              { wifistatus=true;
                               genie.WriteObject(GENIE_OBJ_LED, 1,1);        
                              }else {wifistatus=false;
                              genie.WriteObject(GENIE_OBJ_LED, 1,0);
                                     genie.WriteStr(2,"Press for Portal"); };
             
                               
                           break;
            case 3:       genie.WriteObject(GENIE_OBJ_LED_DIGITS, 7,UNODATA.mlx[0]*10);
                          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 9,UNODATA.mlx[1]*10);
                          genie.WriteObject(GENIE_OBJ_LED_DIGITS,18,UNODATA.mlx[2]*10);
                          genie.WriteObject(GENIE_OBJ_LED_DIGITS,19,UNODATA.mlx[3]*10);
                          break;
            case 4:       if (HDC.DRYP.runningnow == false)
                                {  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 8,HDC.SENSORS.scale*100); 
                                   genie.WriteObject(GENIE_OBJ_LED_DIGITS, 13,0);  
                                } else { genie.WriteObject(GENIE_OBJ_LED_DIGITS, 8,filteredmass*100); 
                                        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 13,HDC.DRYP.massT);  };                              
                              yield();
                          break;
            case 5:         
                          if (HDC.DRYP.runningnow == true)
                             genie.WriteObject(GENIE_OBJ_LED_DIGITS, 6, HDC.DRYP.massG*100);                            
                          genie.WriteObject(GENIE_OBJ_LED, 11, wifistatus);                       
                           break;                 
       };
};


void updaterF()
{ ///////////////STATE machine for FOR THE UPDATES
  if ((millis() - HDC.MCU.STATE[3].milcount) >= 30000)
    {
	if (millis() - screentimer >= 90000)
           {genie.WriteObject(GENIE_OBJ_FORM, 1, 1);
             if (millis() - screentimer >= 120000)
                  genie.WriteContrast(0);
            };
      upd=true;
      updatestate ++;

  switch (updatestate%4)
                    {     

                        case 0:           if (upd == true) 
                                             { write2TSData(820266,"735ZEGBPE2AC5GPD",1,UNODATA.mlx[0],2,UNODATA.mlx[1],3,UNODATA.mlx[2],4,UNODATA.mlx[3],5,HDC.SENSORS.heatavg,6,UNODATA.bmp[0],7,(UNODATA.dhtH[1]/UNODATA.dhtH[2]),8,HDC.DRYP.massG);
                                               HDC.MCU.STATE[3].milcount=millis();
                                             //delay(5);
                                               };
                                             yield();
                                              break;
                       case 1:              if (upd == true) 
                                             { write2TSData(820261,"ECJBK2DNKO25LF84",1,UNODATA.dhtT[2],2,UNODATA.dhtH[2],3,UNODATA.dhtT[1],4,UNODATA.dhtH[1],5,UNODATA.dhtT[0],6,UNODATA.dhtH[0],7,HDC.SENSORS.heatavg,8,HDC.DRYP.massG);
                                               HDC.MCU.STATE[3].milcount=millis();
                                              //elay(5);
                                               };
                                              yield();
                                              break;
                       case 2:              if (upd == true) 
                                             { write2TSData(820272,"PN9JS7U2LY1RCZXX",1,HDC.HEAT.OUT.statusnow,2,HDC.AIR.IN.statusnow,3,HDC.AIR.OUT.statusnow,4,HDC.SENSORS.heatavg,5,UNODATA.bmp[1],6,(UNODATA.dhtH[1]/UNODATA.dhtH[2]),7,(UNODATA.dhtH[1]/UNODATA.dhtH[0]),8,filteredmass);
                                             //delay(5);  
                                               };
                                              yield();
                                              break;            
                       case 3:              if (upd == true) 
                                             { write2TSData(820262,"YI5O0K4Y3JA2BXS9",1,HDC.SENSORS.heatavg,2,UNODATA.dhtT[1],3,starthumid-UNODATA.dhtH[0],4,starttemp-UNODATA.dhtT[0],5,UNODATA.dhtH[1]/UNODATA.dhtH[0],6,compensate,7,filteredGmass,8,filteredmass);
                                             //delay(5);  
                                               };
                                              yield();
                                              break;                                    
                    };              
    };
};


   void myGenieEventHandler(void)
{ 
    if (millis() - screentimer >= 120000)
        genie.WriteContrast(15);

    screentimer= millis(); 
    genieFrame Event;
    genie.DequeueEvent(&Event);
 
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT or Event.reportObject.cmd == GENIE_REPORT_OBJ) 
     {  yield();
     
       if (Event.reportObject.object == GENIE_OBJ_FORM)                                            ///// FORM NUMBER RELATED EVENTS          
          { HDC.DISP.FORM.oldform = HDC.DISP.FORM.newform; 
            HDC.DISP.FORM.newform = Event.reportObject.index;
            HDC.MCU.currentstate=HDC.DISP.FORM.newform;

                   switch (Event.reportObject.index)
                  {              
                      case 0:    break;            
                      case 1:    break;
                      case 2:    if ( HDC.DISP.FORM.oldform == 5 and HDC.DRYP.runningnow == false)      
                                   killdryprog();      
                                  counterk1=0;
                                  counterk2=0;
                                  for(int p=0; p<15;p++)
                                   { keyvalue1[p]=0;
                                     keyvalue2[p]=0;}

                                
                                 break;
                      case 3:    break;          
                      case 4:    break;
                      case 5:    break;
                      case 6:    break;
                      case 7:    break;
                                      
                  };
          };
          
  if (Event.reportObject.object == 0x21)                                                           //userbutton                                             
    {    
      switch (Event.reportObject.index)  
         {
         case 2:   
                     tapcount++;
                     if (tapcount >= 2)
                         { wifiManager.startConfigPortal("HDCconnect"); 
                               genie.WriteObject(GENIE_OBJ_LED,1,wifistatus);        
                          genie.WriteObject(GENIE_OBJ_LED,11,wifistatus);    
                         tapcount=0;
                         };

                                       break;                
          case 4:     break;
          case 5:     break;
          };  
     };

     if (Event.reportObject.object == 30)                                                          //4D button events
       {   
            switch (Event.reportObject.index)     
         {     
            case 2:      HDC.AIR.OUT.ORIDE.enable = genie.GetEventData(&Event); 
                         genie.WriteObject(GENIE_OBJ_USER_LED, 8, HDC.AIR.OUT.ORIDE.enable);
                         break;          
                                       
            case 3:      HDC.HEAT.OUT.ORIDE.enable = genie.GetEventData(&Event); 
                         genie.WriteObject(GENIE_OBJ_USER_LED, 7, HDC.HEAT.OUT.ORIDE.enable);
                         break;                       
                         
           case 5:       if (HDC.DRYP.runningnow != true )      
                          {  LC.tare();
                             genie.WriteStr(3,taremsg);
                                                         }else{  genie.WriteStr(6,"Scale inoperable .Drying Program Running."); };
                         break;

           case 6:       HDC.AIR.IN.ORIDE.enable = genie.GetEventData(&Event); 
                         genie.WriteObject(GENIE_OBJ_USER_LED, 5, HDC.AIR.IN.ORIDE.enable);
                         break;
                         
           case 7:       if (genie.GetEventData(&Event) == true && HDC.DRYP.startF[0] == true && HDC.DRYP.startF[1] == true && HDC.DRYP.startF[2]== true)
                             { HDC.DRYP.runningnow = true;
                                genie.WriteObject(GENIE_OBJ_USER_LED, 0, true); 
                                genie.WriteObject(GENIE_OBJ_USER_LED, 10, true); 
                                genie.WriteObject(GENIE_OBJ_LED, 0, true); 
                                starttemp=UNODATA.dhtT[0];
                                starthumid=UNODATA.dhtH[0];
                                HDC.DRYP.start_time = millis();
                                HDC.DRYP.timerT=millis();
                             } else if(genie.GetEventData(&Event) == false)     
                                        killdryprog();
                      break;                      
           case 9:   if (HDC.DRYP.runningnow == false)
                      {  HDC.DRYP.startF[0]=true;     
                         LC.tare();           
                         genie.WriteObject(GENIE_OBJ_USER_LED, 3, true);
                         HX.filterbegin=true;
                         HDC.DRYP.start_time = millis();  //this is intentional, highjacking start_time so that on activation of user button 11 (below)
                       };                                    // i can cycle throguh 450 ms of load cell data, as the initial value needs more tha none reading to be accurate.                     
                     break;     
          
          case 11:  if (HDC.DRYP.runningnow == false && HDC.DRYP.startF[0]== true)    //Start dry program
                      {  HDC.DRYP.massI = filteredmass; 
                        HDC.DRYP.massG = HDC.DRYP.massI ;
                        filteredGmass=filtermassG(HDC.DRYP.massI,.85);
                        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 14, HDC.DRYP.massI*100);//Build dryer stats
                        HDC.DRYP.massT=(HDC.DRYP.massI*0.05); 
                        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 11, HDC.DRYP.massT*100);
                        yield();
                        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 6, HDC.DRYP.massG*100);  
                        HDC.DRYP.startF[1]=true;
                        genie.WriteObject(GENIE_OBJ_USER_LED,4,1);
                       };
                      break;
                 
           case 12: if (HDC.DRYP.runningnow == false && HDC.DRYP.startF[0]== true && HDC.DRYP.startF[1]== true) 
                         { genie.WriteObject(GENIE_OBJ_USER_LED, 6, 1);
                           if (HDC.DRYP.maxT != 0)
                              HDC.DRYP.startF[2]=true;                                      
                         };
                       break;
                       
            case 17: HDC.DRYP.mode1 = genie.GetEventData(&Event);
                       
				          	 break;
           case 21:   HDC.AIR.IN.ORIDE.statusnow = genie.GetEventData(&Event);
                     break;
           case 22:    HDC.AIR.OUT.ORIDE.statusnow = genie.GetEventData(&Event);
                     break;
           case 23:    HDC.HEAT.OUT.ORIDE.statusnow = genie.GetEventData(&Event);
                     break;         

           case 24:       for(int i=0; i<15;i++)
                        ssid[i] = keyvalue1[i];
                          break; 
                          
           case 25:       for(int i2=0; i2<15;i2++)
                              password[i2] = keyvalue2[i2];
                        //   genie.WriteStr(9,"pw stored");                          
                         // if(WiFi.status() != WL_CONNECTED)
                             WiFi.disconnect(true);
                             WiFi.begin(String(keyvalue1),String(keyvalue2));
                            break;                                     
           };   
       };
     if (Event.reportObject.object == 5)                                                       /////TRACKBAR EVENTS
      {   
         switch (Event.reportObject.index)                          
        {   case 0:  HDC.DRYP.maxT = genie.GetEventData(&Event);
                     HDC.DRYP.maxT = HDC.DRYP.maxT  + 27;
                     genie.WriteObject(GENIE_OBJ_LED_DIGITS, 10, HDC.DRYP.maxT);
                     break;
                     
            case 1 :  heatripple = genie.GetEventData(&Event);
                        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 15, heatripple); 
                        //was heatP
                      // HDC.DRYP.heatP= HDC.DRYP.heatP*25;
                      //  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 15, HDC.DRYP.heatP); 
                          HDC.DRYP.heatP=HDC.DRYP.heatP/100;
                                    
                      break;
           case 2 : humidripple = genie.GetEventData(&Event);
                    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 16,humidripple);
                      //was cycleT
                  
                  //  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 16, HDC.DRYP.cycleT);
                  //  HDC.DRYP.cycleT=HDC.DRYP.cycleT*60;
                    break;
        }
      }

     if (Event.reportObject.object == GENIE_OBJ_KEYBOARD)
       {
        
              if (Event.reportObject.index == 0)                              // If keyboard0
             {   if (counterk1>=15)
                    counterk1=0;
                 if (counterk2>=15) 
                    counterk2=0;
                
                tempk = genie.GetEventData(&Event);
                if( tempk != 8 && tempk != 13)   
                 {       if (string_1 == 0)
                          { keyvalue1[counterk1] = tempk;        // Receive the event data from the keyboard
                            genie.WriteStr(8,keyvalue1);                         //prints to String Object 0      
                            counterk1++; }         // = counterk1 + 1; //increment array 
              
                        if (string_1 == 1)
                          { keyvalue2[counterk2] = tempk;        // Receive the event data from the keyboard
                            genie.WriteStr(9 ,keyvalue2);                         //prints to String Object 1     
                            counterk2++; }
          
                  }else if(tempk == 8){                             //check if backspace
                                             if (string_1==0)
                                              {counterk1--;
                                                keyvalue1[counterk1] = 0;
                                                genie.WriteStr(8,keyvalue1); }
                                
                                            if (string_1==1)
                                              { counterk2--;
                                                keyvalue2[counterk2] = 0;
                                                genie.WriteStr(9,keyvalue2);}
                     }else if(tempk == 13){     // change string
                                            if (string_1==0){
                                               string_1 = 1;
                                                            } else string_1 = 0; };
              }; 
          };
               
        };
    };
    
void scalerun()
 { LC.update();
   
     if (millis() - loadcellT >= 300)
      { HDC.SENSORS.scale = LC.getData();
    //    filteredmass=filterF(LC.getData(),.78);
        filteredmass=filterF(HDC.SENSORS.scale,0.64);
      
      


                 if (safeTmass==true )  
                          if (millis()-safeTmassT >= 1000 and  HDC.AIR.OUT.statusnow == false )
                              { filteredGmass=filtermassG(HDC.SENSORS.scale,0.85);
                                 compensate=filteredGmass*((UNODATA.dhtT[0]-starttemp)/10)*(14.29/12.65);
                                    
                                 if (millis() - safeTmassT >=4000)
                                         safeTmass=false;       
                               };     
                loadcellT = millis();                            
         };
      
 };
  
  void fancontrol()
{   
       if (HDC.AIR.OUT.ORIDE.enable == true)  
              { genie.WriteObject(GENIE_OBJ_USER_LED, 2,HDC.AIR.OUT.ORIDE.statusnow);
                digitalWrite(FANPIN, HDC.AIR.OUT.ORIDE.statusnow);   
                }else {
                        genie.WriteObject(GENIE_OBJ_USER_LED, 2, HDC.AIR.OUT.statusnow);
                        digitalWrite(FANPIN, HDC.AIR.OUT.statusnow); // turn the LED on (HIGH is the voltage level) 
                            }

              if (HDC.AIR.IN.ORIDE.enable == true)
               { genie.WriteObject(GENIE_OBJ_USER_LED, 9,HDC.AIR.IN.ORIDE.statusnow);
                digitalWrite(FAN2PIN, HDC.AIR.IN.ORIDE.statusnow);   
                  }else {
                         genie.WriteObject(GENIE_OBJ_USER_LED, 9, HDC.AIR.IN.statusnow);
                         digitalWrite(FAN2PIN, HDC.AIR.IN.statusnow); // turn the LED on (HIGH is the voltage level)       
                        };
  };


  void heatcontrol()
  {    
       if (HDC.HEAT.OUT.ORIDE.enable == true)
              {  genie.WriteObject(GENIE_OBJ_USER_LED, 1,HDC.HEAT.OUT.ORIDE.statusnow);
                digitalWrite(HEATPIN, HDC.HEAT.OUT.ORIDE.statusnow);   
                  }else{         
                        genie.WriteObject(GENIE_OBJ_USER_LED, 1, HDC.HEAT.OUT.statusnow);
                        digitalWrite(HEATPIN, HDC.HEAT.OUT.statusnow); // turn the LED on (HIGH is the voltage level)
                        };                                         
  };                  
    
      
  void killdryprog()
  {
                    genie.WriteObject(GENIE_OBJ_USER_LED, 3, 0); 
                    HDC.DRYP.startF[0]= false; //RESET TARE STATUS
                    genie.WriteObject(GENIE_OBJ_USER_LED, 4, 0);
                    HDC.DRYP.startF[1]= false; //RESET MASS STATUS
                    HDC.DRYP.startF[2]= false;  //NOT ON FORM 5 ANYMORE
                    HDC.DRYP.runningnow=false;
                    yield();
                    genie.WriteObject(GENIE_OBJ_USER_LED, 6, 0); //led on the page
                    genie.WriteObject(GENIE_OBJ_USER_LED, 11, 0); //led on the page
                    genie.WriteObject(GENIE_OBJ_USER_LED, 14, 0); //led on the page
                    genie.WriteObject(GENIE_OBJ_USER_LED, 13, 0); //led on the page
                    genie.WriteObject(GENIE_OBJ_USER_LED, 10, 0); //led on the page
                    genie.WriteObject(GENIE_OBJ_USER_LED, 0, 0); //led on the page
                    genie.WriteObject(GENIE_OBJ_LED, 0, 0); //led on the page
                    genie.WriteObject(0x1E,7, 0); //4D BUTTON 7
                    HDC.AIR.OUT.statusnow = false;
                    HDC.HEAT.OUT.statusnow = false;
                    if(HDC.DRYP.done==true)  //add a timer todo
                       { HDC.AIR.IN.statusnow = true; 
                          }else{  
                                HDC.AIR.IN.statusnow = false;};                                                    
  };


  void dryprog () 
{
  
  if (HDC.AIR.OUT.statusnow == false)
          { HDC.DRYP.massG=filteredmass;
            percentM=HDC.DRYP.massG/HDC.DRYP.massI;
          };
   
      
        verifydry();  

         temperaturemode();      
     
}; 


void verifydry()
{
   if (percentM>100)
          percentM = 100;
            else  if (percentM<0)
            percentM = 0;
            
   genie.WriteObject (GENIE_OBJ_SCOPE, 0,percentM*100);
   HDC.SENSORS.heatavg=(UNODATA.mlx[0]+UNODATA.mlx[1]+UNODATA.mlx[2]+UNODATA.mlx[3])/4;

                       
                      
        if (percentM <= .05)
            { donecount++;
              if (donecount >= 200)
                  { HDC.DRYP.done=true;
                    killdryprog();
                    genie.WriteObject(GENIE_OBJ_USER_LED, 12,true);
                    return;
                  };
              };                


  
};
                                 
void manualmode()
{
    switch (manST)
     {  case 0 :     manSTstate = true; 
                     if ((millis() - HDC.DRYP.timerT)/1000 > (HDC.DRYP.cycleT*HDC.DRYP.heatP))
                          manST++;   
                      break;
        case 1:   // HDC.DRYP.state = false;
                    manSTstate = false;   
                     if ((millis() - HDC.DRYP.timerT)/1000 >= HDC.DRYP.cycleT+1)
                      {   manST=0;
                         HDC.DRYP.timerT = millis(); 
                      };
                     break;
      };
      
         if (manSTstate==true)
           {      HDC.AIR.OUT.statusnow =  false;
                  HDC.HEAT.OUT.statusnow = true; 
                  if (HDC.SENSORS.heatavg>= HDC.DRYP.maxT)
                        manSTstate=false;
                     
                     } else if (manSTstate == false)
                        {     HDC.AIR.OUT.statusnow = true;
                              HDC.HEAT.OUT.statusnow = false;     
                        };  



                        
};

void gethotgel()
{
   float compare=0;
   hottestgel = max(UNODATA.mlx[0],UNODATA.mlx[1]);
   compare = max(UNODATA.mlx[2],UNODATA.mlx[3]);
   hottestgel = max(hottestgel,compare);
}

void gethighhumid()
{ highesthumid = max(UNODATA.dhtH[0],UNODATA.dhtH[1]);
  highesthumid = max(UNODATA.dhtH[2],highesthumid);  
}







                                                                       
void temperaturemode()
{
  
  gethotgel();
 // gethighhumid();
  
              switch (tempST)    
   {   case 0:      if (HDC.SENSORS.heatavg  <= HDC.DRYP.maxT - (heatripple + 3))
                      {  tempST = 1;
                         break;
                      };
                        
                     HDC.AIR.IN.statusnow = true;
                     HDC.AIR.OUT.statusnow = true; 
                     HDC.HEAT.OUT.statusnow = false;  
                     break;

      
      case 1 :      if  (HDC.SENSORS.heatavg >HDC.DRYP.maxT )
                          {tempST = 0;
                           break;
                          };
                          
                    //HEATING MAIN
                     HDC.AIR.OUT.statusnow = false;
                     HDC.AIR.IN.statusnow = false;
                     HDC.HEAT.OUT.statusnow = true; 


                      if (safeTmass ==false)
                      {
                        if (HDC.AIR.IN.statusnow == false and HDC.AIR.OUT.statusnow == false)
                                                        {  
                                                           safeTmass=true;
                                                           safeTmassT=millis();
                                                           safeTreading=millis();
                                                        };
                      };






                   if (UNODATA.dhtH[1] >= (humidMAX - humidripple))
                        { 
                          tempST = 2;
                        };
                        

                   if  (HDC.SENSORS.heatavg > (HDC.DRYP.maxT - heatripple) )
                      {  
                          tempST = 2;
                      };

                       break;
                   
      case 2:         //HEAT ON, CONDITIONS TO TURN off dans

                     if (HDC.SENSORS.heatavg<= (HDC.DRYP.maxT-(heatripple)))
                        {HDC.HEAT.OUT.statusnow = true;
                            }else if (HDC.SENSORS.heatavg>= HDC.DRYP.maxT-heatripple)
                                     { HDC.HEAT.OUT.statusnow = false;
                                       if (HDC.SENSORS.heatavg >= HDC.DRYP.maxT) 
                                            {tempST = 0;
                                             break;
                                             };    
                                       } ;   
                         
          
                      if (UNODATA.dhtH[1] < (humidMAX - (humidripple)))
                               { 
                                  HDC.AIR.OUT.statusnow = false;
                                  
                                 } else { if (UNODATA.dhtH[1] >= humidMAX - humidripple)
                                              { 
                                                HDC.AIR.OUT.statusnow = true;                                                            
                                                };
                              
                                  };
                               
                     if  ( HDC.SENSORS.heatavg<= HDC.DRYP.maxT - heatripple)
                         { HDC.AIR.IN.statusnow = false;
                          } else if  ( HDC.SENSORS.heatavg>=(HDC.DRYP.maxT-heatripple))
                                              {  HDC.AIR.IN.statusnow = true;
              
                                              };   

                    //   if (HDC.SENSORS.heatavg<=HDC.DRYP.maxT - heatripple and UNODATA.dhtH[1] < humidMAX-humidripple)
                             if (safeTmass ==false)
                      {
                            if (HDC.AIR.IN.statusnow == false and HDC.AIR.OUT.statusnow == false)
                                                        {  tempST = 1;     
                                                           safeTmass=true;
                                                           safeTmassT=millis();
                                                           safeTreading=millis();
                                                        };
                      };
                          break;    
   };
   
     
 };                                 

int write2TSData( long TSChan,char* key,unsigned int TSField1, float field1Data, unsigned int TSField2, float field2Data,
                                        unsigned int TSField3, float field3Data, unsigned int TSField4, float field4Data,
                                        unsigned int TSField5, float field5Data, unsigned int TSField6, float field6Data,
                                        unsigned int TSField7, float field7Data, unsigned int TSField8, float field8Data )
 {int writeSuccess=0;
  upd = false;
  ThingSpeak.begin( client );          
  ThingSpeak.setField( TSField1, field1Data );
  ThingSpeak.setField( TSField2, field2Data );
  ThingSpeak.setField( TSField3, field3Data );
  ThingSpeak.setField( TSField4, field4Data );
  ThingSpeak.setField( TSField5, field5Data );
  ThingSpeak.setField( TSField6, field6Data );
  ThingSpeak.setField( TSField7, field7Data );  
  ThingSpeak.setField( TSField8, field8Data );
  writeSuccess = ThingSpeak.writeFields(TSChan,key);
   client.stop();
   yield();
   return writeSuccess;  
 };
   
float filterF (float _xn,float weight)
{if (HX.filterbegin ==true)
  {
  if (HX.count == 0)
  {HX.yn_1 = _xn;
  }else 
       {
          (HX.yn_1 = HX.yn);
                            };      
HX.yn=(_xn*weight)+((1-weight)*(HX.yn_1));
HX.count++;
  };
  return HX.yn;
};


float filtermassG (float _xn,float weight)
{if (HX2.filterbegin ==true)
  {
  if (HX2.count == 0)
  {HX2.yn_1 = _xn;
  }else 
       {
          (HX2.yn_1 = HX2.yn);
                            };      
HX2.yn=(_xn*weight)+((1-weight)*(HX2.yn_1));
HX2.count++;
  };
  return HX2.yn;
};
