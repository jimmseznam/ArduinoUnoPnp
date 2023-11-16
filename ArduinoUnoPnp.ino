//ISR_Steper

#define MOTOR_ENABLE_PIN 8

#define MOTOR_X_STEP_PIN 2
#define MOTOR_X_DIR_PIN 5
#define MOTOR_X_DIRECTION HIGH  //Set HIGH or LOW to change direction
#define MOTOR_X_ACC 50          //Acceleration

#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6
#define MOTOR_Y_DIRECTION LOW   //Set HIGH or LOW to change direction
#define MOTOR_Y_ACC 50          //Acceleration 

#define MOTOR_Z_STEP_PIN 4
#define MOTOR_Z_DIR_PIN 7
#define MOTOR_Z_DIRECTION HIGH  //Set HIGH or LOW to change direction
#define MOTOR_Z_ACC 50          //Acceleration

#define MOTOR_A_STEP_PIN 12
#define MOTOR_A_DIR_PIN 13
#define MOTOR_A_DIRECTION HIGH  //Set HIGH or LOW to change direction
#define MOTOR_A_ACC 50          //Acceleration

#define VACUUM_PUMP_PWM_PIN 11  //CPU Port PB3,  PWM output for vacu pump motor
#define VACUUM_REALY_PIN A3     //CPU Port PC3

#define MAX_RPS  4.0 // max revolution per s 

#define BAUD_RATE 115200
#define RX_BUF_CNT 10
#define RX_BUF_SIZE 62 + 1 + 1             //62 chars + 1 end null + 1 BYTE for valid chars cnt (without end null)  
#define RX_CHAR_CNT_INDEX RX_BUF_SIZE - 1  //index of last BYTE in each g_RX_buf, where is valid chars cnt (without end null)  

#define CMD_PAUSE '\\'         //Pause PnP
#define CMD_CONTINUE '/'       //Pause end
#define CMD_SET_HOME '='       //Set HOME position
#define CMD_ABORT '*'          //abort PnP 0x18 // ctrl-x.

typedef enum { X_AXE, Y_AXE, Z_AXE, A_AXE} axis_t;
typedef enum { RS_IDLE, RS_RUN, RS_PAUSE, RS_SET_HOME, RS_ABORT} RunState_t;

typedef struct  { uint8_t SpeedCnt;
                   uint8_t AccCnt;
                   
                   int32_t SollWert;   
                   int32_t IstWert;    
                  } t_axis_info;
  
RunState_t g_RunState    = RS_IDLE; //inicialize g_RunState
RunState_t g_ReqRunState = RS_IDLE; //required g_RunState

t_axis_info axis[4]; 

int mot_HWsteps_per_rev = 200;      //HW motor steps/revolution
int microsteps = 16;                // 
float steps_per_rev = mot_HWsteps_per_rev * microsteps; //200 * 16 = 3200 microsteps per revolution
                  //(  4      *     3200     ) = 12800 Hz
float clock_freq = (MAX_RPS  * steps_per_rev); //12800 Hz      
int match_register = (16000000 / clock_freq) - 1;

int       g_tmr1Cnt = 0;
uint16_t  g_LoopCnt = 0;                //"timeout" for Loop()without MSG 'G' - for ackowledging, that realy Idle 
bool      g_JeSend  = false;

char g_RX_buf[RX_BUF_CNT][RX_BUF_SIZE];
volatile uint8_t g_Wr_rx_buff_No  = 0;  //Number Rx buff - only for ISR(USART_RX_vect)
volatile uint8_t g_Wr_rx_charsCnt = 0;  //Number of valid readed chars in g_RX_buf[n]

uint8_t g_Rd_G_buff_No  = 0;            //Number g_RX_buf buff, from whose is read solwert position [mm] in loop() 
uint8_t g_Rd_S_buff_No  = 0;            //Number g_RX_buf buff, from whose is read Setup in loop()


char    g_TXbuf[80];                   // = "<I|0.000|0.000|0.000|0.000|0|0|>\n\r";
char   *g_TXbuf_ptr;

uint8_t g_Pump         = 0;  //PWM value for pump 0..255
uint8_t g_Rele         = 0;  //Air relay state
uint8_t g_SpeedDiv     = 1;  //Global speed divider 0 - disable drivery, 1..16 - speed divider
uint8_t g_EnableSteper = 0;  //Global enable steper - 0 = Disable, 1 = Enable

//_____________________________________________________________________________________________________________
int parse_comma_delimited_str(char *string, char **fields, int max_fields) //in field return pointers to single strings
{
  int i = 0;
  fields[i++] = string;

  while ((i < max_fields) && 0 != (string = strchr(string, '|'))) { //strchr returns a pointer to the first occurrence of character '|' in the string, or NULL if the character is not found.
    *string = '\0';
    fields[i++] = ++string;
  }

  return --i;
}


//___________________________________________________________________________________
void setup() 
{ uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
  int i;

  cli();//disable interrupts
  {//Init serial 
    UCSR0A |= (1 << U2X0);                  // baud doubler on for high baud rates, i.e. 115200
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);   // Turn on the transmission and reception circuitry
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes

    UBRR0H = (UBRR0_value >> 8);            // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
    UBRR0L =  UBRR0_value;                  // Load lower 8-bits of the baud rate value into the low byte of the UBRR register

    UCSR0B |= (1 << RXCIE0);                // Enable the USART Recieve Complete interrupt (USART_RXC)
  }


  {//init timer1 interrupt
    TCCR1A = 0;                             // set entire TCCR1A register to 0                         
    TCCR1B = 0;                             // same for TCCR1B
    TCNT1  = 0;                             //initialize counter value to 0
    // set compare match register for 1hz increments
    OCR1A = match_register;                 // = (16*10^6) / (1*12800) - 1 (must be <65536)
    TCCR1B |= (1 << WGM12);                 // turn on CTC mode
    TCCR1B |= (0 << CS12) | (1 << CS10);    // Set CS10 and CS12 bits for 1 prescaler
    TIMSK1 |= (1 << OCIE1A);                // enable timer compare interrupt
  }
  sei();//enable interrupts

  

  for (i = X_AXE; i <= A_AXE; i++)
  { axis[i].SollWert = 0;
    axis[i].IstWert  = 0;
  }  
  pinMode(MOTOR_ENABLE_PIN,   OUTPUT);
  
  pinMode(MOTOR_X_DIR_PIN,  OUTPUT); 
  pinMode(MOTOR_X_STEP_PIN, OUTPUT); 

  pinMode(MOTOR_Y_DIR_PIN,  OUTPUT); 
  pinMode(MOTOR_Y_STEP_PIN, OUTPUT); 

  pinMode(MOTOR_Z_DIR_PIN,  OUTPUT); 
  pinMode(MOTOR_Z_STEP_PIN, OUTPUT); 

  pinMode(MOTOR_A_DIR_PIN,  OUTPUT); 
  pinMode(MOTOR_A_STEP_PIN, OUTPUT); 
  
  pinMode (VACUUM_REALY_PIN, OUTPUT); //Set Arduino pin A3, bit on CPU portu PC3. (32 pins CPU, pin 25 as Digital output - for control air vacu valve 
  

  digitalWrite(MOTOR_ENABLE_PIN, !g_EnableSteper); //Disable stepers

  g_TXbuf[0]   = 0;
  g_TXbuf_ptr  = g_TXbuf;

}//setup()



/*Form PC is sended:
 * 1. Pause (Esc)
 * 2. Abort - cancer PnP
 * 3. JOG move X, Y, Z, Angle
 * 4. Go HOME
 * 5. Set HOME //Set actual position as reference position 0,0,0
 * 6. linear movement speed
 * 7. Air pump revolution
 * 8. Vacuum On / Off
 * 9. Sequence for placement - Needle up, Needle above pick device, Needle down, Vacuum on, Air Pump On, Needle up, Needle above place device, Needle down  (different height), Vacuum Off, Air pump Off, Needle up
 * 10.En/Dis steppers
 * 11.Delay ?
 */
//___________________________________________________________
void loop() 
{ //Here defined vars have local property - each entry to loop() are undefined
  char *field[10];
  int i;
  uint8_t cnt;
  
  g_LoopCnt++;

  if ((g_RunState == RS_SET_HOME) || (g_RunState == RS_ABORT))    //State RS_SET_HOME or RS_ABORT is only by Pause or Idle
  {
    for (i = 0; i < RX_BUF_CNT; i++) {g_RX_buf[i][RX_CHAR_CNT_INDEX] = 0;}
   
    for (i = X_AXE; i <= A_AXE; i++)
    { 
        if (g_RunState == RS_SET_HOME)
        {
          axis[i].SollWert = 0;               //Set SollWert and IstWert  position zo zero
          axis[i].IstWert  = 0;
        }

        if (g_RunState == RS_ABORT)
        {
          axis[i].SollWert = axis[i].IstWert; //Set SollWert according IstWert
        }
    }
    
    g_Wr_rx_buff_No  = 0;
    g_Wr_rx_charsCnt = 0;
    g_Rd_G_buff_No   = 0;
                        
    g_RunState = RS_IDLE;
  }

  {
        cnt = g_RX_buf[g_Rd_G_buff_No][RX_CHAR_CNT_INDEX];
        if (cnt)      //if chars cnt>0
        { 
            if (g_RX_buf[g_Rd_G_buff_No][1] != 'G')                      //skip MSG, other then G
            {
                g_Rd_G_buff_No++;                                        //Inc read buffer
                if (g_Rd_G_buff_No >= RX_BUF_CNT) {g_Rd_G_buff_No = 0;}  //behind the last buff, turn g_Rd_rx_buff_No on first buff
            }
            else
            {
                 g_LoopCnt = 0;                                          //Zeroing g_LoopCnt, because i g_RX_buf id some 'G' MSG, so that  is not Idle state (RS_IDLE)
           
                 if (g_RunState == RS_IDLE)                              //if (g_RunState == RS_IDLE) (and MSG je G - 'Go') convert coordinate. e.g.  <G|0|0|0|0|0|0|> <G|5.5|8.4|3.51|45|1|1|> <G|100||||||> <G|X|Y|Z|Angle|Pump|Rele|res> to absolut position  in steps 
                 {                           //800 = 3200 / 4  t.j. 3200 mikrosteps_per_revolution / 4 mm shift per 1 revolution
                   i = parse_comma_delimited_str(g_RX_buf[g_Rd_G_buff_No], field, 10); //Parse MSG

                   if (field[1][0]) 
                   {
                     axis[X_AXE].SollWert = 800 * atof(field[1]); //X
                     axis[X_AXE].SpeedCnt = MOTOR_X_ACC;
                     axis[X_AXE].AccCnt = MOTOR_X_ACC;
                   }
                   
                   if (field[2][0]) 
                   {
                     axis[Y_AXE].SollWert = 800 * atof(field[2]); //Y
                     axis[Y_AXE].SpeedCnt = MOTOR_Y_ACC;
                     axis[Y_AXE].AccCnt = MOTOR_Y_ACC;
                   }
                   
                   if (field[3][0]) 
                   {
                     axis[Z_AXE].SollWert = 800 * atof(field[3]); //Z
                     axis[Z_AXE].SpeedCnt = MOTOR_Z_ACC;
                     axis[Z_AXE].AccCnt = MOTOR_Z_ACC;
                   }
                   
                   if (field[4][0]) 
                   {
                     axis[A_AXE].SollWert = ((3200.0 / 360.0) * atof(field[4])); //Angle 3200 mikrosteps_per_revolution
                     axis[A_AXE].SpeedCnt = MOTOR_A_ACC;
                     axis[A_AXE].AccCnt = MOTOR_A_ACC;
                   }
                   
                   if (field[5][0]) {g_Pump = atoi(field[5]); analogWrite(VACUUM_PUMP_PWM_PIN,  g_Pump);} //Vacuum - pump motor Function analogWrite make PWM from Timer2 on Arduino pin digital 11, port PB3, 32 pins SMD CPU pin 125. 70 presents 70 / 255 wide of PWM  
                   if (field[6][0]) {g_Rele = atoi(field[6]); digitalWrite(VACUUM_REALY_PIN, !g_Rele);}   //Vacuum - control air relay

                   
                   g_RX_buf[g_Rd_G_buff_No][RX_CHAR_CNT_INDEX] = 0;          //delete MSG i.e. zaroing char cnt

                   g_Rd_G_buff_No++;                                         //Inc read buffer
                   if (g_Rd_G_buff_No >= RX_BUF_CNT) {g_Rd_G_buff_No = 0;}   //After last  buff, turn to first buf

                   if (field[7][0])
                   {
                       cnt = atoi(field[7]);                                 //Speed divider
                       if (cnt)
                       {
                         digitalWrite(MOTOR_ENABLE_PIN, LOW);                //enable stepers
                         g_EnableSteper = 1;
                         g_SpeedDiv     = cnt; 
                       } 
                       else
                       {
                         digitalWrite(MOTOR_ENABLE_PIN, HIGH);               //disable stepers
                         g_EnableSteper = 0;                                 //when we send speeddiv=0 disable stepers, but set g_SpeedDiv=1 - simulate max.acc, w.o. motor movig
                         g_SpeedDiv     = 1;
                       }   
                   }
                   g_RunState = RS_RUN; 
                 }//if (g_RunState == RS_IDLE) 
             }
         }//if (cnt) 


        cnt = g_RX_buf[g_Rd_S_buff_No][RX_CHAR_CNT_INDEX];
        if (cnt)      //if char countnt>0
        {
            if (g_RX_buf[g_Rd_S_buff_No][1] != 'S')                         //skip MSG, which not '§'
            {
                g_Rd_S_buff_No++;                                           //Inc read buffer
                if (g_Rd_S_buff_No >= RX_BUF_CNT) {g_Rd_S_buff_No = 0;}     //After last buff turn to begin buff
            }
            else //If MSG is S - 'Setup'
            {
                i = parse_comma_delimited_str(g_RX_buf[g_Rd_S_buff_No], field, 10); //Parse MSG

                //MSG example: PauseOn = '<S|P|1|||||>', PauseOff = '<S|P|0|||||>'
        
                g_RX_buf[g_Rd_S_buff_No][RX_CHAR_CNT_INDEX] = 0;            //Zeroing char cnt

                g_Rd_S_buff_No++;                                           //Inc read buffer
                if (g_Rd_S_buff_No >= RX_BUF_CNT) {g_Rd_S_buff_No = 0;}     //After last buff turn to begin buff        

             }
     
         }//if (cnt) 
  } //switch (g_RunState)

     

  if (g_JeSend) //To PC we send:  '<R|X_istWert|Y_istWert|Z_istWert|Angle_istWert|Vakuum_istWert>'
  {
       g_TXbuf[0] = '<'; 

       if (g_RunState == RS_PAUSE) {g_TXbuf[1] = 'P';}
       else if ((g_LoopCnt > 10) && (g_RunState == RS_IDLE)) {g_TXbuf[1] = 'I';} else {g_TXbuf[1] = 'R';} 
       
       g_TXbuf[2] = '|';
       g_TXbuf[3] = 0;
       
       dtostrf((float)axis[0].IstWert / 800, 2, 3, &g_TXbuf[3]);                      //X IstWert push from [3] 
       strcat(g_TXbuf, "|");
       dtostrf((float)axis[1].IstWert / 800, 2, 3, &g_TXbuf[strlen(g_TXbuf)]);        //Y IstWert
       strcat(g_TXbuf, "|");
       dtostrf((float)axis[2].IstWert / 800, 2, 3, &g_TXbuf[strlen(g_TXbuf)]);        //Z IstWert
       strcat(g_TXbuf, "|");
       dtostrf((float)axis[3].IstWert * (float)  (360.0 / 3200.0), 2, 3, &g_TXbuf[strlen(g_TXbuf)]); //Angle IstWert      
       strcat(g_TXbuf, "|");
       dtostrf(g_Pump, 1, 0, &g_TXbuf[strlen(g_TXbuf)]);                              //Pump IstWert
       
       if (g_Rele)  {strcat(g_TXbuf, "|1|");} else {strcat(g_TXbuf, "|0|");}          //Relay IstWert   //if (g_Rele)  {strcat(g_TXbuf, "|1|>\r\n");} else {strcat(g_TXbuf, "|0|>\r\n");}  //Relay IstWert

       if (g_EnableSteper)
       {
         dtostrf(g_SpeedDiv, 1, 0, &g_TXbuf[strlen(g_TXbuf)]);                        //g_SpeedDiv
         strcat(g_TXbuf, ">\r\n");
       }
       else
       {
         strcat(g_TXbuf, "0>\r\n");                                                   //When not g_EnableSteper send out 0, which mean: stepers disabled
       }

       
       g_JeSend = false;
  } 
     
  if (UCSR0A & (1 << UDRE0) && g_TXbuf[0]) //Id TX register empty and is 1. char in g_TXbuf non zero
  {
         UDR0 = *g_TXbuf_ptr++;     //send char,  g_TXbuf_ptr
         if (!*g_TXbuf_ptr)         //when next char is null
         {
            g_TXbuf_ptr = g_TXbuf;  //set g_TXbuf_ptr on begin of g_TXbuf
            g_TXbuf[0]  = 0;        //and zeroing first char in g_TXbuf  
         }
  }
     
}//loop()


//_______________________________________________________________________________
//Serial receive char interupt (from PC)
ISR(USART_RX_vect)
{  char c = UDR0; //Char from UART
//   UDR0 = c;    //Echo

   switch (c) 
   {
     case CMD_PAUSE     : { g_ReqRunState = RS_PAUSE;    return;}
     case CMD_CONTINUE  : { g_ReqRunState = RS_RUN;      return;}
     case CMD_SET_HOME  : { g_ReqRunState = RS_SET_HOME; return;}
     case CMD_ABORT     : { g_ReqRunState = RS_ABORT;    return;}
     case '\n'          : { return;} //discard LINE_FEED (if came)
   }
   
   if ( g_Wr_rx_charsCnt >= RX_BUF_SIZE - 2)                             //i come too long  MSG - error
   {
     g_Wr_rx_charsCnt = 0;
     g_RX_buf[g_Wr_rx_buff_No][RX_CHAR_CNT_INDEX] = 0;                   //discard whole MSG (set zero length) and next MSG will write to same g_RX_buf[g_Wr_rx_buff_No]
   }
   else if (c == '\r')                                                   //When is end MSG (char CR = 0x0D = \r) if ((c == '\n') || (c == '\r')) (LF = 0x0A = \n) 
         { 
           if ((g_RX_buf[g_Wr_rx_buff_No][0] == '<') && (g_RX_buf[g_Wr_rx_buff_No][g_Wr_rx_charsCnt - 1] == '>'))
           {          
             g_RX_buf[g_Wr_rx_buff_No][g_Wr_rx_charsCnt - 1] = 0;             //Replace end '>' #0
             g_RX_buf[g_Wr_rx_buff_No][RX_CHAR_CNT_INDEX] = g_Wr_rx_charsCnt; //To last Bajtu g_RX_buf[g_Wr_rx_buff_No] push length of MSG 
             g_Wr_rx_charsCnt = 0;                                            //Zeroing chars count
             g_Wr_rx_buff_No++;                                               //inc g_Wr_rx_buff_No
             if (g_Wr_rx_buff_No >= RX_BUF_CNT) {g_Wr_rx_buff_No = 0;}        //After last buff turn to begin g_Wr_rx_buff_No
           }
           else
           {
             g_Wr_rx_charsCnt = 0;                                            //When not began with '<' and not terminate with  '>'
             g_RX_buf[g_Wr_rx_buff_No][RX_CHAR_CNT_INDEX] = 0;                //discard whole MSG (set zero length) and next MSG will write to same g_RX_buf[g_Wr_rx_buff_No]
           }
         }
         else g_RX_buf[g_Wr_rx_buff_No][g_Wr_rx_charsCnt++] = c;            //store one char of MSG
}//ISR(USART_RX_vect)

//_______________________________________________________________________________-
ISR(TIMER1_COMPA_vect)     //timer1 interrupt  for steppers (cca 12800Hz) 
{
  if (g_ReqRunState)
  {
    if ((g_ReqRunState == RS_PAUSE)    && (g_RunState == RS_RUN))   {g_RunState = RS_PAUSE;}  
    if ((g_ReqRunState == RS_RUN)      && (g_RunState == RS_PAUSE)) {g_RunState = RS_RUN;}
    if ((g_ReqRunState == RS_SET_HOME) && ((g_RunState == RS_PAUSE) || (g_RunState == RS_IDLE))) {g_RunState = RS_SET_HOME;} //RS_SET_HOME is posible by  Pause or Idle only
    if ((g_ReqRunState == RS_ABORT)    && ((g_RunState == RS_PAUSE) || (g_RunState == RS_IDLE))) {g_RunState = RS_ABORT;}    //RS_ABORT is posible by Pause or Idle only
    g_ReqRunState = RS_IDLE;
  }
  
  
  if (g_RunState == RS_RUN)  
  {   
      //X______________________________________________________
      if (axis[X_AXE].AccCnt) {axis[X_AXE].AccCnt--;}
      if (!axis[X_AXE].AccCnt)
      {    
          if (axis[X_AXE].SpeedCnt > g_SpeedDiv) {axis[X_AXE].SpeedCnt--;}
          
          axis[X_AXE].AccCnt = axis[X_AXE].SpeedCnt;   
          
          if (axis[X_AXE].SollWert > axis[X_AXE].IstWert)
          { 
            digitalWrite(MOTOR_X_DIR_PIN,  MOTOR_X_DIRECTION); // direction 1 (high = CW) FORWARD
            digitalWrite(MOTOR_X_STEP_PIN, HIGH);
            digitalWrite(MOTOR_X_STEP_PIN, LOW);
            axis[X_AXE].IstWert++;
          }

          if (axis[X_AXE].SollWert < axis[X_AXE].IstWert)
          { 
            digitalWrite(MOTOR_X_DIR_PIN, !MOTOR_X_DIRECTION); // direction 0 (high = CCW) REVERSE
            digitalWrite(MOTOR_X_STEP_PIN, HIGH);
            digitalWrite(MOTOR_X_STEP_PIN, LOW);
            axis[X_AXE].IstWert--;
          }
      }
      
      //Y______________________________________________________
      if (axis[Y_AXE].AccCnt) axis[Y_AXE].AccCnt--;
      if (!axis[Y_AXE].AccCnt)
      {   if (axis[Y_AXE].SpeedCnt) 

          if (axis[Y_AXE].SpeedCnt > g_SpeedDiv) {axis[Y_AXE].SpeedCnt--;}
          
          axis[Y_AXE].AccCnt = axis[Y_AXE].SpeedCnt;   
          
          if (axis[Y_AXE].SollWert > axis[Y_AXE].IstWert)
          { 
            digitalWrite(MOTOR_Y_DIR_PIN,  MOTOR_Y_DIRECTION); // direction 1 (high = CW) FORWARD
            digitalWrite(MOTOR_Y_STEP_PIN, HIGH);
            digitalWrite(MOTOR_Y_STEP_PIN, LOW);
            axis[Y_AXE].IstWert++;
          }

          if (axis[Y_AXE].SollWert < axis[Y_AXE].IstWert)
          { 
            digitalWrite(MOTOR_Y_DIR_PIN, !MOTOR_Y_DIRECTION); // direction 0 (high = CCW) REVERSE
            digitalWrite(MOTOR_Y_STEP_PIN, HIGH);
            digitalWrite(MOTOR_Y_STEP_PIN, LOW);
            axis[Y_AXE].IstWert--;
          }
      }
      
      //Z______________________________________________________
      if (axis[Z_AXE].AccCnt) axis[Z_AXE].AccCnt--;
      if (!axis[Z_AXE].AccCnt)
      {   if (axis[Z_AXE].SpeedCnt) 

          if (axis[Z_AXE].SpeedCnt > g_SpeedDiv) {axis[Z_AXE].SpeedCnt--;}
          
          axis[Z_AXE].AccCnt = axis[Z_AXE].SpeedCnt;   
     
          if (axis[Z_AXE].SollWert > axis[Z_AXE].IstWert)
          { 
            digitalWrite(MOTOR_Z_DIR_PIN,  MOTOR_Z_DIRECTION); // direction 1 (high = CW) FORWARD
            digitalWrite(MOTOR_Z_STEP_PIN, HIGH);
            digitalWrite(MOTOR_Z_STEP_PIN, LOW);
            axis[Z_AXE].IstWert++;
          }

          if (axis[Z_AXE].SollWert < axis[Z_AXE].IstWert)
          { 
            digitalWrite(MOTOR_Z_DIR_PIN, !MOTOR_Z_DIRECTION); // direction 0 (high = CCW) REVERSE
            digitalWrite(MOTOR_Z_STEP_PIN, HIGH);
            digitalWrite(MOTOR_Z_STEP_PIN, LOW);
            axis[Z_AXE].IstWert--;
          }
      }    
          
      //Angle__________________________________________________
      if (axis[A_AXE].AccCnt) axis[A_AXE].AccCnt--;
      if (!axis[A_AXE].AccCnt)
      {   if (axis[A_AXE].SpeedCnt) 

          if (axis[A_AXE].SpeedCnt > g_SpeedDiv) {axis[A_AXE].SpeedCnt--;}
          
          axis[A_AXE].AccCnt = axis[A_AXE].SpeedCnt;   
      
          if (axis[A_AXE].SollWert > axis[A_AXE].IstWert)
          { 
            digitalWrite(MOTOR_A_DIR_PIN,  MOTOR_A_DIRECTION); // direction 1 (high = CW) FORWARD
            digitalWrite(MOTOR_A_STEP_PIN, HIGH);
            digitalWrite(MOTOR_A_STEP_PIN, LOW);
            axis[A_AXE].IstWert++;
          }

          if (axis[A_AXE].SollWert < axis[A_AXE].IstWert)
          { 
            digitalWrite(MOTOR_A_DIR_PIN, !MOTOR_A_DIRECTION); // direction 0 (high = CCW) REVERSE
            digitalWrite(MOTOR_A_STEP_PIN, HIGH);
            digitalWrite(MOTOR_A_STEP_PIN, LOW);
            axis[A_AXE].IstWert--;
          }
      } 
  }//if (g_RunState == RS_RUN)  
        
  if ((g_RunState == RS_RUN) && (axis[X_AXE].SollWert == axis[X_AXE].IstWert) && (axis[Y_AXE].SollWert == axis[Y_AXE].IstWert)
                             && (axis[Z_AXE].SollWert == axis[Z_AXE].IstWert) && (axis[A_AXE].SollWert == axis[A_AXE].IstWert)) {g_RunState = RS_IDLE;} //Idle
  
  
  g_tmr1Cnt++;

  if (g_tmr1Cnt > 250 * MAX_RPS)  // if ((tmr1Cnt % 10000) == 0) //modulo 
  {
    g_JeSend = true ;
    g_tmr1Cnt = 0;
  }
  
}//ISR(TIMER1_COMPA_vect)
