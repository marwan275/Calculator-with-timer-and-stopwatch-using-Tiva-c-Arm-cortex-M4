/*
CSE211 Introduction to Embedded Systems Project
Team 10[
Marwan Hatem    19P4259
Kirollos Thabet 19P6754
Mina Naeem      19P3788
Ahmed Amr       19P7696
Eaman Mohamed   19P1197
]
*/
#include "tm4c123gh6pm.h"
#include "LCD.h"
#include "Keypad.h"
#include "types.h"

//ENUMES FOR USED STATES
typedef enum{CALC,TIMER,STOPWATCH,}MODE;

typedef enum {WAITING,WORKING,PAUSED}STATES;

//GLOBAL VARIABLES FOR STATES
uint8 g_stopwatch_state = WAITING;
uint8 g_state = CALC;

// TIME VARIABLES
int sec = 0;
int hr = 0;
int min = 0;

//FUNCTION PROTOTYPE
/*
Function to get calculator input using an array and keypad driver
search in that array to determine two numbers and operation
*/
void get_calc_input(uint8 *input,uint16 *num_1,uint16 *num_2,uint8 *operation);

/*
Function that handle calculator functions and display as it calls the input and
logic functions of calculator
*/
void calculator_mode(void);

/*
Function that handles the calculations and returns the result of it
*/
float calculator_logic(uint16 *num_1,uint16 *num_2,uint8 *operation);

/*
Function that start stopwatch and reset it
*/
void start_stopwatch(void);

/*
Function that takes input for timer using keypad driver
*/
void timer_input(void);

/*
Function that initialize timer0A for 1 second
*/
void Timer0A_Init(void);

/*
Function that initialize timer1A for 1 second
*/
void Time1A_init(void);
/*
Funtion that handles timer variables and display them on LCD using LCD driver
*/
void timer_handle(void);


int main(){
  
  //PortF two buttons Interrupt 
  SYSCTL_RCGCGPIO_R |= 0x20;   /* enable clock to GPIOF */
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlockGPIOCR register
  GPIO_PORTF_CR_R = 0x01;           // Enable GPIOPUR register enable to commit
  GPIO_PORTF_PUR_R |= (1<<4)|(1<<0);        // Enable Pull Up resistor PF4 and PF0
  GPIO_PORTF_DIR_R &= ~(1<<4)|~(1<<0);          //set PF0 as an output and PF4 as an input pin
  GPIO_PORTF_DEN_R |= (1<<4)|(1<<0)|(1<<2);         // Enable PF0 and PF4 and PF2 as a digital GPIO pins
  GPIO_PORTF_DIR_R |= (1<<2); //set buzzer as output
  
  GPIO_PORTF_IS_R  &= ~(1<<4)|~(1<<0);        /* make bit 4, 0 edge sensitive */
  GPIO_PORTF_IBE_R &=~(1<<4)|~(1<<0);         /* trigger is controlled by IEV */
  GPIO_PORTF_IEV_R &= ~(1<<4)|~(1<<0);        /* falling edge trigger */
  GPIO_PORTF_ICR_R |= (1<<4)|(1<<0);          /* clear any prior interrupt */
  GPIO_PORTF_IM_R  |= (1<<4)|(1<<0);          /* unmask interrupt */
  
  /* enable interrupt in NVIC and set priority to 3 */
  NVIC_PRI3_R  = 3 << 5;     /* set interrupt priority to 3 */
  NVIC_EN0_R  |= (1<<30);  /* enable IRQ30 (D30 of ISER[0]) */
  
  
  // Initialization Functions
  LCD_Init();
  keypad_Init();
  LCD_command(CLEAR_DISPLAY);
  
  
  //CONTROL PROCESSOR STATUS INTERRUPT ENABLE
  __asm("CPSIE i");
  
  while(1){
    
    // Manage Modes
    while(g_state == CALC){
      calculator_mode();
    }
    
    while(g_state == STOPWATCH){
      if(keypad_getkey() == '+'){
        TIMER0_CTL_R=0; //Pause timer0 before configuration
        
      }
      if (keypad_getkey() == '-'){
        TIMER0_CTL_R=1; //Resume timer0 (resume stopwatch)
      }
      
    }
    while(g_state == TIMER){
      
    }
  }
}

void get_calc_input(uint8 *input,uint16 *num_1,uint16 *num_2,uint8 *operation)
{
  //Blink cusror is used to indicate input from user
  LCD_command(DISPLAY_ON_CURSOR_BLINK);
  
  // variables to know the location of the operator and equal in the array
  uint8 i = 0;
  uint8 equal_i = 0;
  uint8 operation_i = 0;
  
  /* 
  the loop is set to repaet itself for 16 times as we are using 16x2 LCD
  The user can enter any two numbers but the numbers, operator and result must
  fit in the 16 space of the LCD
  */  
  while(i<17){
    
    // Input is taken and displayed on LCD
    uint8 temp = keypad_getkey();
    LCD_data(temp);
    
    
    input[i]=temp;
    if (temp == '+'||temp == '-'||temp == '/'||temp == '*'){
      // Oprator is taken and its location in the array is saved 
      *operation = temp; 
      operation_i = i;
    }
    
    if (temp == '='){ 
      /*
      when equal is entered the loop will stop and the equal location in array
      is saved
      */
      equal_i = i;
      break;
    }
    i++;
    //delay for keypad
    delay_ms(800);
  }
  
  /*
  loops to extract the two numbers from the input array
  and adjusted to assign any number of digits
  */
  for (int i = 0;i<operation_i;i++){
    *num_1 = *num_1 * 10 + (input[i]-'0');
    //ASCI of zero is subtracted as the input array is uint8
  }
  for (int i = operation_i+1;i<equal_i;i++){
    *num_2 = *num_2 * 10 + (input[i]-'0');
    //ASCI of zero is subtracted as the input array is uint8
    
  }
}

void calculator_mode(){
  
  //CALCULATOR USER-INTERFACE
  LCD_setcursorRowCol(0,0);
  LCD_printString("   Calculator   ");
  LCD_setcursorRowCol(1,0);
  
  //Variables of Calculator
  uint16 num_1= 0;
  uint16 num_2 = 0;
  uint8 operation = 0;
  float result = 0;
  uint8 input[16] = {'\0'};
  
  /*
  Calling the functions of input and calculations
  The variables are passed by address to them
  */
  get_calc_input(input,&num_1,&num_2,&operation);
  result = calculator_logic(&num_1,&num_2,&operation);
  
  //Displaying the Result
  LCD_printFloat(result);
  delay_ms(5000);
  LCD_command(CLEAR_DISPLAY);
}

float calculator_logic(uint16 *num_1,uint16 *num_2,uint8 *operation){
  //switch case for the calculator operation 
  switch (*operation){
  case '+':
    return  *num_1  + *num_2 ;
    break;
  case '-':
    return  *num_1  - *num_2;
    break;
  case '*':
    return  *num_1  * *num_2;
    break;
  case '/':
    //casting was used to get decimals of division
    return (float)(*num_1)  / (float)(*num_2);
    break;
  }
  return 0;
}


void Timer0A_Init(void){
  SYSCTL_RCGCTIMER_R = 0; //disable all timers
  SYSCTL_RCGCTIMER_R=0X1;          //enable clock of timer0
  TIMER0_CTL_R=0;                 //disable timer0 before configuration
  TIMER0_CFG_R=(1<<2);            //configure to be 16 bits
  TIMER0_TAMR_R=(1<<2);           //periodic mode
  TIMER0_TAPR_R=250;              //16Mz/250=64000 Hz
  TIMER0_TAILR_R=64000;           //Load the interval load register
  TIMER0_ICR_R=1;                 //clear the time-out interupt of timerA
  TIMER0_IMR_R=1;                   //enable the time-out interupt of timerA
  NVIC_PRI4_R |=0X80000;          
  NVIC_EN0_R |= (1<<19);
  TIMER0_CTL_R=1;                 //enable timer0 before configuration
  
}
void Timer0A_Handler(void){
  
  TIMER0_ICR_R = 1; //clear the flag
  sec++; //increase time
  if(g_state == STOPWATCH){
    /*
    Display is handled in the handler so that 
    the lcd will refresh every one second and that's more effecient
    */
    start_stopwatch(); 
  }
}
/*
Mode switching is handled by interrupts of the buttons of the tiva
One for the timer and the other for the stopwatch
*/
void PORTF_Handler(void){	
  if (GPIO_PORTF_MIS_R & 0x10) /* check if interrupt causes by PF4/SW1*/
  {   
    GPIO_PORTF_ICR_R |= 0x10; /* clear the interrupt flag */
    g_state = STOPWATCH;
    //if the stopwatch was on and the button was pressed it will reset
    sec=0;
    hr = 0;
    min = 0;
    LCD_command(CLEAR_DISPLAY);
    LCD_printString("STOPWATCH");
    Timer0A_Init();
  } 
  else if (GPIO_PORTF_MIS_R & 0x01) /* check if interrupt causes by PF0/SW2 */
  {   
    GPIO_PORTF_ICR_R |= 0x01;     /* clear the interrupt flag */
    LCD_command(CLEAR_DISPLAY);
    g_state = TIMER;
    timer_input();
    //if the timer was on and the button was pressed it will reset
    Time1A_init();
  }
}

void start_stopwatch(){
  LCD_setcursorRowCol(0,0);
  LCD_printString("STOPWATCH");
  /*
  LCD clear display command was not used here as
  it takes more time refreshing LCD instead we make the LCD
  write on the same bytes to be more effecient
  */ 
  LCD_setcursorRowCol(1,0);
  //Conditions to manage sec, min and hour
  if(sec >=60){
    sec = 0;
    min++;
  }
  if(min>=60){
    min = 0;
    hr ++;
  }
  LCD_printInt(hr);
  LCD_printString(":");
  LCD_printInt(min);
  LCD_printString(":");
  LCD_printInt(sec);
  //Keypad buttons to pasue and resume stopwach
  LCD_printString(" A:OFF");
  LCD_printString(" B:ON");
}

void timer_input(){
  //Global variables must be zeroed as
  // they are used by both timer and stopwatch
  sec=0;
  hr = 0;
  min = 0;
  LCD_printString("TIMER");
  LCD_setcursorRowCol(1,0);
  LCD_printString("00");
  LCD_printString(":");
  LCD_printString("00");
  LCD_printString(":");
  LCD_printString("00");
  delay_ms(500);
  LCD_setcursorRowCol(1,0);
  LCD_command(DISPLAY_ON_CURSOR_BLINK);
  
  /*
  ASCI of zero is subtracted form the user input
  as the keypad driver return char
  */
  
  //Get two digit input from user for hours
  uint8 hr_key = keypad_getkey();
  LCD_data(hr_key);
  hr = (hr_key - '0')*10;
  delay_ms(800);
  hr_key = keypad_getkey();
  LCD_data(hr_key);
  hr = hr + (hr_key - '0');
  LCD_setcursorRowCol(1,3);
  
  //Get two digit input from user for minutes
  delay_ms(800);
  uint8 min_key = keypad_getkey();
  LCD_data(min_key);
  min = (min_key - '0')*10;
  delay_ms(800);
  min_key = keypad_getkey();
  LCD_data(min_key);
  min = min + (min_key - '0');
  LCD_setcursorRowCol(1,6);
  
  //Get two digit input from user for seconds
  delay_ms(800);
  uint8 sec_key = keypad_getkey();
  LCD_data(sec_key);
  sec = (sec_key - '0')*10;
  delay_ms(800);
  sec_key = keypad_getkey();
  LCD_data(sec_key);
  sec = sec + (sec_key - '0');
  delay_ms(1000);
}

void Time1A_init(void)
{
  SYSCTL_RCGCTIMER_R = 0; //disable all timers
  SYSCTL_RCGCTIMER_R |= (1<<1);  /*enable clock Timer1 subtimer A in run mode */
  TIMER1_CTL_R = 0; /* disable timer1 output */
  TIMER1_CFG_R = 0x4; /*select 16-bit configuration option */
  TIMER1_TAMR_R = 0x02; /*select periodic down counter mode of timer1 */
  TIMER1_TAPR_R = 250; /* TimerA prescaler value */
  TIMER1_TAILR_R = 64000 ; /* TimerA counter starting count down value  */
  TIMER1_ICR_R = 0x1;          /* TimerA timeout flag bit clears*/
  TIMER1_IMR_R |=(1<<0); /*enables TimerA time-out  interrupt mask */
  TIMER1_CTL_R |= 0x01;        /* Enable TimerA module */
  NVIC_PRI4_R |=0X80000;          
  NVIC_EN0_R |= (1<<21);
}

void Timer1A_Handler(){
  if(TIMER1_MIS_R & 0x1){ 
    sec--;//decrese time every one sec
    TIMER1_ICR_R = 0x1;          /* Timer1A timeout flag bit clears*/
    
    //Manage LCD every one sec to be more efffecint and clear LCD view
    timer_handle();
  }
}
void timer_handle(void){
  //Timer end condition
  if(min <= 0 && sec <= 0 && hr <=0){
    TIMER1_CTL_R = 0; /* disable timer1 output */
    LCD_command(CLEAR_DISPLAY);
    LCD_printString("TIMER FINISHED");
    GPIO_PORTF_DATA_R |= (1<<2);//Buzzer ON
    delay_ms(3000);
    GPIO_PORTF_DATA_R &=~(1<<2) ;//Buzzer OFF
    LCD_command(CLEAR_DISPLAY);
    timer_input(); //Repeat Calculator
    Time1A_init();
  }
  
  //conditons to handle time variables
  if(sec <= 0){
    sec = 60;
    min--;
    if(min <= 0){
      min = 60;
      hr--;
    }
  }
  
  /*
  LCD clear display command was not used here as
  it takes more time refreshing LCD instead we make the LCD
  write on the same bytes to be more effecient
  */ 
  LCD_setcursorRowCol(0,0);
  LCD_printString("TIMER");
  LCD_setcursorRowCol(1,0);
  LCD_printInt(hr);
  LCD_printString(":");
  LCD_printInt(min);
  LCD_printString(":");
  LCD_printInt(sec);
}