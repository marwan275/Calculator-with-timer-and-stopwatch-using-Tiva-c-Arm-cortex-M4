# Calculator-with-timer-and-stopwatch-using-Tiva-c-Arm-cortex-M4-
We were assigned to design a simple calculator with a timer and a stopwatch as extra features.
Where the calculator mode shall do the basic operations that are addition, subtraction, multiplication, and division.
In addition to, the timer and stopwatch features both will be handled separately by the hardware. 

Starting with the calculator mode, inputs are entered by the user and printed on the LCD. The keypad letters will be associated as the operator signs: 
A: ‘+’ 
B: ‘-’ 
C: ‘/’ 
D: ‘*’ 

A push button in tiva board will be used to switch between the calculator mode to the timer mode using interrupt. 
In this mode, the user will set a time using a keypad, the timer will start counting down as soon it reaches the time zero it will trigger a buzzer. 
Another push button in tiva board will switch to the last mode using interrupt, the stopwatch. 
The user will be using three buttons, one to start the stopwatch, one to pause the stopwatch and one to rest the stopwatch back to 00:00.
It will initially begin with 00:00 and as soon you push the start button, the stopwatch will start incrementing.  

In this project we used a keypad as an input for all the numbers, an LCD as a user interface to print out the inputs and the results,
and a buzzer which will be triggered once the timer reach zero. 
