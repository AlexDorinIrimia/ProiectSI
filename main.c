/*
 * USART2 is connected to the ST-Link virtual COM port.
 * Use Tera Term to interract with STM board
 *
 * By default, the clock is running at 16 MHz.
 * The UART2 is configured for 115200 Baud.
 * PA2 - USART2 TX (AF7)
 * PA3 - USART2 RX (AF7)
 */
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

#include "Types.h"
#include "Drivers.h"

//----------------------------------------
#define RS 1    /* BIT0 mask for reg select */
#define EN 2    /* BIT1 mask for E */
#define CORRECT_PASSWORD 1234



//-------------------------------------------

int corect[4]={2,2,2,2};
int tries=3;
int key=0;
int count=0;
int k=0;



void delay(void);
void delayMs(int);
void LCD_nibble_write(char data, unsigned char control);
void LCD_command(unsigned char command);
void LCD_data(char data);
void LCD_init(void);
void SPI1_write(unsigned char data);
int readRows(void);
int getPasswordFromKeypad(void);
void outputEnableCols(char n);
void writeCols(char n);
void writeLEDs(char n);
void keypad_init(void);
char keypad_getkey(void);
void writeStringLCD(char *line);
void newLine(unsigned int size);
void stopProgram();
int len =4;
U8 TimerCountDown_U16 = SECONDS_TO_COUNT_U8;
U8 PrintInCycleMode_U8 = 0;

void PeriphInit(void)
{

	__disable_irq();
	// Configure PB[7..4] as output
	RCC->AHB1ENR |= 0;			 /* Enable GPIOB clock */
	GPIOB->MODER &= 0; /* Reset GPIOB PB[7..4]  */
	GPIOB->MODER |= 0;	 /* Set GPIOB PB[7..4]  as ouput */

	// Configure PC[11..8] as input
	RCC->AHB1ENR |= 0;			 /* Enable GPIOC clock */
	GPIOC->MODER &= 0; /* Reset GPIOC PC[11..8]  for input mode */

	// Configure PB[15..12] port as input and enables pull-ups
	GPIOB->MODER &= 0; /* Reset GPIOB PB[15..12]  */
	GPIOB->PUPDR |= 0;	 /* Enable pull-ups on GPIOB PB[15..12]  */

	USART2_init();

	__enable_irq();
}


void citire()
{
 while(k<4)
    {
    while((key = keypad_getkey()) == 0);
    
        if(key != corect[k])
        {
             
        }
        else
        {
            ++count;
        }
        ++k;
        while(keypad_getkey() != 0);
    }
}

int main(void)
{
	U8 status = 0;

	PeriphInit();
	
  keypad_init();
	LCD_init();
	RCC->AHB1ENR |=  2;             /* enable GPIOB clock */
  GPIOB->MODER &= ~0x0000ff00;    /* clear pin mode */
  GPIOB->MODER |=  0x00005500;    /* set pins to output mode */
	
		
		
		while(1) 
			{
		LCD_data('P');
		LCD_data('A');
		LCD_data('S');
		LCD_data('S');
		LCD_data('W');
		LCD_data('O');
		LCD_data('R');
		LCD_data('D');
		LCD_data(':');
		delayMs(500);
				if(tries>0)
				{	
					citire();
					
					if(count==4)
			
					{
						 /* clear LCD display */
						LCD_command(1);
						delayMs(500);
					
						//writeLEDs(0x0); 
						LCD_data('U');
						LCD_data('N');
						LCD_data('L');
						LCD_data('O');
						LCD_data('C');
						LCD_data('K');
						LCD_data('E');
						LCD_data('D');
						LCD_data('!');
						k=0;
					  count=0;
						break;
						//while(1);
					}
					else
					{
						 /* clear LCD display */
						LCD_command(1);
						delayMs(500);
						
						--tries;
						int f = tries + 48;
						LCD_data('W');
						LCD_data('R');
						LCD_data('O');
						LCD_data('N');
						LCD_data('G');
						LCD_data(' ');
						LCD_data(f);
						writeLEDs(0xF);
						k=0;
					  count=0;
						delayMs(500);
						LCD_command(1);
					}
			}
		else
				{
				while(1){
				LCD_command(1);
				delayMs(500);
				LCD_data('L');
        LCD_data('O');
				LCD_data('C');
				LCD_data('K');
				LCD_data('E');
				LCD_data('D');
				LCD_data('!');
				delayMs(5000);
				LCD_command(1);
				}
				}
	}
		
}
//-----------------------------------------------------------------------------

/* configure SPI1 and the associated GPIO pins */
void LCD_init(void) {
    RCC->AHB1ENR |= 1;              /* enable GPIOA clock */
    RCC->AHB1ENR |= 4;              /* enable GPIOC clock */
    RCC->APB2ENR |= 0x1000;         /* enable SPI1 clock */

    /* PORTA 5, 7 for SPI1 MOSI and SCLK */
    GPIOA->MODER &= ~0x0000CC00;    /* clear pin mode */
    GPIOA->MODER |=  0x00008800;    /* set pin alternate mode */
    GPIOA->AFR[0] &= ~0xF0F00000;   /* clear alt mode */
    GPIOA->AFR[0] |=  0x50500000;   /* set alt mode SPI1 */

    /* PA12 as GPIO output for SPI slave select */
    GPIOA->MODER &= ~0x03000000;    /* clear pin mode */
    GPIOA->MODER |=  0x01000000;    /* set pin output mode */

    /* initialize SPI1 module */
    SPI1->CR1 = 0x31F;
    SPI1->CR2 = 0;
    SPI1->CR1 |= 0x40;              /* enable SPI1 module */

    /* LCD controller reset sequence */
    delayMs(20);
    LCD_nibble_write(0x30, 0);
    delayMs(5);
    LCD_nibble_write(0x30, 0);
    delayMs(1);
    LCD_nibble_write(0x30, 0);
    delayMs(1);
    LCD_nibble_write(0x20, 0);  /* use 4-bit data mode */
    delayMs(1);
    LCD_command(0x28);          /* set 4-bit data, 2-line, 5x7 font */
    LCD_command(0x06);          /* move cursor right */
    LCD_command(0x01);          /* clear screen, move cursor to home */
    LCD_command(0x0F);          /* turn on display, cursor blinking */
}

void LCD_nibble_write(char data, unsigned char control) {
    data &= 0xF0;       /* clear lower nibble for control */
    control &= 0x0F;    /* clear upper nibble for data */
    SPI1_write (data | control);           /* RS = 0, R/W = 0 */
    SPI1_write (data | control | EN);      /* pulse E */
    delayMs(0);
    SPI1_write (data);
}

void LCD_command(unsigned char command) {
    LCD_nibble_write(command & 0xF0, 0);    /* upper nibble first */
    LCD_nibble_write(command << 4, 0);      /* then lower nibble */

    if (command < 4)
        delayMs(2);         /* command 1 and 2 needs up to 1.64ms */
    else
        delayMs(1);         /* all others 40 us */
}

void LCD_data(char data) {
    LCD_nibble_write(data & 0xF0, RS);      /* upper nibble first */
    LCD_nibble_write(data << 4, RS);        /* then lower nibble */

    delayMs(1);
}

/* This function enables slave select, writes one byte to SPI1, */
/* wait for transmit complete and deassert slave select. */
void SPI1_write(unsigned char data) {
    while (!(SPI1->SR & 2)) {}      /* wait until Transfer buffer Empty */
    GPIOA->BSRR = 0x10000000;       /* assert slave select */
    SPI1->DR = data;                /* write data */
    while (SPI1->SR & 0x80) {}      /* wait for transmission done */
    GPIOA->BSRR = 0x00001000;       /* deassert slave select */
}

/* 16 MHz SYSCLK */
void delayMs(int n) {
    int i;
    for (; n > 0; n--)
        for (i = 0; i < 3195; i++) ;
}


void writeStringLCD(char *line) {
    for(unsigned int i=0; i < strlen(line); i++)
    {
        LCD_data(line[i]);
    }
}

void newLine(unsigned int size){
    for(unsigned int i=0; i < 40-size; i++)
    {
        LCD_data(' ');
    }
}

char keypad_getkey(void)
{
    int row, col;

    /* check to see any key is pressed first */
    outputEnableCols(0xF);      /* enable all columns */
    writeCols(0xF);             /* and drive them high */
    delay();                    /* wait for signal to settle */
    row = readRows();           /* read all rows */
    writeCols(0x0);             /* discharge all columns */
    outputEnableCols(0x0);      /* disable all columns */
    if (row == 0) return 0;     /* if no key pressed, return a zero */

    /* If a key is pressed, it gets here to find out which key.
     * It activates one column at a time and read the rows to see
     * which is active.
     */
    for (col = 0; col < 4; col++) {
        outputEnableCols(1 << col); /* enable one column */
        writeCols(1 << col);        /* turn the active row high */
        delay();                    /* wait for signal to settle */
        row = readRows();           /* read all rows */
        writeCols(0x0);             /* discharge all columns */
        if (row != 0) break;        /* if one of the row is low, some key is pressed. */
    }

    outputEnableCols(0x0);          /* disable all columns */
    if (col == 4)
        return 0;                   /* if we get here, no key is pressed */

    /* gets here when one of the rows has key pressed.
     * generate a unique key code and return it.
     */
    if (row == 0x01) {return 0 + col;}    // key in row 0
    if (row == 0x02) {return 4 + col;  }  // key in row 1
    if (row == 0x04) {return 8 + col;   } // key in row 2
    if (row == 0x08) {return 12 + col;   }// key in row 3

    return 0;   /* just to be safe */
}

/* enable columns according to bit 3-0 of the parameter n */
void outputEnableCols(char n) {
    GPIOB->MODER &= ~0xFF000000;    /* clear pin mode */

    /* make the pin output according to n */
	if (n & 1)
		GPIOB->MODER |=  0x01000000;
    if (n & 2)
		GPIOB->MODER |=  0x04000000;
	if (n & 4)
		GPIOB->MODER |=  0x10000000;
	if (n & 1 << 3)
		GPIOB->MODER |=  0x40000000;
}

/* write columns high or low according to bit 3-0 of the parameter n */
void writeCols(char n) {
    GPIOB->BSRR = 0xF0000000;   // turn off all column pins
    GPIOB->BSRR = n << 12;      // turn on column pins
}

/* read rows and return them in bit 3-0 */
int readRows(void) {
	return (GPIOC->IDR & 0x0F00) >> 8;
}





void writeLEDs(char n) {
    GPIOB->BSRR = 0x00F00000;   // turn off all LEDs
    GPIOB->BSRR = n << 4;       // turn on LEDs
}

/* system clock at 16 MHz delay about 100 us */
void delay(void) {
	int j;

	for (j = 0; j < 300; j++)
		;      /* do nothing */
}

/* This function intializes the pins connected to the keypad. */
void keypad_init(void) {
	/* make rows input first */
    RCC->AHB1ENR |=  4;             /* enable GPIOC clock */
    GPIOC->MODER &= ~0x00FF0000;    /* clear pin mode */

	/* make columns input */
    RCC->AHB1ENR |=  2;             /* enable GPIOB clock */
    GPIOB->MODER &= ~0xFF000000;    /* clear pin mode */
}

//-----------------------------------------------------------------------------

/* ISRs */
void TIM2_IRQHandler(void)
{
	TIM2->SR = 0; /* clear UIF */
	TimerCountDown_U16--;
	PrintInCycleMode_U8 = 1; /* time to print */
}


void SysTick_Handler(void)
{
	GPIOB->ODR ^= 0x00000020;
}
