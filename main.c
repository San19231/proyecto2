/*
 * File:   main.c
 * Proyecto#2 Programación de Microcontroladores 
 * Author: Edgar SAndoval
 *
 * Created on May 11, 2023, 7:03 PM
 */

#pragma config FOSC = INTRC_CLKOUT// Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF      // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF    // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF

//config 2 
#pragma config BOR4V =  BOR40V
#pragma config WRT = OFF
#define _XTAL_FREQ 4000000
#define _tmr0_value 176
#define adressEEPROM 0x00
#include <xc.h>
// VARIABLES
uint8_t VAL;
uint8_t VAL1;
uint8_t VAL2;
uint8_t VAL3;
uint8_t PWM1;
uint8_t PWM2;
uint8_t POT1;
uint8_t POT2;
uint8_t val1;
uint8_t val2;
uint8_t val3;
uint8_t val4;
uint8_t VALOR = 0;
uint8_t VALOR1;
uint8_t VALOR2;
uint8_t FLAG;
uint8_t OP;
unsigned char I[72]=" \n Este es el modo serial, presione un numero para continuar \n";
unsigned char R[63]=" \n Que motor desea mover? \n 1) \n 2) \n 3) \n 4) \n ";
unsigned char M[22]="\n Numero entre 0 y 9 \n";
// funciones
void setup(void);
void switcheo(void);
void write(uint8_t data,uint8_t address);
uint8_t read(uint8_t address);
void UART(void);
void INS(void);
void otr(void);
void mssg(void);
void MTMR0(void);

//Interrupciones
void __interrupt() isr(){
  
   //ADC
    if(PIR1bits.ADIF==1){
        switch(ADCON0bits.CHS){
            //adresh igual al adc
            case 0:
                VAL= ADRESH;
                break;
            case 1:
                VAL1= ADRESH;
                break;
            case 2:
                VAL2= ADRESH;
                break;
            case 3:
                VAL3= ADRESH;
                break;  
        }
        PIR1bits.ADIF =0;
    }
    //TMR0
    if(INTCONbits.T0IF==1){
        PWM1++;
        if(PWM1 <= POT1){
            PORTCbits.RC3=1;
        }else
        {
            PORTCbits.RC3=0;
        }
        if(PWM1 <= POT2){
            PORTCbits.RC4=1;
        }else{
            PORTCbits.RC4=0;
        }
        if(PWM1 >=250){//SI SE CUMPLEN 20 MS AL REINICIAR LA VARIABLE
            PWM1=0;
        }
        TMR0 = _tmr0_value;// reinicar tmr0
        INTCONbits.T0IF=0;            
    }
    //PUERTOB
    if(INTCONbits.RBIF==1)// interrupt on change flag
    {
            PORTDbits.RD0=0;
            PORTDbits.RD1=0;
            PORTDbits.RD2=0;
        if(PORTBbits.RB2==0){// push button en B2 , modo UART
            PORTDbits.RD0=1;
            PORTDbits.RD1=1;
            PORTDbits.RD2=0;
                    
            FLAG=1;
            while(FLAG==1){
                TXSTAbits.TXEN=1;
                UART();
                FLAG=0;
            }
            TXSTAbits.TXEN=0;
             ADCON0bits.ADON=0;
      
            val1= read(0x10);
            val2= read(0x11);
            val3= read(0x12);
            val4= read(0x13);
            
            CCPR1L= val1;
            CCPR2L= val2;
            POT1= val3;
            POT2= val4;
            __delay_ms(3000);
          ADCON0bits.ADON=1;
        }
        if(PORTBbits.RB0==0){// pushbutton b0 modo lectura eeprom 
            PORTDbits.RD0=0;
            PORTDbits.RD1=1;
            PORTDbits.RD2=1;
            //guardar en la eeprom valores
            write(VALOR1,0x10);
            write(VALOR2,0x11);
            write(POT1, 0x12);
            write(POT2, 0x13);
            __delay_ms(5000);
        }
        if(PORTBbits.RB1==0){// PUSHBUTTON b1 modo escritura eeprom
            ADCON0bits.ADON=0;
            PORTDbits.RD0=1;
            PORTDbits.RD1=0;
            PORTDbits.RD2=1;
            //LEER VALORES EEPROM
            val1= read(0x10);
            val2= read(0x11);
            val3= read(0x12);
            val4= read(0x13);
            
            CCPR1L= val1;
            CCPR2L= val2;
            POT1= val3;
            POT2= val4;
            __delay_ms(3000);
          ADCON0bits.ADON=1;
          
        }
        if(PORTBbits.RB3==0){
            FLAG=0;
             ADCON0bits.ADON=0;
      
            val1= read(0x10);
            val2= read(0x11);
            val3= read(0x12);
            val4= read(0x13);
            
            CCPR1L= val1;
            CCPR2L= val2;
            POT1= val3;
            POT2= val4;
            __delay_ms(3000);
          ADCON0bits.ADON=1;
        }
        INTCONbits.RBIF=0;
    }PIR1bits.TMR2IF=0;
}
/// Configuraciones
void setup(void){
    //puertos
    ANSEL = 0B00011111;
    ANSELH= 0x00;
    TRISA=  0b00011111;
    TRISBbits.TRISB0=1;
    TRISBbits.TRISB1=1;
    TRISBbits.TRISB2=1;
    TRISBbits.TRISB3=1;
    TRISC=  0b10000000;
    TRISD=  0b00;
    PORTA=  0X00;
    PORTB=  0X00;
    PORTC=  0X00;
    PORTD=  0X00;
    // PUERTO B PULL UPS
    IOCB= 0XFF;
    OPTION_REGbits.nRBPU=0;//habilitar pull ups
    WPUB= 0B00001111; //PINES 0 AL 3
    // TIMER 0
    OPTION_REG= 0b00001000;
    TMR0= _tmr0_value ;// inicializar tmr0
    INTCONbits.GIE=1;
    INTCONbits.PEIE=1;
    INTCONbits.T0IE=1;// interrupcion timer0
    INTCONbits.T0IF=0;
    INTCONbits.RBIE=1;// change on interrupt puerto b
    INTCONbits.RBIF=0; // bandera portb
    //oscilador
    OSCCONbits.SCS=1;//oscilador interno
    //config a 4mhz:
    OSCCONbits.IRCF2=1;
    OSCCONbits.IRCF1=1;
    OSCCONbits.IRCF0=0;
    PIR1bits.TMR2IF=0;
    T2CON= 0X26; //PRESCALER: 16 POSTSCALER:5
    //ADC
    ADCON0bits.CHS=0;
    ADCON0bits.CHS=2;
    __delay_us(100);
    PIE1bits.ADIE=1;
    PIR1bits.ADIF=0;
    ADCON0bits.ADON=1;
    ADCON0bits.ADCS=1;
    ADCON1bits.ADFM=0;
    ADCON1bits.VCFG0=0;
    ADCON1bits.VCFG1=0;
    //PWM
    PR2=250;
    CCP1CON= 0B00001100;
    CCP2CON= 0B00001111;
    //UART
    PIR1bits.RCIF=0;
    PIE1bits.RCIE=0;
    PIE1bits.TXIE=0;
    TXSTAbits.TX9=0;
    TXSTAbits.TXEN=1;
    TXSTAbits.SYNC=0;
    TXSTAbits.BRGH=1;
    RCSTAbits.RX9=0;
    RCSTAbits.CREN=1;
    RCSTAbits.SPEN=1;
    BAUDCTLbits.BRG16=0;
    SPBRG=25;
    SPBRGH=1;      
}
void main(void) {
    setup();
    while(1){
        switcheo();
    }
}
//funciones 
void UART(void){
    __delay_ms(100);
    VALOR=0;
    do{
        VALOR++;
        TXREG=I[VALOR];
        __delay_ms(50);
        
    }while(VALOR<=72);
    while(RCIF==0);
    INS();
}
void switcheo(){
    if(ADCON0bits.GO==0){
        switch(ADCON0bits.CHS){
            case 0:
                CCPR1L = ((0.247*VAL)+62);
                VALOR1 = CCPR1L;
                ADCON0bits.CHS=1;
                __delay_us(100);
                ADCON0bits.GO=1;
                break;
            case 1:
                POT2=((0.049*VAL1)+7);
                ADCON0bits.CHS=2;
                __delay_us(250);
                ADCON0bits.GO=1;
                break;
            case 2:
                CCPR2L=((0.247*VAL2)+62);
                VALOR2= CCPR2L;
                ADCON0bits.CHS=3;
                __delay_us(100);
                ADCON0bits.GO=1;
                break;
            case 3:
                 POT1=((0.049*VAL3)+7);
                ADCON0bits.CHS=0;
                __delay_us(250);
                ADCON0bits.GO=1;
                break;
            default:
                break;
                
        }
    }
}
void write(uint8_t data, uint8_t address){
    EEADR= address;
    EEDAT= data;
    EECON1bits.EEPGD=0;
    EECON1bits.WREN=1;
    INTCONbits.GIE=0;
    EECON2 = 0X55;
    EECON2 = 0XAA;
    EECON1bits.WR=1;
    while(PIR2bits.EEIF==0);
    PIR2bits.EEIF=0;
    EECON1bits.WREN=0;
    INTCONbits.GIE=0;
}
uint8_t read(uint8_t address){
    EEADR= address;
    EECON1bits.EEPGD=0;// apuntador
    EECON1bits.RD=1;// leer
    uint8_t data= EEDATA;
    return data;
}
void INS(void){
    OP=RCREG;
    switch(OP){
        case 49:
            __delay_ms(300);
            VALOR=0;
            do{
                VALOR++;
                TXREG=R[VALOR];
                __delay_ms(50);
                
            }while(VALOR<=60);
            while(RCIF==0);
            OP=0;
           otr();
            break;
        case 50:
            TXSTAbits.TXEN=0;
            OP=0;
            break;
            
    }
}
void otr(void){
    OP= RCREG;
    switch(OP){
        case 49:
            mssg();
            if(RCREG>=48 && RCREG <=57){
                VAL= RCREG;
                switcheo();
            }
            break;
        case 50:
            mssg();
            if(RCREG>=48 && RCREG<=57){
                VAL1=RCREG;
                switcheo();
            }
            break;
        case 51:
            mssg();
            if(RCREG>=48 && RCREG<=57){
                VAL2=RCREG;
                switcheo();
            }
            break;
        case 52:
            mssg();
            if(RCREG>=48 && RCREG<=57){
                VAL3=RCREG;
                switcheo();
            }
            break;
        
            
    }
}
void mssg(void){
    __delay_ms(500);
    VALOR=0;
    do{
        VALOR++;
        TXREG=M[VALOR];
        __delay_ms(50);
    }while(VALOR<=22);
    while(RCIF==0){
        OP=0;
    }
}