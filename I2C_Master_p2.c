/*
 * File:   I2C_Master_p2.c
 * Author: lisan
 *
 * Created on 27 de mayo de 2022, 11:00 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 8000000
#define I2C_SPEED 100000
#define READ 0b0
#define WRITE 0b1
#define address_s1 0x08 
#define address_s2 0x09
#define address_s3 0x0A
#define address_s4 0x0B

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t data = 0, response = 0; 
uint8_t val_pot1, val_pot2, val_pot3, val_pot4; 
uint8_t modo = 1;               // El modo comienza por Default con el modo manual (ADC y Guardar posición)
uint8_t Entrada;                // Variable que recibe el dato de PC a PIC

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
//Prototipo de funcion de configuraciones
void setup(void);

//EEPROM
uint8_t read_EEPROM(uint8_t address);               //Leemos
void write_EEPROM(uint8_t address, uint8_t data);   //Escribimos

//Deshabilitación de interrupciones
void interupcion_NAN(void);     //Deshabilitamaos la interrupción de ADC
void int_Ninterfaz(void);       //Deshabilitamos la interrupción de interfaz

//Prototipo de función para la interfaz
void enviar_letra(char letra);
void print(unsigned char palabra);
void menusito(void);

//Prototipo para la I2C
void wait_I2C(void);
void start_I2C(void);
void restart_I2C(void);
void stop_I2C(void);
void send_ACK(void);
void send_NACK(void);
__bit write_I2C(uint8_t data);
uint8_t read_I2C(void);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr(void){
    if(INTCONbits.RBIF){                            // Fue interrupción del PORTB, entonces:
        if(!PORTBbits.RB0){                         // cambia modo
            if(modo == 0b00000001){                 // manual -> reproducción
                modo = 0b00000010;
            }
            else if(modo == 0b00000010){            // reproducción -> interfaz
                modo = 0b00000100;
            }
            else if(modo == 0b00000100){            // interfaz -> manual
                modo = 0b00000001;
            }
        }                               // Mostramos el modo en el puerto D    
        INTCONbits.RBIF = 0;                        // Limpiamos bandera  
    }
    if(modo == 0b00000001){                             // Modo Manual
        if(PIR1bits.ADIF){                   // BANDERA = ON --> SIGO ADELANTE
    
            if(ADCON0bits.CHS == 0)         // ELIJIÓ EL CH0?
            {
                val_pot1 = ADRESH;
                CCPR1L = (val_pot1>>1)+120;
                CCP1CONbits.DC1B1 = val_pot1 & 0b01;
                CCP1CONbits.DC1B0 = (val_pot1>>7);
            }else if(ADCON0bits.CHS == 1)   // ELIJIÓ EL CH1?
            {
                val_pot2 = ADRESH;
                CCPR2L = (val_pot2>>1)+120;   // VALOR == 124
                CCP1CONbits.DC1B1 = val_pot2 & 0b01;
                CCP1CONbits.DC1B0 = (val_pot2>>7);
            }else if(ADCON0bits.CHS == 2)   // ELIJIÓ EL CH2?
            {
                val_pot3 = (ADRESH & 0b11111110);
               }else if(ADCON0bits.CHS == 3)   // ELIJIÓ EL CH3?
            {
                val_pot4 = ((ADRESH & 0b11111110) | 0b00000001);
            }
            PIR1bits.ADIF = 0;
        }
        if(INTCONbits.RBIF){             // Fue interrupción del PORTB, entonces:
            if(!PORTBbits.RB1){          // Guarda el valor del servo1 
                write_EEPROM(address_s1, val_pot1);     //Escribimos el valor del potenciometro en la dirección 
            }
            if(!PORTBbits.RB2){          // Guarda el valor del servo2 
                write_EEPROM(address_s2, val_pot2);     //Escribimos el valor del potenciometro en la dirección 
            }
            INTCONbits.RBIF = 0;           // Limpiamos bandera  
        }
    }   
    
    if(modo == 0b00000010){                             // Modo Reproducción
        if(INTCONbits.RBIF){             // Fue interrupción del PORTB, entonces:
            if(!PORTBbits.RB1){          // Guarda el valor del servo1 
                val_pot1 = read_EEPROM(address_s1);     //Escribimos el valor del potenciometro en la dirección 
            }
            if(!PORTBbits.RB2){          // Guarda el valor del servo2 
                val_pot2 = read_EEPROM(address_s2);     //Escribimos el valor del potenciometro en la dirección 
            }
            INTCONbits.RBIF = 0;           // Limpiamos bandera  
        }
    }
    
    if(modo == 0b00000100){              //Modo interfaz
        if(RCIF){                        // Si recibimos datos, entonces:
            Entrada = RCREG;             // Guardamos el valor recibido
            RCREG = 0;                   // RCREG lo seteamos
        }
    }    
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){ 
        PORTD = modo;
        if(modo == 0b00000001){         //Si es modo manual, entonces:
            if(GO == 0){    
                if(ADCON0bits.CHS == 0){    //CH0 -> CH1
                    ADCON0bits.CHS = 1;
                }
                if(ADCON0bits.CHS == 1){    //CH1 -> CH2
                    ADCON0bits.CHS = 2;
                }
                if(ADCON0bits.CHS == 2){    //CH2 -> CH3
                    ADCON0bits.CHS = 3;
                }
                if(ADCON0bits.CHS == 3){    //CH3 -> CH0
                    ADCON0bits.CHS = 0;
                }
                GO =1;
            }    
            
            if(!PORTBbits.RB1){          // Guarda el valor del servo1 
                __delay_ms(10);          // Antirrebote
                write_EEPROM(address_s1, val_pot1);     //Escribimos el valor del potenciometro1 en la dirección1 
            }
            if(!PORTBbits.RB2){          // Guarda el valor del servo2 
                __delay_ms(10);          // Antirrebote
                write_EEPROM(address_s2, val_pot2);     //Escribimos el valor del potenciometro2 en la dirección2 
            }
            
            //COMUNICACIÓN MAESTRO -> ESCLAVO EN I2C:
           
            //ENVIAMOS EL VALOR DEL POTENCIOMETRO 3
            data = (uint8_t)((address_s3<<1)+READ);
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(val_pot3);        // Enviamos dato del 3er potenciometro al esclavo 
            stop_I2C();
            
            //ENVIAMOS EL VALOR DEL POTENCIOMETRO 4
            data = (uint8_t)((address_s4<<1)+READ);
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(val_pot4);        // Enviamos dato del 4to potenciometro al esclavo
            stop_I2C();                 //STOP Y VOLVEMOS A COMUNICAR PARA ENVIAR OTRO DATO
            }
        
        else if(modo == 0b00000010){         // Si es modo reproducción, entonces:
            /*interupcion_NAN();          // Deshabilitamos interrupción ADC
            int_Ninterfaz();            // Deshabilitamos interrupción del EUSART*/
            if(!PORTBbits.RB1){          // Guarda el valor del servo1 
                __delay_ms(10);          // Antirrebote
                val_pot1 = read_EEPROM(address_s1);     // Leemos el valor del potenciometro de la dirección
            }
            if(!PORTBbits.RB2){          // Guarda el valor del servo2 
                __delay_ms(10);          // Antirrebote
                val_pot2 = read_EEPROM(address_s2);     // Leemos el valor del potenciometro de la dirección
            }
            val_pot4 = (val_pot4 & 0b11111110 | 0b00000001); //codificación 01
            val_pot3 = (val_pot3 & 0b11111110 );             //codificación 00
            //ENVIAMOS EL VALOR DEL POTENCIOMETRO 3
            data = (uint8_t)((address_s3<<1)+READ);
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(val_pot3);        // Enviamos dato del 3er potenciometro al esclavo 
            stop_I2C();
            
            //ENVIAMOS EL VALOR DEL POTENCIOMETRO 4
            data = (uint8_t)((address_s4<<1)+READ);
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(val_pot4);        // Enviamos dato del 4to potenciometro al esclavo
            stop_I2C();                 //STOP Y VOLVEMOS A COMUNICAR PARA ENVIAR OTRO DATO
        }
        
        else if(modo == 0b00000100){    // Modo interfaz (no reproduce, ni guarda, solo potenciometros de la interfaz)
            /*
            TMRT para TSR 1 = Vacio y TSR 0 = Ocupado
            TXIF para TXREG 1 = Vacio y TXREG 0 = ocupado
            */
            if(Entrada == 0b00)         // Si el valor recibido es XXXX XX00, entonces:
            {
            print("1");  
            print("");
            val_pot1 = Entrada;                       // Entrada guarda el valor del potenciometro 
            print(val_pot1);
            CCPR1L = (val_pot1>>1)+120;
            CCP1CONbits.DC1B1 = val_pot1 & 0b01;
            CCP1CONbits.DC1B0 = (val_pot1>>7);
            menusito();                               // Muestra en la terminal de pregunta

            }    
            else if(Entrada == 0b01){    // Si el valor recibido es XXXX XX01, entonces:
            print("2");  
            print("");
            val_pot2 = Entrada;                       // Entrada guarda el valor del potenciometro 
            CCPR2L = (val_pot2>>1)+120;   // VALOR == 124
            CCP1CONbits.DC1B1 = val_pot2 & 0b01;
            CCP1CONbits.DC1B0 = (val_pot2>>7);
            menusito();                               // Muestra la terminal de pregunta
            }
            else if(Entrada == 0b10){    // Si el valor recibido es XXXX XX10, entonces:
            print("3");  
            print("");
            val_pot3 = Entrada;                       // Entrada guarda el valor del potenciometro 
            val_pot3 = (uint8_t)(val_pot3 & 0b11111110);
            //ENVIAMOS EL VALOR DEL POTENCIOMETRO 3
            data = (uint8_t)((address_s3<<1)+READ);
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(val_pot3);        // Enviamos dato del 3er potenciometro al esclavo 
            stop_I2C();
            menusito();                               // Muestra la terminal de pregunta
            }
            else if(Entrada == 0b11){    // Si el valor recibido es XXXX XX11, entonces:
            print("4");  
            print("");
            val_pot4 = Entrada;                       // Entrada guarda el valor del potenciometro 
            val_pot4 = (uint8_t)(val_pot4 & 0b11111110 | 0b00000001);
            //ENVIAMOS EL VALOR EL POTENCIOMETRO 4
            data = (uint8_t)((address_s4<<1)+READ);
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(val_pot4);        // Enviamos dato del 4to potenciometro al esclavo
            stop_I2C();                 //STOP Y FINALIZAMOS LA COMUNICACIÓN
            menusito();                               // Muestra la terminal de pregunta
            }
        }
        else{
            stop_I2C();                 // Finalizamos la comunicación
            PORTD = 0b111;              // Error no hay ningun modo
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
//---------------------- CONFIGURACIÓN MODO MANUAL -----------------------------//    
if(modo == 0b00000001){
//Config entradas y salidas
    ANSEL = 0x0F;               // AN0, AN1, AN2 Y AN3 ENTRADAS ANALÓGICAS
    ANSELH = 0;                 // I/O digitales
    
    TRISA = 0x0F;               // los nibble bajo son entradas.
    TRISB = 0b00000111;         // ENTRADA -> PULSADORES (RB0,RB1,RB2)    
    TRISC = 0;                  // Salida
    TRISD = 0;                  // Salida
    TRISE = 0;                  // Salida
    PORTA = 0;                  // SETEAMOS ALL PORTs 
    PORTB = 0;
    PORTC = 0;
    PORTD = 0b00000001;
    PORTE = 0;
    
//Config Reloj
    OSCCONbits.SCS  = 1;        // Reloj interno
    OSCCONbits.IRCF = 0b111;    // 8MHz
    
//Config IOCB
    OPTION_REGbits.nRBPU = 0;
    WPUB = 0b00000111;
    IOCB = 0b00000111;
    
//Config ADC
    ADCON0bits.ADCS = 0b11;         // FRC
    ADCON1bits.VCFG0 = 0;           // Referencia VDD
    ADCON1bits.VCFG1 = 0;           // Referencia VSS
    ADCON0bits.CHS = 0;             // Se selecciona PORTA0/AN0
    ADCON1bits.ADFM = 0;            // Se indica que se tendrá un justificado a la izquierda
    ADCON0bits.ADON = 1;            // Se habilita el modulo ADC
    __delay_us(40);                 // Delay para sample time
    
//Config PWM
    TRISCbits.TRISC2 = 1;           // RC2/CCP1 como salida deshabilitado
    TRISCbits.TRISC1 = 1;           // Se deshabilita salida de PWM (CCP2)
    CCP1CON = 0;                    // Se apaga CCP1
    CCP2CON = 0;                    // Se apaga CCP2
    PR2 = 155;                      // Período de 20 ms  
    
//Config CCP
    CCP1CONbits.P1M = 0;            // Modo single output
    CCP1CONbits.CCP1M = 0b1100;     // Modo PWM
    CCP2CONbits.CCP2M = 0b1100;     // Modo PWM
//Servo 1
    CCPR1L = 30>>2;                 //Ciclo de trabajo base pues se va a variar
    CCP1CONbits.DC1B = 30 & 0b11;       

//Servo 2
    CCPR2L = 30>>2;                 //Ciclo de trabajo base pues se va a variar
    CCP2CONbits.DC2B0 = 30 & 0b01;      
    CCP2CONbits.DC2B1 = (30 & 0b10)>>1; 

//Config TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Se enciende TMR2
    while(!PIR1bits.TMR2IF);    // Se espera un ciclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Se habilita salida de PWM (CCP1)
    TRISCbits.TRISC1 = 0;       // Se habilita salida de PWM (CCP2)

//Config I2C
    SSPADD = ((_XTAL_FREQ)/(4*I2C_SPEED)) - 1;  // 100 kHz
    SSPSTATbits.SMP = 1;        // Velocidad de rotación
    SSPCONbits.SSPM = 0b1000;   // I2C master mode, clock= Fosc/(4*(SSPADD+1))
    SSPCONbits.SSPEN = 1;       // Habilitamos pines de I2C
    PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción de I2C
    
//Interrupciones
    INTCONbits.GIE  = 1;        // Hab. int. generales
    INTCONbits.RBIF = 0;        // Limpiamos bandera de PORTB
    INTCONbits.RBIE = 1;        // Habilitamos int. PORTB
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
}

//-------------------- CONFIGURACIÓN DE REPRODUCCIÓN -----------------------------//
if(modo == 0b00000010){
//Config entradas y salidas
    ANSEL = 0x0F;               // AN0, AN1, AN2 Y AN3 ENTRADAS ANALÓGICAS
    ANSELH = 0;                 // I/O digitales
    
    TRISB = 0b00000111;         // ENTRADA -> PULSADORES (RB0,RB1,RB2)    
    TRISC = 0;                  // Salida
    TRISD = 0;                  // Salida
    TRISE = 0;                  // Salida
    PORTB = 0;
    PORTC = 0;
    PORTD = 0b00000010;
    PORTE = 0;   	// Comienza en modo reproducción de posiciones	
    
//Config Reloj
    OSCCONbits.SCS  = 1;        // Reloj interno
    OSCCONbits.IRCF = 0b111;    // 8MHz
    
//Config IOCB
    OPTION_REGbits.nRBPU = 0;
    WPUB = 0b00000111;
    IOCB = 0b00000111;   
    
//Config ADC
    ADCON0bits.ADCS = 0b11;         // FRC
    ADCON1bits.VCFG0 = 0;           // Referencia VDD
    ADCON1bits.VCFG1 = 0;           // Referencia VSS
    ADCON0bits.CHS = 0;             // Se selecciona PORTA0/AN0
    ADCON1bits.ADFM = 0;            // Se indica que se tendrá un justificado a la izquierda
    ADCON0bits.ADON = 1;            // Se habilita el modulo ADC
    __delay_us(40);                 // Delay para sample time
        
//Config PWM
    TRISCbits.TRISC2 = 1;           // RC2/CCP1 como salida deshabilitado
    TRISCbits.TRISC1 = 1;           // Se deshabilita salida de PWM (CCP2)
    CCP1CON = 0;                    // Se apaga CCP1
    CCP2CON = 0;                    // Se apaga CCP2
    PR2 = 155;                      // Período de 20 ms  
    
//Config CCP
    CCP1CONbits.P1M = 0;            // Modo single output
    CCP1CONbits.CCP1M = 0b1100;     // Modo PWM
    CCP2CONbits.CCP2M = 0b1100;     // Modo PWM
//Servo 1
    CCPR1L = 30>>2;                 //Ciclo de trabajo base pues se va a variar
    CCP1CONbits.DC1B = 30 & 0b11;       

//Servo 2
    CCPR2L = 30>>2;                 //Ciclo de trabajo base pues se va a variar
    CCP2CONbits.DC2B0 = 30 & 0b01;      
    CCP2CONbits.DC2B1 = (30 & 0b10)>>1; 

//Config TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Se enciende TMR2
    while(!PIR1bits.TMR2IF);    // Se espera un ciclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Se habilita salida de PWM (CCP1)
    TRISCbits.TRISC1 = 0;       // Se habilita salida de PWM (CCP2)
//Config I2C
    SSPADD = ((_XTAL_FREQ)/(4*I2C_SPEED)) - 1;  // 100 kHz
    SSPSTATbits.SMP = 1;        // Velocidad de rotación
    SSPCONbits.SSPM = 0b1000;   // I2C master mode, clock= Fosc/(4*(SSPADD+1))
    SSPCONbits.SSPEN = 1;       // Habilitamos pines de I2C
    PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción de I2C
    
//Interrupciones
    INTCONbits.GIE  = 1;        // Hab. int. generales
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.RBIF = 0;        // Limpiamos bandera int. portb
    INTCONbits.RBIE = 1;        // Habilitamos int del PORTB
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
}
//-------------------- CONFIGURACIÓN DE LA INTERFAZ -----------------------------//
if(modo == 0b00000100){
//Config entradas y salidas
    ANSEL = 0x0F;               // AN0, AN1, AN2 Y AN3 ENTRADAS ANALÓGICAS
    ANSELH = 0;                 // I/O digitales
    
    TRISB = 0b00000111;         // ENTRADA -> PULSADORES (RB0,RB1,RB2)    
    TRISC = 0;                  // Salida
    TRISD = 0;                  // Salida
    TRISE = 0;                  // Salida
    PORTB = 0;
    PORTC = 0;
    PORTD = 0b00000100;
    PORTE = 0;   	// Comienza en modo reproducción de posiciones	
    
//Config Reloj
    OSCCONbits.SCS  = 1;        // Reloj interno
    OSCCONbits.IRCF = 0b111;    // 8MHz
    
//Config IOCB
    OPTION_REGbits.nRBPU = 0;
    WPUB = 0b00000111;
    IOCB = 0b00000111;   
    
//Config ADC
    ADCON0bits.ADCS = 0b11;         // FRC
    ADCON1bits.VCFG0 = 0;           // Referencia VDD
    ADCON1bits.VCFG1 = 0;           // Referencia VSS
    ADCON0bits.CHS = 0;             // Se selecciona PORTA0/AN0
    ADCON1bits.ADFM = 0;            // Se indica que se tendrá un justificado a la izquierda
    ADCON0bits.ADON = 1;            // Se habilita el modulo ADC
    __delay_us(40);                 // Delay para sample time
        
//Config PWM
    TRISCbits.TRISC2 = 1;           // RC2/CCP1 como salida deshabilitado
    TRISCbits.TRISC1 = 1;           // Se deshabilita salida de PWM (CCP2)
    CCP1CON = 0;                    // Se apaga CCP1
    CCP2CON = 0;                    // Se apaga CCP2
    PR2 = 155;                      // Período de 20 ms  
    
//Config CCP
    CCP1CONbits.P1M = 0;            // Modo single output
    CCP1CONbits.CCP1M = 0b1100;     // Modo PWM
    CCP2CONbits.CCP2M = 0b1100;     // Modo PWM
//Servo 1
    CCPR1L = 30>>2;                 //Ciclo de trabajo base pues se va a variar
    CCP1CONbits.DC1B = 30 & 0b11;       

//Servo 2
    CCPR2L = 30>>2;                 //Ciclo de trabajo base pues se va a variar
    CCP2CONbits.DC2B0 = 30 & 0b01;      
    CCP2CONbits.DC2B1 = (30 & 0b10)>>1; 

//Config TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Se enciende TMR2
    while(!PIR1bits.TMR2IF);    // Se espera un ciclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Se habilita salida de PWM (CCP1)
    TRISCbits.TRISC1 = 0;       // Se habilita salida de PWM (CCP2)
    
//Config EUSART TRANSMITER
    SPBRGH  =   0;              // Byte Superior 9600 Baud Rate
    SPBRG   =   12;             // Byte inferior
    BRGH    =   1;              // Baud rate a alta velocidad
    BRG16   =   1;              // 16bits para generar el baud rate
    TXSTAbits.SYNC = 0;         // Comunicación Asíncrono (full-duplex))
    
    RCSTAbits.SPEN = 1;         // Habilita EUSART y configura como salida a TX/CK 
    TXSTAbits.TXEN = 1;         // Activa circuito para trasmisor del EUSART
    TXSTAbits.TX9  = 0;         // Usamos solo 8 bits
    RCSTAbits.CREN = 1;         // Activa circuito para receptor del EUSART
    
//Config I2C
    SSPADD = ((_XTAL_FREQ)/(4*I2C_SPEED)) - 1;  // 100 kHz
    SSPSTATbits.SMP = 1;        // Velocidad de rotación
    SSPCONbits.SSPM = 0b1000;   // I2C master mode, clock= Fosc/(4*(SSPADD+1))
    SSPCONbits.SSPEN = 1;       // Habilitamos pines de I2C
    PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción de I2C
    
//Config INTERRUPCIONES
    PIE1bits.RCIE  = 1;         // Es la interrupción de receptor
    INTCONbits.GIE  = 1;        // Hab. int. generales
    INTCONbits.PEIE = 1;        // Hab. int. perifericas
    
}
}
/*------------------------------------------------------------------------------
 * Funciones 
 ------------------------------------------------------------------------------*/
//----------------------- FUNCIONES DE LA EEPROM -------------------------------
uint8_t read_EEPROM(uint8_t address){
    EEADR = address;            // Guardamos la dirección en el valor de dirección que tiene (0x10))
    EECON1bits.EEPGD = 0;       // Lectura a la EEPROM
    EECON1bits.RD = 1;          // Obtenemos dato de la EEPROM
    return EEDAT;               // Regresamos dato 
}

void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;            //Elegimos la dirección que vamos a escribir
    EEDAT = data;               //Elegimos el dato que vamos a guardar
    EECON1bits.EEPGD = 0;       // Escritura a la EEPROM
    EECON1bits.WREN = 1;        // Habilitamos escritura en la EEPROM
    
    INTCONbits.GIE = 0;         // Deshabilitamos interrupciones
    EECON2 = 0x55;              //Según el fabricante dice que se tienen que poner estos dos valores en el registro de EECON2
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;          // Iniciamos escritura
    
    EECON1bits.WREN = 0;        // Deshabilitamos escritura en la EEPROM
    INTCONbits.RBIF = 0;        //
    INTCONbits.GIE = 1;         // Habilitamos interrupciones

}

/*******************************************************************************
 * Funciones de ADC
 *******************************************************************************/
void interupcion_NAN(void){
    PIE1bits.ADIE = 0;          // Deshabilitamos interrupcion del ADC
}

/*******************************************************************************
 * Funciones del modulo MSSP (I2C)
 *******************************************************************************/
void wait_I2C(void){
    while(!PIR1bits.SSPIF);     // Esperamos a que se ejecute instruccion de I2C
    PIR1bits.SSPIF = 0;         // Limpimos bandera
}

void start_I2C(void){
    SSPCON2bits.SEN = 1;        // Inicializar comunicación
    wait_I2C();
}

void stop_I2C(void){
    SSPCON2bits.PEN = 1;        // Finalizar comunicación
    wait_I2C();
}

__bit write_I2C(uint8_t data){
    SSPBUF = data;              // Cargar dato a enviar en el buffer
    wait_I2C();
    return ACKSTAT;             // Obtener ACK del esclavo
}


/*******************************************************************************
 * Funciones del modulo EUSART
 *******************************************************************************/
void int_Ninterfaz(void){
    PIE1bits.RCIE = 0;          // Deshabilitamos Interrupciones de recepción
}

void print(unsigned char palabra){
    while((palabra) != '\0'){
        while(!TXIF);
        TXREG = (palabra);
        palabra++;
    }    
    enviar_letra('\r');         
}

void enviar_letra(char letra){
    while(!TXIF);              // 
    TXREG = letra;
}

void menusito(void){
    print("");
    print("");
    print("¿Qué servo se movera?");
    print("");
    print("");
}
