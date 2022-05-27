/* 
 * File:   main_I2C_master.c
 * Author: Christopher Chiroy
 * 
 * Comunicación I2C, con contador ascendente que se envía al esclavo
 * 
 * Valor enviado del contador -> PORTA
 * Dato recibido del esclavo -> PORTD
 *
 * Created on 15 mei 2022, 20:06
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
#define ADDRESS_S1 0x08
#define ADDRESS_S2 0x09
#define ADDRESS_S3 0x10
#define ADDRESS_S4 0x11
#define ADDRESS_MODO 0x12
#define READ 0b0
#define WRITE 0b1
#define _tmr0_value 255         // TMR0 a 2 ms
#define LEN_MSG 9               // Constante para definir largo de mensaje e iteraciones al enviarlo por el serial


/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t data = 0, response = 0, val_pot1, val_pot2, val_pot3, val_pot4, modo;
unsigned short CCPR = 0;        // Variable para ancho de pulso 1
unsigned short CCPR_2 = 0;      // Variable para ancho de pulso 2
unsigned short cont_tmr0;       // Contador de interrupciones de TMR0
/*unsigned short pulse_w1;         // Ancho de pulso variado por POT3
unsigned short pulse_w2;         // Ancho de pulso variado por POT4*/
char mensaje[LEN_MSG] = {'D', 'a', 't', 'o', ':', ' ', ' ', 0x0D, 0x0A};
uint8_t indice = 0;             // Variable para saber que posición del mensaje enviar al serial
uint8_t valor_old = 0;          // Variable para guardar el valor anterior recibido

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
void interupcion_AN(void);
void interupcion_NAN(void);
void wait_I2C(void);
void start_I2C(void);
void restart_I2C(void);
void stop_I2C(void);
void send_ACK(void);
void send_NACK(void);
__bit write_I2C(uint8_t data);
uint8_t read_I2C(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t max_in, unsigned short min_out, unsigned short max_out);
void config_interfaz(void);
void main_interfaz(void);
void int_Ninterfaz(void);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr(void){
    
    if (PIR1bits.ADIF){
        if (ADCON0bits.CHS == 0){                   // Se verifica canal AN0  
            val_pot1 = ADRESH;
            CCPR = map(val_pot1, 0, 90, 20, 64);     // Valor de ancho de pulso variable
            CCPR1L = (uint8_t)(CCPR>>2);            // Se guardan los 8 bits más significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11;         // Se guardan los 2 bits menos significativos en DC1B
        }
        else if (ADCON0bits.CHS == 1){              // Se verifica canal AN1        
            val_pot2 = ADRESH;
            CCPR_2 = map(val_pot2, 0, 90, 20, 64);   // Valor de ancho de pulso variable
            CCPR2L = (uint8_t)(CCPR_2>>2);          // Se guardan los 8 bits más significativos en CPR2L
            CCP2CONbits.DC2B0 = CCPR_2 & 0b01;      // Se guardan los 2 bits menos significativos en DC2B
            CCP2CONbits.DC2B1 = CCPR_2 & 0b10;      // Se guardan los 2 bits menos significativos en DC2B
        }
        else if (ADCON0bits.CHS == 2){              // Se verifica canal AN2        
            val_pot3 = ADRESH;                      // Valor de ancho de pulso variable
        }
        else if (ADCON0bits.CHS == 3){              // Se verifica canal AN2        
            val_pot4 = ADRESH;                      // Valor de ancho de pulso variable
        }
        PIR1bits.ADIF = 0;                          // Limpiamos bandera ADC
    }
    if(INTCONbits.RBIF){             // Fue interrupción del PORTB, entonces:
        if(!PORTBbits.RB0){          // cambia modo
            modo = modo<<1;          // Se acarrea un bit hacia la izquierda.   
            if(modo > 0x04){         // Si al momento de acarriar el uno es mayor a 4, entonces:
                modo = 0x01;          // Modo vuelve a ser modo manual.
            }
        }
    }    
    if(modo == 0b100){
        if(PIR1bits.RCIF){          // Hay datos recibidos?
           mensaje[6] = RCREG;     // Guardamos valor recibido en el arreglo mensaje
           PORTD = mensaje[6];     // Mostramos valor recibido en el PORTD
       }
    }   
    /* if (INTCONbits.T0IF){ 
       cont_tmr0++;                     // Contador de 0.2 ms
       if (cont_tmr0 == pulse_w1) {      // Se verifica si contador TMR0 igual al ancho de pulso
        PORTDbits.RD0 = 0;              // PORTD0 pasa a 0
        return;
       }
       if (cont_tmr0 == 10) {           // Se verifica período de 20 ms
        PORTDbits.RD0 = 1;              // PORTD0 pasa a 1
        cont_tmr0 = 0;                  // Reinicio de contador
       }
       __delay_ms(20);
       if (cont_tmr0 == pulse_w2) {      // Se verifica si contador TMR0 igual al ancho de pulso
        PORTDbits.RD1 = 0;              // PORTD1 pasa a 0
        return;
       }
       if (cont_tmr0 == 10) {           // Se verifica período de 20 ms
        PORTDbits.RD1 = 1;              // PORTD1 pasa a 1
        cont_tmr0 = 0;                  // Reinicio de contador
       }
       TMR0 = _tmr0_value;              // Reinicio de TMR0
       INTCONbits.T0IF = 0;             // Limpieza de bandera
    }*/
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    
    while(1){ 
        if(modo == 0b00000001){         //Si es modo manual, entonces:
            int_Ninterfaz();
            
            //COMUNICACIÓN MAESTRO -> ESCLAVO EN I2C
            //ENVÍA EL MODO EN EL QUE ESTÁ
            data = (uint8_t)((ADDRESS_MODO<<1)+READ);
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(modo);            // Enviamos dato del modo al esclavo 
            stop_I2C();                 // Finalizamos la comunicación
            //ENVIAMOS EL VALOR DEL POTENCIOMETRO 1
            data = (uint8_t)((ADDRESS_S1<<1)+READ);
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(val_pot1);        // Enviamos dato del 1er potenciometro al esclavo 
            stop_I2C();                 // Finalizamos la comunicación
            //ENVIAMOS EL VALOR DEL POTENCIOMETRO 2
            data = (uint8_t)((ADDRESS_S2<<1)+READ);
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(val_pot2);        // Enviamos dato del 2do potenciometro al esclavo
            stop_I2C();
            //ENVIAMOS EL VALOR DEL POTENCIOMETRO 3
            data = (uint8_t)((ADDRESS_S3<<1)+READ);
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(val_pot3);        // Enviamos dato del 3er potenciometro al esclavo 
            stop_I2C();                 // Finalizamos la comunicación
            //ENVIAMOS EL VALOR DEL POTENCIOMETRO 4
            data = (uint8_t)((ADDRESS_S4<<1)+READ);
            //val_pot4 = (val_pot4<<2) + 0b11;    // Se acarrea dos bits hacia la izquierda para que rellene esos dos bits con XXXX XX11
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(val_pot4);        // Enviamos dato del 4to potenciometro al esclavo
            stop_I2C();
            
            //COMUNICACIÓN ESCLAVO -> MAESTRO EN I2C
            //PEDIMOS EL VALOR DEL POTENCIOMETRO 1
            data = (uint8_t)((ADDRESS_S4<<1)+WRITE);
            start_I2C();                // Iniciamos comunicación
            
            stop_I2C();
            //PEDIMOS EL VALOR DEL POTENCIOMETRO 2
            data = (uint8_t)((ADDRESS_S4<<1)+WRITE);
            start_I2C();                // Iniciamos comunicación
            
            stop_I2C();
        }
        else if(modo == 0b010){         //Si es modo reproducción, entonces:
            interupcion_NAN();          // Deshabilitamos interrupción ADC
            int_Ninterfaz();            // Deshabilitamos interrupción del EUSART
            
            //COMUNICACIÓN MAESTRO -> ESCLAVO EN I2C
            //ENVIAMOS EL VALOR DEL MODO EN EL QUE ESTAMOS
            data = (uint8_t)((ADDRESS_MODO<<1)+READ);
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(modo);            // Enviamos dato del modo al esclavo 
            stop_I2C();                 // Finalizamos la comunicación
            
            //COMUNICACIÓN ESCLAVO -> MAESTRO EN I2C
            //PEDIMOS AL SLAVE QUE ENVIE EL VALOR DEL SERVO 4 Y QUE SE GUARDE (FALTAAAAA HACERLO))
            data = (uint8_t)((ADDRESS_S4<<1)+WRITE);
            start_I2C();                // Iniciamos comunicación
            
            stop_I2C();
        }
        else if(modo == 0b00000100){    // Modo interfaz (no reproduce, ni guarda, solo potenciometros)
            //COMUNICACIÓN MAESTRO -> ESCLAVO EN I2C
            //ENVIAMOS EL VALOR DEL MODO EN EL QUE ESTAMOS
            data = (uint8_t)((ADDRESS_MODO<<1)+READ);
            start_I2C();                // Iniciamos comunicación
            write_I2C(data);            // Enviamos dirección de esclavo a recibir datos
            write_I2C(modo);            // Enviamos dato del modo al esclavo 
            stop_I2C();                 // Finalizamos la comunicación
            
            //Módulo EUSART
            config_interfaz();
            main_interfaz();
            //Falta cambiar progra de la interfaz (TODO, solo está el esqueleto) y no sé si está correcto lo de la comunicación
        }
        else{
            stop_I2C();                 // Finalizamos la comunicación
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0x0F;               // AN0, AN1, AN2 Y AN3 ENTRADAS ANALÓGICAS
    ANSELH = 0;                 // I/O digitales
    
    OSCCONbits.IRCF = 0b0111;   // 8MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    // --------------- CONFIGURACION DEL ADC --------------- //    
    ADCON0bits.ADCS = 0b11;         // FRC
    ADCON1bits.VCFG0 = 0;           // Referencia VDD
    ADCON1bits.VCFG1 = 0;           // Referencia VSS
    ADCON0bits.CHS = 0;             // Se selecciona PORTA0/AN0
    ADCON1bits.ADFM = 0;            // Se indica que se tendrá un justificado a la izquierda
    ADCON0bits.ADON = 1;            // Se habilita el modulo ADC
    __delay_us(40);                 // Delay para sample time
    
// --------------- CONFIGURACION DE PWM --------------- //
    TRISCbits.TRISC2 = 1;           // RC2/CCP1 como salida deshabilitado
    TRISCbits.TRISC1 = 1;           // Se deshabilita salida de PWM (CCP2)
    CCP1CON = 0;                    // Se apaga CCP1
    CCP2CON = 0;                    // Se apaga CCP2
    PR2 = 155;                      // Período de 20 ms  
    
// --------------- CONFIGURACION DEL CCP --------------- //
    CCP1CONbits.P1M = 0;            // Modo single output
    CCP1CONbits.CCP1M = 0b1100;     // Modo PWM
    CCP2CONbits.CCP2M = 0b1100;     // Modo PWM
    //Servo 1
    CCPR1L = 30>>2;                     
    CCP1CONbits.DC1B = 30 & 0b11;       

    //Servo 2
    CCPR2L = 30>>2;                 //Ciclo de trabajo base pues se va a variar
    CCP2CONbits.DC2B0 = 30 & 0b01;      
    CCP2CONbits.DC2B1 = (30 & 0b10)>>1; 

// --------------- CONFIGURACION DE TIMER 2 --------------- //
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Se enciende TMR2
    while(!PIR1bits.TMR2IF);    // Se espera un ciclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2 nuevamente
    
    //TRISCbits.TRISC3 = 0;       // Se habilita salida de PWM (Con TMR0)
    TRISCbits.TRISC2 = 0;       // Se habilita salida de PWM (CCP1)
    TRISCbits.TRISC1 = 0;       // Se habilita salida de PWM (CCP2)
    
    //TMR0 CONFIG
    OPTION_REGbits.T0CS = 0;    // Uso de reloj interno
    OPTION_REGbits.PSA = 0;     // Uso de Prescaler con TMR0
    OPTION_REGbits.PS = 0b0111; // Prescaler de 1:256 
    TMR0 = _tmr0_value;         // preset for timer register a 0.2 ms
    
    //I2C CONFIG
    TRISB = 0;
    TRISC = 0b00011000;         // SCL and SDA as input
    TRISD = 0;
    TRISE = 0;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTEbits.RE0 = 1;
    PORTEbits.RE1 = 0;
    PORTEbits.RE2 = 0;
    

    SSPADD = ((_XTAL_FREQ)/(4*I2C_SPEED)) - 1;  // 100 kHz
    SSPSTATbits.SMP = 1;        // Velocidad de rotación
    SSPCONbits.SSPM = 0b1000;   // I2C master mode, clock= Fosc/(4*(SSPADD+1))
    SSPCONbits.SSPEN = 1;       // Habilitamos pines de I2C
    PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción de I2C
    interupcion_AN();
}

/*------------------------------------------------------------------------------
 * Funciones 
 ------------------------------------------------------------------------------*/
void interupcion_AN(void){
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion del ADC
    PIR1bits.ADIF = 0;          // Limpiamos la bandera del ADC
}

void interupcion_NAN(void){
    PIE1bits.ADIE = 0;          // Deshabilitamos interrupcion del ADC
}

void wait_I2C(void){
    while(!PIR1bits.SSPIF);     // Esperamos a que se ejecute instruccion de I2C
    PIR1bits.SSPIF = 0;         // Limpimos bandera
}

void start_I2C(void){
    SSPCON2bits.SEN = 1;        // Inicializar comunicación
    wait_I2C();
}

void restart_I2C(void){
    SSPCON2bits.RSEN = 1;       // Reiniciar de comunicación
    wait_I2C();
}

void stop_I2C(void){
    SSPCON2bits.PEN = 1;        // Finalizar comunicación
    wait_I2C();
}

void send_ACK(void){
    SSPCON2bits.ACKDT = 0;      // Confirmar que se recibió la data
    SSPCON2bits.ACKEN = 1;      // Envio de ack al esclavo
    wait_I2C();
}

void send_NACK(void){
    SSPCON2bits.ACKDT = 1;      // Confirmar recepción al finalizar comunicación
    SSPCON2bits.ACKEN = 1;      // Envio de nack al esclavo
    wait_I2C();
}
__bit write_I2C(uint8_t data){
    SSPBUF = data;              // Cargar dato a enviar en el buffer
    wait_I2C();
    return ACKSTAT;             // Obtener ACK del esclavo
}

uint8_t read_I2C(void){
    SSPCON2bits.RCEN = 1;       // Pedir dato al esclavo  
    wait_I2C();
    return SSPBUF;              // Regresar dato recibido
}

void main_interfaz(void){  
       setup();
       while(1){
           //__delay_ms(1000);

           indice = 0;                             // Reiniciamos indice para enviar todo el mensaje
           if (valor_old != mensaje[6]){           // Verificamos que el nuevo valor recibido en el serial 
                                                   //   sea diferente al anterior, para imprimir solo 
               while(indice<LEN_MSG){              // Loop para imprimir el mensaje completo
                   if (PIR1bits.TXIF){             // Esperamos a que esté  libre el TXREG para poder enviar por el serial
                       TXREG = mensaje[indice];    // Cargamos caracter a enviar
                       indice++;                   // Incrementamos indice para enviar siguiente caracter
                   }
               }
               valor_old = mensaje[6];             // Guardamos valor recibido para comparar en siguiente iteración
                                                   //   si el nuevo valor recibido es diferente al anterior. 
           }
       }
       return;
   }
void config_interfaz(void){
       ANSEL = 0x0F;
       ANSELH = 0;                 // I/O digitales

       TRISD = 0;
       PORTD = 0;                  // PORTD como salida

       OSCCONbits.IRCF = 0b100;    // 1MHz
       OSCCONbits.SCS = 1;         // Oscilador interno

       // Configuraciones de comunicacion serial
       //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
       TXSTAbits.SYNC = 0;         // Comunicaci n ascincrona (full-duplex)?
       TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
       BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate

       SPBRG = 25;
       SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%

       RCSTAbits.SPEN = 1;         // Habilitamos comunicación
       TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
       TXSTAbits.TXEN = 1;         // Habilitamos transmisor
       RCSTAbits.CREN = 1;         // Habilitamos receptor

       // Configuraciones de interrupciones
       INTCONbits.GIE = 1;         // Habilitamos interrupciones globales
       INTCONbits.PEIE = 1;        // Habilitamos interrupciones de perifericos
       PIE1bits.RCIE = 1;          // Habilitamos Interrupciones de recepción
   }

void int_Ninterfaz(void){
    PIE1bits.RCIE = 0;          // Deshabilitamos Interrupciones de recepción
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}