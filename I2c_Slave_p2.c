/*
 * File:   I2c_Slave_p2.c
 * Author: lisan
 *
 * Created on 27 de mayo de 2022, 11:36 PM
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

#define ADDRESS1_S1 0x08
#define ADDRESS1_S2 0x09
#define ADDRESS1_S3 0x0A
#define ADDRESS1_S4 0x0B

#define ADDRESS2_S1 0x0C
#define ADDRESS2_S2 0x0D
#define ADDRESS2_S3 0x0E
#define ADDRESS2_S4 0x0F

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t val_pot1, val_pot3, val_pot4, modo;                // Valor inicial del contador
uint8_t address_s1 = 0x08, address_s3 = 0x09;
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES
 ------------------------------------------------------------------------------*/
void setup(void);
void interfaz_mode(void);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    
    if (PIR1bits.SSPIF){
        SSPCONbits.CKP = 0;         // Mantenemos el reloj en 0 para que se configure el esclavo
        
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){   // Si hay un desbordamiento o colisión entonces:
            uint8_t buff = SSPBUF;   // Limpiamos el buffer
            SSPCONbits.SSPOV = 0;   // Limpiamos bandera de overflow
            SSPCONbits.WCOL = 0;    // Limpiamos indicador de colisión
            SSPCONbits.CKP = 1;     // Habilitamos reloj para recibir datos
        }
        
        // Verificamos lo recibido fue un dato y no una dirección
        // Verificamos si el esclavo tiene que recibir datos del maestro
        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW){
            SSPSTATbits.BF = 0;     // Limpiamos bandera para saber cuando se reciben los datos
            while(!SSPSTATbits.BF); // Esperamos a recibir los datos
            if((SSPBUF & 0b00000001)  == 0){// SSPBUF SE HACE UN AND AL BIT MENOS SIGNIFICATIVO PARA SABER SI ES VALOR DE S3 O S4, SI ES 0 ES DE S3 Y SI ES 1 ES DEL S4
            CCPR1L = (SSPBUF>>1)+ 120;
            CCP1CONbits.DC1B1 = SSPBUF & 0b01;
            CCP1CONbits.DC1B0 = (SSPBUF>>7);
            }
            //ENVIA EL VALOR DE S4
            if((SSPBUF & 0b00000001)  == 1){
            CCPR2L = (SSPBUF>>1)+ 120;
            CCP2CONbits.DC2B1 = SSPBUF & 0b01;
            CCP2CONbits.DC2B0 = (SSPBUF>>7);
            }   
        }
        // Verificamos lo recibido fue un dato y no una dirección
        // Verificamos si el esclavo tiene que enviar datos al maestro
        else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            SSPCONbits.CKP = 1;     // Habilitamos reloj para el envío
            while(SSPSTATbits.BF);  // Esperamos a que se envíe el dato
            }
        PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción
    }
    return;
}    

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL PARA EL MAESTRO
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){}
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0;
    ANSELH = 0;                 // I/O digitales
    
    OSCCONbits.IRCF = 0b111;    // 8MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    TRISC = 0b00011000;         // SCL and SDA as input
    PORTC = 0;
    
    //Configurar PWM
    PR2 = 250; //Valor inicial de PR2
    CCP1CONbits.P1M = 0; //PWM bits de salida
    CCP1CONbits.CCP1M = 0b00001100; //Se habilita PWM   
    CCP2CONbits.CCP2M = 0b00001100;   
    
    CCPR1L = 0x0F; 
    CCPR2L = 0x0F;
    CCP1CONbits.DC1B = 0; //Bits menos significativos del Duty Cycle
    CCP2CONbits.DC2B1 = 0;
    CCP2CONbits.DC2B0 = 0;
    
    PIR1bits.TMR2IF = 0; //Se limpia la bandera
    T2CONbits.T2CKPS1 = 1; //Prescaler de 16
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.TMR2ON = 1; //Se enciende el TMR2
    
    while (!PIR1bits.TMR2IF); //Se espera una interrupción
    PIR1bits.TMR2IF = 0;
    //Interrupciones Config
    PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción de I2C
    PIE1bits.SSPIE = 1;         // Habilitamos interrupcion de I2C
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones de perifericos
    INTCONbits.GIE = 1;         // Habilitamos interrupciones globales

}

