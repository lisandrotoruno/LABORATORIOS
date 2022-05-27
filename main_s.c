/* 
 * File:   main_s.c
 * Author: LISANDRO TORUÑO
 *
 * Created on 19 de mayo de 2022, 10:07 AM
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
#define _XTAL_FREQ 4000000

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
uint8_t pot1, pot2, pot3, pot4, modo;                // Valor inicial del contador

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
            uint8_t var = SSPBUF;   // Limpiamos el buffer
            SSPCONbits.SSPOV = 0;   // Limpiamos bandera de overflow
            SSPCONbits.WCOL = 0;    // Limpiamos indicador de colisión
            SSPCONbits.CKP = 1;     // Habilitamos reloj para recibir datos
        }
        
        // Verificamos lo recibido fue un dato y no una dirección
        // Verificamos si el esclavo tiene que recibir datos del maestro
        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW){
            SSPSTATbits.BF = 0;     // Limpiamos bandera para saber cuando se reciben los datos
            while(!SSPSTATbits.BF); // Esperamos a recibir los datos
            CCPR1L = (SSPBUF>>1)+ 55;
            CCP1CONbits.DC1B1 = SSPBUF & 0b01;
            CCP1CONbits.DC1B0 = (SSPBUF>>7);
            //PORTD = SSPBUF;         // Mostramos valor recibido del maestro en PORTD
            SSPCONbits.CKP = 1;     // Habilitamos el reloj
        }
        
        // Verificamos lo recibido fue un dato y no una dirección
        // Verificamos si el esclavo tiene que enviar datos al maestro
        else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            SSPBUF = pot1;         // Preparamos dato a enviar
            SSPCONbits.CKP = 1;     // Habilitamos reloj para el envío
            while(SSPSTATbits.BF);  // Esperamos a que se envíe el dato
            //PORTA = cont;           // Mostramos dato enviado en PORTA
            //cont--;                 // Actualizamos valor del contador
        }
        PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción
    }
    //ES MAESTRO NO ESCLAVO
    if(INTCONbits.RBIF){             // Fue interrupción del PORTB, entonces:
        //GRABADO DE POSICIONES
        if(!PORTBbits.RB0 && PORTEbits.RE0){             // Guardamos la posición que tiene el Servo 1
            write_EEPROM(ADDRESS1_S1, pot1);    
        }
        if(!PORTBbits.RB1&& PORTEbits.RE0){             // Guardamos la posición que tiene el Servo 3
            write_EEPROM(ADDRESS1_S3, pot3);
        }
        //MODO REPRODUCCIÓN
        if(!PORTBbits.RB0 && PORTEbits.RE1){             // Guardamos la posición que tiene el Servo 1
            read_EEPROM(ADDRESS1_S1);    
        }
        if(!PORTBbits.RB1&& PORTEbits.RE2){             // Guardamos la posición que tiene el Servo 3
            read_EEPROM(ADDRESS1_S3);
        }
        
        INTCONbits.RBIF = 0;             // Limpiamos bandera de interrupción
        SLEEP();                         // Al menos que no haya una interrupción está en bajo consumo el PIC
    }    
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL PARA EL MAESTRO
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        PORTE = modo;
        if(PORTEbits.RE0){
            if(!PORTBbits.RB0){
               write_EEPROM(ADDRESS1_S1, pot1); 
            }
            if(!PORTBbits.RB1){
               write_EEPROM(ADDRESS1_S3, pot3); 
            }
        }
        if(PORTEbits.RE1){
            if(!PORTBbits.RB0){
                read_EEPROM(ADDRESS1_S1);
            }
            if(!PORTBbits.RB1){
                read_EEPROM(ADDRESS1_S3);
            }
        }
        if(PORTEbits.RE2){
            interfaz_mode();
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0;
    ANSELH = 0;                 // I/O digitales
    
    OSCCONbits.IRCF = 0b110;    // 4MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    TRISC = 0b00011000;         // SCL and SDA as input
    PORTC = 0;
    
    TRISD = 0;
    PORTD = 0x00;
    
    TRISA = 0;
    PORTA = 0x00;

    SSPADD = 0x10;              // Dirección de esclavo: 0x08 0b0001 000x
    SSPSTATbits.SMP = 1;        // Velocidad de rotación
    SSPCONbits.SSPM = 0b0110;   // I2C slave mode, 7-bit address
    SSPCONbits.SSPEN = 1;       // Habilitamos pines de I2C

    PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción de I2C
    PIE1bits.SSPIE = 1;         // Habilitamos interrupcion de I2C
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones de perifericos
    INTCONbits.GIE = 1;         // Habilitamos interrupciones globales
}

//FALTA ESTÁ PARTE
void interfaz_mode(void){
    
}
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
