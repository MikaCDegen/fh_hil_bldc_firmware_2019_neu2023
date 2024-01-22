#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motorsteuerungs-Funktionen
#include "hw.h" // Pinbelegung unserer Hardware
#include "timeout.h" // Timeout Reset
#include "commands.h" // printf für BLDC Terminal
#include "app_spicontrol.h" // Funktionsdeklarationen
#include "mcpwm.h"  // Funktionen zum Messen von Live-Daten
#include "mcpwm_foc.h"  // Funktionen zum Messen von Live-Daten
#include "mc_interface.h" // Funktionen zum Motorsteuerung

// Example thread
static THD_FUNCTION(example_thread, arg);
static THD_WORKING_AREA(example_thread_wa, 2048); // 2kb Stack für den Thread

// Einstellung der Samplerate in HZ
#define SPICONTROL_SAMPLE_RATE_HZ 1000

uint32_t spi_val = 0;
uint32_t testvalue = 0;
uint32_t merker[2];

double pwm_rpm = 0;
double pwm_current = 0;

// SPI Control Initialisierung
void app_spicontrol_init(void) {

  // Definition der Timer Struktur
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  // Setzen der SPI-Pins als Ein- bzw. Ausgänge
  palSetPadMode(HW_SPI_PORT_MISO, HW_SPI_PIN_MISO, PAL_MODE_INPUT);
  palSetPadMode(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(HW_SPI_PORT_SCK, HW_SPI_PIN_SCK, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

  // Setzen der NSS (CS), SCK und MOSI Pins auf LOW
  palClearPad(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS);
  palClearPad(HW_SPI_PORT_SCK, HW_SPI_PIN_SCK);
  palClearPad(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI);

  // Clock enable
	HW_ENC_TIM_CLK_EN();

  // Timer Einstellungen
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ((168000000 / 2 / SPICONTROL_SAMPLE_RATE_HZ) - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(HW_ENC_TIM, &TIM_TimeBaseStructure);
	TIM_ITConfig(HW_ENC_TIM, TIM_IT_Update, ENABLE);
	TIM_Cmd(HW_ENC_TIM, ENABLE);

  // Setzt die Priorität des Interrupt Handlers und aktiviert ihn
	nvicEnableVector(HW_ENC_TIM_ISR_CH, 6);

	// Start des Threads
	chThdCreateStatic(example_thread_wa, sizeof(example_thread_wa),
		NORMALPRIO, example_thread, NULL);
}

// Übertragung der Messdaten in unsere SPI-Struktur
// 12 Bit auf 20 Bit
// Setzen von Start-, End-, Vorzeichen- und Identifikations-Bit
uint32_t daten_to_spi(int32_t daten, uint32_t typ){
  if(daten<(-4095))daten=-4095;   //Daten dürfen nicht kleiner 4095 sein

  daten+=4095;          // offset
  daten<<=2;            // wegen startstopbit
  daten=daten|0x40002;    // setzen von start/stop

  //Setzen des Identifikations-Bits
  typ<<=15;
  daten=daten|typ;

  return daten;
}

// Interrupt Service Routine (ISR)
// Senden / Empfangen von SPI-Daten bei Interrupt
void spicontrol_tim_isr(void) {
  uint32_t pos = 0;

  //SPI-NSS (Chip Select) auf LOW
  spicontrol_begin();

  //Gleichzeitiger Transfer und Empfang von SPI-Daten
  spicontrol_transfer(&pos, &testvalue, 1);

  //SPI-NSS (Chip Select) auf HIGH
	spicontrol_end();

  //Übertragung empfangener Daten an globale Variable
	spi_val = pos;
}

// Funktion zum Auslesen der Empfangen SPI-Daten
uint32_t spicontrol_get_val(void) {
	return spi_val;
}

// Funktion zum Senden und Empfangen von SPI-Daten
void spicontrol_transfer(uint32_t *in_buf, const uint32_t *out_buf, int length) {
	for (int i = 0;i < length;i++) {
		uint32_t send = out_buf ? out_buf[i] : 0xFFFFF;
		uint32_t recieve = 0;

		for (int bit = 0;bit < 20;bit++) {
			palWritePad(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, send >> 19);
			send <<= 1;

			spicontrol_delay();
			palSetPad(HW_SPI_PORT_SCK, HW_SPI_PIN_SCK);
			spicontrol_delay();

			recieve <<= 1;
			if (palReadPad(HW_SPI_PORT_MISO, HW_SPI_PIN_MISO)) {
				recieve |= 1;
			}

			palClearPad(HW_SPI_PORT_SCK, HW_SPI_PIN_SCK);
			spicontrol_delay();
		}

		if (in_buf) {
			in_buf[i] = recieve;
		}
	}
}

// Funktion zum Setzen von SPI-NSS (CS) auf LOW
void spicontrol_begin(void) {
	palClearPad(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS);
}

// Funktion zum Setzen von SPI-NSS (CS) auf HIGH
void spicontrol_end(void) {
	palSetPad(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS);
}

// Funktion zum Auslösen eines minimalen Delays
void spicontrol_delay(void) {
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

// Funktion zur Anwendung unseres Algorithmus auf die empfangenen SPI-Daten
// Überprüfung der Position des Start- und End-Bits
uint32_t spi_to_data(void){
  uint32_t daten=spicontrol_get_val();

  if(daten&0x80000){
    if(daten&0x0004 && (daten&0x0003) == 0){
      daten=daten&0x7FFF8;
      daten>>=3;
      return daten;
    }
  }
  else{
    if(daten&0x40000){
      if(daten&0x0002 && (daten&0x0001) == 0){
        daten=daten&0x3FFFC;
        daten>>=2;
        return daten;
      }
    }
    else{
      if(daten&0x20000){
        if(daten&0x0001 && (daten&0xC0000) == 0){
          daten=daten&0x1FFFE;
          daten>>=1;
          return daten;
        }
      }
    }
  }

  return 4096;
}

// Thread
static THD_FUNCTION(example_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_EXAMPLE");

  uint32_t newdata=0;

	for(;;) {

    // Auslesen der Live-Daten des Frequenzumrichters
    // Funktionen aus der "mcpwm.c" und "mcpwm_foc.c"

    //FOC Get Current
	//Aktuellen Stromwert des Motors auslesen
    pwm_current = mcpwm_foc_get_tot_current();

    // Empfangene SPI-Daten in Variable speichern
    newdata=spi_to_data();
    if(!(newdata>=4096))
		commands_printf("SPI Receive: %d", newdata);
    commands_printf("SPI Receive: %lf",((double)newdata/1000.0));

	// Merker, zur Abfrage ob sich der zu setztende Stromwert geändert hat
    if(merker[0]!=newdata){
      merker[0]=newdata;
      merker[1]=1;
    }else{
      merker[1]=0;
    }
   
    //FOC Set Current
	//Funktion zum Setzen eines neuen Stromwertes
	//Diese wird nur einmalig bei einer Änderung aufgerufen
    if(merker[1]){
      commands_printf("Set Current: %d", merker[1]);
      mc_interface_set_current((float)newdata/1000.0);
    }

	//Debug: Loopen des gesendeten Signals von Matlab
    //pwm_current=(float)newdata/1000.0;
	
	//Debug: Setzen eines festen Stromwertes
    //pwm_current = -1;
	
	//Stromsignal in mA umrechnen
    pwm_current*=1000;            
	
	//Aufrufen der Funktion zum Senden der Messdaten an Matlab
    testvalue=daten_to_spi((pwm_current),1);
    commands_printf("pwm_current: %lf", pwm_current);
    commands_printf("SPI Send: %d", testvalue);

    // Kleines Verzögerung, damit der Thread den STM nicht überlastet
	chThdSleepMilliseconds(100);

	// Reset des Timeouts
	timeout_reset();
	}
}
