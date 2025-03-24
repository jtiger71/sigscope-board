// Sample from the ADC continuously at a particular sample rate
// and then compute an FFT over the data
//
// much of this code is from pico-examples/adc/dma_capture/dma_capture.c
// the rest is written by Alex Wulff (www.AlexWulff.com)
  
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "kiss_fftr.h"

#define READABLE_DISPLAY_FLAG 0


// set this to determine sample rate, 0 is not working
// 96     = 500,000 Hz
// 960    = 50,000 Hz
// 9600   = 5,000 Hz
// 96*5   = 100,000 Hz
// 96*50  = 10,000Hz
// 96*500 = 1,000Hz

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0
#define LED_PIN 25

// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly


// globals

dma_channel_config cfg;
uint dma_chan;
//float freqs[NSAMP];
float CLOCK_DIV = 96.0;
#define NSAMP 5000
float FSAMP = 500000.0;

uint16_t cap_buf[NSAMP];	
kiss_fft_scalar fft_in[NSAMP]; 
kiss_fft_cpx fft_out[NSAMP];
kiss_fftr_cfg kiss_cfg;

//..........declaration of 2 functions..............
void setup();
void sample(uint16_t *capture_buf);
short *genSample(int *length);
void buildSignalData(int ch, short *data, int dataCount, char *jsonBuff, int bufSize);
//..................................................


#define UART_ID uart0
void uart_input_task() {
     while(uart_is_readable(UART_ID)) {
          char c = uart_getc(UART_ID);
          printf("Received: %c\n", c);     
     }
}

/* -------------------------------------------------------------------------------------------------------------------- */



uint64_t sum;
float avg, sum1, variance, power, max_power, max_freq, std_deviation;
int max_idx, startpoint, displaysize, diff, slope;

void init_adc_fft() 
{
     srand(time(NULL));
     setup(); // setup ports and outputs
     kiss_cfg = kiss_fftr_alloc(NSAMP,false,0,0);
}


void run_adc_fft(char *resultBuff, int size) 
{              
     //=============== sampling with 500KHz(maximum vaues) to find out frequency ranges

     FSAMP = 500000.0;  CLOCK_DIV = 96.0; 
     adc_set_clkdiv(CLOCK_DIV);
     sample(cap_buf); 

     // return NULL;

     //================ converting the sampled data into 8-bit
     for(int i=0; i<NSAMP; i++) {cap_buf[i] = cap_buf[i]>>4;}
          
     // ============== compute 1st variance and standard deviation to detect DC signal
     sum = 0; sum1 = 0;  avg = 0.0;
     for (int i=0; i<NSAMP; i++) {sum += cap_buf[i];}
     avg = (float)sum/NSAMP;
     for (int i = 0; i<NSAMP; i++) {sum1 = sum1 + pow((cap_buf[i]-avg), 2);}
     variance = sum1/NSAMP;
     std_deviation = sqrt(variance);
     // DC component remove
     for (int i=0; i<NSAMP; i++) {fft_in[i] = (float)cap_buf[i] - avg;}
          
     // compute fast fourier transform 1st
     kiss_fftr(kiss_cfg , fft_in, fft_out);
     
     // compute power and calculate max freq component
     // any frequency bin over NSAMP/2 is aliased (nyquist sampling theorum)
     max_power = 0.0;  max_idx = 0.0;
     for (int i = 0; i < NSAMP/2; i++) {   
          power = fft_out[i].r*fft_out[i].r+fft_out[i].i*fft_out[i].i;
          if (power > max_power) {max_power = power;  max_idx = i;}
     }                                 
     max_freq = (max_idx * (FSAMP/NSAMP));

     // return NULL;

     if(READABLE_DISPLAY_FLAG) {
          printf("===================================================================================================================\n");
          printf("1st sampling freq: %0.2f,  CLOCK: %0.2f,   VAR: %0.2f,   Max Freq: %0.2fHz,   MAX_IDX: %d,   Avg: %0.2f,   NSAMP:%d\n", 
                    FSAMP, CLOCK_DIV, variance, max_freq, max_idx, avg, NSAMP);
     }

     // return NULL;

     //------------------------------------------------------------------------------------------------------------
     //        Check the input frequency range: larger than 25KHz with 500KHz sampling. Resultion is 250Hz with NSAMP=2000.
     //------------------------------------------------------------------------------------------------------------  
     
     if (max_freq >= 25000) {						// start if > 25KHz		22222
          if(READABLE_DISPLAY_FLAG) {
               printf("=========================================\n");
               printf("The signal frequency is larger than 25KHz\n");
          }
     }									// end   if > 25KHz		22222 xxxxxx

     // -----------------------------------------------------------------------------------------------------------
     //        The followingelse routine is to adjust the new sampling frequency in case of less than 25KHz signals.
     //        The 2nd sampling frequency, FSAMP, is determined from the first fft's max_freq * 20.
     //        That is, if the input signal freqency is 1KHz, then the FSAMP shold be 1KHz*20 = 20KHz.
     //        The followings, CLOCK_DIV = 48000000/(max_freq*20.0) and FSAMP = 48000000/CLOCK_DIV, are  equations to get the FSAMP  
     // -----------------------------------------------------------------------------------------------------------  
     else {              							// start else < 25KHz		33333       
          CLOCK_DIV = 48000000/(max_freq*20.0);
          FSAMP = 48000000/CLOCK_DIV;         
          adc_set_clkdiv(CLOCK_DIV);
          sample(cap_buf);
               
          //================ converting the sampled data into 8-bit
          for(int i=0; i<NSAMP; i++) {cap_buf[i] = cap_buf[i]>>4;}
                    
          // ============== compute 1st variance and standard deviation to detect DC signal
          sum = 0; sum1 = 0;  avg = 0.0;
          for (int i=0; i<NSAMP; i++) {sum += cap_buf[i];}
          avg = (float)sum/NSAMP;
          for (int i = 0; i<NSAMP; i++) {sum1 = sum1 + pow((cap_buf[i]-avg), 2);}
          variance = sum1/NSAMP;
          std_deviation = sqrt(variance);
          // ============== DC component remove
          for (int i=0; i<NSAMP; i++) {fft_in[i] = (float)cap_buf[i] - avg;}
                              
          // ============== compute fast fourier transform 1st
          kiss_fftr(kiss_cfg , fft_in, fft_out);
               
          // =============== compute power and calculate max freq component
          // =============== any frequency bin over NSAMP/2 is aliased (nyquist sampling theorum)
          max_power = 0.0;  max_idx = 0;
          for (int i = 0; i < NSAMP/2; i++) {  
               power = fft_out[i].r*fft_out[i].r+fft_out[i].i*fft_out[i].i;
               if (power > max_power) {max_power = power;  max_idx = i;}
          }  
          max_freq = (max_idx * (FSAMP/NSAMP));
          
          if(READABLE_DISPLAY_FLAG) {
               printf("===================================================================================================================\n");                                                                         
               printf("2nd sampling freq: %0.2f,  CLOCK: %0.2f,   VAR: %0.2f,   Max Freq: %0.2fHz,   MAX_IDX: %d,   Avg: %0.2f,   NSAMP:%d\n", 
                         FSAMP, CLOCK_DIV, variance, max_freq, max_idx, avg, NSAMP);
          }

          //-----------------------------------------------------------------------------------------------------------------
          //      The above FSAMP and other values are calculated with 2nd sampling  frequency, which depends on 
          //      the max_idx value. The max_idx=1 means signal frequency is less than or equal to 100 Hz at FSAMP=500KHz and NSAMP=5000.
          //      While, the max_idx=2 means that 100Hz < input signal frequency <= 200Hz. is 200Hz. If the NSAMP is changed, than the 
          //      is changed to 250Hz unit instead of 100Hz.
          //      There is resolution drawback when the sampling frequency is increasing to find more precise waveform, because the resolution 
          //      is determined by the FSAMP/NSAMP. That is, The NSAMP must be increased to maintained the same resolutions as increasing
          //      the sampling frequency.
          //-----------------------------------------------------------------------------------------------------------------
/*              
          if (max_freq <= 150) { 	                                   
               CLOCK_DIV = 16000;
               FSAMP = 3000;                               // When NSAMP=5000, this FSAMP is not necessary because resolution is 100Hz
*/
          if (max_freq < 50 ) {

//                want to change NSAMP=5000 with NSAMP=2000 when the frequency is lower than 50Hz

               CLOCK_DIV = 48000;
               FSAMP = 1000;
          } 
                         
          adc_set_clkdiv(CLOCK_DIV);
          sample(cap_buf);
          
          //================ converting the sampled data into 8-bit
          for(int i=0; i<NSAMP; i++) {cap_buf[i] = cap_buf[i]>>4;}
               
          // ============== compute 1st variance and standard deviation to detect DC signal
          sum = 0; sum1 = 0;  avg = 0.0;
          for (int i=0; i<NSAMP; i++) {sum += cap_buf[i];}
          avg = (float)sum/NSAMP;
          for (int i = 0; i<NSAMP; i++) {sum1 = sum1 + pow((cap_buf[i]-avg), 2);}
          variance = sum1/NSAMP;
          std_deviation = sqrt(variance);
          // ============== DC component remove
          for (int i=0; i<NSAMP; i++) {fft_in[i] = (float)cap_buf[i] - avg;}
                         
          // ============== compute fast fourier transform 1st
          kiss_fftr(kiss_cfg , fft_in, fft_out);
          
          // =============== compute power and calculate max freq component
          // =============== any frequency bin over NSAMP/2 is aliased (nyquist sampling theorum)
          max_power = 0.0;  max_idx = 0;
          for (int i = 0; i < NSAMP/2; i++) {  
               power = fft_out[i].r*fft_out[i].r+fft_out[i].i*fft_out[i].i;
               if (power > max_power) {max_power = power;  max_idx = i;}
          }  
          max_freq = (max_idx * (FSAMP/NSAMP));
          
          if(READABLE_DISPLAY_FLAG) {
               printf("===================================================================================================================\n");                                                            
               printf("3rd sampling freq: %0.2f,  CLOCK: %0.2f,   VAR: %0.2f,   Max Freq: %0.2fHz,   MAX_IDX: %d,   Avg: %0.2f,   NSAMP:%d\n", 
                         FSAMP, CLOCK_DIV, variance, max_freq, max_idx, avg, NSAMP); 
          }

// needs } when NSAMP is 2000		
                                                                                               
     }                                                    			// end else < 25KHz		33333 xxxxx

     // return NULL;                                                                          

     //================= printing routine of sampled data ==========================
                    

          
     //================= finding the first data point that close to the average value
     
     /*	diff is to detect a starting point that is very close to average value, while slope is to find positive cycle by subtracting 2 samples.
     slop1, slop2, and slop3 are to detect square wave form. These values are 0 or equal to FSAMP if the 3 consecutive input signals are flat or DC.
          std_deviation, standard deviation, is used to detect DC signal.
     If a sinusoidal waveform's frequency is very low, it may be regarded as square waveform at the upper high region.
     Thus, you can see a wavform starting not average.  
     */ 

     // Finding the start point of data to be displayed


     bool square_wave = false;
     for (int i = 0; i < NSAMP; i++) {
          diff = abs(cap_buf[i] - (int)avg);
          slope = cap_buf[i+2] - cap_buf[i];

          if (((diff <= 1 && slope > 0 ) && (slope < std_deviation))|| ((cap_buf[i+1]-cap_buf[i]) >= std_deviation) || (std_deviation < 5)) {
               startpoint = i;
               if ((cap_buf[i+1]-cap_buf[i]) > std_deviation) square_wave = true;

               if (startpoint >= NSAMP) {   
                    if(READABLE_DISPLAY_FLAG) {
                         printf("============\n");
                         printf("Out of range\n");
                         printf("============\b");
                    }
               }
               break;
          }
     }

     // ================ calculating the one cycle data range and resetting variables. If it needs 2 cycles, 2 times the displaysize 

     if ( 1.0 > max_freq) displaysize=1000-startpoint;
     else displaysize = (int)FSAMP/(int)max_freq;  

     // return NULL;
                                                                                                                        
     // ================ printing data of ADC 
     if(READABLE_DISPLAY_FLAG) {
          printf("===================================================================================================================\n");
          printf("Max freq: %0.2f and Display Size: %d  and Start: %d Difference from Avg: %d Slope:=%d Standard Deviation: %0.2f, Square Wave:%d\n", 
                    max_freq, displaysize, startpoint, diff, slope, std_deviation, square_wave);
          printf("================================================================================================================================\n"); 
                    
          for (int j = 0, i = startpoint; j <= displaysize; j++, i++) {  
               printf("0x%03x   ", cap_buf[i]);  
               if(j % 10 == 9) printf("\n");
          }
          printf("\n");
          // return NULL;

          // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
          // sleep_ms(500);                                                     

          // return NULL;

          // ================= plotting data of ADC =======================
               
          for (int i = startpoint; i <= (startpoint+displaysize); i++) {   
               int count = cap_buf[i]/4;
               for (int j = 0; j < count; j++) { printf("%c", (char)32u);}                                                           
               printf("%c\n", (char)42u);
          }

          //==================== one more plot if the final data is less than avg                       

          for (int i = 0; i < 3; i++) { 
               diff = cap_buf[startpoint+displaysize + i] - (int)avg;      
               if (diff <= 0) {
                    int count = cap_buf[startpoint+displaysize + i+1]/4;
                    for (int j = 0; j < count; j++) { printf("%c", (char)32u);}                                                           
                    printf("%c\n", (char)42u);
               }
          }                                                                                

          printf("\n");
                    
                    
          if (startpoint >= NSAMP) {
               printf("Out of range of startpoint= %d at FSAMP= %0.2f and max_freq= %0.2f due to diff=%d and slope=%d\n", startpoint, FSAMP, max_freq, diff, slope);
          } 

          /* EMPTY RESULT */
          strcpy(resultBuff, "");
     }
     else {
          strcpy(resultBuff, "");
          char jsonBuf1[1000] = "";
          char jsonBuf2[1000] = "";

          int lengthCh1 = 0, lengthCh2 = 0;
          short *dataCh1 = genSample(&lengthCh1);

          buildSignalData(1, dataCh1, lengthCh1, jsonBuf1, sizeof(jsonBuf1));
          snprintf(resultBuff, size, "@@{{%d}}@@{\"signals\":[%s]}",  14 + strlen(jsonBuf1), jsonBuf1);

          free(dataCh1);
     }
}                

/*
     {
          "ch": "1",
          "data": [ ... ]
     }
*/
void buildSignalData(int ch, short *data, int dataCount, char *jsonBuff, int bufSize) 
{
     char digBuf[20] = "";
     
     strcat(jsonBuff, "{\"ch\": \"");
     
     sprintf(digBuf, "%d", ch);
     strcat(jsonBuff, digBuf);
     
     strcat(jsonBuff, "\",");
     strcat(jsonBuff, "\"data\": [");
     for (int xx = 0; xx < dataCount; xx++) {  
          char digBuf[20] = "";
          if(xx != 0) strcat(jsonBuff, ",");
          sprintf(digBuf, "%d", data[xx]);
          strcat(jsonBuff, digBuf);
     }
     strcat(jsonBuff, "]}");
}                                                                            

int sendData(int ch, short *data, int dataCount) {
     /* SEND HEAD */
     printf("{{=>%d", ch); write(STDOUT_FILENO, &dataCount, sizeof(dataCount)); printf("}}");

     /* SEND DATA */
     for (int xx = 0; xx < dataCount; xx++) {  
          write(STDOUT_FILENO, &data[xx], sizeof(short));
     }

     return 0;
}


#define SAMPLE_RATE 8800   // 샘플링 주파수
#define DURATION 1         // 파형 지속 시간 (초)
#define GRAPH_WIDTH 80     // 그래프의 너비
#define FREQUENCY 440.0    // 출력할 파형의 주파수 (Hz)

int sinGraph() {
    double amplitude = 2.0;    // 파형의 진폭
    int num_samples_per_cycle = SAMPLE_RATE / FREQUENCY;

    for (int i = 0; i < num_samples_per_cycle; i++) {
        double t = (double)i / SAMPLE_RATE;
        double value = amplitude * sin(2 * M_PI * FREQUENCY * t);

        printf("%.2f ", value);
        if(i%8 == 0) printf("\n");  
    }
    printf("\n");

    for (int i = 0; i < num_samples_per_cycle; i++) {
        double t = (double)i / SAMPLE_RATE;
        double value = amplitude * sin(2 * M_PI * FREQUENCY * t);

        // 그래프의 스케일링 및 출력
        int scaled_value = (int)((value + 1.0) * 0.5 * GRAPH_WIDTH);
        for (int j = 0; j < scaled_value; j++) {
            putchar('*');  // 양의 값은 '*' 문자로 표시
        }
        putchar('\n');
    }

    return 0;
}

short *genSample(int *length) {
     double amplitude = rand()%5 + 1;
     int num_samples_per_cycle = SAMPLE_RATE / FREQUENCY;

     short* buf = (short *) malloc(sizeof(short)*num_samples_per_cycle);
     for (int i = 0; i < num_samples_per_cycle; i++) {
          double t = (double)i / SAMPLE_RATE;
          double value = amplitude * sin(2 * M_PI * FREQUENCY * t);

          buf[i] = (short) (value*100);
    }
    *length = num_samples_per_cycle;
    return buf;
}


/*

{{=>1####}}
{{=>2####}}

*/



/* 
// =================for later amp gain adj. =====================
  //Enable SPI0 at 1 MHz
   spi_init (spi_default, 1 * 1000000);

  // Assign SPI functions to the default SPI pins
   gpio_set_function (PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
   gpio_set_function (PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
   gpio_set_function (PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
   gpio_set_function (PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);

   uint16_t pga_gain[2], dummy_buf[2];
   pga_gain[0] = 40;
   pga_gain[1] = 00;
   dummy_buf [0] = 0;
   dummy_buf [1] = 0;

   spi_write_read_blocking (spi_default, pga_gain, dummy_buf, 1);
// ============================================================
*/








//.................. ADC sample function ...............
void sample(uint16_t *capture_buf) {
  adc_fifo_drain();
  adc_run(false);
      
  dma_channel_configure(dma_chan, &cfg,
			capture_buf,    // dst
			&adc_hw->fifo,  // src
			NSAMP,          // transfer count
			true            // start immediately
			);

  gpio_put(LED_PIN, 1);
  adc_run(true);
  dma_channel_wait_for_finish_blocking(dma_chan);
  gpio_put(LED_PIN, 0);
}
//................................................

//................... set up funcion..............
void setup() {
  stdio_init_all();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  adc_gpio_init(26 + CAPTURE_CHANNEL);

  adc_init();
  adc_select_input(CAPTURE_CHANNEL);
  adc_fifo_setup(
		 true,    // Write each completed conversion to the sample FIFO
		 true,    // Enable DMA data request (DREQ)
		 1,       // DREQ (and IRQ) asserted when at least 1 sample present
		 false,   // We won't see the ERR bit because of 8 bit reads; disable.
		 false     // Shift each sample to 16 bits when pushing to FIFO
		 );

  // set sample rate
  adc_set_clkdiv(CLOCK_DIV);

  sleep_ms(1000);
  // Set up the DMA to start transferring data as soon as it appears in FIFO
  uint dma_chan = dma_claim_unused_channel(true);
  cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);

  // calculate frequencies of each bin
//  float f_max = FSAMP;
//  float f_res = f_max / NSAMP;
//  for (int i = 0; i < NSAMP; i++) {freqs[i] = f_res*i;}
}
//.....................................................

