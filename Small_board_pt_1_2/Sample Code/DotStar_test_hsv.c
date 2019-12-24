/*
 * File:        DotStar test
 * Author:      Bruce Land
 * For use with Sean Carroll's Little Board
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
// threading library
#include "pt_cornell_1_2_1.h"
#include <stdlib.h>
#include <math.h>

// APA102 datasheet:
// 32 bits of zeros is a start frame
// 32 bits of ones is a stop frame
// LED frame:
// 111_5bit_global_intensity_8bitBlue_8bitGreen_8bitRed
// so 0xff_00_ff_00 is full intnsity green
#define START_FRAME 0x00000000
#define STOP_FRAME  0xffffffff
#define PIXEL_FRAME(i,r,g,b)(0xe0000000 | (((0x1f & (i)))<<24) | ((0xff & (b))<<16) | ((0xff & (g))<<8) | (0xff & (r)))
//#define PIXEL_FRAME(i,r,g,b)(0xe0000000 | ((i)<<24) | ((b)<<16) | ((g)<<8) | (r))
#define FULL_ON 0x1e
#define HALF_ON 0x0f
#define QUAR_ON 0x07

// number of pixels
#define PixelNum 144

typedef struct pixel pixel;
struct pixel{
    char red;
    char green;
    char blue;
    char intensity;
};
// and the whole string
pixel pixel_array[PixelNum];

// === display the LEDs ===========================================
// copies the contents of pixel_array to SPI
void write_pixels(void){ 

    // start frame
    WriteSPI2(START_FRAME);
    // wait for end of transaction
    while (SPI2STATbits.SPIBUSY); 
    
    int i;
    //payload
    for (i=0; i<PixelNum; i++){
        WriteSPI2(PIXEL_FRAME(pixel_array[i].intensity, pixel_array[i].red, pixel_array[i].green, pixel_array[i].blue));
        // wait for end of transaction
        while (SPI2STATbits.SPIBUSY); 
    }
    //stop frame
    WriteSPI2(STOP_FRAME);
    // wait for end of transaction
    while (SPI2STATbits.SPIBUSY); 
}

// === write a RGBI value to the pixel array =======================
void set_pixel_rgb(int i, char r, char g, char b, char intensity){
    if (i<0 || i>=PixelNum) return ;
    pixel_array[i].intensity = intensity  ;  //enforce max 
    pixel_array[i].red = r   ;
    pixel_array[i].green = g  ;
    pixel_array[i].blue = b ;
}

// === write a HSVI value to the pixel array =======================
void set_pixel_hsv(int i, float h, float s, float v, char intensity){
    float C, X, m, rp, gp, bp ;
    unsigned char r, g, b ;
    // index range check
    if (i<0 || i>=PixelNum) return ;
    // hsv to rgb conversion from
    // http://www.rapidtables.com/convert/color/hsv-to-rgb.htm
    C = v * s;
    //X = C * (1 - abs((int)(h/60)%2 - 1));
    // (h/60) mod 2  = (h/60 - (int)(h/60))
    X = C * (1.0 - fabsf(fmodf(h/60.0, 2.0) - 1.));
    m = v - C;
    if      ((0<=h) && (h<60))   { rp = C; gp = X; bp = 0;}
    else if ((60<=h) && (h<120)) { rp = X; gp = C; bp = 0;}
    else if ((120<=h) && (h<180)){ rp = 0; gp = C; bp = X;}
    else if ((180<=h) && (h<240)){ rp = 0; gp = X; bp = C;}
    else if ((240<=h) && (h<300)){ rp = X; gp = 0; bp = C;}
    else if ((300<=h) && (h<360)){ rp = C; gp = 0; bp = X;}
    else                         { rp = 0; gp = 0; bp = 0;}
    
    r = (unsigned char)((rp+m)*255) ;
    g = (unsigned char)((gp+m)*255) ;
    b = (unsigned char)((bp+m)*255) ;
            
    pixel_array[i].intensity = intensity  ;  //enforce max 
    pixel_array[i].red = r   ;
    pixel_array[i].green = g  ;
    pixel_array[i].blue = b  ;
}

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer ;


// === Timer Thread =================================================
// update a 1 second tick counter
int position=0, dir=1;
#define DDS_sample_time 30

static PT_THREAD (protothread_timer(struct pt *pt)){
    PT_BEGIN(pt);
      static int i ;
      static int sine[256], c[256] ;
      static float h, s, v, m[256];
      // with a 16 bit DDS and 8-bit sine table index
      // frequency of sine output is related to increment as
      // inc = Fout * 2^16 * DDS_sample_time
      // e.g. for 2 Hz and sample time 0f 30 millisec: 
      // inc = 2 * 2^16 * 0.030 = 3932
      // or an increment of about 2000 per Hz. (with 30 mS sample time)
      static unsigned short dds_inc_r=1500, dds_inc_g=1500, dds_inc_b=500, dds_inc_m=600;
      static unsigned short dds_acc_r, dds_acc_g, dds_acc_b, dds_acc_m;
      static char r,g,b,intensity;
      // set up DDS tables
      // 256 entries of 8-bits each
      for(i=0; i<256; i++){
        sine[i] = (int)(120.*sin((float)i*6.28/256.)+ 120);
        c[i] = (int)(120.*cos((float)i*6.28/256.)+ 120); 
        m[i] = (360.*((float)i/256.)); //  i to h in degrees
      }
      
      while(1) {
        // yield time 
        PT_YIELD_TIME_msec(DDS_sample_time) ;
        // DDS phase incrementers
        dds_acc_r += dds_inc_r ; 
        dds_acc_g += dds_inc_g ;
        dds_acc_b += dds_inc_b ;
        dds_acc_m += dds_inc_m ;
        
        for(i=0; i<PixelNum; i++){
            // shift dds_acc by 8 for index into table
            // add array index for motion
            // mask with 0xff for moduluo 256 operation
            
            
//            r = sine[((dds_acc_r>>8) + i) & 0xff ]; 
//            g = c[((dds_acc_g>>8) + i) & 0xff ] ;
//            b = sine[((dds_acc_b>>8)) & 0xff ] ; //(dds_acc_b>>8)
//            intensity = HALF_ON ;
//            set_pixel_rgb(i,r,g,b,intensity);
            
            h = m[((dds_acc_m>>8)+i) & 0xff];
            v = 1.0;
            s = 1.0;
            intensity = HALF_ON ;
            set_pixel_hsv(i, h, s, v, intensity);
        }
        
        write_pixels();
       
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// === Main  ======================================================
void main(void) {
  
  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    SpiChnOpen(2, SPI_OPEN_ON | SPI_OPEN_MODE32 | SPI_OPEN_MSTEN | SPICON_CKP, 4);
    // SCK2 is pin 26 
    // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);
        
  // init the threads
  PT_INIT(&pt_timer);
  
  // round-robin scheduler for threads
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
  }
} // main

// === end  ======================================================

