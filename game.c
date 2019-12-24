/*
 * File:        ECE4760 POV Dino Game
 * Author:      Zesun Yang and John Ly
 * For use with Sean Carroll's Little Board
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
// threading library
#include "pt_cornell_1_2_1.h"
//i2c for accelerometer
#include "i2c_header.h"
#include "dino.h"
#include "fire.h"
#include <stdlib.h>
#include <math.h>
#include <plib.h>
// need for rand function
#include <stdlib.h>
// need for sin function
#include <math.h>

////////////////////////////////////

// lock out timer 2 interrupt during spi comm to port expander
// This is necessary if you use the SPI2 channel in an ISR.
// The ISR below runs the DAC using SPI2
#define start_spi2_critical_section INTEnable(INT_T2, 0)
#define end_spi2_critical_section INTEnable(INT_T2, 1)

////////////////////////////////////

char buffer[60];
// generate max 3 fireballs
#define nObs 3

// struct for fireballs
typedef struct{
    int x;
    int xrange;
    int y;
    int alive;
}GameObject;

// struct for dinosaur
typedef struct{
    int feetHeight;
    //left_xpos is update counter
    int left_xpos;
    //xrange is image width
    int xrange;
}Human;

//number of nFireBall on the POV
int nFireBallsNow = 1;

// dinosaur init
Human human={.feetHeight=29,.left_xpos=25,.xrange=11};

GameObject map[nObs];

unsigned int oldTime;
volatile unsigned int newTime;
volatile unsigned int capture_period;
int frameCount =0;
int fireUpdate = 0;
int jump = 0;
int dieFlag = 0;

volatile float xAccl;
volatile float yAccl;
volatile float zAccl;

volatile char r,g,b,intensity;
volatile int sine[256], c[256] ;
static unsigned short dds_inc_r=1500, dds_inc_g=1500, dds_inc_b=500, dds_inc_m=600;
volatile unsigned short dds_acc_r, dds_acc_g, dds_acc_b, dds_acc_m;
volatile float m[256];

int startGame = 0;


// === DOTSTAR COMMANDS =================
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
#define PixelNum 64

volatile int counter =0;
volatile unsigned int pwm_on_time;

volatile int current_time;

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

// erase entire dotstar strip
void clearDotStar(){
    static int i;
    for(i = 0; i <64; i++){
        r = 0;
        g = 0;
        b = 0;
        intensity =0;
        set_pixel_rgb(i,r,g,b,intensity);
    }
    write_pixels();
}

volatile int ready;
volatile int update_counter =0 ;
// ISR for input capture - IR sensors
void __ISR(_INPUT_CAPTURE_4_VECTOR,ipl3) Timer23Handler(void)
{
    mIC4ClearIntFlag();
    capture_period = mIC4ReadCapture();
    ready=1;
    // clear the timer every pass
    WriteTimer23(0);
    update_counter = 0;

}


void __ISR(_TIMER_4_VECTOR,ipl4) Timer4Handler(void)
{
    mT4ClearIntFlag();

    if(startGame){
        if(dieFlag){
        //blend the dinosuar
           static int i;
           for(i=21; i<29; i++){
               // a dinosaur color
               r = 0x2c;
               g = 0x90;
               b = 0x0f ;
               intensity = QUAR_ON ;
               set_pixel_rgb(i,r,g,b,intensity);
           }
           write_pixels();
        }else{
            current_time = ReadTimer23();
            clearDotStar();
            static int row;
            static int m;
            // write fireball based on updated location - phase lock
            for(m=0;m<nObs;m++){
              if(update_counter>=map[m].x && update_counter< (map[m].x+ map[m].xrange)){
                for(row = 0; row <11; row++){
                    r = ((fire_color[row][update_counter-map[m].x]>>16) & 0xff);
                    g = ((fire_color[row][update_counter-map[m].x]>>8) & 0xff);
                    b = (fire_color[row][update_counter-map[m].x] & 0xff);
                    intensity =QUAR_ON;
                    set_pixel_rgb(29-11+row,r,g,b,intensity);
                }
              }

            }

            // draw dinosaur based on dinosaur feet position
            if( (update_counter<human.left_xpos+human.xrange) && update_counter>=human.left_xpos ){
                for(row = 0; row <11; row++){
                    r = ((dino_color[row][update_counter-human.left_xpos]>>16) & 0xff);
                    g = ((dino_color[row][update_counter-human.left_xpos]>>8) & 0xff);
                    b = (dino_color[row][update_counter-human.left_xpos] & 0xff);
                    intensity =QUAR_ON;
                    set_pixel_rgb(human.feetHeight-11+row,r,g,b,intensity);
                }
            }
            write_pixels();
            // only if ready to play game and the dinosaur is not dead
            // we can play the game
            if(ready && !dieFlag){
                update_counter++;
            }
        }
    }else{

        // before game starts, run Bruce's dot star test code
        static int i ;
        for(i=0; i<PixelNum; i++){
            // shift dds_acc by 8 for index into table
            // add array index for motion
            // mask with 0xff for moduluo 256 operation
            r = sine[((dds_acc_r>>8) + i) & 0xff ];
            g = c[((dds_acc_g>>8) + i) & 0xff ] ;
            b = sine[((dds_acc_b>>8)) & 0xff ] ; //(dds_acc_b>>8)
            intensity = QUAR_ON ;
            set_pixel_rgb(i,r,g,b,intensity);
        }

        write_pixels();
    }

}

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_acc, pt_anim ;

// system 1 second interval tick
int sys_time_seconds ;
int period_ms;
int fireFrameCounter = 0;

static PT_THREAD (protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt);

     while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(100) ;

        // button for start game
        if(PORTAbits.RA0){
            startGame = 1;
        }
        // if reset game button is pressed, clear dead status
        if(PORTAbits.RA1){
            dieFlag = 0;
        }
        if(!dieFlag){
            frameCount++;

            //dino jump
            if((int)zAccl<13000){
               jump = 1;
            }
            if(frameCount>0 && frameCount<5 && jump==1){
               human.feetHeight-=5;
            }

            if(human.feetHeight<29&& frameCount>=5){
                human.feetHeight+=2;
            }
            if(human.feetHeight>=29){
                human.feetHeight=29;
                frameCount=0;
                jump = 0;
            }

            fireFrameCounter++;


            srand(sys_time_seconds);

            // if we don't current have 3 fire balls on screen
            // and the first fireball has gone at least 20 frames ahead
            // create a new fireball at the end of update counter
            if(fireFrameCounter >=20 && nFireBallsNow <3 ){
                fireFrameCounter = 0;
                GameObject newObj = {.x=89,.xrange=10, .y=15, .alive=1};
                map[nFireBallsNow] = newObj;
                nFireBallsNow++;

            }

            // status1 dino stand still, obsticle far (update)
            // status2 dino stand still, obsticle infront (die)
            // status3 dino jump and doged (update)
            // status4 dino jump and nothing is under

            static int m;
            if(human.left_xpos>=map[0].x+1 && human.feetHeight > 20){
                ready =0;
                dieFlag = 1;
            }else{
              // if the obsticle successfully went underneath the dino
              // shift down the obsticles
              if(map[0].x< human.left_xpos && human.feetHeight <= 20){
                for(m=0;m<nObs-1;m++){
                    map[m]=map[m+1];
                }
                GameObject newObj = {.x=89,.xrange=10, .y=15, .alive =1};
                map[nObs-1]= newObj;
              }else{
                // move the obsticle forward
                for(m=0;m<nObs-1;m++){
                    if (map[m].alive==1 ){
                      map[m].x-=2;
                    }
                }
              }

            }
        }

    }
//
    PT_END(pt);
// animation thread
}

// === Accelerometer Thread =================================================
static PT_THREAD (protothread_acc(struct pt *pt))
{
    PT_BEGIN(pt);
    int hall;


      while(1) {
          PT_YIELD_TIME_msec(30) ;
          //holding Xacc, Yacc and Zacc
          float values[3]= {0,0,0};
          readImuValues(values);
          hall = PORTBbits.RB3;
          //parse data
          xAccl = values[0];
          yAccl = values[1];
          zAccl = values[2];


        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// === Timer Thread =================================================
// update a 1 second tick counter
int position=0, dir=1;
#define DDS_sample_time 30

static PT_THREAD (protothread_timer(struct pt *pt)){
    PT_BEGIN(pt);

      // with a 16 bit DDS and 8-bit sine table index
      // frequency of sine output is related to increment as
      // inc = Fout * 2^16 * DDS_sample_time
      // e.g. for 2 Hz and sample time 0f 30 millisec:
      // inc = 2 * 2^16 * 0.030 = 3932
      // or an increment of about 2000 per Hz. (with 30 mS sample time)
      // set up DDS tables
      // 256 entries of 8-bits each
      static int i ;
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

      } // END WHILE(1)
  PT_END(pt);
} // timer thread


// === Main  ======================================================
void main(void) {
 //SYSTEMConfigPerformance(PBCLK);

  ANSELA = 0; ANSELB = 0;
   // our hall effect sensor pin
    mPORTBSetBits(BIT_3);
    mPORTBSetPinsDigitalIn(BIT_3);
    CNPDB = BIT_3;

    // game buttons
    mPORTASetBits(BIT_0 | BIT_1);
    mPORTASetPinsDigitalIn(BIT_0 | BIT_1);
    CNPDA = BIT_0 | BIT_1;



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

  // === Hall Effect Sensor ISR =====
    // open a 16bit timer with 2s timeout period65535
    OpenTimer23(T23_ON| T23_PS_1_1 |T23_SOURCE_INT, 10000000);

    //45283 -- 100 updates /rev
    OpenTimer4(T4_ON |T4_SOURCE_INT|T4_PS_1_1,45283);
    ConfigIntTimer4(T4_INT_ON|T4_INT_PRIOR_2);

    //==set up input capture
    OpenCapture4(IC_EVERY_FALL_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC |IC_ON| IC_CAP_32BIT);
    // turn on the interrupt so the captures are recorded
    ConfigIntCapture4(IC_INT_ON | IC_INT_PRIOR_2| IC_INT_SUB_PRIOR_3);
    INTClearFlag(INT_IC1);

    //ConfigINT4(EXT_INT_ENABLE|FALLING_EDGE_INT| EXT_INT_PRI_1);
    PPSInput(1, IC4, RPB3);


  // === Open i2c ================
  OpenI2C1(I2C_ON,0x02c);
  //set up liner acceleration sensor control register
  // 1.66khz 2g scale 400Hz filter BW:0x80
  char data[] ={0x80};

  i2c_write(CTRL1_XL,data,1);

  char data2[] ={56};
  i2c_write(CTRL9_XL,data2,1);

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_anim);
  PT_INIT(&pt_acc);

  // seed random color
  srand(1);

  GameObject newObj = {.x=89,.xrange=10, .y=15, .alive= 1};
  map[0]=newObj;

  // round-robin scheduler for threads
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      PT_SCHEDULE(protothread_anim(&pt_anim));
      PT_SCHEDULE(protothread_acc(&pt_acc));
      }
  } // main

// === end  ======================================================
