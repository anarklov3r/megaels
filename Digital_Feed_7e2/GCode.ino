/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** GCode mode ********** //////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Constants
#define VERSION        (1)  // firmware version
#define MAX_BUF        (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN (200)  // depends on your stepper motor.  most are 200.
//#define MIN_STEP_DELAY (50.0)
//#define MAX_FEED   (1000000.0/MIN_STEP_DELAY)
//#define MIN_FEED   (0.01)

//char  serialBuffer[MAX_BUF];  // where we store the message until we get a newline
//int   sofar;            // how much is in the serialBuffer
//float px, pz;      // location

// speeds
//float fr =     0;  // human version
//long  step_delay;  // machine version

// settings
//char mode_abs=1;   // absolute mode?


// for arc directions
#define ARC_CW          (1)
#define ARC_CCW         (-1)
// Arcs are split into many line segments.  How long are the segments?
#define MM_PER_SEGMENT  (10)


//Digital_Feed_7e2 Pins for steppers
#define Z_STEP 49
#define Z_DIR  43
#define Z_ENA  45

#define X_STEP 48
#define X_DIR  44
#define X_ENA  46

//------------------------------------------------------------------------------
// Stepper Driver Methods - Need to be updated to reflect Digital_Feed_7e2 Logic
//------------------------------------------------------------------------------

//was zstep
void zstep(int dir) { 
  digitalWrite(Z_ENA,LOW);
  digitalWrite(Z_DIR,dir==1? HIGH:LOW);
  digitalWrite(Z_STEP,HIGH);
  digitalWrite(Z_STEP,LOW);
}

//was xstep
void xstep(int dir) {
  digitalWrite(X_ENA,LOW);
  digitalWrite(X_DIR,dir==1?HIGH:LOW);
  digitalWrite(X_STEP,HIGH);
  digitalWrite(X_STEP,LOW);
}

void disable() {
  digitalWrite(Z_ENA,HIGH);
  digitalWrite(X_ENA,HIGH);
}


void setup_controller() {
  pinMode(Z_ENA,OUTPUT);
  pinMode(X_ENA,OUTPUT);
  pinMode(Z_STEP,OUTPUT);
  pinMode(X_STEP,OUTPUT);
  pinMode(Z_DIR,OUTPUT);
  pinMode(X_DIR,OUTPUT);
}

/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if(fr==nfr) return;  // same as last time?  quit now.

  if(nfr>MAX_FEED || nfr<MIN_FEED) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEED);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEED);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = 1000000.0/nfr;
  fr = nfr;
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npz new position z
 */
void position(float npx,float npz) {
  // here is a good place to add sanity tests
  px=npx;
  pz=npz;
}


/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newz the destination z position
 **/
void line(float newx,float newz) {
  long i;
  long over= 0;
  
  long dx  = newx-px;
  long dz  = newz-pz;
  int dirx = dx>0?1:-1;
  #ifdef INVERT_Z
  int dirz = dz>0?-1:1;  // because the motors are mounted in opposite directions
  #else
  int dirz = dz>0?1:-1;
  #endif
  dx = abs(dx);
  dz = abs(dz);

  if(dx>dz) {
    over = dx/2;
    for(i=0; i<dx; ++i) {
      zstep(dirx);
      over += dz;
      if(over>=dx) {
        over -= dx;
        xstep(dirz);
      }
      pause(step_delay);
    }
  } else {
    over = dz/2;
    for(i=0; i<dz; ++i) {
      xstep(dirz);
      over += dx;
      if(over >= dz) {
        over -= dz;
        zstep(dirx);
      }
      pause(step_delay);
    }
  }

  px = newx;
  pz = newz;
}


// returns angle of dz/dx as a value from 0...2PI
float atan3(float dz,float dx) {
  float atan = atan2(dz,dx);
  if(atan<0) atan = (PI*2.0)+a;
  return atan;
}


// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
void arc(float cx,float cz,float x,float z,float dir) {
  // get radius
  float dx = px - cx;
  float dz = pz - cz;
  float radius=sqrt(dx*dx+dz*dz);

  // find angle of arc (sweep)
  float angle1=atan3(dz,dx);
  float angle2=atan3(z-cz,x-cx);
  float theta=angle2-angle1;
  
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  
  theta=angle2-angle1;
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = ceil( len * MM_PER_SEGMENT );
 
  float nx, nz, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    nz = cz + sin(angle3) * radius;
    // send it to the planner
    line(nx,nz);
  }
  
  line(x,z);
}


/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parseNumber(char code,float val) {
  char *ptr=serialBuffer;  // start at the beginning of buffer
  while((long)ptr > 1 && (*ptr) && (long)ptr < (long)serialBuffer+sofar) {  // walk to the end
    if(*ptr==code) {  // if you find code on your walk,
      return atof(ptr+1);  // convert the digits that follow into a float and return it
    }
    ptr=strchr(ptr,' ')+1;  // take a step from here to the letter after the next space
  }
  return val;  // end reached, nothing found, return default val.
}


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(const char *code,float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px);
  output("Z",pz);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("Digital Feed 7e2 GCode Mode"));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00 [X(steps)] [Z(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G01 [X(steps)] [Z(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G02 [X(steps)] [Z(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - clockwise arc"));
  Serial.println(F("G03 [X(steps)] [Z(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - counter-clockwise arc"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(steps)] [Z(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
}

/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  int cmd = parseNumber('G',-1);
  switch(cmd) {
  case  0:
  case  1: { // line
    feedrate(parseNumber('F',fr));
    line( parseNumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parseNumber('Z',(mode_abs?pz:0)) + (mode_abs?0:pz) );
    break;
    }
  case 2:
  case 3: {  // arc
      feedrate(parseNumber('F',fr));
      arc(parseNumber('I',(mode_abs?px:0)) + (mode_abs?0:px),
          parseNumber('J',(mode_abs?pz:0)) + (mode_abs?0:pz),
          parseNumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parseNumber('Z',(mode_abs?pz:0)) + (mode_abs?0:pz),
          (cmd==2) ? -1 : 1);
      break;
    }
  case  4:  pause(parseNumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parseNumber('X',0),
              parseNumber('Z',0) );
    break;
  default:  break;
  }

  cmd = parseNumber('M',-1);
  switch(cmd) {
  case 18:  // disable motors
    disable();
    break;
  case 100:  help();  break;
  case 114:  where();  break;
  default:  break;
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}