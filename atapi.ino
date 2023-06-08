/* Wieza stereo
 * 
 * maniek86, 2023
 * 
 * 
 */

int datalines[16]={22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37};
int RST=47;
int DA0=40;
int DA1=41;
int DA2=42;
int CS0=43;
int CS1=44;
int DIOR=46;
int DIOW=45;

#define debug true


// notes:
/* Document used: https://www.bswd.com/sff8020i.pdf
 *  
 * While doing READ/WRITE, ALWAYS release control pins (not releasing them can cause weird things to happen, lost few hours because of this)
 * Also SET DA,CS LINES FIRST BEFORE DIOR/DIOW !!
 * 
 * Original author forgot to write A2 (Service Command) after packet data read. Because of that any next packet gonna fail with 0xB4. Just iadd writeIDE(ComSReg,0xA2,0xFF); 
 * Actually because of my errors I discovered some units seems to not comply with standards 
 */


// IDE Register addresses
const byte DataReg = 0xF0;         // Addr. Data register of IDE device.
const byte ErrFReg = 0xF1;         // Addr. Error/Feature (rd/wr) register of IDE device.
const byte SecCReg = 0xF2;         // Addr. Sector Count register of IDE device.
const byte SecNReg = 0xF3;         // Addr. Sector Number register of IDE device.
const byte CylLReg = 0xF4;         // Addr. Cylinder Low register of IDE device.
const byte CylHReg = 0xF5;         // Addr. Cylinder High register of IDE device.
const byte HeadReg = 0xF6;         // Addr. Device/Head register of IDE device.
const byte ComSReg = 0xF7;         // Addr. Command/Status (wr/rd) register of IDE device.
const byte AStCReg = 0xEE;         // Addr. Alternate Status/Device Control (rd/wr) register of IDE device.

// Program Variables
byte dataLval;                     // dataLval and dataHval hold data from/to 
byte dataHval;                     // D0-D15 of IDE
byte regval;                       // regval holds addr. of reg. to be addressed on IDE
byte reg;                          // Holds the addr. of the IDE register with adapted
                                   // nDIOR/nDIOW/nRST values to suit purpose.
byte cnt;                          // packet byte counter
byte idx;                          // index used as pointer within packet array
byte paclen = 12;                  // Default packet length
byte s_trck;                       // Holds start track
byte e_trck;                       // Holds end track
byte c_trck;                       // Follows current track while reading TOC
byte c_trck_m;                     // MSF values for current track
byte c_trck_s;
byte c_trck_f;
byte a_trck = 1;                   // Holds actual track from reading subchannel data
byte MFS_M;                        // Holds actual M value from reading subchannel data
byte MFS_S;                        // Holds actual S value from reading subchannel data
byte R_MFS_M;                       // relative values
byte R_MFS_S;
byte d_trck;                       // Destination track
byte d_trck_m;                     // MSF values for destination track
byte d_trck_s;
byte d_trck_f;
byte aud_stat = 0xFF;              // subchannel data: 0x11=play, 0x12=pause, 0x15=stop 
byte asc;
byte ascq;

byte returnASC() {
  return asc;
}
byte returnASCQ() {
  return asc;
}
byte returnAUD_STAT() {
  return aud_stat;
}
byte returnA_TRCK() {
  return a_trck;
}
byte returnR_MFS_M() {
  return R_MFS_M;
}
byte returnR_MFS_S() {
  return R_MFS_S;
}
byte returnE_TRCK() {
  return e_trck;
}

byte fnc[]= {
  0x1B,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=0 Open tray
  0x1B,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=16 Close tray
  0x1B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=32 Stop unit
  0x47,0x00,0x00,0x00,0x02,0x00,0x4C,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=48 Start PLAY
  0x4B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=64 PAUSE play
  0x4B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=80 RESUME play
  0x43,0x02,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=96 Read TOC (format field 00b)
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=112 unit ready
  0x5A,0x00,0x01,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=128 mode sense
  0x42,0x02,0x40,0x01,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=144 rd subch.
  0x03,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=160 req. sense
  0x4E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=176 Stop disk
};

// RD SUBCH:
/* 0x42 0x02 0x40 0x[01] ...
 * 
 * 01: 01 CD CURRENT POS
 *     02 MEDIA CATALOG NUM
 *     03 TRACK I. R. CODE (ISRC)
 */
// sense codes: (ASC)
/*
 * 3Ah - medium not present
 * 04h - NOT READY (ascq additional status but not present here)
 * 28h - NOT READY (TOC) (Given once?)
 * 4Eh - Overlapped command attempted (no status yet?)
 * 
 * Also returned after READ TOC:
 * 20,24,28,29,04,06,3A,57 are errors?
 */

byte mfs[3][99]; // store for every track (not memory effiecent but we have a lot of free memory anyway)

void reset_IDE(){
  pinMode(RST,OUTPUT);
  digitalWrite(RST,LOW);
  delay(40);
  digitalWrite(RST,HIGH);
  delay(20);
}

void highZ() {
  for (int i = 0; i <= 15; i++) {
    pinMode(datalines[i],INPUT);
  }
  
}

void releaseZ() {
  reg=0xff;
  digitalWrite(DIOR,bitRead(reg,7));
  digitalWrite(DIOW,bitRead(reg,6));
  digitalWrite(RST,bitRead(reg,5));
  digitalWrite(CS1,bitRead(reg,4));
  digitalWrite(CS0,bitRead(reg,3));
  digitalWrite(DA2,bitRead(reg,2));
  digitalWrite(DA1,bitRead(reg,1));
  digitalWrite(DA0,bitRead(reg,0));  
}

void readIDE (byte regval){
  reg = regval & B01111111;             // set nDIOR bit LOW preserving register address
                             // set all I/O pins to HIGH -> impl. nDIOR release
  highZ();
  digitalWrite(CS1,bitRead(reg,4));
  digitalWrite(CS0,bitRead(reg,3));
  digitalWrite(DA2,bitRead(reg,2));
  digitalWrite(DA1,bitRead(reg,1));
  digitalWrite(DA0,bitRead(reg,0));   
  delay(1); // important!!
  digitalWrite(DIOR,bitRead(reg,7));
  digitalWrite(DIOW,bitRead(reg,6));
  digitalWrite(RST,bitRead(reg,5));
  delay(5);
  byte a=0;
  for (int i = 0; i <= 7; i++) {
    a |= digitalRead(datalines[i]) << i; // odczytanie bitu i ustawienie go w odpowiedniej pozycji w bajcie
  }
  dataLval=a;
  a=0;
  for (int i = 0; i <= 7; i++) {
    a |= digitalRead(datalines[i+8]) << i; // odczytanie bitu i ustawienie go w odpowiedniej pozycji w bajcie
  }
  dataHval = a;
  delay(3);
  releaseZ();
  
}

void writeIDE (byte regval, byte dataLval, byte dataHval){
  reg = regval | B01000000;             // set nDIOW bit HIGH preserving register address
  
  digitalWrite(CS1,bitRead(reg,4));
  digitalWrite(CS0,bitRead(reg,3));
  digitalWrite(DA2,bitRead(reg,2));
  digitalWrite(DA1,bitRead(reg,1));
  digitalWrite(DA0,bitRead(reg,0));
  for (int i = 0; i < 8; i++) {
    pinMode(datalines[i+8],OUTPUT);
    digitalWrite(datalines[i+8], (dataHval >> i) & 0x01);
  }
  for (int i = 0; i < 8; i++) {
    pinMode(datalines[i],OUTPUT);
    digitalWrite(datalines[i], (dataLval >> i) & 0x01); // ustawienie wartości bitu na odpowiedniej linii
  }
  delay(1);
  digitalWrite(DIOR,bitRead(reg,7));
  digitalWrite(DIOW,bitRead(reg,6));
  digitalWrite(RST,bitRead(reg,5));
  delay(5);
  reg = regval & B10111111;             // set nDIOW LOW preserving register address
  digitalWrite(DIOR,bitRead(reg,7));
  digitalWrite(DIOW,bitRead(reg,6));
  digitalWrite(RST,bitRead(reg,5));
  digitalWrite(CS1,bitRead(reg,4));
  digitalWrite(CS0,bitRead(reg,3));
  digitalWrite(DA2,bitRead(reg,2));
  digitalWrite(DA1,bitRead(reg,1));
  digitalWrite(DA0,bitRead(reg,0));
  highZ();                              // All I/O pins to high impedance -> impl. nDIOW release
  releaseZ();
  delay(3);
}

void BSY_clear_wait(){
  do{
    readIDE(ComSReg);
  } while(dataLval & (1<<7));
}
void DRY_set_wait(){
     do{
        readIDE(ComSReg);
     }while((dataLval & ~(1<<6)) == true);
}
void DRQ_set_wait(){
     do{
        readIDE(ComSReg);
     }while((dataLval & ~(1<<3)) == true);
}

void DRQ_clear_wait(){
  do{
    readIDE(ComSReg);
  } while(dataLval & (1<<3));
}

bool DRQ_set_wait_ret() {
  readIDE(ComSReg);
  return ((dataLval & ~(1<<3)) == true);
}

void SendPac(){
     paclen=12;
     writeIDE(AStCReg, B00001010, 0xFF);     // Set nIEN before you send the PACKET command! 
     delay(2);
     writeIDE(ComSReg, 0xA0, 0xFF);           // Write Packet Command Opcode
     delay(4);
     for (cnt=0;cnt<paclen;cnt=cnt+2){        // Send packet with length of 'paclen' 
     dataLval = fnc[(idx + cnt)];             // to IDE Data Registeraccording to idx value
     dataHval = fnc[(idx + cnt + 1)];
     writeIDE(DataReg, dataLval, dataHval);
     delay(1);
     readIDE(AStCReg);                         // Read alternate stat reg.     
     readIDE(AStCReg);                         // Read alternate stat reg.          
     }
     BSY_clear_wait();
     delay(1);
}

void req_sense(){                                 // Request Sense Command is used to check
  idx=160;                                        // the result of the Unit Ready command.
  SendPac();                                      // The Additional Sense Code is used,
  delay(10);                                      // see table 71 in sff8020i documentation
  DRQ_set_wait();
  cnt=0;
  do{
       readIDE(DataReg);
       if (cnt == 6){
           asc=dataLval;                          // Store Additional Sense Code
           ascq=dataHval; //?
       }
       cnt++;
       readIDE(AStCReg);       
       readIDE(ComSReg);
     } while(dataLval & (1<<3));                  // Skip rest of packet
  writeIDE(ComSReg,0xA2,0xFF); 
}

void req_sense_noblock() {
  idx=160;                                        // the result of the Unit Ready command.
  SendPac();
}

void req_sense_noblock2() {
  cnt=0;
  do{
       readIDE(DataReg);
       if (cnt == 6){
           asc=dataLval;                          // Store Additional Sense Code
           ascq=dataHval; //?
       }
       cnt++;
       readIDE(AStCReg);       
       readIDE(ComSReg);
     } while(dataLval & (1<<3));                  // Skip rest of packet
  writeIDE(ComSReg,0xA2,0xFF);
}

void unit_ready(){                                // Reuests unit to report status
    idx=112;                                  // used to check_unit_ready
    SendPac();     
}

int checkError() {
  readIDE(ErrFReg);
    if(dataLval!=0) {
      if(debug) {
        Serial.print("ERROR: ");
        Serial.println(dataLval,HEX);
      }
    }
  return dataLval;
}

int play_disk(){
    idx = 48;                                     // pointer to play function and Play
    SendPac();                                    // from MSF location stored at idx=(51-56)
}                                                 // See also doc. sff8020i table 76

void stop_play(){
    idx = 32;                                     // pointer to stop unit function (dont use)
    SendPac();
}
void eject_tray(){
    idx = 0;                                      // pointer to eject function
    SendPac();
}
void load_tray(){
    idx = 16;                                     // pointer to load
    SendPac();
}
void pause_disk(){
     idx = 64;                                    // pointer to hold
     SendPac();
}
void resume_disk(){
     idx = 80;                                    // pointer to resume
     SendPac();
}
void stop_disk(){
    idx = 176;                                    // pointer to stop disk function
    SendPac();
}
void play_from_beg() {
  fnc[51] = 0;
  fnc[52] = 2;
  fnc[53] = 0;
  play_disk();
  delay(100);
  play_disk();
}

void cd_next() { // we could read all toc contents while loading but i already coded this
  int thetrack=a_trck;
  if(thetrack+1>e_trck) return;
  thetrack++;
  fnc[102]=thetrack;
  get_TOC();
  if(debug) {
    Serial.print("Trying to play ");
    Serial.println(thetrack,DEC);
  }
  fnc[102]=0;
  fnc[51] = mfs[0][thetrack];
  fnc[52] = mfs[1][thetrack];
  fnc[53] = mfs[2][thetrack];
  play_disk();
  delay(100);
  play_disk();
}

void cd_prev() {
  int thetrack=a_trck;
  if(thetrack-1<s_trck) return;
  thetrack--;
  fnc[102]=thetrack;
  get_TOC();
  if(debug) {
    Serial.print("Trying to play ");
    Serial.println(thetrack,DEC);
  }
  fnc[102]=0;
  fnc[51] = mfs[0][thetrack];
  fnc[52] = mfs[1][thetrack];
  fnc[53] = mfs[2][thetrack];
  play_disk();
  delay(100);
  play_disk();
}

void init_task_file(){
  writeIDE(ErrFReg, 0x00, 0xFF);            // Set Feature register = 0 (no overlapping and no DMA)
  writeIDE(CylHReg, 0x02, 0xFF);            // Set PIO buffer to max. transfer length (= 200h)
  writeIDE(CylLReg, 0x00, 0xFF);
  writeIDE(AStCReg, 0x02, 0xFF);            // Set nIEN, we don't care about the INTRQ signal
  BSY_clear_wait();                         // When conditions are met then IDE bus is idle,
  DRQ_clear_wait();                         // this check may not be necessary (???)
}

void get_TOC(){
  idx =  96;                             // Pointer to Read TOC Packet
  SendPac();                             // Send read TOC command packet
  delay(10);
  DRQ_set_wait();
  read_TOC();                            // Fetch result
}

void dumpLH() {
  if(!debug) return;
  Serial.print(dataLval,HEX);
  Serial.print(" ");
  Serial.print(dataHval,HEX);
  Serial.print(" : ");
}


bool read_TOC(){ 
        readIDE(ErrFReg);
        if(dataLval!=0) return false;

        readIDE(DataReg);                      // TOC Data Length not needed, don't care
        readIDE(DataReg);                      // Read first and last session
        s_trck = dataLval;
        e_trck = dataHval;  
        int tr=e_trck;      
        do{
           readIDE(DataReg);                   // Skip Session no. ADR and control fields
           readIDE(DataReg);                   // Read curent track number

           c_trck = dataLval;
           readIDE(DataReg);                   // Read M

           c_trck_m = dataHval;                // Store M of curent track
           readIDE(DataReg);                   // Read S and F
 
           c_trck_s = dataLval;                // Store S of current track
           c_trck_f = dataHval;                // Store F of current track
           
           if (c_trck == s_trck){              // Store MSF of first track
               //fnc[51] = c_trck_m;             // 
               //fnc[52] = c_trck_s;
               //fnc[53] = c_trck_f;            
           }           
           mfs[0][c_trck]=c_trck_m;
           mfs[1][c_trck]=c_trck_s;
           mfs[2][c_trck]=c_trck_f;

           if (c_trck == a_trck){              // Store MSF of actual track
               d_trck_m = c_trck_m;            // 
               d_trck_s = c_trck_s;
               d_trck_f = c_trck_f;            
           }                      
           if (c_trck == 0xAA){                // Store MSF of end position
               //fnc[54] = c_trck_m;
               //fnc[55] = c_trck_s;
               //fnc[56] = c_trck_f;
           }

           readIDE(ComSReg);
        } while(dataLval & (1<<3));            // Read data from DataRegister until DRQ=0
        writeIDE(ComSReg,0xA2,0xFF); 
        return true;

}

bool read_subch_cmd(){ // this reads current track info
        idx=144;                             // Pointer to read Subchannel Packet
        SendPac();                           // Send read Subchannel command packet

        DRQ_set_wait();
        readIDE(ErrFReg);
        if(dataLval!=0) return false;

        readIDE(DataReg);                    // Get Audio Status
        
        if(dataHval==0x13){                  // Play operation successfully completed
          dataHval=0x15;                     // means drive is neither paused nor in play
        }                                    // so treat as stopped
        if(dataHval==0x11|                   // playing
           dataHval==0x12|                   // paused
           dataHval==0x15)                   // stopped
           {aud_stat=dataHval;               // 
        }else{
            aud_stat=0;                      // all other values will report "NO DISC"
        }
        readIDE(DataReg);                    // Get (ignore) Subchannel Data Length
        readIDE(DataReg);                    // Get (ignore) Format Code, ADR and Control

        readIDE(DataReg);                    // Get actual track

        a_trck = dataLval;
        
        readIDE(DataReg);                    // Get M field of actual MFS data and
        MFS_M = dataHval;                    // store M it
        readIDE(DataReg);                    // get S and F fields
        MFS_S = dataLval;                    // Store S value
        // (relative)
        readIDE(DataReg);                    // Get M field of actual MFS data and
        R_MFS_M = dataHval;                    // store M it
        readIDE(DataReg);                    // get S and F fields
        R_MFS_S = dataLval;                    // Store S value
        do{
          readIDE(DataReg);
          readIDE(ComSReg);
        } while(dataLval & (1<<3));          // Read rest of data from Data Reg. until DRQ=0
        writeIDE(ComSReg,0xA2,0xFF); 
        return true;
}


bool initIDE() {
  // put your setup code here, to run once:
  int i=0;
  highZ();
  pinMode(RST,OUTPUT);
  pinMode(DA0,OUTPUT);
  pinMode(DA1,OUTPUT);
  pinMode(DA2,OUTPUT);
  pinMode(CS0,OUTPUT);
  pinMode(CS1,OUTPUT);
  pinMode(DIOR,OUTPUT);
  pinMode(DIOW,OUTPUT);
  if(debug) Serial.println("RESET");
  reset_IDE();
  delay(500);
  if(debug) Serial.println("BSY WAIT");
  BSY_clear_wait();
  if(debug) Serial.println("DRY WAIT");
  DRY_set_wait();
  
  readIDE(CylLReg);
  if(dataLval == 0x14){
  readIDE(CylHReg);
    
  if(dataLval == 0xEB){
    if(debug) Serial.println("Found ATAPI Dev.");
  } else {
    if(debug) Serial.println("No ATAPI Device!");
    return false;
  }
  } else {
    if(debug) Serial.print("No ATAPI Device!");
    return false;
  }

  writeIDE(HeadReg, 0x00, 0xFF);

  init_task_file();

  delay(500);

  writeIDE(ComSReg, 0x90, 0xFF);            // Issue Run Self Diagnostic Command
  readIDE(ErrFReg);
  if(dataLval == 0x01){
    if(debug) Serial.print("OK");
  }else{
    if(debug) Serial.print("Fail: ");            // Units failing this may still work fine
    if(debug) Serial.println(dataLval,HEX);
    if(debug) Serial.print("WAIT TILL READY ");
    int b=0;
    while(1) {
      delay(1000);
      if(debug) Serial.print(".");
      b++;
      if(b>5) return false;
      writeIDE(ComSReg, 0x90, 0xFF);            // Issue Run Self Diagnostic Command
      readIDE(ErrFReg); 
      if(dataLval==0x01) break;
    }
    if(debug) Serial.println("OK");
  }
  if(debug) Serial.println("IDENT");
  writeIDE (ComSReg, 0xA1, 0xFF);           // Issue Identify Device Command
  delay(500);                               // Instead of wait for IRQ. Needed by some dev.  
  //readIDE(AStCReg); //
  do{
    delay(1);
    readIDE(DataReg);
    if (cnt == 0){                                // Get supported packet lenght
      if(dataLval & (1<<0)){                      // contained in lower byte of first word
        paclen = 16;                              // 1st bit set -> use 16 byte packets
      }
    }

    if(cnt > 26 & cnt < 47){                      // Read Model
        if(debug) Serial.print(char(dataHval));
        if(debug) Serial.print(char(dataLval));
    }
    cnt++;
    readIDE(ComSReg);// Read Status Register and check DRQ,

  } while(dataLval & (1<<3));                     // skip rest of data until DRQ=0
  readIDE(AStCReg);
  DRQ_clear_wait();

  delay(1000);

  if(debug) Serial.println("\nUNIT READY");
  unit_ready();
  if(debug) Serial.println(" OK\nREQUEST SENSE");
  req_sense();  

  if(asc == 0x29){                                // Req. Sense returns 'HW Reset' 
    unit_ready();                                 // (ASC=29h) at first since we had one.
    req_sense();                                  // New Req. Sense returns if media
  }                                               // is present or not.
  do{
     unit_ready();                                // Wait until drive is ready.
     req_sense();                                 // Some devices take some time
  }while(asc == 0x04);                            // ASC=04h -> LOGICAL DRIVE NOT READY
  if(debug) Serial.println("INIT PASS!!");
  return true;
}
