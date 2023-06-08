/* Wieza stereo
 * 
 * maniek86, 2023
 * 
 * 
 */


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

int bplus=A1;
int bminus=A3;
int bplay=A0;
int bmode=A2;

int omute=2;
int oswitch=3;

LiquidCrystal_I2C lcd(0x3F,16,2);

uint32_t dfreq;
byte freq=87;
byte freq_t=5;

void setup() {
  pinMode(bplus,INPUT_PULLUP);
  pinMode(bminus,INPUT_PULLUP);
  pinMode(bplay,INPUT_PULLUP);
  pinMode(bmode,INPUT_PULLUP);
  pinMode(omute,OUTPUT);
  pinMode(oswitch,OUTPUT);

  digitalWrite(oswitch,HIGH);
  digitalWrite(omute,HIGH);
  
  lcd.init();
  Wire.begin();
  tsa5523_setMute();
  Serial.begin(115200);
  //initIDE();
  lcd.backlight();
  lcd.print("INITIALIZING...");
  lcd.setCursor(0,1);
  if(!initIDE()) {
    lcd.print("IDE/ATA Error!");
    while(1) {
      
    }
  }
  
  freq=EEPROM.read(0);
  freq_t=EEPROM.read(1);
  bool invalid=false;
  if(freq>108&&freq<87) {
    invalid=true;
  }
  if(freq_t>9) {
    invalid=true;
  }

  if(invalid) {
    EEPROM.update(0,87);
    EEPROM.update(1,5);
    freq=87;
    freq_t=5;
  }
}

// order: plus, minus, play, mode
bool bhold[4]={0,0,0,0}; // holding status
bool bstate[4]={0,0,0,0}; // single push status (unclick)
bool brstate[4]={0,0,0,0}; // single push status (click, used only for CD)
int bholdticks[4]={0,0,0,0}; // hold time (one tick=one func call)

void resetButtons() {
  bstate[0]=0;
  bstate[1]=0;
  bstate[2]=0;
  bstate[3]=0;
  bhold[0]=0;
  bhold[1]=0;
  bhold[2]=0;
  bhold[3]=0;
  bholdticks[0]=0;
  bholdticks[1]=0;
  bholdticks[2]=0;
  bholdticks[3]=0;
}


void refreshButtons_loop() { //(for CD mode)
  if(!digitalRead(bplus)) {
    //bhold[0]=true;
    brstate[0]=true;
    //bholdticks[0]=bholdticks[0]+1;
  } 

  if(!digitalRead(bminus)) {
    //bhold[1]=true;
    brstate[1]=true;
    //bholdticks[1]=bholdticks[1]+1;
  } 

  if(!digitalRead(bplay)) {
    //bhold[2]=true;
    brstate[2]=true;
    //bholdticks[2]=bholdticks[2]+1;
  } 

  if(!digitalRead(bmode)) {
    //bhold[3]=true;
    brstate[3]=true;
    //bholdticks[3]=bholdticks[3]+1;
  } 
}

void refreshButtons() {
  bstate[0]=0;
  bstate[1]=0;
  bstate[2]=0;
  bstate[3]=0;
  if(!digitalRead(bplus)) {
    bhold[0]=true;
    bholdticks[0]=bholdticks[0]+1;
  } else {
    if(bhold[0]) {
      bstate[0]=1;
      bholdticks[0]=0;
    }
    bhold[0]=false;
  }

  if(!digitalRead(bminus)) {
    bhold[1]=true;
    bholdticks[1]=bholdticks[1]+1;
  } else {
    if(bhold[1]) {
      bstate[1]=1;
      bholdticks[1]=0;
    }
    bhold[1]=false;
  }

  if(!digitalRead(bplay)) {
    bhold[2]=true;
    bholdticks[2]=bholdticks[2]+1;
  } else {
    if(bhold[2]) {
      bstate[2]=1;
      bholdticks[2]=0;
    }
    bhold[2]=false;
  }

  if(!digitalRead(bmode)) {
    bhold[3]=true;
    bholdticks[3]=bholdticks[3]+1;
  } else {
    if(bhold[3]) {
      bstate[3]=1;
      bholdticks[3]=0;
    }
    bhold[3]=false;
  }
}



void fm_upFreq() {
  if(freq==108) {
      // nothin
    } else {
      if(freq_t==9) {
        freq++;
        freq_t=0;
      } else {
        freq_t++;
      }
    }
}

void fm_downFreq() {
  if(freq==87) {
    if(freq_t>5) {
      freq_t--;
    }
  } else {
    if(freq_t==0) {
      freq--;
      freq_t=9;
    } else {
      freq_t--;
    }
  }
}

bool fm_cancelFmChange=false;
bool fm_bits[2]={0,0};
bool fm_bits_c[2]={0,0};

void fm_mode() {
  dfreq=(uint32_t)((uint32_t)freq*(uint32_t)1000)+(uint32_t)((uint32_t)freq_t*(uint32_t)100);
  tsa5523_setFMFrequency(dfreq);
  digitalWrite(oswitch,HIGH);
  digitalWrite(omute,LOW);
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("|FM|");
  if(freq<100) {
    lcd.setCursor(2,1);
  } else {
    lcd.setCursor(1,1);
  }
  lcd.print(freq);
  lcd.print(".");
  lcd.print(freq_t);
  lcd.print(" MHz");
  int noUpdateTicks=0;

  while(1) {
    refreshButtons();

    // update FM bits
    byte a=fm_readRegister();
    fm_bits[0]=bitRead(a,4);
    fm_bits[1]=bitRead(a,0);
    if(!fm_bits[0]&&fm_bits[1]) fm_bits[1]=false;

    // button code
    if(bholdticks[0]>30) {// 1.5 secs 
      fm_upFreq();
      fm_cancelFmChange=true;
    }

    if(bholdticks[1]>30) {// 
      fm_downFreq();
      fm_cancelFmChange=true;
    }
    // mode changer
    if(bholdticks[3]>60) {// 3 secs
      tsa5523_setMute();
      return;
    }
    
    if(bstate[0]&&!fm_cancelFmChange) {
      fm_upFreq();
    }
    if(bstate[1]&&!fm_cancelFmChange) {
      fm_downFreq();
    }
    // save Frequency after some time
    noUpdateTicks++;
    if(noUpdateTicks==200) {// 10 secs
      EEPROM.update(0,freq);
      EEPROM.update(1,freq_t);
    }
    

    // display freq and update freq
    if(bstate[0]||bstate[1]||fm_cancelFmChange) {
      lcd.setCursor(1,1);
      if(freq<100) {
        lcd.print(" ");
      } 
      lcd.print(freq);
      lcd.print(".");
      lcd.print(freq_t);
      lcd.print(" MHz");
      dfreq=(uint32_t)((uint32_t)freq*(uint32_t)1000)+(uint32_t)((uint32_t)freq_t*(uint32_t)100);
      fm_cancelFmChange=false;
      noUpdateTicks=0;
      Serial.println(dfreq);
      tsa5523_setFMFrequency(dfreq);
      //tsa5523_setFMFrequency(93800);
    }
    // display fm_bits
    if(fm_bits[0]!=fm_bits_c[0]) {
      fm_bits_c[0]=fm_bits[0];
      lcd.setCursor(5,0);
      if(fm_bits_c[0]) {
        lcd.print("TUNE");
      } else {
        lcd.print("    ");
      }
    }
    if(fm_bits[1]!=fm_bits_c[1]) {
      fm_bits_c[1]=fm_bits[1];
      lcd.setCursor(10,0);
      if(fm_bits_c[1]) {
        lcd.print("STEREO");
      } else {
        lcd.print("      ");
      }
    }
    //end
    delay(50);
  }
}

bool cd_startplay=false;

void cd_mode() {
  digitalWrite(oswitch,LOW);
  digitalWrite(omute,LOW);
  cd_startplay=false;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("|CD|");
  uint32_t mil=millis();
  while(1) {
      Serial.print("A");
      mil=millis();
     get_TOC(); // to get sense
     req_sense_noblock();
     do{
      refreshButtons_loop(); 
    } while(DRQ_set_wait_ret());
    while(1) {
      refreshButtons_loop(); 
      if(millis()-mil>50) break;
    }
     req_sense_noblock2();
     byte asc=returnASC();
     byte ascq=returnASCQ();
     Serial.println("B");
     Serial.println(asc,HEX);
     if(asc==0x3A) {
      lcd.setCursor(0,1);
      lcd.print("NO DISC           ");
      cd_startplay=false;
     }
     if(asc==0x04) {
      if(ascq==0x00) {
        lcd.setCursor(0,1);
        lcd.print("INVALID DISC         ");  
      }
      if(ascq==0x01) {
        lcd.setCursor(0,1);
        lcd.print("LOADING.         ");  
      }
      
     }
     if(asc==0x28) { // valid TOC detected?
        lcd.setCursor(0,1);
        lcd.print("READING.    ");  
        cd_startplay=false;
      }
      byte aud_stat=returnAUD_STAT();
      byte a_trck=returnA_TRCK();
      byte R_MFS_M=returnR_MFS_M();
      byte R_MFS_S=returnR_MFS_S();
      byte e_trck=returnE_TRCK();
      if(asc==0x4E) {
        lcd.setCursor(0,1);
        if(!cd_startplay) {
          lcd.print("TRACKS: ");
          lcd.print(e_trck, DEC);
          if(brstate[2]) {
            cd_startplay=true;
            play_from_beg();
            if(checkError()!=0) {
              lcd.setCursor(0,1);
              lcd.print("PLAY ERROR   ");
              cd_startplay=false;
              delay(3000);
            }
          }
        } else {
          read_subch_cmd();
          aud_stat=returnAUD_STAT();
          a_trck=returnA_TRCK();
          R_MFS_M=returnR_MFS_M();
          R_MFS_S=returnR_MFS_S();
          e_trck=returnE_TRCK();
          lcd.setCursor(0,1);
          if(aud_stat==0x11||aud_stat==0x12) { // playing
            if(a_trck<10) lcd.print("0");
            lcd.print(a_trck, DEC);
            lcd.print(" ");
            if(R_MFS_M<10) lcd.print("0");
            lcd.print(R_MFS_M,DEC);
            lcd.print(":");
            if(R_MFS_S<10) lcd.print("0");
            lcd.print(R_MFS_S,DEC);
            if(aud_stat==0x12) {
              lcd.print(" PAUSED      ");
            } else {
              lcd.print("            ");
            }
          }
          if(aud_stat==0x15) {
            lcd.print("STOPPED      ");
          }

          if(brstate[2]) {
            if(aud_stat==0x11) {
              pause_disk();
              lcd.setCursor(8,1);
              lcd.print(" PAUSE      ");
            }
            if(aud_stat==0x12) {
              resume_disk();
              lcd.setCursor(8,1);
              lcd.print("            ");
            }
            if(aud_stat==0x15) {
              play_from_beg();
            }
          }
          if(brstate[0]&&aud_stat!=0x15) {
            cd_next();
          }
          if(brstate[1]&&aud_stat!=0x15) {
            cd_prev();
          }
        }
        
      }
      Serial.print("C");
    //refreshButtons(); 
    refreshButtons();
    brstate[0]=false;
    brstate[1]=false;
    brstate[2]=false;
    brstate[3]=false;
    if(bholdticks[3]>2) {// about 3 secs
      stop_play(); // this stops unit
      return;
    }
    //delay(50);
  }
}

void aux_mode() {
  digitalWrite(oswitch,LOW);
  digitalWrite(omute,HIGH);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("|AUX/MUTE|");
  while(1) {
    refreshButtons();
    if(bholdticks[3]>60) {// 3 secs
      return;
    }
    delay(50);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  //lcd.setCursor(0,1);
  resetButtons();
  fm_mode();
  resetButtons();
  cd_mode();
  resetButtons();
  aux_mode();
}
