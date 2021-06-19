
/*
  xdrv_46_somfy.ino - Somfy RF Rolling Code Support
  Based on https://github.com/Nickduino/Somfy_Remote 

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  Based on
  https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
  https://github.com/arendst/Tasmota/pull/7969
  https://github.com/arendst/Tasmota/issues/7870
  https://sourceforge.net/p/culfw/code/HEAD/tree/trunk/culfw/clib/somfy_rts.c#l76
  https://github.com/LSatan/Simu_Remote_CC1101/blob/master/Simu_Remote_CC1101.cpp
  https://pushstack.wordpress.com/somfy-rts-protocol/
  http://www.bastelbudenbuben.de/2017/03/21/espressif-esp8266-12f-mit-cc1101-transceiver-im-asynchronous-mode-betreiben/
  https://github.com/Nickduino/Somfy_Remote/blob/master/Somfy_Remote.ino
  https://github.com/LSatan/SmartRC-CC1101-Driver-Lib/blob/master/img/Esp8266_CC1101.png
*/

#ifdef USE_SOMFY

#include <ELECHOUSE_CC1101_SRC_DRV.h>

#define XDRV_46 46

/*********************************************************************************************\
 * Send Somfy RTS commands via a connected 433MHz transmitter module
\*********************************************************************************************/
const char kSomfyCommands[] PROGMEM = "Somfy|" // prefix
  "Up|Down|Stop|Prog|Set|Values";

void (* const somfyCommand[])(void) PROGMEM = {
  &CmdUp, &CmdDown, &CmdStop, &CmdProg, &CmdSomfySet, &CmdValues};

#define SYMBOL 640
#define SOMFY_UP 0x2
#define SOMFY_STOP 0x1
#define SOMFY_DOWN 0x4
#define SOMFY_PROG 0x8

byte frame[7];

void BuildFrame(byte *frame, byte button);
void sendResponse ( int slot );


void somfysetup() {
  pinMode(Pin(GPIO_CC1101_GDO0), OUTPUT);
  digitalWrite(Pin(GPIO_CC1101_GDO0), LOW);
}

uint8_t sync = 0;
uint8_t cyclicStep = 0x00;
int cyclicWait = 0;
int repeats = 0;
int somfySlot = 0;


void Somfy_FUNC_EVERY_50_MSECOND(void)
{
  if (cyclicStep == 0x00) { return; }
  switch ( cyclicStep )
  {
    case 0x01:
      //
      // Send the wake-up signal ...
      //
      AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("SFY: %lu ********* Sending Wakeup"), millis());
      
      digitalWrite(Pin(GPIO_CC1101_GDO0), HIGH);
      delayMicroseconds(9415);
      digitalWrite(Pin(GPIO_CC1101_GDO0), LOW);

      //delayMicroseconds(89565);
      // now we need to wait for almost 90ms
      // we do this by return control back to tasmota and expect to be resumed after 50ms of waiting
      //
      cyclicWait = 1; // = 50ms
      cyclicStep = 0x02; // next step.
      sync = 2;
      break;

    case 0x02:
      //AddLog_P(LOG_LEVEL_DEBUG, PSTR("SFY: %lu ********* wait"), millis());
      cyclicWait--; 
      if ( cyclicWait <= 0 ) {
        //AddLog_P(LOG_LEVEL_DEBUG, PSTR("SFY: %lu ********* wait (end)"), millis());

        // we have waited for 50ms ... so we need to wait the remaining time to have waited ~89565us
        // calculating with 53000 (although it would be 50000) as we have no hard real time here, a little more 
        // time will have passed than 50ms
        if (sync == 2 ) delayMicroseconds(89565-53000);
    
        //AddLog_P(LOG_LEVEL_DEBUG, PSTR("SFY: %lu ********* transmission"), millis());
        // Hardware sync: two sync for the first frame, seven for the following
        // ones.
        for (int i = 0; i < sync; i++) {
          digitalWrite(Pin(GPIO_CC1101_GDO0), HIGH);
          delayMicroseconds(4 * SYMBOL);
          digitalWrite(Pin(GPIO_CC1101_GDO0), LOW);
          delayMicroseconds(4 * SYMBOL);
        }
        sync = 7; 

        // Software sync
        digitalWrite(Pin(GPIO_CC1101_GDO0), HIGH);
        delayMicroseconds(4550);
        digitalWrite(Pin(GPIO_CC1101_GDO0), LOW);
        delayMicroseconds(SYMBOL);

        // Data: bits are sent one by one, starting with the MSB.
        for (byte i = 0; i < 56; i++) {
          if (((frame[i / 8] >> (7 - (i % 8))) & 1) == 1) {
            digitalWrite(Pin(GPIO_CC1101_GDO0), LOW);
            delayMicroseconds(SYMBOL);
            digitalWrite(Pin(GPIO_CC1101_GDO0), HIGH);
            delayMicroseconds(SYMBOL);
          } else {
            digitalWrite(Pin(GPIO_CC1101_GDO0), HIGH);
            delayMicroseconds(SYMBOL);
            digitalWrite(Pin(GPIO_CC1101_GDO0), LOW);
            delayMicroseconds(SYMBOL);
          }
        }

        // send inter-frame gap
        // if last bit = 0, silence is 1/2 symbol longer
        // delayMicroseconds(30415);  // Inter-frame silence
        digitalWrite(Pin(GPIO_CC1101_GDO0), LOW);
        //delayMicroseconds(30415 + ((frame[6] >> 7) & 1) ? 0 : SYMBOL); // it seems to be non critical - as we now wait 50ms

        cyclicWait = 0;

        repeats--;
        if ( repeats <= 0 ) {

          // we're done
          // put radio to sleep
          ELECHOUSE_cc1101.SpiStrobe(0x36);//Exit RX / TX, turn off frequency synthesizer and exit
          ELECHOUSE_cc1101.SpiStrobe(0x39);//Enter power down mode when CSn goes high.

          // terminate loop
          cyclicStep = 0x00;

          // store next rolling code
          Settings.somfyRemoteSettings[somfySlot].rolling_code++; // increment rolling code by one

          AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("SFY: %lu ********* Done with Transmission"), millis());

        } else {
          // terminate.
          // We stay in this step for an other round ...
        }
      }
  }

  
}

void BuildFrame(byte *frame, byte button, unsigned int code, unsigned int remote) {

  
  frame[0] = 0xA1; // Encryption key. Doesn't matter much
  frame[1] = button << 4;  // Which button did you press? The 4 LSB will be the checksum
  frame[2] = code >> 8;    // Rolling code (big endian)
  frame[3] = code;         // Rolling code
  frame[6] = remote >> 16; // Remote address
  frame[5] = remote >>  8; // Remote address
  frame[4] = remote;       // Remote address



// Checksum calculation: a XOR of all the nibbles
  byte checksum = 0;
  for(byte i = 0; i < 7; i++) {
    checksum = checksum ^ frame[i] ^ (frame[i] >> 4);
  }


  checksum &= 0b1111; // We keep the last 4 bits only
//Checksum integration
  frame[1] |= checksum; //  If a XOR of all the nibbles is equal to 0, the blinds will
                        // consider the checksum ok.


// Obfuscation: a XOR of all the bytes
  for(byte i = 1; i < 7; i++) {
    frame[i] ^= frame[i-1];
  }

}


void sendResponse ( int slot )
{
  Response_P(PSTR("{\"%s\":\"%s\", \"RollingCode\":\"0x%04x\", \"Address\":\"0x%06x\"}"),
                        XdrvMailbox.command,
                        D_JSON_DONE,
                        Settings.somfyRemoteSettings[slot].rolling_code,
                        Settings.somfyRemoteSettings[slot].address
                        );
}



void CmdUp()
{
  AddLog_P(LOG_LEVEL_DEBUG, PSTR("SFY: Payload Up: %d, i %d"), XdrvMailbox.payload, XdrvMailbox.index);
  Cmd(SOMFY_UP);
}

void CmdDown()
{
  AddLog_P(LOG_LEVEL_DEBUG, PSTR("SFY: Payload Down: %d, i %d"), XdrvMailbox.payload, XdrvMailbox.index);
  Cmd(SOMFY_DOWN);
}

void CmdStop()
{
  AddLog_P(LOG_LEVEL_DEBUG, PSTR("SFY: Payload Stop: %d, i %d"), XdrvMailbox.payload, XdrvMailbox.index);
  Cmd(SOMFY_STOP);
}

void CmdProg()
{
  AddLog_P(LOG_LEVEL_DEBUG, PSTR("SFY: Payload Prog: %d, i %d"), XdrvMailbox.payload, XdrvMailbox.index);
  Cmd(SOMFY_PROG);
}


void CmdValues()
{
  AddLog_P(LOG_LEVEL_DEBUG, PSTR("SFY: Payload Values: %d, i %d"), XdrvMailbox.payload, XdrvMailbox.index);
  int slot = XdrvMailbox.index;
  if ( slot > MAX_SOMFY_REMOTES ) 
  {
    AddLog_P(LOG_LEVEL_ERROR, PSTR("SFY: Index out of bounds"));
    return;
  }
  else slot = slot -1;
  
  sendResponse(slot);
}

void prepareCC1101() {
  
  AddLog_P(LOG_LEVEL_DEBUG, PSTR("SFY:setting params for CC1101"));

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setGDO(Pin(GPIO_CC1101_GDO0), 0);
  ELECHOUSE_cc1101.setCCMode(0);
  ELECHOUSE_cc1101.setModulation(2);  // set modulation mode ASK/OOK
  // ELECHOUSE_cc1101.setChsp(350.00);
  ELECHOUSE_cc1101.setDRate(828);         // Set the Data Rate in kBaud.
  //ELECHOUSE_cc1101.setPA(10);             // might be important to increase radio power
  ELECHOUSE_cc1101.setMHZ(433.42);
  ELECHOUSE_cc1101.setPktFormat(1);  // Data on GDO0
  ELECHOUSE_cc1101.SetTx();
}

/*
 * Determine the current requested slot id
 */
int getSlot()
{
  int slot = XdrvMailbox.index;
  if ( slot > MAX_SOMFY_REMOTES ) 
  {
    AddLog_P(LOG_LEVEL_ERROR, PSTR("SFY: Index out of bounds"));
    return -1;
  }
  else slot = slot -1;  
  return slot;
}

/*
 * Main function: Send command.
 */
void Cmd(byte cmdCode)
{
  int slot = getSlot();
  if ( slot < 0 ) return; // We have no valid slot, do nothing, just return.
  
  prepareCC1101();
 
  AddLog_P(LOG_LEVEL_INFO, PSTR("Sending 0x%02x using remote 0x%06x. Current Somfy Rolling Code: 0x%x"), cmdCode, Settings.somfyRemoteSettings[slot].address, Settings.somfyRemoteSettings[slot].rolling_code);

  somfysetup();
  BuildFrame(frame, cmdCode, Settings.somfyRemoteSettings[slot].rolling_code, Settings.somfyRemoteSettings[slot].address);

  // Prepare command for asynchronuous processing ...
  cyclicStep = 0x01; 
  repeats = 7; // we want the command sent 7 times
  somfySlot = slot;
  
  sendResponse(slot);
}

void CmdSomfySet(void)
{
  AddLog_P(LOG_LEVEL_DEBUG, PSTR("SFY: Payload Set: %d, i %d"), XdrvMailbox.payload, XdrvMailbox.index);
  int slot = getSlot();
  if ( slot < 0 ) return; // We have no valid slot, do nothing, just return.
  
  if (XdrvMailbox.data_len > 0) {
    if (XdrvMailbox.payload > 0) {
      char *p;
      uint32_t i = 0;
      uint32_t param[2] = { 0 };
      for (char *str = strtok_r(XdrvMailbox.data, ", ", &p); str && i < 2; str = strtok_r(nullptr, ", ", &p)) {
        param[i] = strtoul(str, nullptr, 0);
        i++;
      }
      
      Settings.somfyRemoteSettings[slot].address = param[0];
      Settings.somfyRemoteSettings[slot].rolling_code = param[1];

      sendResponse(slot);

    } else {
      DEBUG_DRIVER_LOG(LOG_LEVEL_DEBUG_MORE, PSTR("no payload"));
    }
  } else {
    DEBUG_DRIVER_LOG(LOG_LEVEL_DEBUG_MORE, PSTR("no param"));
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/
bool Xdrv46(uint8_t function) {
  bool result = false;

  // if(99 == Pin(GPIO_CC1101_GDO0)) return result;

  switch (function) {
    case FUNC_COMMAND:
      if (!PinUsed(GPIO_CC1101_GDO0)) {
        AddLog_P(LOG_LEVEL_ERROR, PSTR("Cannot execute command - IOs not defined!"));
        ResponseCmndChar("Error");
        return false;
      }
      AddLog_P(LOG_LEVEL_DEBUG_MORE, PSTR("calling command"));
      result = DecodeCommand(kSomfyCommands, somfyCommand);
      break;
    case FUNC_INIT:
      DEBUG_DRIVER_LOG(LOG_LEVEL_DEBUG, PSTR("Somfy init done."));
      break;
    case FUNC_EVERY_50_MSECOND:
      Somfy_FUNC_EVERY_50_MSECOND();
      break;
  }

  return result;
}
#endif  // USE_SOMFY
