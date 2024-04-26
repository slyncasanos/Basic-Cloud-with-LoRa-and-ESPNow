#include "thingProperties.h"
#include  <SPI.h>              // include libraries
#include <LoRa.h>

const int  csPin = 18;          // LoRa radio chip select
const int resetPin = 14;       //  LoRa radio reset
const int irqPin = 26;         // change for your board; must  be a hardware interrupt pin

byte msgCount = 0;            // count of outgoing  messages
byte localAddress = 0x01;     // address of this device
byte destination  = 0xFF;      // destination to send to
long lastSendTime = 0;        // last  send time
int interval = 1000;          // interval between sends

String  incoming;
String outgoing;              // outgoing message

void setup()  {
  // Initialize serial and wait for port to open:
  Serial.begin(115200);
  // This delay gives the chance to wait for a Serial Monitor without blocking if  none is found
  delay(1000);
   
  // override the default CS, reset, and  IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset,  IRQ pin

  if (!LoRa.begin(868E6)) {             // initialize ratio at 915  MHz
    Serial.println("LoRa init failed. Check your connections.");
    while  (true);                       // if failed, do nothing
  }
  
  Serial.println("LoRa  init succeeded.");

  delay(1500);

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the  higher number the more granular information you’ll get.
     The default is 0  (only errors).
     Maximum is 4
 */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void  loop() {
  ArduinoCloud.update();
  // Your code here
  // parse for a  packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());  
  //ciaoString = incoming;
  
  
}



/*
  Since CiaoString  is READ_WRITE variable, onCiaoStringChange() is
  executed every time a new value  is received from IoT Cloud.
*/
void onCiaoStringChange()  {
  // Add your  code here to act upon CiaoString change
  sendMessage(ciaoString);
}

void  sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start  packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        //  add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void onReceive(int  packetSize) {
  if (packetSize == 0) return;          // if there's no packet,  return

  // read packet header bytes:
  int recipient = LoRa.read();          //  recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength  = LoRa.read();    // incoming msg length

  Serial.println("Received from:  0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient,  HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message  length: " + String(incomingLength));
  Serial.println();

  incoming =  "";
  byte incomingData[20];
  int i=0;
  
  if (sender != 0x2) {
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
/*    if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error:  message length does not match length");
    return;                             //  skip rest of function
    } */
  }
  else {
    Serial.println("We  are in the case of data rceiving");    
    while (LoRa.available()) {
    incomingData[i]  = LoRa.read();
    i++;
    }    
  }
  
  

  // if the recipient  isn't this device or broadcast,
  if (recipient != localAddress && recipient  != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  if (sender  != 0x2) {
    // if message is for this device, or broadcast, print details:
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent  to: 0x" + String(recipient, HEX));
    Serial.println("Message ID: " + String(incomingMsgId));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message:  " + incoming);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();  
    ciaoString = "Device " + String (sender,DEC) + ": " + incoming;
    }
  else {
    humidity = ((incomingData[2] << 8) | incomingData[3]);
    humidity = humidity/10;
    temperature = ((incomingData[0] << 8) | incomingData[1]);
    temperature = temperature/10;
    Serial.println("Final Temperature: " + String (temperature));
    Serial.println("Final  Humidity: " + String (humidity));
  }
}
