#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>              //  include libraries
#include <LoRa.h>

const  int csPin = 18;          // LoRa radio chip select
const int resetPin = 14;       //  LoRa radio reset
const int irqPin = 26;         // change for your board; must  be a hardware interrupt pin

String outgoing;              // outgoing message

byte  msgCount = 0;            // count of outgoing messages
byte localAddress = 0x02;     // address of this device
byte destination = 0x01;      // destination to  send to

int Temperature = 0;        // Temperature Integer
int Humidity  = 0;           // Humidity Integer
float temperature, humidity;

int payloadLenght = 19;
byte Data[19];

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  float x;
  float y;
}struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;

// Create an array with all the structures
struct_message boardsStruct[3] = {board1, board2};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id-1].x = myData.x;
  boardsStruct[myData.id-1].y = myData.y;

    // Update the structure with the new incoming data
  if (myData.id == 1) {
    boardsStruct[0].x = myData.x;
    boardsStruct[0].y = myData.y;
    Serial.printf("Board 1 - Temperature: %.2f °C\n", boardsStruct[0].x);
    Serial.printf("Board 1 - Humidity: %.2f %%\n", boardsStruct[0].y);
  } else {
    // Handle other boards if needed
    Serial.println("Unknown board ID");
  }
}

void setup()  {
  Serial.begin(115200);                   // initialize serial
  delay (1000);  
  
  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(868E6))  {             // initialize ratio at 868 MHz
    Serial.println("LoRa init failed.  Check your connections.");
    while (true);                       // if failed,  do nothing
  }

  Serial.println("LoRa init succeeded.");

  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void  loop() {

  delay(4000);
  
  temperature = boardsStruct[0].x;
  humidity  = boardsStruct[0].y;

  Serial.println("Temperature  and Humidity before the calculation");
  Serial.print(temperature);
  Serial.print("  ");
  Serial.println(humidity);

  Temperature = int (temperature*10);
  Humidity = int (humidity*10);
  // prepare and schedule data for transmission  
    Data[0] = Temperature >> 8; 
    Data[1] = Temperature;
    Data[2] = Humidity >> 8;
    Data[3] = Humidity;
    Serial.println("Data of Temp and Hum after manipulation");
    Serial.println(Temperature);
    Serial.println(Humidity);
     
  sendMessage(Data);
}

void sendMessage(byte* outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             //  add sender address
  LoRa.write(msgCount);                 // add message ID
  //LoRa.write(outgoing.length());        // add payload length
  LoRa.write(payloadLenght);        // add payload length
  LoRa.write(outgoing, payloadLenght);                 //  add payload
  LoRa.endPacket();                     // finish packet and send  it
  msgCount++;                           // increment message ID
}
