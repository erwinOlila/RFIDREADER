/*
This is the code for the RFID reader where its functions aside from detecting the
occupancy are sending message to its 3 connected
clients to control the loads as well as fetching the load status, connecting to
the repeater to send the accumulated data which include the occupancy of the room and the
the status of each load. It also has an actuator connected to it which enables it to
control the a lighting load.

Load definition:
1LI:  lighting load connected to the rfid sensor located at the front of the clasroom
2LI:  Another lighting load at the back side of the classroom
1AC:  Airconditioning unit at the front
2AC:  Airconditioning unit at the back


Scenario: When the faculty enters the room and inserts the rfid card to the sensor,
(the card stays with the sensor), 1LI will turn on. It then sends signal to its 3 clients
to also turn on the loads (appr. 5 seconds after 1LI is turned on) to turn 2LI, 1AC and
2AC. Approx 20 seconds after, the transceiver will send the fetched data it is connected
to which will then be sent by the repeater to the basestation to perform the algorithm.
Based on the algorithm, the repeater will send the command to the rfid reader whether
the load will stay On or not.
*/

#include "MFRC522.h"
#include <ESP8266WiFi.h>
#include <MyCommonFun.h>

#define LOADNUM 0

#define IND 2

MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance
WiFiClient client[MAX_CONN];

int readIDCounter = 0;
int idRemovedCounter = 0;
int counts = 0;

// Initialization for arrays
char messageCommand[] = "1100";      //Set initial state of loads ON |1LI|2LI|1AC|2AC|
char messageToRepeater[] = "0000000000000000000000000";   //24 characters; 25 size
char lightCon[] = "11";             // Lighting states
char loadMessage[] = "0000000000000000"; // Array for receiving message from clients
char rfid[] = "00000000";          // array for rfid
char listen[] = "xxx";             // array for receiving message from light sensors

int i = 0;
int c = 0;

unsigned long cardRemovedMillis = 0;  // Counter for how long the card is removed
unsigned long cardInsertedMillis = 0; // Counter for how long the card is inserted
unsigned long cardInsertedTimeOut = 0;
unsigned long cardRemovedTimeOut = 0;


WiFiServer server(PORT); // Create instance for server

/**************************Continued on the next page**************************/

void setup() {
  Serial.begin(115200);    // Initialize serial communications
  SPI.begin();            // Star SPI bus
  mfrc522.PCD_Init();     // Start MFRC522

  //Initialize digital pins
  pinMode(IND, OUTPUT);
  pinMode(LOADCON, OUTPUT);
  pinMode(LOADSTA, INPUT);

  //Set the device as both AP and Station mode
  WiFi.mode(WIFI_AP_STA);

  //Connect to the repeater device
  conBuffer(ssidRep, passwordRep, IND);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());

  //Setup AP configuration
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(ip, gateway, subnet);
  server.begin();
  Serial.print("WiFi is created with IP: ");
  IPAddress myIP = WiFi.softAPIP();
  Serial.println(myIP);
}

void loop() {
  // IF connection is interrupted, reconnect
  if (WiFi.status() != WL_CONNECTED) {
    conBuffer(ssidRep, passwordRep, IND);
  }

  //Detect for any inserted card
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
    if (millis() - cardRemovedMillis > 5000){   // If no card deteced after 5 seconds
      readIDCounter = 0;
      idRemovedCounter += 1;           // If no card is inserted, increment a counter
      if(idRemovedCounter == CARDREM_PERIOD){
        //if it reaches the period, this will report that room is unoccupied
        send_card_empty();
        idRemovedCounter = 0;     // reset counter
      }
    }
    delay(1000);
    return;
  }
  if ( ! mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  //If card is detected, a counter is incremented
  idRemovedCounter = 0;     // Reset this counter
  readIDCounter += 1;
  digitalWrite(IND, LOW);   // Trigger LED (pin IND is source mode)

  /*************************Continued on the next page*************************/
  // This is ran every cycle
  if(readIDCounter ==1){
    Serial.println("Card Inserted");
    cardInsertedMillis = millis();
    send_when_inserted();     //This will send command to the clients to turn on
  }

  // This is ran every cycle
  if(readIDCounter == CARDINS_PERIOD){
    dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    client_start(messageCommand); // Communicate with Clients
    readIDCounter = 1;            // reset the counter back to 1
  }
  cardRemovedMillis = millis();
  if (messageCommand[LOADNUM] == '0') {
    digitalWrite(LOADCON, LOW); // Turn off
  }
  else {
    digitalWrite(LOADCON, HIGH); // Turn on
  }
  delay(1000);
}

// Send report to the clients to turn on
// This should run only once and at the start of card insertion
void send_when_inserted(){
  digitalWrite(LOADCON, 1);       // Turn 1LI ON
  commandSet(messageCommand, 0);  // Turn ON other remaining loads
  // Read rfid for 3 seconds
  do {
    dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size); // read and convert rfid
    client_start(messageCommand);
  }while(millis()-cardInsertedMillis < TIMEFETCH);
    Serial.println("Exit after 3 seconds of card insertion");
}

/**
* READ RFID
* @params *buffer: byte - message
* @ params bufferSize: byte - message size
**/
byte dump_byte_array(byte *buffer, byte bufferSize) {
  char msgType[] = "OCC";
  String rfidString = "";

  // Read each byte
  for (byte i = 0; i < bufferSize; i++) {
    rfidString  += String(buffer[i], HEX);
  }

  rfidString.toCharArray(rfid, rfidString.length() + 1); // Convert to char array
  sendToRepeater(rfid, msgType);      // Process data to be sent to the repeater
  Serial.println("Sending that card is inserted...");
}

/*************************Continued on the next page*************************/
// Fetch data from the clients and then send data to the repeater
/**
* FETCH DATA FROM CLIENTS AND REPEATER AND DISTRIBUTE
* @params control - pointer to the array that handles message from repeater
**/
void client_start(char* control){   //control: pointer to messageCommand
  char msgType[] = "STA";
  int c = 0;

  Serial.println("Initializing...");
  Serial.print("Sending clients to: ");
  Serial.println(control);

  client[0].connect(serverRep,PORT);
  Serial.println(loadMessage);

  // Loop through clients
  for(i = 1; i < MAX_CONN; i++){
      client[i] = server.available();     // Open connection
      digitalWrite(IND, HIGH);

      // When a client has connected
      if(client[i]){
        digitalWrite(IND, LOW);
        Serial.println("Client " +String(i)+ " is connected");
        client[i].print(control); // Distribute the message

        // If data is available to read
        if (client[i].available()) {
          while (client[i].available()) {
            Serial.println("reading...");
            char* m = loadMessage;
            *(m + c) = client[i].read(); // Read data per byte

            // Read light sensor data
            /*
            * X: indoor sunlight read by 1AC is high
            * x: indoor sunlight read by 1AC is low
            * Y: indoor sunlight read by 2AC is high
            * y: indoor sunlight read by 2AC is low
            */
            if ((*(m + c)) == 'X') {
              lightCon[0] = '0'; // turn off 1LI
              c--;
            }
            if ((*(m + c)) == 'x') {
              lightCon[0] = '1'; // turn on 1LI
              c--;
            }
            if ((*(m + c)) == 'Y') {
              lightCon[1] = '0'; // turn off 2LI
              c--;
            }
            if ((*(m + c)) == 'y') {
              lightCon[1] = '1'; // turn on 2LI
              c--;
            }
/*************************Continued on the next page*************************/
            else {
            }
            c++;      // Gets the number of characters of the message
          }
        }
        Serial.println(c);
        Serial.println(loadMessage);
        delay(100);
      }
   }

   strcpy(&loadMessage[c], loadType); // Insert "1LI"
   Serial.println("Sending load status...");
   Serial.println(loadMessage);

   // Check load status
   if (digitalRead(LOADCON) == 1) {
    strcpy(&loadMessage[c + 3], "1");
   }
   else {
    strcpy(&loadMessage[c + 3], "0");
   }
   sendToRepeater(loadMessage, msgType);
}

/**
* Turn off loads if no card is present after cut-off time
**/
void send_card_empty(){
  char message[] = "OUT";
  char msgType[] = "OCC";
  if (messageCommand[0] == '0') {
    digitalWrite(LOADCON, LOW);
  }
  else {
    digitalWrite(LOADCON, HIGH);
  }
  client_start(messageCommand);
  sendToRepeater(message, msgType);
  Serial.println("Sending that card is removed...");
}

/**
* @params message: pointer to array that handles load status/occupancy message
* @params dataType: poiner to array that handles dataType (OCC/STA)
**/
void sendToRepeater(char* message, char* dataType) {
  client_stop();

  // Process message
  strcpy(messageToRepeater, roomNum); // Add roomNum at the start of the message
  strcat(messageToRepeater, message); // Insert occupancy or load status message
  strcat(messageToRepeater, dataType); // Insert data type

  Serial.println("The message to repeater is: ");
  Serial.println(messageToRepeater);
  client[0].connect(serverRep,PORT);
/*************************Continued on the next page*************************/
  if(client[0].connected()) {
    Serial.println("Ready for sending...");
    client[0].print(messageToRepeater);     //Send Occupancy/Load status of the room
    unsigned long timeOut = millis();

    while (!client[0].available()) {
        delay(100);
        Serial.print(".");

        // If connection takes too long due to error, exit
        if ((millis() - timeOut) >= 5000) {
          c = 3;
          break;
        }
    }

    // read control essage
    for (c = 0; c < 3; c++) {
      listen[c] = client[0].read();
    }

  }

  listenRepeater();
  messageReset(messageToRepeater, sizeof(messageToRepeater));
  messageReset(loadMessage, sizeof(loadMessage));
}

/**
* Stop connections to clients
**/
void client_stop(){
   int i;
    for(i = 0; i < MAX_CONN; i++){
      if(client[i]){
        client[i].flush();
        client[i].stop();
      }
    }
}

/**
* Evaluate the message from repeater and update arrays accordingly
**/
void listenRepeater() {
  //client_stop();
  //client[0].connect(serverRep, PORT);

    Serial.print("Received message from repeater: ");
    Serial.println(listen);
    if (listen[SECTION] == '1') {
      commandSet(messageCommand, 0);      // Turn ON ALL LOADS

      // Update array based on indoor sunlight status
      if (lightCon[0] == '0') {
          commandSet(messageCommand, 1);      // Turn OFF 1LI
      }
      if (lightCon[1] == '0') {
          commandSet(messageCommand, 2);      // Turn OFF 2LI
      }
/*************************Continued on the next page*************************/
    }
    if (listen[SECTION] == '0') {
      commandSet(messageCommand, 4);      // Turn OFF all loads
    }
    Serial.print("The command is: ");
    Serial.println(messageCommand);
}

/**
* Update messageCommand array
* @params msgPtr: pointer to array that handles command message
* @params num: int - load number
**/
void commandSet(char* msgPtr, int num) {
  if (num == 0) {
    for (c = 0; c < LOADNUMS; c++) {
      *(msgPtr + c) = '1';
    }
  }
  if (num == 4) {     // Turn all loads off
    for (c = 0; c < LOADNUMS; c++) {
      *(msgPtr + c) = '0';
    }
  }
  else {
    *(msgPtr + (num-1)) = '0'; // Set the first character (2LI) to 0
  }
}

int sunLight () {
  char* shine = lightCon;
  if (*(shine + 0) == '1') {      // Check value at the 1st element of lightCon
    return 1;
  }
  else {
    return 0;
  }
}
/*************************************End**************************************/
