#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <addons/RTDBHelper.h>




//for time module
const long utcOffsetInSeconds = 0; // utc +5.30
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);



//Define credentials
#define WIFI_SSID "Janith"
#define WIFI_PASSWORD "mmmmmmmm"
#define API_KEY "AIzaSyAoxkXg7dG9hV7z8WoUc1duIbOa0d4NB-Y"
#define FIREBASE_PROJECT_ID "patient-assisting-bed-app"
#define USER_EMAIL "janithravi7@gmail.com"
#define USER_PASSWORD "12345678"
#define PATIENT_NAME "Janith"  //change user name
#define DATABASE_SECRET "Vg3CNDIHe7ghlgGP6OULfkNT6jz0aao4Q9UEaiVa"
#define DATABASE_URL "https://patient-assisting-bed-app-default-rtdb.asia-southeast1.firebasedatabase.app/"



// Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;


int bpm = 0;
float bTemp = 0;
float roomTemp = 0;
float humidity = 0;


unsigned long lastDataSent = 0;
unsigned long lastWarningDataRetrieve = 0;
unsigned long lastDosagesDataRetrieve = 0;



// The Firestore payload upload callback function
void fcsUploadCallback(CFS_UploadStatusInfo info)
{
    if (info.status == fb_esp_cfs_upload_status_init)
    {
        Serial.printf("\nUploading data (%d)...\n", info.size);
    }
    else if (info.status == fb_esp_cfs_upload_status_upload)
    {
        Serial.printf("Uploaded %d%s\n", (int)info.progress, "%");
    }
    else if (info.status == fb_esp_cfs_upload_status_complete)
    {
        Serial.println("Upload completed ");
    }
    else if (info.status == fb_esp_cfs_upload_status_process_response)
    {
        Serial.print("Processing the response... ");
    }
    else if (info.status == fb_esp_cfs_upload_status_error)
    {
        Serial.printf("Upload failed, %s\n", info.errorMsg.c_str());
    }
}



void splitString(String str) {
  String key = "";
  String value = "";
  bool isKey = true;
  for (int i = 0; i < str.length(); i++) {
    if (str[i] == '=') {
      isKey = false;
      continue;
    }
    if (isKey) {
      key += str[i];
    } else {
      value += str[i];
    }
  }
  //covert value into float
  key = value.toFloat();
}


String epochTimeConverter(unsigned long epochTime){
    time_t rawtime = epochTime; //
    struct tm *ptm = gmtime(&rawtime); //convert to UTC
    char utcTime[30];
    sprintf(utcTime, "%d-%02d-%02dT%02d:%02d:%02dZ", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    Serial.println(utcTime);
    return String(utcTime);
}


void setup() {
    Serial.begin(9600);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED){
        Serial.write(".");
        delay(300);
    }
    // Serial.write();
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.print("Wifi connected");
    Serial.print(WiFi.localIP());
    Serial.println();  

    timeClient.begin();
    
    //firebase
    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    config.cfs.upload_callback = fcsUploadCallback;

}

void loop() {
  if(Serial.available()){
    String str = Serial.readStringUntil('\n');
    splitString(str);
    Serial.println(str);
  }
  if(roomTemp > 0 && Firebase.ready() && ((millis() - lastDataSent)>=60000)){
    FirebaseJson content;
    String documentPath = PATIENT_NAME + String("/") + String(timeClient.getFormattedTime()); //document path is patient name and time

    //set data to json object
    content.set("fields/timestamp/timestampValue",epochTimeConverter(timeClient.getEpochTime()));//use firebase.now() to get current time
    content.set("fields/bpm/integerValue", bpm);
    content.set("fields/body temp/doubleValue", bTemp);
    content.set("fields/room temp/doubleValue", roomTemp);

    //send data to firestore
    Serial.print("Create a document...");
    if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), content.raw()))
        Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
    else
        Serial.println(fbdo.errorReason());  
  }

    if (Firebase.ready()){

        FirebaseJson json;

        if((millis() - lastDosagesDataRetrieve)>=10000 ){
          Firebase.RTDB.getJSON(&fbdo, "/alerts/Janith/warnings"); //only every 10 s
          Serial.println(fbdo.stringData().c_str());
          if (fbdo.httpCode() == FIREBASE_ERROR_HTTP_CODE_OK){
              // printResult(fbdo);
            //   Serial.print(fbdo.stringData());
            //   Serial.write(fbdo.stringData());
            //send data to mega through serial
            Serial1.write(fbdo.stringData().c_str());

          }
          Firebase.RTDB.deleteNode(&fbdo, "/alerts/Janith/warnings");
          lastDosagesDataRetrieve = millis();
        }

        if((millis() - lastWarningDataRetrieve)>=300000 ){ //check every 5 minutes
          Firebase.RTDB.getJSON(&fbdo, "/dosages/Janith/dosages");
          if (fbdo.httpCode() == FIREBASE_ERROR_HTTP_CODE_OK){
              // printResult(fbdo); // see addons/RTDBHelper.h
            //   Serial.print(fbdo.stringData());
            Serial1.write(fbdo.stringData().c_str());
          }
          Firebase.RTDB.deleteNode(&fbdo, "/dosages/Janith/dosages");
          lastWarningDataRetrieve = millis();
        }

  }
  
  
}