#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
// #include <softwareSerial.h>



//Define credentials
#define WIFI_SSID "Janith"
#define WIFI_PASSWORD "mmmmmmmm"
#define API_KEY "AIzaSyAoxkXg7dG9hV7z8WoUc1duIbOa0d4NB-Y"
#define FIREBASE_PROJECT_ID "patient-assisting-bed-app"
#define USER_EMAIL "janithravi7@gmail.com"
#define USER_PASSWORD "12345678"
#define PATIENT_NAME "Janith"  //change user name


// Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;


int bpm;
int bTemp;
int roomTemp;





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
  key = value;
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


}

void loop() {
  if(Serial.available()){
    String str = Serial.readStringUntil('\n');
    splitString(str);
    Serial.println(str);
  }
}