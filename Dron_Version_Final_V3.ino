/*
 * Autor : Ruben Ingles Gimeno
 * Titulo : Modificación de Drone de competición para entrega de paquetería
 */

 
 
/* GMAIL UTILIZA EL PUERTO 465 (SSL) y SMTP Server smtp.gmail.com
 * SE DEBE DESBLOQUEAR LA SEGURIDAD DE GMAIL  
 * Link: https://myaccount.google.com/lesssecureapps?pli=1
 * CAMPOS  "*****" PARA EVITAR HAKEOS
 */
#define emailSenderAccount    "**********" //Introduce la dirección de correo 
#define emailSenderPassword   "**********." //Introduce la contraseña de la dirección de correo 
#define emailRecipient        "***********" //Introduce la dirección de correo del receptor 

//Setup WiFi
const char* ssid = "****************";//Introduce el nombre de tu red WiFi
const char* password = "************"; //Introduce la contraseña de tu red WiFi

// Incluir Librarias
#include <Servo.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include "ESP32_MailClient.h"
#include "SPI.h"
#include "ESPino32CAM.h"
#include "ESPino32CAM_QRCode.h"
#include "esp_camera.h"

// Creación de Objectos
Servo myservo;      //Objecto para servo motor
SMTPData smtpData;  //Objecto para SMTPData (Mail)
ESPino32CAM cam;    //Objecto para captura de imagen
ESPino32QRCode qr;  //Objecto para decodificacion de imagen

// Setup Data
int redLED = 33;    //ESP32 Builtin GPIO Pin 33
int i=0;
int pos=0;          //Posicion inicial 
int go=0;
String data1;       //Variable para almacenar la palabra decodificada 
int lock;
String reading="";
int mailSendVal=0;  //Variable para activar el envio del mail

//Definicion de variables
/** The smtp port e.g. 
 * 25  or esp_mail_smtp_port_25
 * 465 or esp_mail_smtp_port_465
 * 587 or esp_mail_smtp_port_587
*/
#define smtpServerPort 465
#define smtpServer "smtp.gmail.com"   //Protocolo simple de transferencia de correo de Gmail
#define emailSubject "ESP32 Test Email with Attachments"  //Titulo del mail
#define Mail_Button 12               // Boton para el envio del email (12)
#define servoPin 13                 // Servo motor pin (13). Cable Naranja

//Set camera pins
#define CAMERA_MODEL_M5STACK_NO_PSRAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define flash 4

void sendCallback(SendStatus info); //Conocer el estado del email
//void startCameraServer(); //Descomentar para ver la camara a traves de una IP

/* Funcion para conectar al WiFi */
void connectWiFi()
{
  delay(500);
  WiFi.begin(ssid, password);
  delay(100);
  Serial.println("Connecting WiFi ...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.println(".");
  }
  Serial.println("WiFi Connected!");
  delay(500);
}

/* Funcion para montar el SPI Flash File system  */
void mountSPIFFS()
{
  delay(100);
  Serial.println("Montando SPIFFS ...");
  delay(500);
  if(!SPIFFS.begin(true)) {
    Serial.println("SPIFFS error de montaje ...");
  }
  else{
    Serial.println("SPIFFS Montado con exito!");
  }
  delay(100);
}

/* Funcion para el envio del email */
void mailSend() 
{
  smtpData.setLogin(smtpServer, smtpServerPort, emailSenderAccount, emailSenderPassword); //ESTABLISHES : SMTP Server - Email host, port, cuenta y contraseña 
  smtpData.setSender("ESP32-CAM", emailSenderAccount); // "Nombre del emisor"
  smtpData.setPriority("High"); // Introduce la prioridad del mail : High, Normal, Low o 1 hasta 5 (1 High,5 Low)
  smtpData.setSubject(emailSubject); //Setear el titulo del email

  /* Introducir el formato del texto */
  smtpData.setMessage("<div style=\"color:#2f4468;\"><h1>Estimado señor/a su paquete esta siendo enviado </h1><p>- Sent from ESP32 board</p></div>", true);
  smtpData.addRecipient(emailRecipient); //Setea el receptor,se puede añadir varios receptores - //smtpData.addRecipient("MAIL2@example.com");    
  smtpData.addAttachFile("/qr_code.jpg", "qr_code/jpg"); // la foto del Qr
  

  //Introduce que tipo del almacenamieento quieres (SPIFFS or SD Card)
  smtpData.setFileStorageType(MailClientStorageType::SPIFFS);
  smtpData.setSendCallback(sendCallback);
      
  // Empezar a enviar el e-mail y el seguimiento del mismo 
  if (!MailClient.sendMail(smtpData))
  {
    Serial.println("Error sending Email, " + MailClient.smtpErrorReason());
    smtpData.empty(); //ERASE MEMORY
  }
}

/* Función que commprueba el estado  */  
void sendCallback(SendStatus msg) 
{
  Serial.println(msg.info()); //Print the current status
  
  // Si esta ompletado eenviar el mensaje
  if (msg.success()) 
  {
    Serial.println("Email enviado con exito!");
  }
}

/* Función en setup ESP32-CAM */
void setupESP32Camera()
{
  delay(100);
  Serial.println("Setting up ESP32-CAM ...");
  delay(500);
  //Set camera pins
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //config.frame_size = FRAMESIZE_VGA;
  //config.jpeg_quality = 4;
  //config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  // for larger pre-allocated frame buffer.
  if(psramFound())
    {
      config.frame_size = FRAMESIZE_UXGA;
      config.jpeg_quality = 10;
      config.fb_count = 2;
    } 
  else 
    {
      config.frame_size = FRAMESIZE_SVGA;
      config.jpeg_quality = 12;
      config.fb_count = 1;
    }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif
  
  //Iniciación de la camara
  esp_err_t err = esp_camera_init(&config); 
  if (err != ESP_OK) 
  {
    Serial.printf("Camera ha fallado con este error 0x%x", err); //Muestra los errores si no inicia correctamente
    delay(1000);
    Serial.println();
    ESP.restart();// Reinicia ESP
  }

  //Inicia la decodificación 
  qr.init(&cam);
  sensor_t *s = cam.sensor();
  s->set_framesize(s, FRAMESIZE_CIF);
  s->set_whitebal(s, true);
  delay(1000);
  Serial.println("ESP32-CAM Connected!");
  delay(100);
}

/* Función de giro del servo */
void moveServo()
{
  if (data1 == "mensajero")
  {
    Serial.println("Opening the door ...");
    digitalWrite(redLED, HIGH); //LED on
    Serial.println("LED Activated!");
    delay(1000);
    for (pos = 0; pos <= 180; pos += 1) 
        {
          myservo.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15);                       // waits 15ms for the servo to reach the position
        }
    Serial.println("Door Open.");
    Serial.println("The Door remain open for 10 seconds.");
    delay(10000);

    Serial.println("Closing the door ...");
    digitalWrite(redLED, LOW); //LED off
    Serial.println("LED Deactivated!");
    delay(1000);
    for (pos = 180; pos >= 0; pos -= 1) 
        { 
          myservo.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15);                       // waits 15ms for the servo to reach the position
        }
    Serial.println("Door Close");
    data1="O";
  }
}

void qrScan()
{
  Serial.println("Started Scanning ...");
  unsigned long pv_time  = millis();
  camera_fb_t *fb = cam.capture(); //Capture the image
  if (!fb)
  {
    Serial.println("Image capture failed!");
    return;
  }
  dl_matrix3du_t *rgb888, *rgb565;
  if (cam.jpg2rgb(fb, &rgb888))
  {
    rgb565 = cam.rgb565(rgb888);
  }
  cam.clearMemory(rgb888);
  cam.clearMemory(rgb565);
  dl_matrix3du_t *image_rgb;
  if (cam.jpg2rgb(fb, &image_rgb))
  {
    cam.clearMemory(fb);

    qrResoult res = qr.recognition(image_rgb); // decodifica la imagen del qr
    if (res.status)//Si decodifica la imagen , muestra el mensaje en la pantalla
    {
      if (lock == 0) 
      {
        lock = 1;
        reading = "QR Code Read: " + res.payload; //Variable que muestra el texto que contiene el Qr
        data1 = res.payload;
        Serial.println();
        Serial.println(reading);  //Imprime el texto en la pantalla
      }
    }
    else 
    {
      //Si no recibe el codigo Qr imprime : 
      lock = 0;
      Serial.println();
      Serial.println("Muestre el código QR porfavor ... ");
      qrScan();
    }
  }
  moveServo();
  cam.clearMemory(image_rgb); //limpia la imagen para poder recibir otra
}

/* funcion del envio del email por pulsador */

void buttonPress()
{
    mailSendVal = digitalRead(Mail_Button);
    if(mailSendVal == 1)
    {
      Serial.println("Enviando Email ...");
      mailSend();
      mailSendVal=0;
      return;
    }
}

/* Setup Function */
void setup() 
{
  Serial.println("Starting Setup ...");
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  
  //pinMode(redLED,OUTPUT);
  pinMode(Mail_Button,INPUT);
  pinMode(flash, OUTPUT);           //Define pin to do flash
  
  digitalWrite(redLED,LOW);         //Turning Off Built-In LED
  digitalWrite(flash, LOW);         //Turn off the flash
  
  myservo.attach(servoPin,2,0,180); //Attach Servo with ESP32-CAM (ESPPin, ESPChannel, MinRotation, MaxRotation)
  delay(500);
  
  connectWiFi();
  Serial.println();
  mountSPIFFS();
  Serial.println();
  setupESP32Camera();
  Serial.println();
  //startCameraServer();
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  Serial.println("Setup complete!");
}

/* Loop Function */
void loop()
{
  //Serial.println("Press button to get QR ...");
  buttonPress();
  delay(1000);
  if(mailSendVal == 1)
  {
    Serial.println("Sending Email ...");
    mailSend();
    mailSendVal=0;
    return;
  }
  Serial.println();
  Serial.println("Scan the QR ..."); 
  qrScan();
  delay(100);
}
