/*
 * Autor : Ruben Ingles Gimeno
 * Titulo : Modificación de Drone de competición para entrega de paquetería
 * Año :2020-2021
 */

/*Incluir Librarias*/
#include <Servo.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include "ESP32_MailClient.h"
#include "SPI.h"
#include "ESPino32CAM.h"
#include "ESPino32CAM_QRCode.h"
#include "esp_camera.h"
 
/*  ---------------Definicion de parametros del e-mail-------------------
 *   
 * GMAIL UTILIZA EL PUERTO 465 (SSL) y SMTP Server smtp.gmail.com
 * SE DEBE DESBLOQUEAR LA SEGURIDAD DE GMAIL  
 * Link: https://myaccount.google.com/lesssecureapps?pli=1
 * CAMPOS  "*****" PARA EVITAR HAKEOS

 
 * The smtp port ejemplos : 
 * 25  o esp_mail_smtp_port_25  Para Outlook
 * 465 o esp_mail_smtp_port_465 Para Gmail
 * 587 o esp_mail_smtp_port_587 Puerto por defecto para los demas
*/
#define emailSenderAccount    "********************"                        //Introduce tu dirección de e-mail
#define emailSenderPassword   "************"                                //Introduce tu contraseña de la dirección e-mail 
#define emailRecipient        "***********************"                     //Introduce el e-mail de la persona a la que le envias el paquete
#define smtpServerPort 465                                                  // Puerto para Gmail
#define smtpServer "smtp.gmail.com"                                         //Gmail Simple Mail Transfer Protocol
#define emailSubject "Dron de Paqueteria"                                   //Asunto del mail


/*Nombre y Contraseña Wifi*/

const char* ssid = "********************";                  //Introduce el nombre de tu red Wifi
const char* password = "***************";                   //Introduce la contraseña de tu red Wifi


/* Creación de Objetos*/

Servo myservo;      //Objecto para servo motor
SMTPData smtpData;  //Objecto para SMTPData (Mail)
SMTPData smtpData2;  //Objecto para SMTPData de recepcion del paquete(Mail)
ESPino32CAM cam;    //Objecto para captura de imagen
ESPino32QRCode qr;  //Objecto para decodificacion de imagen

/* declaracion de variables*/

int i=0;
int pos=0;          //Posicion inicial 
int go=0;
String data1;       //Variable para almacenar la palabra decodificada 
int lock;
String reading="";
int mailSendVal=0;  //Variable para activar el envio del e-mail



//Definicion de los pines de salida/entrada

#define Mail_Button 13                 // Boton para el envio del email (13)
#define servoPin 12                    // Servo motor pin (12). Cable Naranja
#define GreenLed 15                    // ESP32 contiene en  GPIO Pin (15) un led Verde
#define RedLed 14                      // ESP32 contiene en  GPIO Pin (14) un led Rojo

//Set camara pins

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

void sendCallback(SendStatus info);                 //Conocer el estado del email
//void startCameraServer();                         //Descomentar para ver la camara a traves de una IP

/* Funcion para conectar al WiFi */
void connectWiFi()
{
  delay(500);
  WiFi.begin(ssid, password);
  delay(1000);
  Serial.println("Conectando WiFi ...Espere");
  delay(1000);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conectando....");
    
    //----------------------Alarma1---------------------------------//
    
    digitalWrite(RedLed, HIGH);                     // Enciende el LED mientras no este conectado al Wifi
    delay(1000);
  }
  Serial.println("WiFi Conectedo!");
  digitalWrite(RedLed, LOW);                        // Apaga el LED al conectarse a la red wifi
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

    //-----------Alarma 2---------------------------//
    digitalWrite(RedLed, HIGH);                     // enciende el LED.
    delay(500);
    digitalWrite(RedLed, LOW);                      // enciende el LED.
    delay(500);
    digitalWrite(RedLed, HIGH);                     // enciende el LED.
    delay(500);
    digitalWrite(RedLed, LOW);                      // enciende el LED.
    delay(500);
    digitalWrite(RedLed, HIGH);                     // enciende el LED.
    delay(500);
    digitalWrite(RedLed, LOW);                      // enciende el LED.
    delay(500);
    digitalWrite(RedLed, HIGH);                     // enciende el LED.
    delay(500);
    digitalWrite(RedLed, LOW);                      // enciende el LED.
    delay(500);
    }
  
  else{
    Serial.println("SPIFFS Montado con exito!");
  }
  delay(100);
}

/* Funcion para el envio del email */
void mailSend() 
{
  smtpData.setLogin(smtpServer, smtpServerPort, emailSenderAccount, emailSenderPassword); //ESTABLECIDO : SMTP Server - Email host, port, cuenta y contraseña 
  smtpData.setSender("ESP32-CAM", emailSenderAccount); // "Nombre del emisor"
  smtpData.setPriority("High"); // Introduce la prioridad del mail : High, Normal, Low o 1 hasta 5 (1 High,5 Low)
  smtpData.setSubject(emailSubject); //Setear el titulo del email

  /* Introducir el formato del texto */
  smtpData.setMessage("<div style=\"color:#2f4468;\"><h1>Estimado señor/a su paquete esta siendo enviado </h1><p>- Sent from ESP32 board</p></div>", true);
  smtpData.addRecipient(emailRecipient); //Setea el receptor,se puede añadir varios receptores - //smtpData.addRecipient("MAIL2@example.com");    
  smtpData.addAttachFile("/qr_code.jpg", "qr_code/jpg"); // la foto del Qr
  

  //Introduce que tipo del almacenamieento (SPIFFS or SD )
  smtpData.setFileStorageType(MailClientStorageType::SPIFFS);
  smtpData.setSendCallback(sendCallback);
      
  // Empezar a enviar el e-mail y el seguimiento del mismo 
  if (!MailClient.sendMail(smtpData))
  {
    Serial.println("Error enviando Email, " + MailClient.smtpErrorReason());
    smtpData.empty(); //ERASE MEMORY
  }
}
/*funcion para enviar un mail con el paquete recibido*/
void MailEntrega() 
{
  smtpData2.setLogin(smtpServer, smtpServerPort, emailSenderAccount, emailSenderPassword);       //ESTABLECIDO : SMTP Server - Email host, port, cuenta y contraseña 
  smtpData2.setSender("ESP32-CAM", emailSenderAccount); // titulo ,"Nombre del emisor"
  smtpData2.setPriority("High");                                                                  // Introduce la prioridad del mail : High, Normal, Low o 1 hasta 5 (1 High,5 Low)
  smtpData2.setSubject(emailSubject);                                                             //Setear el titulo del email

  /* Introducir el formato del texto */
  smtpData2.setMessage("<div style=\"color:#2f4468;\"><h1>Estimado señor/a su paquete ha sido enviado con éxito </h1><p>- Sent from ESP32 board</p></div>", true);
  smtpData2.addRecipient(emailRecipient); //Setea el receptor,se puede añadir varios receptores -    //smtpData.addRecipient("MAIL2@example.com");    
  smtpData2.addAttachFile("/gracias.txt", "gracias/txt");                                            //Archivo txt. con un mensaje de agradecimiento
  

  //Introduce que tipo del almacenamieento (SPIFFS or SD )
  smtpData2.setFileStorageType(MailClientStorageType::SPIFFS);
  smtpData2.setSendCallback(sendCallback);
      
  // Empezar a enviar el e-mail y el seguimiento del mismo 
  if (!MailClient.sendMail(smtpData2))
  {
    Serial.println("Error enviando Email, " + MailClient.smtpErrorReason());
    smtpData.empty();         //Borrado de memoria
  }
}

/* Función que commprueba el estado  */  
void sendCallback(SendStatus msg) 
{
  Serial.println(msg.info()); //Imprime el estado del correo
  
  // Si esta ompletado envio del correo el mensaje
  if (msg.success()) 
  {
    Serial.println("Email enviado con exito!");
  }
}

/* Función en setup ESP32-CAM */
void setupESP32Camera()
{
  delay(100);
  Serial.println("Setendo ESP32-CAM ...");
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

  //Inicia la decodificación del codigo QR
  qr.init(&cam);
  sensor_t *s = cam.sensor();
  s->set_framesize(s, FRAMESIZE_CIF);
  s->set_whitebal(s, true);
  delay(1000);
  Serial.println("ESP32-CAM Conectedo!");
  delay(100);
}

/* Función de giro del servo */
void moveServo()
{
  if (data1 == "mensajero")
  {
    Serial.println("Abriendo el cierre ...");
    digitalWrite(GreenLed, HIGH); //LED on
    Serial.println("LED Activado!");
    delay(1000);
    for (pos = 0; pos <= 180; pos += 1) 
        {
          myservo.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15);                       // waits 15ms for the servo to reach the position
        }
    Serial.println("Cierre abierto.");
    Serial.println("El cierre permanecera abierto durante 10 seg.");
    delay(10000);

    Serial.println("Pinza abierta ...");
    digitalWrite(GreenLed, LOW);          
    Serial.println("LED Desactivatedo!");
    delay(1000);
    for (pos = 180; pos >= 0; pos -= 1) 
        { 
          myservo.write(pos);              // indica al servo que vaya a la posición en la variable 'pos'
          delay(15);                       // espera 15ms para que el servo alcance la posición
        }
    Serial.println("Pinza cerrada");
    data1="O";
    delay(500);
    MailEntrega();
    delay(500);
  }
  else if (data1 != "mensajero"){                     // Palabra clave
    //-----alarma3---------//
    digitalWrite(RedLed, HIGH);
    delay(2000);
    digitalWrite(RedLed, LOW);
    delay(500);
  }
  else{
    Serial.println("No se encontro nada");
  }
}

void qrScan()
{
  Serial.println("Empezando a escanear...");
  unsigned long pv_time  = millis();
  camera_fb_t *fb = cam.capture();        //Captura la imagen del codigo qr
  if (!fb)
  {
    Serial.println("Fallo al capturar la imagen !!");
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

    qrResoult res = qr.recognition(image_rgb);      // decodifica la imagen del qr
    if (res.status)                                       //Si decodifica la imagen ...
    {
        if (lock == 0) {
      
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
      Serial.println("Muestre el código de nuevo QR porfavor ... ");
      qrScan();
    }
  }
  moveServo();
  cam.clearMemory(image_rgb);        //limpia la imagen para poder recibir otra
}



//-----------------------------------Setup------------------------------------------------ //
void setup()  
{
  Serial.println("Inciando Setup ...");
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  //----Pines----------------//
  pinMode(GreenLed,OUTPUT);           //Define pin para led verde
  pinMode(RedLed,OUTPUT);             //Define pin para led verde
  pinMode(Mail_Button,INPUT);         //Define pin para el pulsador
 
  
  digitalWrite(GreenLed,LOW);       // LED verde apagado
  digitalWrite(RedLed, LOW);         //led rojo apagado
  
  myservo.attach(servoPin,2,0,180);   //Adjunto Servo con  ESP32-CAM (ESPPin, ESPChannel, MinRotación, MaxRotación)
  delay(500);

  //-------Ejecución de funciones--------//
  connectWiFi();
  Serial.println();
  mountSPIFFS();
  Serial.println();
  setupESP32Camera();
  Serial.println();
  //startCameraServer();                                                //Activar en caso de que se quiera testear la camara o conocer su direccion IP
  //Serial.print("Camera Ready! Use 'http://");
  //Serial.print(WiFi.localIP()); 
  //Serial.println("' para conectar");
  Serial.println("Setup completada con éxito !");
}

//----------------------------------Función Loop-----------------------------//
void loop()
{
  Serial.println("Pulsa el boton para enviar la imagen QR ...");
  delay(1000);
  
  mailSendVal = digitalRead(Mail_Button);
  if(mailSendVal == 1){
    Serial.println("Enviando Email ...");
    
    mailSend();
    delay(3000);
    mailSendVal=0;
    delay(100);
    return;
    
  }
  Serial.println();
  Serial.println("Muestre el codigo QR ..."); 
  delay(2000);
  qrScan(); 
  delay(2000);
 
  Serial.println("Empieza de nuevo el loop");
  delay(1000);
}
