/*Connecting the BME280 Sensor:
Sensor              ->  Board
-----------------------------
Vin (Voltage In)    ->  3.3V
Gnd (Ground)        ->  Gnd
SDA (Serial Data)   ->  D2 on NodeMCU / Wemos D1 PRO
SCK (Serial Clock)  ->  D1 on NodeMCU / Wemos D1 PRO */

//BME280 definition and Mutichannel_Gas_Sensor
#include <EnvironmentCalculations.h>
#include <Wire.h>
#include "cactus_io_BME280_I2C.h"
//Create BME280 object
BME280_I2C bme(0x76);			//I2C using address 0x76

//for OTA
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
bool OTAConfigured = 0;

//#define BLYNK_DEBUG			//Optional, this enables lots of prints
//#define BL+YNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
WidgetTerminal terminal(V40);				//Attach virtual serial terminal to Virtual Pin V40
#include <SimpleTimer.h>
SimpleTimer TimerBlynkCheck;				//Do sprawdzana połączenia z Blynkiem uruchamiany do 30s
SimpleTimer TimerMainFunction;				//dla MainFunction uruchamiany do 3s
SimpleTimer TimerSedes;					//dla iluminacji sedesu
int timerID;						//Przetrzymuje ID Timera

float		SetHumid		= 75;		//Wilgotności przy której załączy się wentylator w trybie manualnym
float		SetHumidAuto		= 75;		//Wilgotności przy której załączy się wentylator w trybie automatycznym
float		RoomHumid		= 0;		//Wilgotności w pokoju, potrzebna do wysnaczenie watrości wilgotności przy której ma się załączyć wentylator
int		Fan_Manual		= 0;		//Manualne włączenie wentylatora
int		Fan_State		= 0;
int		PhotoResValue		= 0;		//Store value from photoresistor (0-1023)
int		ProgPhotoresistor	= 450;		//Próg jasności od którego zacznie działać iluminacja sedesu (nie powinno podświetlać jeśli światło w łazience zapalone)
float temp(NAN), hum(NAN), pres(NAN), dewPoint(NAN), absHum(NAN), heatIndex(NAN);

//STAŁE
const char	ssid[]			= "XXXX";
const char	pass[]			= "XXXX";
const char	auth[]			= "XXXX";	//Token Łazienka Rymanowska
const int	checkInterval		= 30000;	//Co 30s zostanie sprawdzony czy jest sieć Wi-Fi i czy połączono z serwerem Blynk
const int	BathFan			= D5;		//Deklaracja pinu na który zostanie wysłany sygnał załączenia wentylatora
const int	Piec_CO			= D6;		//Deklaracja pinu na którym będzie włączany piec CO
const int	PIR_Sensor		= D7;		//Deklaracja pinu z sensorem ruchu AM312
const int	LED_Light		= D8;		//Deklaracja pinu z MOSFETem do iluminacji sedesu
const int	PhotoResistor		= A0;		//Deklaracja pinu z sensorem światła
const float	HumidHist		= 4;		//histereza dla wilgotności

#include <LightDimmer.h>				//Rozjaśnianie i przygasanie PWMem
LightDimmer SedesDimmer;				//Deklaracja do rozjaśniania i ściemniania

BLYNK_CONNECTED()			//Informacja że połączono z serwerem Blynk, synchronizacja danych
{
	Serial.println("Reconnected, syncing with cloud.");
	Blynk.syncAll();
}

void blynkCheck()			//Sprawdza czy połączone z serwerem Blynk
{
	if (WiFi.status() == WL_CONNECTED)		//WL_CONNECTED: assigned when connected to a WiFi network
	{
		if (!Blynk.connected())
		{
			Serial.println("WiFi OK, trying to connect to the Blynk server...");
			Blynk.connect();
		}
	}

	if (WiFi.status() == WL_NO_SSID_AVAIL)		//WL_NO_SSID_AVAIL: assigned when no SSID are available
	{
		Serial.println("No WiFi connection, offline mode.");
	}

	if (WiFi.status() == WL_IDLE_STATUS)		//WL_IDLE_STATUS is a temporary status assigned when WiFi.begin() is called and remains active until the number of attempts expires (resulting in WL_CONNECT_FAILED) or a connection is established (resulting in WL_CONNECTED)
	{
		Serial.println("WL_IDLE_STATUS: WiFi.begin() is called");
	}

	if (WiFi.status() == WL_SCAN_COMPLETED)		//WL_SCAN_COMPLETED: assigned when the scan networks is completed
	{
		Serial.println("WL_SCAN_COMPLETED: networks is completed");
	}

	if (WiFi.status() == WL_CONNECT_FAILED)		//WL_CONNECT_FAILED: assigned when the connection fails for all the attempts
	{
		Serial.println("WL_CONNECT_FAILED: connection fails for all the attempts");
	}

	if (WiFi.status() == WL_CONNECTION_LOST)	//WL_CONNECTION_LOST: assigned when the connection is lost
	{
		Serial.println("WL_CONNECTION_LOST: the connection is lost");
	}

	if (WiFi.status() == WL_DISCONNECTED)		//WL_DISCONNECTED: assigned when disconnected from a network
	{
		Serial.println("WL_DISCONNECTED: disconnected from a network");
	}
}

void OTA_Handle()			//Deklaracja OTA_Handle:
{
	if (OTAConfigured == 1)
	{
		if (WiFi.waitForConnectResult() == WL_CONNECTED) {
			ArduinoOTA.handle();
		}
	}
	else
	{
		if (WiFi.waitForConnectResult() == WL_CONNECTED)
		{
			// Port defaults to 8266
			// ArduinoOTA.setPort(8266);

			// Hostname defaults to esp8266-[ChipID]
			ArduinoOTA.setHostname("Lazienka_Rymanowska");

			// No authentication by default
			// ArduinoOTA.setPassword("admin");

			// Password can be set with it's md5 value as well
			// MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
			// ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

			ArduinoOTA.onStart([]() {
				String type;
				if (ArduinoOTA.getCommand() == U_FLASH) { type = "sketch";}
				else { type = "filesystem";}  // U_SPIFFS
				// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
				Serial.println("Start updating " + type);
			});
			
			ArduinoOTA.onEnd([]() {
				Serial.println("\nEnd");
			});
			
			ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
			Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
			});
			
			ArduinoOTA.onError([](ota_error_t error) {
				Serial.printf("Error[%u]: ", error);
				if (error == OTA_AUTH_ERROR) { Serial.println("Auth Failed");}
				else if (error == OTA_BEGIN_ERROR) { Serial.println("Begin Failed");}
				else if (error == OTA_CONNECT_ERROR) { Serial.println("Connect Failed");}
				else if (error == OTA_RECEIVE_ERROR) { Serial.println("Receive Failed");}
				else if (error == OTA_END_ERROR) { Serial.println("End Failed");}
			});
			
			ArduinoOTA.begin();

			Serial.println("Ready");
			Serial.print("OTA IP address: ");
			Serial.println(WiFi.localIP());

			OTAConfigured = 1;
		}
	}
}

void MainFunction()			//Robi wszystko co powinien
{
	Read_BME280_Values();			//Odczyt danych z czujnika BME280
	Bathrum_Humidity_Control();		//Włącza wentylator jeśli wilgotność przekracza próg ale Piec CO jest wyłączony
	Wyslij_Dane();				//Wysyła dane do serwera Blynk
}

void Bathrum_Humidity_Control()		//Załączanie wentylatora w łazience jeśji warunek spełnionyBathFan_Value
{
	if (Fan_Manual == 0)
	{
		if (hum >= SetHumidAuto + HumidHist)		//Jeśli wilgotność w pokoju + 15% + HumidHist
		{
			digitalWrite(BathFan, LOW);		//turn on relay with voltage LOW
			Blynk.virtualWrite(V8, 255);		//Wentylator włączony
		}
		else if (hum <= SetHumidAuto - HumidHist)	//Jeśli wilgotność w pokoju - 15% + HumidHist
		{
			digitalWrite(BathFan, HIGH);		//turn on relay with voltage HIGH
			Blynk.virtualWrite(V8, 0);		//Wentylator Wyłączony
		}
	}
	
	else if (Fan_Manual == 1)
	{
		if (Fan_State == 1)
		{
			digitalWrite(BathFan, LOW);		//turn on relay with voltage LOW
			Blynk.virtualWrite(V8, 255);		//Wentylator Wyłączony
		}
		else if (Fan_State == 0)
		{
			digitalWrite(BathFan, HIGH);		//turn on relay with voltage HIGH
			Blynk.virtualWrite(V8, 0);		//Wentylator włączony
		}
		else if (Fan_State == 2)
		{
			if (hum >= SetHumid + HumidHist)
			{
				digitalWrite(BathFan, LOW);	//turn on relay with voltage LOW
				Blynk.virtualWrite(V8, 255);	//Wentylator Wyłączony
			}
			else if (hum <= SetHumid - HumidHist)
			{
				digitalWrite(BathFan, HIGH);	//turn on relay with voltage HIGH
				Blynk.virtualWrite(V8, 0);	//Wentylator włączony
			}
		}
	}
}

void Read_BME280_Values()		//Odczyt wskazań z czujnika BME280
{
	bme.readSensor(); 		//Odczyt wskazań z czujnika BME280
	pres = bme.getPressure_MB();
	hum = bme.getHumidity();
	temp = bme.getTemperature_C();
	EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
	EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;
	//Dane obliczane na podstawie danych z czujnika
	dewPoint = EnvironmentCalculations::DewPoint(temp, hum, envTempUnit);
	absHum = EnvironmentCalculations::AbsoluteHumidity(temp, hum, envTempUnit);
	heatIndex = EnvironmentCalculations::HeatIndex(temp, hum, envTempUnit);
}

void Wyslij_Dane()			//Wysyła dane na serwer Blynk
{
	Blynk.virtualWrite(V0, temp);			//Temperatura [°C]
	Blynk.virtualWrite(V1, hum);			//Wilgotność [%]
	Blynk.virtualWrite(V2, pres);			//Ciśnienie [hPa]
	Blynk.virtualWrite(V3, dewPoint);		//Temperatura punktu rosy [°C]
	Blynk.virtualWrite(V4, absHum);			//Wilgotność bezwzględna [g/m³]
	Blynk.virtualWrite(V5, heatIndex);		//Temperatura odczuwalna [°C]
	Blynk.virtualWrite(V6, SetHumidAuto);		//Wilgotności przy której załączy się wentylator w trybie automatycznym [%] 

	Blynk.virtualWrite(V25, constrain(map(WiFi.RSSI(), -105, -40, 0, 100), 0, 100)); //Siła sygnału Wi-Fi [%], constrain() limits range of sensor values to between 0 and 100
}

BLYNK_WRITE(V40)			//Obsługa terminala
{
	String TerminalCommand = param.asStr();
	TerminalCommand.toLowerCase();

	if (String("ports") == TerminalCommand)
	{
		terminal.clear();
		terminal.println("PORT     DESCRIPTION        UNIT");
		terminal.println("V0   ->  Temperature        °C");
		terminal.println("V1   ->  Humdity            %");
		terminal.println("V2   ->  Pressure           HPa");
		terminal.println("V3   ->  DewPoint           °C");
		terminal.println("V4   ->  Abs Humdity        g/m3");
		terminal.println("V5   ->  Heat Index         °C");
		terminal.println("V10  <-  SetHumid           %");
		terminal.println("V11  <-  Fan Manual & State 1,2,3,4");
		terminal.println("V25  ->  WiFi Signal        %");
		terminal.println("V40 <->  Terminal           String");
	}
	else if (String("values") == TerminalCommand)
	{
		terminal.clear();
		terminal.println("PORT   DATA              VALUE");
		terminal.print("V0     Temperature   =   ");
		terminal.print(temp);
		terminal.println(" °C");
		terminal.print("V1     Humdity       =   ");
		terminal.print(hum);
		terminal.println(" %");
		terminal.print("V2     Pressure      =   ");
		terminal.print(pres);
		terminal.println(" HPa");
		terminal.print("V3     DewPoint      =   ");
		terminal.print(dewPoint);
		terminal.println(" °C");
		terminal.print("V4     Abs Humdity   =   ");
		terminal.print(absHum);
		terminal.println(" g/m3");
		terminal.print("V5     Heat Index    =   ");
		terminal.print(heatIndex);
		terminal.println(" °C");
		terminal.print("V10    SetHumid  =   ");
		terminal.print(SetHumid );
		terminal.println(" %");
		terminal.print("V25    WiFi Signal   =   ");
		terminal.print(constrain(map(WiFi.RSSI(), -105, -40, 0, 100), 0, 100));
		terminal.println(" %");
	}
	else if (String("hello") == TerminalCommand)
	{
		terminal.clear();
		terminal.println("Hi Łukasz. Have a great day!");
	}
	else
	{
		terminal.clear();
		terminal.println("Type 'PORTS' to show list") ;
		terminal.println("Type 'VALUES' to show list") ;
		terminal.println("or 'HELLO' to say hello!") ;
	}
	// Ensure everything is sent
	terminal.flush();
}

BLYNK_WRITE(V10)			//Ustawienie progu wilgotności powyżej którego włączy się wentylator (plus próg)
{
	SetHumid = param.asInt(); 
}

BLYNK_WRITE(V21)			//Wilgotność w pokoju, przesyłana z Wemos D1
{
	RoomHumid = param.asInt();
	SetHumidAuto = RoomHumid + 15;
}

BLYNK_WRITE(V11)			//Sterowanie wentylatorem z aplikacji
{
	switch (param.asInt()){
		case 1:					//Ogrzewanie w trybie automatycznym na podstawie harmonogramu SetTemp
			Fan_Manual = 0;
			Fan_State = 0;
			break;
		case 2:					//Ogrzewanie w trybie ręcznym ON
			Fan_Manual = 1;
			Fan_State = 1;
			break;
		case 3:					//Ogrzewanie w trybie ręcznym OFF
			Fan_Manual = 1;
			Fan_State = 0;
			break;
		case 4:					//Ogrzewanie w trybie ręcznym z zadaną temperaturą 'SetTemp'
			Fan_Manual = 1;
			Fan_State = 2;
			break;
			default:			//Wartość domyślna to tryb automatyczny
			Fan_Manual = 0;
			Fan_State = 0;
	}
}

/***********************************************************************************************/

void setup()
{
	Serial.begin(115200);
	WiFi.begin(ssid, pass);
	Blynk.config(auth);

	//Inicjalizacja Timerów
	TimerBlynkCheck.setInterval(checkInterval, blynkCheck);		//Multiple timer https://codebender.cc/example/SimpleTimer/SimpleTimerAlarmExample#SimpleTimerAlarmExample.ino
	TimerMainFunction.setInterval(3000, MainFunction);		//1000 = 1s

	Wire.begin();

	//Ustawianie pinów
	pinMode(BathFan, OUTPUT);			//Deklaracja pinu na który zostanie wysłany sygnał załączenia wentylatora
	digitalWrite(BathFan, HIGH);			//Domyślnie wyłączony (stan wysoki HIGH)
	pinMode(Piec_CO, OUTPUT);			//Deklaracja pinu na którym będzie włączany piec CO
	digitalWrite(Piec_CO, HIGH);			//Domyślnie wyłączony (stan wysoki HIGH)
	pinMode(PIR_Sensor, INPUT);			//Deklaracja pinu z sensorem ruchu AM312

	pinMode(LED_Light, OUTPUT);			//Deklaracja pinu z MOSFETem do iluminacji sedesu
	SedesDimmer.begin(LED_Light,HIGH);		//Deklaracja pinu z MOSFETem do iluminacji sedesu
	SedesDimmer.setFadingTime(1000);
	SedesDimmer.setBrighteningTime(1000);

	pinMode(PhotoResistor, INPUT);			//Set pResistor - A0 pin as an input (optional)
	attachInterrupt(digitalPinToInterrupt(PIR_Sensor), handleInterrupt, HIGH);	//Obsługa przerwań dla czujnika ruchu

	//inicjowanie czujnika BME280
	if (!bme.begin())
	{
		Serial.println("Could not find a valid BME280 sensor, check wiring!"); 
		while (1);
	}

	bme.setTempCal(0.7);				//Temp was reading high so subtract 0.7 degree
}

void loop()
{
	TimerBlynkCheck.run();
	TimerMainFunction.run();
	TimerSedes.run();
	OTA_Handle();			//Obsługa OTA (Over The Air) wgrywanie nowego kodu przez Wi-Fi
	LightDimmer::update();		//updates all FadeLed objects

	if (TimerSedes.isEnabled(timerID) && digitalRead(PIR_Sensor) == 1 && analogRead(PhotoResistor) < ProgPhotoresistor)			//Returns true if the specified timer is enabled
	{
		TimerSedes.restartTimer(timerID);		//Wydłuża illuminacje sedesu o kolejne 30s
	}
	else if (!TimerSedes.isEnabled(timerID) && digitalRead(PIR_Sensor) == 1 && analogRead(PhotoResistor) < ProgPhotoresistor)		//Returns true if the specified timer is enabled
	{
		SedesDimmer.on();				//Włącza iluminację sedesu
		timerID = TimerSedes.setTimeout(20000, SedesIlluminationOFF);									//Wyłączy iluminacje sedesu za 30s
	}
	else if ( TimerSedes.isEnabled(timerID) && analogRead(PhotoResistor) > ProgPhotoresistor)						//Returns true if the specified timer is enabled
	{
		TimerSedes.deleteTimer(timerID);		//Wyłącza Timer 
		SedesIlluminationOFF();				//Wyłącza iluminację sedesu
	}

	if (Blynk.connected()) Blynk.run();
}

void handleInterrupt()			//Obsługa przerwań wywoływanych przez czujnik PIR AM312
{
	if ( !TimerSedes.isEnabled(timerID) && analogRead(PhotoResistor) < ProgPhotoresistor )	//Returns true if the specified timer is enabled
	{
		SedesDimmer.on();
		timerID = TimerSedes.setTimeout(10000, SedesIlluminationOFF);			//Wyłączy iluminacje sedesu za 30s
	}
}

void SedesIlluminationOFF()		//Powolne wygaszenie iluminacji sedesu, czas 0.5s
{
	SedesDimmer.off();
	Serial.println("Illuminacja wyłączona");
}