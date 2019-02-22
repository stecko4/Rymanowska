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

//Wyświetlacz OLED
#include <Arduino.h>
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//for OTA
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
bool OTAConfigured = 0;

//#define BLYNK_DEBUG			//Optional, this enables lots of prints
//#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>
#include <TimeLib.h>
#include <WidgetRTC.h>
WidgetBridge bridge1(V20);				//Initiating Bridge Widget on V20 of Device A
WidgetTerminal terminal(V40);				//Attach virtual serial terminal to Virtual Pin V40
WidgetLED LED_CO(V8);					//Inicjacja diody LED dla załączania CO
SimpleTimer TimerBlynkCheck;				//Do sprawdzana połączenia z Blynkiem uruchamiany do 30s
SimpleTimer TimerMainFunction;				//dla MainFunction uruchamiany do 3s
WidgetRTC rtc;						//Inicjacja widgetu zegara czasu rzeczywistego RTC

int		Tryb_Sterownika		= 0;		//Tryb_Sterownika 0 = AUTO, 1 = ON, 2 = OFF, 3 = MANUAL
float		SetTempManual		= 21;		//Temperatura nastawiana manualnie z aplikacji Blynka 
float		SetTempActual		= 18.5;		//Temperatura według której sterowana jest temperatura (auto lub manual)
float		SetTempSchedule[7][24]	= {
//00:00 01:00 02:00 03:00 04:00 05:00 06:00 07:00 08:00 09:00 10:00 11:00 12:00 13:00 14:00 15:00 16:00 17:00 18:00 19:00 20:00 21:00 22:00 23:00
  {18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 18.5 },  //Niedziela
  {18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 20.8, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 18.5 },  //Poniedziałek
  {18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 20.8, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 18.5 },  //Wtorek
  {18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 20.8, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 18.5 },  //Środa
  {18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 20.8, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 18.5 },  //Czwartek
  {18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 20.8, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 20.8, 18.5 },  //Piątek
  {18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5, 18.5 }   //Sobota
}; //Ustawienia temperatury dla poszczególnych dni/godzin, temperatura musi być pomiędzy 14 a 26 stopni Celsjusza z dokładnością do 2 miejsc po przecinku

int		OLED_ON			= 1;		//Przyjmuje wartość '1' dla OLED włączony i '0' dla OLED wyłączony
boolean		Podlane			= true;		//Przyjmuje wartość true gdy podlane czyli wilgotność >80% i false gdy sucho wilgotność <60%
boolean		SoilNotification60	= false;	//Przyjmuje wartość true gdy wysłano notyfikacje o wilgotności 60%, true po podlaniu
boolean		SoilNotification50	= false;	//Przyjmuje wartość true gdy wysłano notyfikacje o wilgotności 50%, true po podlaniu
boolean		SoilNotification40	= false;	//Przyjmuje wartość true gdy wysłano notyfikacje o wilgotności 40%, true po podlaniu
float		RH 			= 0;		//Soil Relative Humidity in % 
long		CZAS_START_MANUAL	= 0;		//Ustawienie czasu przejścia sterowania w trym MANUAL (wartość w sekundach)
long		CZAS_START_AUTO		= 0;		//Ustawienie czasu przejścia sterowania w trym AUTO (wartość w sekundach)
int		dzien			= 0;		// day of the week (1-7), Sunday is day 1
int		godz			= 1;		// the hour now (0-23)
float		temp(NAN), hum(NAN), pres(NAN), dewPoint(NAN), absHum(NAN), heatIndex(NAN);	//Zmienne dla danych z czujnika BME280

//STAŁE
const char  ssid[]      = "XXXX";
const char  pass[]      = "XXXX";
const char  auth[]      = "XXXX"; //Token Pokój Rymanowska
const int	checkInterval		= 30000;	//Co 30s zostanie sprawdzony czy jest sieć Wi-Fi i czy połączono z serwerem Blynk
const int	HeatCO			= D6;		//Pin do włączania CO w sterowniku w łazience
const int	MinTemp			= 14;		//Najniższa możliwa temperatura do ustawienia
const int	MaxTemp			= 26;		//Najwyższa możliwa temperatura do ustawienia
const float	TemtHist		= 0.2;		//histereza dla temperatury
const float	HumidHist		= 5;		//histereza dla wilgotności

BLYNK_CONNECTED()			//Informacja że połączono z serwerem Blynk, synchronizacja danych
{
	Serial.println("Reconnected, syncing with cloud.");
	bridge1.setAuthToken("XXXX"); // Token of the hardware B (Łazienka)
	rtc.begin();
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
			ArduinoOTA.setHostname("Pokoj_Rymanowska");

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
	Read_BME280_Values();		//Odczyt danych z czujnika BME280
	TrybManAuto();			//Ustawienie trybów sterowania i temperatury do załączenia pieca CO
	RH = ReadSoilMoisture();	//Odczyt z czujnika wilgotności gleby i konwersja do wartości 0 - 100%
	Room_Temp_Control();		//kontrola temperatury na podstawie odczytów z BME280
	OLED_Display();			//Wyświetlanie na ekranie OLED 0.96"
	Wyslij_Dane();			//Wysyła dane do serwera Blynk
}

void TrybManAuto()			//Ustawienie trybów sterowania i temperatury do załączenia pieca CO
{
	//Start Manual za pomocą Time Input
	if (CZAS_START_MANUAL > 0 && hour(now()) * 60 + minute(now()) == CZAS_START_MANUAL / 60)	//Tryb_Sterownika 0 = AUTO, 1 = ON, 2 = OFF, 3 = MANUAL
	{
		CZAS_START_MANUAL = 0;		//Zerowanie czasu uruchomienia tryby MANUALNEGO, czyli wyłączenie załączanie czasowego
		Blynk.virtualWrite(V11, 4);	//Ustawienie widgetu BLYNK w tryb MAN
		Tryb_Sterownika = 3;		//Tryb_Sterownika 0 = AUTO, 1 = ON, 2 = OFF, 3 = MANUAL
		//reset Time Input widget in app
		int startAt = - 1;
		int stopAt;
		if (CZAS_START_AUTO == 0)
		{
			stopAt = - 1;
		}
		else
		{
			stopAt = CZAS_START_AUTO;
		}
		Blynk.virtualWrite(V13, startAt, stopAt, "Europe/Warsaw");
	}
	// Start Auto za pomocą Time Input
	if (CZAS_START_AUTO > 0 && hour(now()) * 60 + minute(now()) == CZAS_START_AUTO / 60)		//Tryb_Sterownika 0 = AUTO, 1 = ON, 2 = OFF, 3 = MANUAL
	{
		CZAS_START_AUTO = 0;		//Zerowanie czasu uruchomienia tryby AUTOMATYCZNEGO, czyli wyłączenie załączanie czasowego
		Blynk.virtualWrite(V11, 1);	//Ustawienie widgetu BLYNK w tryb AUTO
		Tryb_Sterownika = 0;		//Tryb_Sterownika 0 = AUTO, 1 = ON, 2 = OFF, 3 = MANUAL
		//reset Time Input widget in app
		int startAt = - 1;
		int stopAt = - 1;
		Blynk.virtualWrite(V13, startAt, stopAt, "Europe/Warsaw");
	}

	dzien = weekday(now()) - 1;			//day of the week (1-7), Sunday is day 1
	godz = hour(now());					//the hour now  (0-23)
	//Ustawienie temperatury sterowania
	if (Tryb_Sterownika == 0)		//Tryb AUTO,  temperatura w zmiennej SetTempActual ustawiana tylko jeśli różna od zadanej w SetTempSchedule
	{
		if(SetTempActual != SetTempSchedule[dzien][godz])
		{
			SetTempActual = SetTempSchedule[dzien][godz]; 
		}
	}
	else if (Tryb_Sterownika == 1)		//Tryb ON,  temperatura w zmiennej SetTempActual ustawiana tylko jeśli różna od zadanej w SetTempSchedule
	{
		if(SetTempActual != temp)
		{
			SetTempActual = temp;
		}
	}
	else if (Tryb_Sterownika == 2)		//Tryb OFF,  temperatura w zmiennej SetTempActual ustawiana tylko jeśli różna od 0 (zera)
	{
		if(SetTempActual != 0)
		{
			SetTempActual = 0;
		}
	}
	else if (Tryb_Sterownika == 3)		//Tryb MAN,  temperatura w zmiennej SetTempActual ustawiana tylko jeśli różna od zadanej w SetTempManual
	{
		if (SetTempActual != SetTempManual)
		{
			SetTempActual = SetTempManual;
		}
	}
}

void Read_BME280_Values()		//Odczyt wskazań z czujnika BME280
{
	bme.readSensor();		//Odczyt wskazań z czujnika BME280
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

float ReadSoilMoisture()		//Odczyt z czujnika wilgotności gleby i konwersja do wartości 0 - 100%
{
	int i;
	float sval = 0;

	for (i = 0; i < 5; i++)
	{		//Uśrednianie wartości z czujnika analogowego
		sval = sval + analogRead(A0);	//sensor on analog pin 0
	}
	sval = sval / 5;
	sval = map(sval, 818, 427, 0, 100);	//Convert to Relative Humidity in % (818 -> sensor in air, 427 -> sensor in water)
	sval = constrain(sval, 0, 100);		//Limits range of sensor values to between 10 and 150
	//informacja o podlaniu
	if (sval > 80 && Podlane == false)
	{
		Podlane = true;			//Ustawia wartość na true czyli podlano kwiatka
		SoilNotification40 = false;	//Ustawia wartość na true czyli zeruje ilość powiadomień po podlaniu kwiatka
		SoilNotification50 = false;	//Ustawia wartość na true czyli zeruje ilość powiadomień po podlaniu kwiatka
		SoilNotification60 = false;	//Ustawia wartość na true czyli zeruje ilość powiadomień po podlaniu kwiatka
		Blynk.notify("Kwiatek został podlany :)");
	}
	else if (sval < 60 && Podlane == true)
	{
		Podlane = false;		//Ustawia wartość na false czyli kwiatek ma sucho
	}
	//Powiadomienia na smartfona
	if (sval < 40 && Podlane == false && SoilNotification40 == false)
	{
		Blynk.notify("Podlej tego kwiatka bo uschnie! Wilgotność spadła już do 40%.");
		SoilNotification40 = true;	//Ustawia wartość na true czyli powiadomienie przy wilgotności 40% poszło
	}
	else if (sval < 50 && Podlane == false  && SoilNotification50 == false)
	{
		Blynk.notify("Wilgotność w kwiatku spadła poniżej 50%. Teraz już trzeba podlać!");
		SoilNotification50 = true;	//Ustawia wartość na true czyli powiadomienie przy wilgotności 50% poszło
	}
	else if (sval < 60 && Podlane == false && SoilNotification60 == false)
	{
		Blynk.notify("Wilgotność w kwiatku spadła poniżej 60%. Może trzeba podlać?");
		SoilNotification60 = true;	//Ustawia wartość na true czyli powiadomienie przy wilgotności 60% poszło
	}

return sval;
}

void Room_Temp_Control()		//Sterowanie piecem w zależności od temperatury
{
	if (Tryb_Sterownika == 1)
	{
		bridge1.digitalWrite(HeatCO, 0);	//Wysłanie sygnału do włączenia pieca
		LED_CO.on();				//Piec CO grzeje
		digitalWrite(BUILTIN_LED, LOW);		//Niebieska dioda WEMOSA gaśnie
	}
	else if (temp < SetTempActual - TemtHist)
	{
		bridge1.digitalWrite(HeatCO, 0);	//Wysłanie sygnału do włączenia pieca
		LED_CO.on();				//Piec CO grzeje
		digitalWrite(BUILTIN_LED, LOW);		//Niebieska dioda WEMOSA gaśnie
	}
	else if (temp > SetTempActual + TemtHist)
	{
		bridge1.digitalWrite(HeatCO, 1023);	//Wysłanie sygnału do wyłączenia pieca (1023 bo piny obsługują PWM i nadanie "1" nie działa)
		LED_CO.off();				//Piec CO nie grzeje
		digitalWrite(BUILTIN_LED, HIGH);	//Niebieska dioda WEMOSA świeci
	}
}

void OLED_Display()			//Wyświetlanie na ekranie OLED 0.96"
{
	if (OLED_ON == 1)
	{
		//Wyświetlanie dane na OLED
		String DispTemp = "Temp:               " + String(temp) + "     "+ char(176) + "C";
		String DispPress = "Press:            " + String(pres) +"   hPa";
		String DispHum = "Hum:               "+String(hum) + "      %";
		String DispdewPoint = "DewPoint:       " + String(dewPoint) + "     " + char(176) + "C";
		String DispabsHum = "AbsHum:         " + String(absHum) +"   g/m" + char(179);

		String currentTime = String(hour()) + ":" + minute() + ":" + second();
		String currentDate = String(day()) + " " + month() + " " + year();

		u8g2.clearBuffer();
		u8g2.setFontMode(1);
		u8g2.setFont(u8g2_font_helvR08_tf);
		u8g2.drawStr(0,9,"Set:");

		if (Tryb_Sterownika != 2)
		{
			u8g2.setCursor(40,28);
			u8g2.print("°C");
		}
		u8g2.setCursor(40+64,28);
		u8g2.print("°C");
		u8g2.drawStr(64,9,"Actual:");
		u8g2.setFont(u8g2_font_logisoso16_tf); 
		u8g2.setCursor(3,35);

		if (Tryb_Sterownika != 2)
		{
			u8g2.print(SetTempActual,1);
		}
		else
		{
			u8g2.print("OFF");
		}
		//u8g2.print(SetTempActual,1);
		u8g2.setCursor(64,35);
		u8g2.print(temp,1);
		//Wyświetlanie aktualnego czasu
		u8g2.setFont(u8g2_font_helvR08_tf);   
		u8g2.setCursor(0,64);
		u8g2.print(currentTime);
		//Wyświetlanie aktualnej daty  
		u8g2.setCursor(75,64);
		u8g2.print(currentDate);
		u8g2.sendBuffer();
	}
	else if (OLED_ON == 0)
	{
		u8g2.clearBuffer();
		u8g2.sendBuffer();
	}
}

void Wyslij_Dane()			//Wysyłanie danych na serwer Blynka
{
	Blynk.virtualWrite(V0, temp);				//Temperatura [°C]
	Blynk.virtualWrite(V1, hum);				//Wilgotność [%]
	Blynk.virtualWrite(V2, pres);				//Ciśnienie [hPa]
	Blynk.virtualWrite(V3, dewPoint);			//Temperatura punktu rosy [°C]
	Blynk.virtualWrite(V4, absHum);				//Wilgotność bezwzględna [g/m³]
	Blynk.virtualWrite(V5, heatIndex);			//Temperatura odczuwalna [°C] 
	Blynk.virtualWrite(V6, RH);				//Wilgotność gleby [%]  
	Blynk.virtualWrite(V18, SetTempActual);			//Temperatura zadana [°C]
	Blynk.virtualWrite(V25, constrain(map(WiFi.RSSI(), -105, -40, 0, 100), 0, 100)); //Siła sygnału Wi-Fi [%], constrain() limits range of sensor values to between 0 and 100
	bridge1.virtualWrite(V21, hum);				//Wilgotność w pokoju wysyłana do sterownika w łazience [%]
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
		terminal.println("V6   ->  Wilgotność gleby   %");
		terminal.println("V10  <-  OLED_ON            1/0");
		terminal.println("V11  <-  Tryb_Sterownika    1,2,3,4");
		terminal.println("V12  <-  SetTempManual      °C");
		terminal.println("V13  <-  BLYNK Timer        sec");
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
		terminal.print("V10    OLED_ON       =   ");
		terminal.print(OLED_ON);
		terminal.println(" ");
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

BLYNK_WRITE(V10)			//Włączanie i wyłączanie wyświetlacza
{
	OLED_ON = param.asInt(); 
}

BLYNK_WRITE(V11)			//Sterowanie ogrzewaniem z aplikacji (AUTO, ON, OFF, MAN)
{
	switch (param.asInt())
	{
		case 1:				//AUTO
			Tryb_Sterownika = 0;
			break;
		case 2:				//ON
			Tryb_Sterownika = 1;
			break;
		case 3:				//OFF
			Tryb_Sterownika = 2;
			break;
		case 4:				//MAN
			Tryb_Sterownika = 3;
			break;
			default:		//Wartość domyślna AUTO
			Tryb_Sterownika = 0;
	}
}

BLYNK_WRITE(V12)			//Ustawienie progu temperatury poniżej której załączy się CO (plus próg)
{
	SetTempManual = param.asFloat();
}

BLYNK_WRITE(V13)			//Obsługa timera Start Manual i Start Auto (Time Input Widget)
{
	CZAS_START_MANUAL = param[0].asLong();		//Ustawienie czasu przejścia sterowania w trym MANUAL (wartość w sekundach)
	CZAS_START_AUTO = param[1].asLong();		//Ustawienie czasu przejścia sterowania w trym AUTO (wartość w sekundach)
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
	pinMode(BUILTIN_LED, OUTPUT);					//Będzie mrugał diodą

	//inicjowanie wyświetlacza
	u8g2.begin();
	u8g2.enableUTF8Print();
	u8g2.setDisplayRotation(U8G2_R0);
	u8g2.setFlipMode(1);						//Odwrócenie ekrany o 180 stopnie jeśli parametr = 1

	u8g2.clearBuffer();
	u8g2.setFontMode(1);
	u8g2.setFont(u8g_font_helvB24);
	u8g2.setCursor(0,33);
	u8g2.print(F("BME280"));
	u8g2.setFont(u8g_font_helvR08);
	//u8g2.setCursor(0, 50);
	//u8g2.print(clock.dateFormat("d-m-Y H:i:s", dt));
	u8g2.setCursor(0, 64);
	u8g2.print("Connecting to: "+String(ssid));
	u8g2.sendBuffer();

	//inicjowanie czujnika BME280
	if (!bme.begin())
	{
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}

	bme.setTempCal(-5.8);						// Temp was reading high so subtract 5.8 degree 
}

void loop()
{
	TimerBlynkCheck.run();
	TimerMainFunction.run();
	OTA_Handle();			//Obsługa OTA (Over The Air) wgrywanie nowego kodu przez Wi-Fi
	if (Blynk.connected()) Blynk.run();
}