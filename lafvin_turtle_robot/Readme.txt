Erforderliche Hardware Anpassungen (im assembly manual nicht beschrieben):

- Das dem Roboter-Kit beigelegte Bluetooth-Modul ("HC-06" mit Chip BC352) ist nur Bluetooth 2.x f�hig. Wir ben�tigen (u.A wegen iOS) jedoch BLE (4.x) - das Bluetooth-Modul muss daher ausgetauscht werden (Chip CC2541)
- Das Bluetooth-Modul vertr�gt nur LVTTL (3.3V). Daher sollte in der TX Leitung ("Dupont-Wire") vom Shield zum Bluetooth-Modul ein Widerstand eingel�tet werden (1k...10kOhm)
- RX und TX zwischen Bluetooth-Modul und Shield m�ssen gekreuzt angeschlossen werden (logisch ;-))
- Um das Display direkt unter dem Ultraschallsensor (HC-SR04) zu montieren, die vier Pins des Ultraschallmodules sorgf�ltig um 90� biegen.
- Das Display kann direkt unterhalb des Ultraschallsensors mit Kabelbindern befestigt werden
- Die Kabel an den beiden Motoren m�ssen angel�tet werden. Es wird empfohlen, die L�tstellen mit etwas Schrumpfschlauch sichern.
- Um die Batteriebox zu montieren, statt der mitgelieferten Panhead Schrauben 4 St�ck Senkkopf M3x6 verwenden

--------------------

Verdrahtung:

- Roboter gem�ss "AxRobot_wiring.png" verdrahten - die Farben der Dr�hte darf nat�rlich von dem Diagramm abweichen. Siehe auch RobotFirmware/src/Pinout.h
- Falls der linke und rechte Motor vertauscht sind, Pins 8/9 mit 10/A2 tauschen
- Falls ein Motor verkehrt l�uft, die Pins 8 mit 9 resp. 10 mit A2 tauschen

--------------------

Arduino Firmware:
- Visual Studio Code plus PlatformIO installieren
- CH340G USB to Serial Treiber installieren, sofern notwendig: http://www.wch.cn/download/CH341SER_EXE.html
- axRobot.ino mittels PlatformIO kompilieren und auf dem Arduino UNO installieren

--------------------

Mobile App Arduino/iOS:
- Wir verwenden Visual Studio f�r das Kompilieren der Mobile Applikation (iOS, Android). Xamarin Studio (Starter m�sste gen�gen) sollte auch funktionieren.
- axRobot.sln kompilieren und auf das Mobile kopieren.

--------------------

Allgemeine Hinweise:

F�r den Betrieb werden zwei 18650 LiIon Akkus ben�tigt. Diese haben _keinen_ Tiefentladeschutz. Nicht Tiefentladen (Roboter nach Gebrauch immer ausschalten)! 
Die Akkus sollten nie l�ngere Zeit vollst�ndig entladen herumliegen. 
Es empfiehlt sich, die Akkus nach jedem Gebrauch, bei Nichtgebrauch mindestens alle 6 Monate vollst�ndig aufzuladen. 
F�r das Aufladen nur f�r LiIon Akkus geeignete Ladeger�te verwenden (z.B. Liitokala Lii-202)


	
	
	
	
