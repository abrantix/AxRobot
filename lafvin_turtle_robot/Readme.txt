Erforderliche Hardware Anpassungen (im assembly manual nicht beschrieben):

- Das dem Roboter-Kit beigelegte Bluetooth-Modul ("HC-06" mit Chip BC352) ist nur Bluetooth 2.x fähig. Wir benötigen (u.A wegen iOS) jedoch BLE (4.x) - das Bluetooth-Modul muss daher ausgetauscht werden (Chip CC2541)
- Das Bluetooth-Modul verträgt nur LVTTL (3.3V). Daher sollte in der TX Leitung ("Dupont-Wire") vom Shield zum Bluetooth-Modul ein Widerstand eingelötet werden (1k...10kOhm)
- RX und TX zwischen Bluetooth-Modul und Shield müssen gekreuzt angeschlossen werden (logisch ;-))
- Um das Display direkt unter dem Ultraschallsensor (HC-SR04) zu montieren, die vier Pins des Ultraschallmodules sorgfältig um 90° biegen.
- Das Display kann direkt unterhalb des Ultraschallsensors mit Kabelbindern befestigt werden
- Die Kabel an den beiden Motoren müssen angelötet werden. Es wird empfohlen, die Lötstellen mit etwas Schrumpfschlauch sichern.
- Um die Batteriebox zu montieren, statt der mitgelieferten Panhead Schrauben 4 Stück Senkkopf M3x6 verwenden

--------------------

Verdrahtung:

- Roboter gemäss "AxRobot_wiring.png" verdrahten - die Farben der Drähte darf natürlich von dem Diagramm abweichen. Siehe auch RobotFirmware/src/Pinout.h
- Falls der linke und rechte Motor vertauscht sind, Pins 8/9 mit 10/A2 tauschen
- Falls ein Motor verkehrt läuft, die Pins 8 mit 9 resp. 10 mit A2 tauschen

--------------------

Arduino Firmware:
- Visual Studio Code plus PlatformIO installieren
- CH340G USB to Serial Treiber installieren, sofern notwendig: http://www.wch.cn/download/CH341SER_EXE.html
- axRobot.ino mittels PlatformIO kompilieren und auf dem Arduino UNO installieren

--------------------

Mobile App Arduino/iOS:
- Wir verwenden Visual Studio für das Kompilieren der Mobile Applikation (iOS, Android). Xamarin Studio (Starter müsste genügen) sollte auch funktionieren.
- axRobot.sln kompilieren und auf das Mobile kopieren.

--------------------

Allgemeine Hinweise:

Für den Betrieb werden zwei 18650 LiIon Akkus benötigt. Diese haben _keinen_ Tiefentladeschutz. Nicht Tiefentladen (Roboter nach Gebrauch immer ausschalten)! 
Die Akkus sollten nie längere Zeit vollständig entladen herumliegen. 
Es empfiehlt sich, die Akkus nach jedem Gebrauch, bei Nichtgebrauch mindestens alle 6 Monate vollständig aufzuladen. 
Für das Aufladen nur für LiIon Akkus geeignete Ladegeräte verwenden (z.B. Liitokala Lii-202)


	
	
	
	
