# Traffic Sign
Ziel dieses Projekts ist die autonome Steuerung eines Roboters auf einer festgelegten Strecke. Dabei soll der Roboter Verkehrsschilder erkennen und darauf reagieren. Zudem soll der Roboter einer markierten Fahrbahn folgen.

## Inhalt
- [Roboter](#roboter)
- [Strecke](#strecke)
- [Verkehrsschilder](#verkehrsschilder)
- [Schilderkennung](#schilderkennung)
	- [Funktionsweise](#funktionsweise)
	- [Haar Cascaden](#haar-cascaden)
- [ROS](#ros)
	- [Package erstellen](#package-erstellen)
	- [Publisher und Subscriber](#publisher-und-subscriber)
- [Quellen](#quellen)

## Roboter
Bei dem Roboter handelt es sich um einen Turtlebot 3 Burger. Dieser ist Open Source und wurde entwickelt bei Willow Garage von Melonee Wise und Tully Foote. Er dient als günstiger Einstieg in die Welt der Robotik und ist durch seinen modularen Aufbau leicht zu erweitern. Der verwendete Roboter wurde um eine Raspberry Cam erweitert, um dadurch Verkehrsschilder und die Strecke erkennen zu können.

![Roboterbild](img/turtlebot3.png "Turtlebot 3")
https://spectrum.ieee.org/image/Mjg5Mzk3MQ.png

## Strecke
Die verwendete Strecke orientiert sich am [Turtlebot3 Autorace](http://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#autonomous-driving) Projekt. Dies ist ein Wettbewerb, bei dem verschiedene Teams gegeneinander mit Turtlebot3 eine definierte Strecke abfahren und auf Schilder reagieren.

<img src="img/autorace_map.png" alt="Schild 1" width="600"/>
http://emanual.robotis.com/assets/images/platform/turtlebot3/autonomous_driving/autorace_map.png

## Verkehrsschilder
Die in dem Projekt verwendeten Verkehrsschilder sind Modellverkehrsschilder

<img src="img/Schild1.jpg" alt="Schild 1" width="200"/>
<img src="img/Schild2.jpg" alt="Schild 2" width="200"/>
<img src="img/Schild3.jpg" alt="Schild 3" width="200"/>
<img src="img/Schild4.jpg" alt="Schild 4" width="200"/>
<img src="img/Schild5.jpg" alt="Schild 5" width="200"/>
<img src="img/Schild6.jpg" alt="Schild 6" width="200"/>
<img src="img/Schild7.jpg" alt="Schild 7" width="200"/>
<img src="img/Schild8.jpg" alt="Schild 8" width="200"/>
<img src="img/Schild9.jpg" alt="Schild 9" width="200"/>
<img src="img/Schild10.jpg" alt="Schild 10" width="200"/>
<img src="img/Schild11.jpg" alt="Schild 11" width="200"/>

## Schilderkennung

### Funktionsweise

### Haar Cascaden

## ROS
>Robot Operating System (ROS) ist ein Software-Framework für persönliche Roboter. Die Entwicklung begann 2007 am Stanford Artificial Intelligence Laboratory im Rahmen des Stanford-AI-Robot-Projektes (STAIR) und wurde ab 2009 hauptsächlich am Robotikinstitut Willow Garage weiterentwickelt. Seit April 2012 wird ROS von der neu gegründeten, gemeinnützigen Organisation Open Source Robotics Foundation (OSRF) unterstützt und seit Beendigung der operativen Tätigkeit von Willow Garage 2013 von dieser koordiniert, gepflegt und weiterentwickelt. Seit 2013 beschäftigt sich das ROS Industrial Consortium mit der Förderung und Unterstützung von ROS für Anwendungen in der Industrierobotik. In Europa koordiniert das Fraunhofer IPA die Aktivitäten des ROS Industrial Consortium Europe.

https://de.wikipedia.org/wiki/Robot_Operating_System am 16.12.2018 um 12.16 Uhr

### Package erstellen

### Publisher und Subscriber

## Quellen
Roboter
- http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

OpenCV
- https://pythonprogramming.net/haar-cascade-object-detection-python-opencv-tutorial/
- https://docs.opencv.org/3.3.0/dc/d88/tutorial_traincascade.html
