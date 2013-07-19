arduino_sonars.py published all 7 readings from the sensors and published them to a topic called sonarReadings in meters. It uses the SegbotSensorsStatus.msg

sonar_listener.cpp reads only the first 5 readings from the sonarReadings topic and publishes them to pointcloud2 topic
