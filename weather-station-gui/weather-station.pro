PROJECT =       Weather Station GUI
TEMPLATE =      app
TARGET          += 
DEPENDPATH      += .
QT              += widgets
QT              += serialport

OBJECTS_DIR     = obj
MOC_DIR         = moc
UI_DIR          = ui
LANGUAGE        = C++
CONFIG          += qt warn_on release

# Input
FORMS           += weather-station-main.ui
#FORMS           += weather-station-record.ui
HEADERS         += weather-station.h
HEADERS         += weather-station-main.h
#HEADERS         += weather-station-record.h
SOURCES         += weather-station.cpp
SOURCES         += weather-station-main.cpp
#SOURCES         += weather-station-record.cpp
