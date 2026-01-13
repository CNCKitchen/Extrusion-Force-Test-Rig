# Extrusionskraftprüfstand (3D-Druck) – Firmware + GUI

Ein Prüfstand zur Untersuchung von **Extrusionsverhalten** im FDM/FFF-3D-Druck:  
Wir erfassen (u.a.) **Temperatur**, **Vorschub**, **Extrusionskraft** (über Wägezelle) und **Filament-Schlupf** (über Encoder) und visualisieren die Messung live in einer **Python-GUI**.

> Status: Prototyp / Laboraufbau  
> Ziel: reproduzierbare Messreihen für Material- und Parametervergleiche

---

## Inhalte / Features

- **Live-Messung** & Plotting (GUI)
- Parametrierung (z.B. Solltemperatur, Vorschub / Feedrate, Messdauer – je nach Firmwarestand)
- Logging/Export der Messdaten (z.B. CSV)
- Modulare Firmware-Architektur (Hotend / LoadCell / Stepper / Encoder / Kommunikation)

---

## Repository-Struktur

Dieses Repo ist als PlatformIO-Projekt + Python-GUI organisiert:

- `src/` – Firmware (Entry/Tasks/Main)
- `include/` – Header / Schnittstellen (Klassen für HotEnd, LoadCell, ExtruderStepper, Encoder, GUI-Kommunikation)
- `lib/` – Externe/zusätzliche Libraries (falls genutzt)
- `Python/` – PC-Software (GUI, Plotting, Logging, Tools/Notebooks)
- `Manual/` – Doku/Anleitungen/Build-Infos
- `test/` – (Optional) Tests/Versuche
- `platformio.ini` – PlatformIO Build-Konfiguration

---

## Hardware (typischer Aufbau)

> **Passe die Liste an deinen finalen Stand an.**

- Mikrocontroller: **ESP32-S3 DevKitC-1** (oder kompatibel)
- Extruder-Antrieb: Stepper + Treiber (**TMC2209**)
- Hotend/Heater: Heizpatrone + Lüfter + Temperaturmessung (z.B. **NTC 100k**)
- Kraftmessung: Wägezelle + **HX711**
- Filamentweg/Schlupf: Rotary-Encoder am Filamentpfad
- Leistungselektronik: MOSFET/Heater-Treiber
- Netzteil passend zu Heater/Stepper (Sicherheit beachten!)

---

## Software-Voraussetzungen

### Firmware (PlatformIO)
- VS Code + **PlatformIO** Extension

### PC-GUI (Python)
- Python **3.10+** empfohlen
- Pakete: `pyserial`, `matplotlib`, `numpy` (ggf. weitere – siehe unten)

---

## Quickstart

### 1) Firmware flashen (PlatformIO)

1. Repo klonen:
   ```bash
   git clone https://github.com/iamGabriel08/Extrusionskraftpruefstand.git
   cd Extrusionskraftpruefstand
