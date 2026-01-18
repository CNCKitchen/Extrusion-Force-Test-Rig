# PID Temperature Controller - Dokumentation

## Übersicht

Das System verwendet jetzt einen vollständigen PID-Regler (Proportional-Integral-Derivative) für die Temperaturkontrolle des Hot-Ends. Der alte einfache On-Off-Controller wurde durch einen professionellen PID-Regler mit Auto-Tuning-Funktion ersetzt.

## Funktionen

### 1. PID-Regler

Der PID-Regler bietet:
- **P-Anteil (Proportional)**: Reagiert direkt auf die Abweichung von der Zieltemperatur
- **I-Anteil (Integral)**: Eliminiert bleibende Regelabweichungen
- **D-Anteil (Derivative)**: Dämpft Überschwinger und verbessert die Stabilität
- **Anti-Windup**: Verhindert Integral-Sättigung
- **Sicherheitsgrenzen**: Automatisches Abschalten bei Über-/Unterschreitung

### 2. Auto-Tuning (Ziegler-Nichols Relay-Methode)

Das System kann automatisch die optimalen PID-Parameter für eine gegebene Zieltemperatur ermitteln.

#### Funktionsweise:
1. Das System erzeugt eine Oszillation durch Relay-Kontrolle (Ein/Aus)
2. Misst die Amplitude und Periode der Oszillation
3. Berechnet nach Ziegler-Nichols die optimalen Kp, Ki, Kd Werte
4. Speichert die Parameter automatisch

## Verwendung

### Auto-Tuning durchführen

Um die PID-Parameter für eine bestimmte Temperatur zu optimieren:

1. **Wichtig**: Stelle sicher, dass KEINE Messung läuft
2. Sende folgenden Befehl über die serielle Schnittstelle:
   ```
   AUTOTUNE:<Temperatur>
   ```
   Beispiel für 200°C:
   ```
   AUTOTUNE:200
   ```

3. Der Prozess dauert mehrere Minuten und gibt fortlaufende Statusmeldungen aus
4. Am Ende werden die optimierten Parameter angezeigt:
   ```
   === Neue PID-Parameter ===
   Kp: 8.5234
   Ki: 0.4123
   Kd: 2.1567
   === Auto-Tuning abgeschlossen ===
   ```

### Manuelle PID-Parameter setzen

Falls du eigene Parameter verwenden möchtest:

```
SETPID:<Kp>,<Ki>,<Kd>
```

Beispiel:
```
SETPID:10.0,0.5,2.0
```

### Aktuelle PID-Parameter abfragen

```
GETPID
```

Ausgabe:
```
=== Aktuelle PID-Parameter ===
Kp: 8.0000
Ki: 0.5000
Kd: 2.0000
```

## Standard-Parameter

Die Standard-PID-Parameter beim Start sind:
- **Kp** = 8.0
- **Ki** = 0.5
- **Kd** = 2.0

Diese sollten für die meisten Anwendungen bereits gute Ergebnisse liefern, können aber durch Auto-Tuning optimiert werden.

## Tipps für bestes Ergebnis

1. **Auto-Tuning bei Betriebstemperatur**: Führe das Auto-Tuning bei der Temperatur durch, die du am häufigsten verwendest
2. **Umgebungsbedingungen**: Führe das Auto-Tuning unter ähnlichen Bedingungen durch wie deine Messungen (z.B. gleiche Lüftereinstellung)
3. **Geduld**: Das Auto-Tuning braucht Zeit (5-10 Minuten), um genaue Ergebnisse zu liefern
4. **Wiederholung**: Bei sehr unterschiedlichen Temperaturbereichen (z.B. 180°C vs. 250°C) kann es sinnvoll sein, separate Tunings durchzuführen

## Fehlerbehandlung

### Auto-Tuning schlägt fehl
- Prüfe, ob die Zieltemperatur im gültigen Bereich liegt (50-290°C)
- Stelle sicher, dass keine Messung läuft
- Überprüfe die Heizelement- und NTC-Verkabelung

### Temperatur oszilliert stark
- Führe ein Auto-Tuning durch
- Reduziere ggf. den D-Anteil manuell

### Temperatur erreicht Sollwert nicht
- Erhöhe den I-Anteil leicht
- Prüfe die Heizleistung

### Überschwinger beim Aufheizen
- Erhöhe den D-Anteil
- Reduziere den P-Anteil leicht

## Debug-Ausgaben

Während des Betriebs gibt der PID-Regler kontinuierlich Telemetrie-Daten aus:
```
>Temp:199.5
>Setpoint:200.0
>PWM:150
>Power:23.5
```

Diese können mit dem Serial Plotter von Arduino IDE visualisiert werden.

## Technische Details

- **Update-Rate**: 100ms (HEATER_DELAY)
- **Temperaturbereich**: 0-290°C
- **PWM-Bereich**: 0-255
- **Anti-Windup**: Ja, mit dynamischer Begrenzung
- **Sicherheitsabschaltung**: Bei >290°C oder <0°C
