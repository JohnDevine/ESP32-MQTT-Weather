# ESP32 + SN-3000-FSJT-N01 Wind Speed Sensor + MAX485 Wiring Diagram

## Component List
- ESP32 DevKit V1 (or similar)
- SN-3000-FSJT-N01 Wind Speed Sensor (RS485 output)
- MAX485 TTL to RS485 Converter Module
- 12V Power Supply (for wind sensor)
- 3.3V/5V Power Supply (for ESP32 and MAX485)
- Twisted pair cable for RS485 communication
- Various jumper wires

## Wiring Connections

### MAX485 Module to ESP32
```
MAX485 Pin    →    ESP32 Pin    →    Purpose
VCC           →    3.3V          →    Power (3.3V or 5V depending on module)
GND           →    GND           →    Ground
DI (Data In)  →    GPIO17        →    Serial TX (data to RS485)
RO (Rec Out)  →    GPIO16        →    Serial RX (data from RS485)
DE (Data En)  →    GPIO4         →    Driver Enable (TX control)
RE (Rec En)   →    GPIO4         →    Receiver Enable (tied to DE)
```

### SN-3000-FSJT-N01 Wind Sensor to MAX485
```
Wind Sensor   →    MAX485        →    Purpose
A+ (RS485+)   →    A             →    RS485 Differential Pair +
B- (RS485-)   →    B             →    RS485 Differential Pair -
VCC           →    12V Supply    →    Sensor Power (10-30V DC)
GND           →    Common GND    →    Ground Reference
```

### Power Connections
```
12V Supply+   →    Wind Sensor VCC, USB-C Mini BMS VIN
12V Supply-   →    Common GND
USB-C Mini BMS 5V OUT  →    ESP32 VIN (via USB-C connector)
ESP32 3.3V    →    MAX485 VCC
Common GND    →    ESP32 GND, MAX485 GND, Wind Sensor GND, BMS GND
```

## Power Supply Notes
- **Primary Power**: 12V DC supply
- **Wind Sensor**: 12V DC direct from main supply (10-30V range)
- **ESP32**: 5V from USB-C Mini BMS (converts 12V→5V via USB-C connector)
- **MAX485**: 3.3V from ESP32's onboard regulator
- **USB-C Mini BMS**: Converts 12V input to 5V USB-C output for ESP32
- **Common GND**: All components share common ground reference

## GPIO Pin Assignments (from specifications)
```
GPIO Pin    Function           Connection
GPIO16      UART2 RX          MAX485 RO (Receive Out)
GPIO17      UART2 TX          MAX485 DI (Data In)
GPIO4       RS485 DE/RE       MAX485 DE and RE (tied together)
```

## RS485 Communication Details
- **Baud Rate**: 4800 (default for SN-3000-FSJT-N01)
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Slave Address**: 01 (default for SN-3000-FSJT-N01)
- **Register**: 0x0000 (wind speed data register)
- **Protocol**: Modbus RTU

## Cable Specifications
- **RS485 Cable**: Twisted pair cable (Cat5/Cat6 or dedicated RS485 cable)
- **Maximum Distance**: Up to 1200m (4000ft) for RS485
- **Termination**: 120Ω termination resistors at both ends for long runs

## Power Requirements
- **12V Supply**: Powers wind sensor directly + USB-C Mini BMS input
- **USB-C Mini BMS**: Converts 12V to 5V, powers ESP32 via USB-C connector
- **ESP32**: ~240mA (active), ~10µA (deep sleep), powered by 5V from BMS
- **MAX485**: 3.3V from ESP32's onboard regulator, ~1mA consumption
- **Wind Sensor**: 10-30V DC, <50mA (powered directly from 12V supply)

## Safety Notes
1. Ensure proper grounding of all components
2. Use appropriate gauge wire for power connections
3. Protect outdoor connections from moisture
4. Consider lightning protection for outdoor installations
5. Verify voltage levels before connecting

## Testing Procedure
1. Connect components as shown in diagram
2. Power on system
3. Use serial monitor to verify RS485 communication
4. Send Modbus query: `01 03 00 00 00 01 84 0A`
5. Expect response with wind speed data
6. Verify data parsing and MQTT publishing

## Integration with Existing Weather Station
This wind sensor will integrate with the existing BME680 and BH1750 sensors, adding wind speed data to the JSON payload published to MQTT topic:
`sensor/baanfarang/weather/office01`
