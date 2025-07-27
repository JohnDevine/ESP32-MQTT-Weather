# MQTT Topic Structure for Multi-Location Sensor Network

This structure supports multiple sites with various types of devices and sensors, each possibly having multiple instances. The topic format categorizes data by transmission type (iot/sensor), location, and device type, making it hierarchical and wildcard-friendly for efficient dashboard processing.

## ✅ Topic Format

```
[type_of_data]/[site_name]/[device_type]/[device_id]
```

**Note:** All measurements for a device are published as a JSON payload to the device_id topic, not as separate MQTT topics.

---

## 🧭 Components Breakdown

| Element        | Description                                                                 |
|----------------|-----------------------------------------------------------------------------|
| `type_of_data` | `iot`, `sensor`, etc. (categorizes the type of data being transmitted)      |
| `site_name`    | Name of location .. baanfarang, solar_blue (lowercase, spaces replaced with underscores)              |
| `device_type`  | Device type, e.g. `bms`, `engine`, `solar`, `temp`, `camera`, `weather` etc.       |
| `device_id`    | Device identifier (`bms1`,`wsensor01`,`jrelay01`, `engine2`, or `main` if only one exists). All sensor data published as JSON payload to this topic. |

---

## 📦 Example Topics

### Yacht: "Solar Blue"

```
iot/solar_blue/bms/bms1
Payload: {
  "voltage": 12.6,
  "amps": -5.2,
  "soc": 85,
  "cell1_voltage": 3.15,
  "cell2_voltage": 3.20,
  "timestamp": "2025-07-26T10:30:00Z"
}

iot/solar_blue/engine/engine1
Payload: {
  "rpm": 1800,
  "temp": 78.5,
  "exhaust_temp": 145.2,
  "alternator": {
    "volts": 14.2,
    "amps": 12.5
  },
  "battery": {
    "volts": 12.8,
    "amps": 3.2
  },
  "timestamp": "2025-07-26T10:30:00Z"
}

iot/solar_blue/camera/engine_bay
Payload: {
  "ir_image": "base64_encoded_image_data",
  "temperature_range": {"min": 45.2, "max": 89.1},
  "timestamp": "2025-07-26T10:30:00Z"
}
```

### Land Location: "Na House"

```
sensor/baanfarang/bms/bms1
Payload: {
  "voltage": 24.8,
  "amps": -2.1,
  "soc": 92,
  "timestamp": "2025-07-26T10:30:00Z"
}

sensor/baanfarang/weather/main
Payload: {
  "temperature": 22.5,
  "humidity": 65.2,
  "pressure": 1013.25,
  "lux": 340.8,
  "gas_resistance": 0,
  "timestamp": "2025-07-26T10:30:00Z"
}

iot/baanfarang/solar/inverter1
Payload: {
  "volts": 230.5,
  "amps": 8.2,
  "power": 1890,
  "frequency": 50.1,
  "timestamp": "2025-07-26T10:30:00Z"
}
```

---

## 🔍 Subscription Patterns

| Use Case                            | MQTT Subscription Topic               |
|-------------------------------------|----------------------------------------|
| All BMS data from Solar Blue        | `iot/solar_blue/bms/+`                |
| All BMS data from all IoT systems   | `iot/+/bms/+`                          |
| All engine data from any site       | `+/+/engine/+`                         |
| All camera data from all locations  | `+/+/camera/+`                         |
| All weather data from all sites     | `+/+/weather/+`                        |
| All data from Baanfarang            | `+/baanfarang/+/+`                     |
| Specific device from Solar Blue     | `iot/solar_blue/bms/bms1`              |
| **Weather Dashboard Node-RED**       | `+/+/weather/+`                        |

---

## 🧩 Optional Metadata Topics

Use retained topics for static metadata:

```
meta/solar_blue/location = {"lat": 8.1234, "lon": 98.5678}
meta/solar_blue/owner = "John D"
meta/solar_blue/engine/engine1/model = "Yanmar 3YM30"
```

---

## 🛠️ Naming Rules

- Use all **lowercase** for topic components
- Replace **spaces with underscores** in site names
- Stick to **descriptive device IDs** (`bms1`, `main`, `inverter1`)
- Use **JSON payloads** for all sensor data with consistent field naming
- Include **timestamps** in all payloads for data correlation
- Use **nested JSON objects** for complex devices with multiple subsystems

---

## 🏁 Summary

This structure provides:

- **Simplified topic hierarchy** - only 4 levels deep, ending at device_id
- **Consolidated data** - all measurements for a device in a single JSON payload
- **Easy filtering** by site, system, or specific device
- **Reduced MQTT overhead** - fewer topics, larger but more efficient payloads
- **Flexibility** for future expansion (more systems, locations, sensors)
- **Better data correlation** with timestamps in every payload

**Key Benefits:**
- Node-RED flows are simpler with single topic subscriptions per device
- InfluxDB ingestion is more efficient with batch data points
- Grafana queries can access all device metrics from single measurement
- Reduced MQTT broker load with fewer topic subscriptions
- **System-level filtering** enables device-type specific dashboards (weather, bms, engine)

---

## 📊 InfluxDB Integration

Data is stored in InfluxDB v2 with the following structure:

**Tags:** `type_of_data`, `site_name`, `device_type`, `device_id`
**Measurement:** `Readings`
**Fields:** All sensor data (temperature, humidity, voltage, etc.)

### Example InfluxDB Line Protocol:
```
Readings,type_of_data=sensor,site_name=baanfarang,device_type=weather,device_id=main temperature=22.5,humidity=65.2,pressure=1013.25,lux=340.8,gas_resistance=0 1643208600000000000
```

---

## 🎯 Grafana Dashboard Strategy

### Variable Hierarchy:
1. **site_name** - Primary selector (baanfarang, solar_blue)
2. **device_id** - Cascading selector (populated based on site_name)
3. **device_type** - Hard-coded in queries for sensor-specific dashboards

### Dashboard Types:
- **Weather Dashboard** - `device_type = "weather"` hardcoded
- **BMS Dashboard** - `device_type = "bms"` hardcoded  
- **Engine Dashboard** - `device_type = "engine"` hardcoded
- **Overview Dashboard** - All device types with device_type variable

### Node-RED Flow Strategy:
Each Node-RED flow subscribes to a specific `device_type` using wildcards:
- **Weather Flow**: `+/+/weather/+` (captures all weather devices from any site)
- **BMS Flow**: `+/+/bms/+` (captures all BMS devices from any site)  
- **Engine Flow**: `+/+/engine/+` (captures all engine devices from any site)

This approach allows each flow to process all devices of its type regardless of location or device_id.

You can now use this as a reference file or template in your VS Code project or documentation.