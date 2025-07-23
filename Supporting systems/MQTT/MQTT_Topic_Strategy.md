# MQTT Topic Structure for Multi-Location Sensor Network

This structure supports multiple physical locations (yachts and land sites) with various types of sensors, each possibly having multiple instances. The topic format is hierarchical and wildcard-friendly.

## ✅ Topic Format

```
[site_type]/[site_name]/[system]/[device_id]/[measurement]
```

---

## 🧭 Components Breakdown

| Element        | Description                                                                 |
|----------------|-----------------------------------------------------------------------------|
| `site_type`    | `yacht`, `land`, etc. (for clarity and future scalability)                  |
| `site_name`    | Name of location (lowercase, spaces replaced with underscores)              |
| `system`       | Subsystem type, e.g. `bms`, `engine`, `solar`, `temp`, `camera`, `weather` etc.       |
| `device_id`    | Device identifier (`bms1`, `engine2`, or `main` if only one exists)         |
| `measurement`  | Metric name, e.g. `voltage`, `amps`, `soc`, `rpm`, `exhaust_temp`, `Lumens` etc.     |

---

## 📦 Example Topics

### Yacht: "Solar Blue"

```
yacht/solar_blue/bms/bms1/voltage
yacht/solar_blue/bms/bms1/amps
yacht/solar_blue/bms/bms1/soc
yacht/solar_blue/bms/bms1/cell1_voltage
yacht/solar_blue/bms/bms1/cell2_voltage
yacht/solar_blue/engine/engine1/rpm
yacht/solar_blue/engine/engine1/temp
yacht/solar_blue/engine/engine1/exhaust_temp
yacht/solar_blue/engine/engine1/alternator/volts
yacht/solar_blue/engine/engine1/alternator/amps
yacht/solar_blue/engine/engine1/battery/volts
yacht/solar_blue/engine/engine1/battery/amps
yacht/solar_blue/engine/engine1/engine_bay/ir_image
```

### Land Location: "Na House"

```
land/na_house/bms/bms1/voltage
land/na_house/engine/engine1/temp
land/na_house/solar/inverter1/volts
land/na_house/temp/outdoor
```

---

## 🔍 Subscription Patterns

| Use Case                            | MQTT Subscription Topic               |
|-------------------------------------|----------------------------------------|
| All BMS data from Solar Blue        | `yacht/solar_blue/bms/+/+#`            |
| All volts from all yachts           | `yacht/+/+/+/voltage`                  |
| All engine data from any site       | `+/+/engine/#`                         |
| All IR images from all locations    | `+/+/engine/+/engine_bay/ir_image`     |
| All data from Na House              | `land/na_house/#`                      |

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

- Use all **lowercase**
- Replace **spaces with underscores**
- Stick to **singular, lowercase measurement names** (`voltage`, not `voltages`)
- Use `/` to create sub-categories (`battery/volts`, `engine_bay/ir_image`)

---

## 🏁 Summary

This structure provides:

- Clear navigation of topics
- Easy filtering by site, system, or metric
- Flexibility for future expansion (more systems, locations, sensors)

You can now use this as a reference file or template in your VS Code project or documentation.