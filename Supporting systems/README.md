# Supporting Systems

## Custom Instructions

This assumes you have already installed docker.

Do the following in this order:

1. In the Docker folder: 
    Follow the README.md then:
        docker compose up
    This will create the containers for Influx, Grafana, Node Red etc.

2. go to the "Influx" folder and do what is in the README.md

3. go to the Node Red folder.
   Start Node Red from the browser with 
    http://localhost:1880

    use the pallet manager and install "node-red-contrib-influxdb"
    the use the import to import the "Take_data_from_MQTT_and_write_to_INFLUX.txt"
    the hit the "Deploy" button

## If you add in the Optional Supporting system this is what you can get Dashboard Screenshot

This is for my dual BMS system. 

![Grafana BMS Dashboard](Images/GrafanaBMS.jpg)