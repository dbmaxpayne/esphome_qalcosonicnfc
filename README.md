# esphome_qalcosonicnfc
ESPHome component for reading an Axioma Qalcosonic W1 water meter via a PN5180 NFC chip

## Needed components
- ESP32
- PN5180-NFC module (Can be easily obtained via AliExpress. I paid around 5 USD in August 2025.)
- Some breaboard cables
- (Perfboards)

## Wiring
| ESP32 Pin | PN5180 Pin |
| :---      | :---       |
| VIN / 5V  | 5V         |
| 3.3V      | 3.3V       |
| GND       | GND        |
| SCLK, 18  | SCLK       |
| MISO, 19  | MISO       |
| MOSI, 23  | MOSI       |
| 14        | NSS        |
| 16        | BUSY       |
| 17        | RST        |

## Example configuration
```
external_components:
  - source:
      type: local
      path: my_components
    components: [ qalcosonicnfc ]

esphome:
  name: qalcosonic-w1-nfc-reader
  friendly_name: Qalcosonic W1 NFC Reader

esp32:
  #board: esp32-c3-devkitm-1
  board: esp32dev
  framework:
    type: arduino

qalcosonicnfc:
  update_interval: 60s # How often should the component query the water meter for a value. I am not sure how this affects its battery life!
  pn5180_mosi_pin: GPIO23 # Currently ignored, uses default pin
  pn5180_miso_pin: GPIO19 # Currently ignored, uses default pin
  pn5180_sck_pin: GPIO18 # Currently ignored, uses default pin
  pn5180_nss_pin: GPIO14
  pn5180_busy_pin: GPIO16
  pn5180_rst_pin: GPIO17
  water_usage_sensor:
    name: "Wasserverbrauch"
  # This sensor allows me to maybe find more useful data in the future.
  # You should not need it and can disable it.
  raw_data_sensor:
    name: "Rohdaten"
    disabled_by_default: true

# Enable logging
logger:
  logs:
    # This following can be set to DEBUG to get a lot of information like the raw SPI frames being exchanged between the ESP32 and the PN5180
    qalcosonicnfc: INFO
    PN5180: INFO
    PN5180ISO15693: INFO
```
