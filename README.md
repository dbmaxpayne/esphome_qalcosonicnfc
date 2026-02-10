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

## Special Thanks
Special thanks goes to @ATrappmann for his PN5180-Library (https://github.com/ATrappmann/PN5180-Library).
Without his work, this project would not have been possible.
Also I thank everyone who has contributed to this repository for their work.

## Example configuration
```
external_components:
  - source:
      type: git
      url: https://github.com/dbmaxpayne/esphome_qalcosonicnfc
      #ref: refs/pull/5/head # Uncomment to test an active pull request 
  #- source:
  #    type: local
  #    path: my_components
    components: [ qalcosonicnfc ]
    #refresh: 1min # Refresh interval. Leave this commented if you're not testing any new pull requests
  

esphome:
  name: qalcosonic-w1-nfc-reader
  friendly_name: Qalcosonic W1 NFC Reader

esp32:
  #board: esp32-c3-devkitm-1
  board: esp32dev
  framework:
    type: arduino

qalcosonicnfc:
  update_interval: 300s # How often should the component query the water meter for a value.
                        # Battery drain:
                        # 60s: ~ 1% per 75 days (added 10.02.2026, tested by dbmaxpayne)
  pn5180_mosi_pin: GPIO23
  pn5180_miso_pin: GPIO19
  pn5180_sck_pin:  GPIO18
  pn5180_nss_pin:  GPIO14
  pn5180_busy_pin: GPIO16
  pn5180_rst_pin:  GPIO17
  water_usage_sensor:
    name: "Wasserverbrauch"
  water_usage_positive_sensor:
    name: "Volume (Only Positive)"
    disabled_by_default: True
  water_usage_negative_sensor:
    name: "Volume (Only Negative)"
    disabled_by_default: True
  water_flow_sensor:
    name: "Wasserdurchfluss"
  water_temperature_sensor:
    name: "Wassertemperatur"
  external_temperature_sensor:
    name: "External Temperature"
  battery_level_sensor:
    name: "Batteriestand"
  timepoint_sensor:
    name: "Timepoint"
    disabled_by_default: True
  # This sensor allows me to maybe find more useful data in the future.
  # You should not need it and can disable it.
  raw_data_sensor:
    name: "M-BUS Rohdaten"
    disabled_by_default: True

# Enable logging
logger:
  logs:
    # This following can be set to DEBUG to get a lot of information like the raw SPI frames being exchanged between the ESP32 and the PN5180
    qalcosonicnfc: INFO
    PN5180: INFO
    PN5180ISO15693: INFO
```

## Images
<img src="./media/esp32_pn5180_1.jpg" width="200" /> <img src="./media/esp32_pn5180_2.jpg" width="200" /> <img src="./media/esp32_pn5180_3.jpg" width="200" />
