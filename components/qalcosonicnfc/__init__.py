import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor, text_sensor#, spi
from esphome.const import CONF_ID, CONF_NAME, CONF_PROTOCOL, CONF_UPDATE_INTERVAL, UNIT_CUBIC_METER, UNIT_CUBIC_METER_PER_HOUR, UNIT_CELSIUS, UNIT_PERCENT, ICON_WATER, ICON_THERMOMETER, ICON_BATTERY, STATE_CLASS_TOTAL_INCREASING, STATE_CLASS_MEASUREMENT, DEVICE_CLASS_WATER, DEVICE_CLASS_TEMPERATURE, DEVICE_CLASS_BATTERY

CODEOWNERS = ["@dbmaxpayne"]

AUTO_LOAD = ["sensor", "text_sensor", "spi"]

DEPENDENCIES = ["network"]

MULTI_CONF = True

# Define constants for configuration keys
CONF_PN5180_MOSI_PIN = "pn5180_mosi_pin"
CONF_PN5180_MISO_PIN = "pn5180_miso_pin"
CONF_PN5180_SCK_PIN = "pn5180_sck_pin"
CONF_PN5180_NSS_PIN = "pn5180_nss_pin"
CONF_PN5180_BUSY_PIN = "pn5180_busy_pin"
CONF_PN5180_RST_PIN = "pn5180_rst_pin"
CONF_WATER_USAGE_SENSOR = "water_usage_sensor"
CONF_WATER_FLOW_SENSOR = "water_flow_sensor"
CONF_WATER_TEMPERATURE_SENSOR = "water_temperature_sensor"
CONF_BATTERY_LEVEL_SENSOR = "battery_level_sensor"
CONF_RAW_DATA_SENSOR = "raw_data_sensor"

qalcosonicnfc_ns = cg.esphome_ns.namespace("qalcosonicnfc")
QalcosonicNfc = qalcosonicnfc_ns.class_("QalcosonicNfc", cg.PollingComponent)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(QalcosonicNfc),
            cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_WATER_USAGE_SENSOR, default={ CONF_NAME: "Water Usage",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER,
                icon=ICON_WATER,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                device_class=DEVICE_CLASS_WATER,),
            cv.Optional(CONF_WATER_FLOW_SENSOR, default={ CONF_NAME: "Water Flow",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER_PER_HOUR,
                icon=ICON_WATER,
                state_class=STATE_CLASS_MEASUREMENT,
                device_class=DEVICE_CLASS_WATER,),
            cv.Optional(CONF_WATER_TEMPERATURE_SENSOR, default={ CONF_NAME: "Water Temperature",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                state_class=STATE_CLASS_MEASUREMENT,
                device_class=DEVICE_CLASS_TEMPERATURE,),
            cv.Optional(CONF_BATTERY_LEVEL_SENSOR, default={ CONF_NAME: "Battery Level",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                icon=ICON_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
                device_class=DEVICE_CLASS_BATTERY,),
            cv.Optional(CONF_RAW_DATA_SENSOR, default={ CONF_NAME: "Raw M-BUS Data",}): text_sensor.text_sensor_schema(),
            cv.Required(CONF_PN5180_MOSI_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_PN5180_MISO_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_PN5180_SCK_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_PN5180_NSS_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_PN5180_BUSY_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_PN5180_RST_PIN): pins.gpio_output_pin_schema
        }
    )
)


async def to_code(config):
    pn5180_mosi_pin = await cg.gpio_pin_expression(config[CONF_PN5180_MOSI_PIN])
    pn5180_miso_pin = await cg.gpio_pin_expression(config[CONF_PN5180_MISO_PIN])
    pn5180_sck_pin = await cg.gpio_pin_expression(config[CONF_PN5180_SCK_PIN])
    pn5180_nss_pin = await cg.gpio_pin_expression(config[CONF_PN5180_NSS_PIN])
    pn5180_busy_pin = await cg.gpio_pin_expression(config[CONF_PN5180_BUSY_PIN])
    pn5180_rst_pin = await cg.gpio_pin_expression(config[CONF_PN5180_RST_PIN])
    var = cg.new_Pvariable(config[CONF_ID], pn5180_mosi_pin, pn5180_miso_pin, pn5180_sck_pin, pn5180_nss_pin, pn5180_busy_pin, pn5180_rst_pin)
    await cg.register_component(var, config)
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    
    water_usage_sensor = await sensor.new_sensor(config.get(CONF_WATER_USAGE_SENSOR))
    cg.add(var.set_water_usage_sensor(water_usage_sensor))
    
    water_flow_sensor = await sensor.new_sensor(config.get(CONF_WATER_FLOW_SENSOR))
    cg.add(var.set_water_flow_sensor(water_flow_sensor))
    
    water_temperature_sensor = await sensor.new_sensor(config.get(CONF_WATER_TEMPERATURE_SENSOR))
    cg.add(var.set_water_temperature_sensor(water_temperature_sensor))
    
    battery_level_sensor = await sensor.new_sensor(config.get(CONF_BATTERY_LEVEL_SENSOR))
    cg.add(var.set_battery_level_sensor(battery_level_sensor))
    
    raw_data_sensor = await text_sensor.new_text_sensor(config.get(CONF_RAW_DATA_SENSOR))
    cg.add(var.set_raw_data_sensor(raw_data_sensor))