import copy
from importlib import resources
import logging
from typing import Optional

import tzlocal

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor, text_sensor, binary_sensor
from esphome.const import CONF_ID, CONF_NAME, CONF_UPDATE_INTERVAL, UNIT_CUBIC_METER, UNIT_CUBIC_METER_PER_HOUR, UNIT_CELSIUS, UNIT_PERCENT, UNIT_SECOND, ICON_WATER, ICON_THERMOMETER, ICON_BATTERY, STATE_CLASS_TOTAL_INCREASING, STATE_CLASS_MEASUREMENT, DEVICE_CLASS_WATER, DEVICE_CLASS_VOLUME_FLOW_RATE, DEVICE_CLASS_TEMPERATURE, DEVICE_CLASS_BATTERY, DEVICE_CLASS_DURATION, DEVICE_CLASS_PROBLEM, ENTITY_CATEGORY_DIAGNOSTIC, DEVICE_CLASS_TIMESTAMP, CONF_TIMEZONE, CONF_DEVICE_CLASS

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@dbmaxpayne"]

AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor", "spi"]

DEPENDENCIES = ["network"]

MULTI_CONF = False

# Define constants for configuration keys
CONF_PN5180_MOSI_PIN = "pn5180_mosi_pin"
CONF_PN5180_MISO_PIN = "pn5180_miso_pin"
CONF_PN5180_SCK_PIN = "pn5180_sck_pin"
CONF_PN5180_NSS_PIN = "pn5180_nss_pin"
CONF_PN5180_BUSY_PIN = "pn5180_busy_pin"
CONF_PN5180_RST_PIN = "pn5180_rst_pin"
CONF_WATER_USAGE_SENSOR = "water_usage_sensor"
CONF_WATER_USAGE_POSITIVE_SENSOR = "water_usage_positive_sensor"
CONF_WATER_USAGE_NEGATIVE_SENSOR = "water_usage_negative_sensor"
CONF_WATER_FLOW_SENSOR = "water_flow_sensor"
CONF_WATER_TEMPERATURE_SENSOR = "water_temperature_sensor"
CONF_EXTERNAL_TEMPERATURE_SENSOR = "external_temperature_sensor"
CONF_BATTERY_LEVEL_SENSOR = "battery_level_sensor"
CONF_OPERATING_TIME_SENSOR = "operating_time_sensor"
CONF_ON_TIME_SENSOR = "on_time_sensor"
CONF_METER_VERSION_SENSOR = "meter_version_sensor"
CONF_TIMEPOINT_SENSOR = "timepoint_sensor"
CONF_TIMEPOINT_SENSOR_RAW = "timepoint_sensor_raw"
CONF_RAW_DATA_SENSOR = "raw_data_sensor"
CONF_SERIAL_NUMBER_SENSOR = "serial_number_sensor"
CONF_METER_ID_SENSOR = "meter_id_sensor"
CONF_MANUFACTURER_ID_SENSOR = "manufacturer_id_sensor"
CONF_ERROR_FLAGS_RAW = "error_flags_raw"
CONF_ERROR_RECONFIGURATION_WARNING = "error_reconfiguration_warning"
CONF_ERROR_NO_CONSUMPTION = "error_no_consumption"
CONF_ERROR_DAMAGE_METER_HOUSING = "error_damage_meter_housing"
CONF_ERROR_CALCULATOR_HARDWARE_FAILURE = "error_calculator_hardware_failure"
CONF_ERROR_LEAKAGE = "error_leakage"
CONF_ERROR_BURST = "error_burst"
CONF_ERROR_OPTICAL_COMMUNICATION = "error_optical_communication"
CONF_ERROR_LOW_BATTERY = "error_low_battery"
CONF_ERROR_SOFTWARE_FAILURE = "error_software_failure"
CONF_ERROR_HARDWARE_FAILURE = "error_hardware_failure"
CONF_ERROR_NO_SIGNAL = "error_no_signal"
CONF_ERROR_REVERSE_FLOW = "error_reverse_flow"
CONF_ERROR_FLOW_RATE = "error_flow_rate"
CONF_ERROR_FREEZE_ALERT = "error_freeze_alert"
CONF_CONSECUTIVE_ERRORS_SENSOR = "consecutive_errors_sensor"
CONF_CONSECUTIVE_ERRORS_LIMIT = "consecutive_errors_limit"

qalcosonicnfc_ns = cg.esphome_ns.namespace("qalcosonicnfc")
QalcosonicNfc = qalcosonicnfc_ns.class_("QalcosonicNfc", cg.PollingComponent)

# taken from ESPHome's time component
# Home Assistant requires a correctly set timezone to parse a string as a datetime instead of a bunch of constantly changing strings.
# All this timezone code is necessary so users don't have to manually set a timezone (but they can), and continue using an existing configuration witout changes
# If all logic fail, it falls back to sending the time point as a string as with previous versions
def _load_tzdata(iana_key: str) -> Optional[bytes]:
    # From https://tzdata.readthedocs.io/en/latest/#examples
    try:
        package_loc, resource = iana_key.rsplit("/", 1)
    except ValueError:
        return None
    package = "tzdata.zoneinfo." + package_loc.replace("/", ".")

    try:
        return (resources.files(package) / resource).read_bytes()
    except (FileNotFoundError, ModuleNotFoundError):
        return None

def _extract_tz_string(tzfile: bytes) -> str:
    try:
        return tzfile.split(b"\n")[-2].decode()
    except (IndexError, UnicodeDecodeError):
        _LOGGER.error("Could not determine TZ string. Please report this issue.")
        _LOGGER.error("tzfile contents: %s", tzfile, exc_info=True)
        raise

def detect_tz() -> str:
    iana_key = tzlocal.get_localzone_name()
    if iana_key is None:
        raise cv.Invalid(
            "Could not automatically determine timezone, please set timezone manually."
        )
    _LOGGER.info("Detected timezone '%s'", iana_key)
    tzfile = _load_tzdata(iana_key)
    if tzfile is None:
        raise cv.Invalid(
            "Could not automatically determine timezone, please set timezone manually."
        )
    ret = _extract_tz_string(tzfile)
    _LOGGER.debug(" -> TZ string %s", ret)
    return ret

def validate_tz(value: str) -> str:
    value = cv.string_strict(value)

    tzfile = _load_tzdata(value)
    if tzfile is None:
        # Not a IANA key, probably a TZ string
        return value

    return _extract_tz_string(tzfile)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(QalcosonicNfc),
        cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.positive_time_period_milliseconds,
        cv.Required(CONF_PN5180_MOSI_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_PN5180_MISO_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_PN5180_SCK_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_PN5180_NSS_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_PN5180_BUSY_PIN): pins.gpio_input_pin_schema,
        cv.Required(CONF_PN5180_RST_PIN): pins.gpio_output_pin_schema,
        # Hier ist die wichtige Änderung: cv.maybe_simple_value stellt sicher,
        # dass die Sensoren korrekt mit IDs initialisiert werden.
        cv.Optional(CONF_WATER_USAGE_SENSOR): sensor.sensor_schema().extend(cv.Schema({cv.GenerateID(): cv.declare_id(sensor.Sensor)})),
        cv.Optional(CONF_WATER_USAGE_POSITIVE_SENSOR): sensor.sensor_schema().extend(cv.Schema({cv.GenerateID(): cv.declare_id(sensor.Sensor)})),
        cv.Optional(CONF_WATER_USAGE_NEGATIVE_SENSOR): sensor.sensor_schema().extend(cv.Schema({cv.GenerateID(): cv.declare_id(sensor.Sensor)})),
        cv.Optional(CONF_WATER_FLOW_SENSOR): sensor.sensor_schema().extend(cv.Schema({cv.GenerateID(): cv.declare_id(sensor.Sensor)})),
        cv.Optional(CONF_WATER_TEMPERATURE_SENSOR): sensor.sensor_schema().extend(cv.Schema({cv.GenerateID(): cv.declare_id(sensor.Sensor)})),
        cv.Optional(CONF_EXTERNAL_TEMPERATURE_SENSOR): sensor.sensor_schema().extend(cv.Schema({cv.GenerateID(): cv.declare_id(sensor.Sensor)})),
        cv.Optional(CONF_BATTERY_LEVEL_SENSOR): sensor.sensor_schema().extend(cv.Schema({cv.GenerateID(): cv.declare_id(sensor.Sensor)})),
        cv.Optional(CONF_CONSECUTIVE_ERRORS_SENSOR): sensor.sensor_schema().extend(cv.Schema({cv.GenerateID(): cv.declare_id(sensor.Sensor)})),
        cv.Optional(CONF_CONSECUTIVE_ERRORS_LIMIT, default=5): cv.uint8_t,
    }
).extend(cv.COMPONENT_SCHEMA)

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

    # Sicherheits-Check für jeden Sensor: Nur erstellen, wenn er in der Config existiert
    for conf_key, setter in [
        (CONF_WATER_USAGE_SENSOR, var.set_water_usage_sensor),
        (CONF_WATER_USAGE_POSITIVE_SENSOR, var.set_water_usage_positive_sensor),
        (CONF_WATER_USAGE_NEGATIVE_SENSOR, var.set_water_usage_negative_sensor),
        (CONF_WATER_FLOW_SENSOR, var.set_water_flow_sensor),
        (CONF_WATER_TEMPERATURE_SENSOR, var.set_water_temperature_sensor),
        (CONF_EXTERNAL_TEMPERATURE_SENSOR, var.set_external_temperature_sensor),
        (CONF_BATTERY_LEVEL_SENSOR, var.set_battery_level_sensor),
        (CONF_CONSECUTIVE_ERRORS_SENSOR, var.set_consecutive_errors_sensor),
    ]:
        if conf_key in config:
            s = await sensor.new_sensor(config[conf_key])
            cg.add(setter(s))

    cg.add(var.set_consecutive_errors_limit(config[CONF_CONSECUTIVE_ERRORS_LIMIT]))
    return var
