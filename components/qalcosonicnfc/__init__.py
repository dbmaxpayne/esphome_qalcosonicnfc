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

MULTI_CONF = True

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


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(QalcosonicNfc),
            cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_WATER_USAGE_SENSOR, default={ CONF_NAME: "Water usage",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER,
                icon=ICON_WATER,
                accuracy_decimals=3,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                device_class=DEVICE_CLASS_WATER,),
            cv.Optional(CONF_WATER_USAGE_POSITIVE_SENSOR, default={ CONF_NAME: "Water usage (positive)",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER,
                icon=ICON_WATER,
                accuracy_decimals=3,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                device_class=DEVICE_CLASS_WATER,),
            cv.Optional(CONF_WATER_USAGE_NEGATIVE_SENSOR, default={ CONF_NAME: "Water usage (negative)",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER,
                icon=ICON_WATER,
                accuracy_decimals=3,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                device_class=DEVICE_CLASS_WATER,),
            cv.Optional(CONF_WATER_FLOW_SENSOR, default={ CONF_NAME: "Water flow",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER_PER_HOUR,
                icon=ICON_WATER,
                accuracy_decimals=3,
                state_class=STATE_CLASS_MEASUREMENT,
                device_class=DEVICE_CLASS_VOLUME_FLOW_RATE,),
            cv.Optional(CONF_WATER_TEMPERATURE_SENSOR, default={ CONF_NAME: "Water temperature",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
                device_class=DEVICE_CLASS_TEMPERATURE,),
            cv.Optional(CONF_EXTERNAL_TEMPERATURE_SENSOR, default={ CONF_NAME: "External temperature",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
                device_class=DEVICE_CLASS_TEMPERATURE,),
            cv.Optional(CONF_BATTERY_LEVEL_SENSOR, default={ CONF_NAME: "Battery level",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                icon=ICON_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
                device_class=DEVICE_CLASS_BATTERY,),
            cv.Optional(CONF_SERIAL_NUMBER_SENSOR, default={ CONF_NAME: "Serial number",}): text_sensor.text_sensor_schema(
                icon="mdi:numeric",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_METER_ID_SENSOR, default={ CONF_NAME: "Meter ID",}): text_sensor.text_sensor_schema(
                icon="mdi:numeric",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_MANUFACTURER_ID_SENSOR, default={ CONF_NAME: "Manufacturer ID",}): text_sensor.text_sensor_schema(
                icon="mdi:factory",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_FLAGS_RAW, default={ CONF_NAME: "Error flags raw",}): text_sensor.text_sensor_schema(
                icon="mdi:alert",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_OPERATING_TIME_SENSOR, default={ CONF_NAME: "Operating Time",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_SECOND,
                icon="mdi:clock",
                state_class=STATE_CLASS_TOTAL_INCREASING,
                device_class=DEVICE_CLASS_DURATION,),
            cv.Optional(CONF_ON_TIME_SENSOR, default={ CONF_NAME: "On Time",}): sensor.sensor_schema(
                unit_of_measurement=UNIT_SECOND,
                icon="mdi:clock",
                state_class=STATE_CLASS_TOTAL_INCREASING,
                device_class=DEVICE_CLASS_DURATION,),
            cv.Optional(CONF_METER_VERSION_SENSOR, default={ CONF_NAME: "Meter version",}): text_sensor.text_sensor_schema(
                icon="mdi:update",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_RAW_DATA_SENSOR, default={ CONF_NAME: "M-BUS raw data",}): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_TIMEPOINT_SENSOR, default={}): cv.Schema({
                cv.Optional(CONF_NAME, default="Time point"): cv.string,
                cv.Optional(CONF_TIMEZONE): validate_tz,
            }).extend(
                text_sensor.text_sensor_schema()
            ),
            cv.Optional(CONF_TIMEPOINT_SENSOR_RAW, default={ CONF_NAME: "Time point (raw)",}): text_sensor.text_sensor_schema(
                icon="mdi:clock",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_RECONFIGURATION_WARNING, default={ CONF_NAME: "Reconfiguration warning",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_NO_CONSUMPTION, default={ CONF_NAME: "No consumption",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_DAMAGE_METER_HOUSING, default={ CONF_NAME: "Damage of meter housing",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_CALCULATOR_HARDWARE_FAILURE, default={ CONF_NAME: "Calculator's hardware failure detected",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_LEAKAGE, default={ CONF_NAME: "Leakage",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_BURST, default={ CONF_NAME: "Pipe is cracked (Burst)",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_OPTICAL_COMMUNICATION, default={ CONF_NAME: "Optical communication temporarily stopped",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_LOW_BATTERY, default={ CONF_NAME: "Low battery, less than 12 months left",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_SOFTWARE_FAILURE, default={ CONF_NAME: "Software failure detected",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_HARDWARE_FAILURE, default={ CONF_NAME: "Hardware failure detected",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_NO_SIGNAL, default={ CONF_NAME: "No signal; the flow sensor is not filled with water",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_REVERSE_FLOW, default={ CONF_NAME: "Reverse flow",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_FLOW_RATE, default={ CONF_NAME: "Flow rate is greater than 1.25×Q₄",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_ERROR_FREEZE_ALERT, default={ CONF_NAME: "Freeze alert",}): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,),
            cv.Optional(CONF_CONSECUTIVE_ERRORS_SENSOR, default={ CONF_NAME: "Consecutive Errors",}): sensor.sensor_schema(
                icon="mdi:alert-circle",
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_CONSECUTIVE_ERRORS_LIMIT, default=5): cv.uint8_t,
            cv.Required(CONF_PN5180_MOSI_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_PN5180_MISO_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_PN5180_SCK_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_PN5180_NSS_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_PN5180_BUSY_PIN): pins.gpio_input_pin_schema,
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
    
    water_usage_positive_sensor = await sensor.new_sensor(config.get(CONF_WATER_USAGE_POSITIVE_SENSOR))
    cg.add(var.set_water_usage_positive_sensor(water_usage_positive_sensor))

    water_usage_negative_sensor = await sensor.new_sensor(config.get(CONF_WATER_USAGE_NEGATIVE_SENSOR))
    cg.add(var.set_water_usage_negative_sensor(water_usage_negative_sensor))

    water_flow_sensor = await sensor.new_sensor(config.get(CONF_WATER_FLOW_SENSOR))
    cg.add(var.set_water_flow_sensor(water_flow_sensor))
    
    water_temperature_sensor = await sensor.new_sensor(config.get(CONF_WATER_TEMPERATURE_SENSOR))
    cg.add(var.set_water_temperature_sensor(water_temperature_sensor))
    
    external_temperature_sensor = await sensor.new_sensor(config.get(CONF_EXTERNAL_TEMPERATURE_SENSOR))
    cg.add(var.set_external_temperature_sensor(external_temperature_sensor))

    battery_level_sensor = await sensor.new_sensor(config.get(CONF_BATTERY_LEVEL_SENSOR))
    cg.add(var.set_battery_level_sensor(battery_level_sensor))

    operating_time_sensor = await sensor.new_sensor(config.get(CONF_OPERATING_TIME_SENSOR))
    cg.add(var.set_operating_time_sensor(operating_time_sensor))

    on_time_sensor = await sensor.new_sensor(config.get(CONF_ON_TIME_SENSOR))
    cg.add(var.set_on_time_sensor(on_time_sensor))

    meter_version_sensor = await text_sensor.new_text_sensor(config.get(CONF_METER_VERSION_SENSOR))
    cg.add(var.set_meter_version_sensor(meter_version_sensor))

    conf_timepoint_sensor = config.get(CONF_TIMEPOINT_SENSOR)
    conf_timepoint_sensor = copy.deepcopy(conf_timepoint_sensor)
    tz = None
    if CONF_TIMEZONE in conf_timepoint_sensor:
        tz = conf_timepoint_sensor[CONF_TIMEZONE]
        _LOGGER.debug("Using configured timezone: %s", tz)
    else:
        try:
            tz = detect_tz()
            _LOGGER.debug("Auto-detected timezone: %s", tz)
        except Exception:
            _LOGGER.warning("Could not detect timezone, disabling timestamp device_class")
            tz = None
    if tz:
        conf_timepoint_sensor[CONF_DEVICE_CLASS] = DEVICE_CLASS_TIMESTAMP
    else:
        # Ensure it's not set
        conf_timepoint_sensor.pop(CONF_DEVICE_CLASS, None)
    timepoint_sensor = await text_sensor.new_text_sensor(conf_timepoint_sensor)
    cg.add(var.set_timepoint_sensor(timepoint_sensor))
    if tz:
        cg.add(var.set_timezone(tz))

    timepoint_sensor_raw = await text_sensor.new_text_sensor(config.get(CONF_TIMEPOINT_SENSOR_RAW))
    cg.add(var.set_timepoint_sensor_raw(timepoint_sensor_raw))

    raw_data_sensor = await text_sensor.new_text_sensor(config.get(CONF_RAW_DATA_SENSOR))
    cg.add(var.set_raw_data_sensor(raw_data_sensor))

    serial_number_sensor = await text_sensor.new_text_sensor(config.get(CONF_SERIAL_NUMBER_SENSOR))
    cg.add(var.set_serial_number_sensor(serial_number_sensor))

    meter_id_sensor = await text_sensor.new_text_sensor(config.get(CONF_METER_ID_SENSOR))
    cg.add(var.set_meter_id_sensor(meter_id_sensor))

    manufacturer_id_sensor = await text_sensor.new_text_sensor(config.get(CONF_MANUFACTURER_ID_SENSOR))
    cg.add(var.set_manufacturer_id_sensor(manufacturer_id_sensor))

    error_flags_raw = await text_sensor.new_text_sensor(config.get(CONF_ERROR_FLAGS_RAW))
    cg.add(var.set_error_flags_raw(error_flags_raw))

    error_reconfiguration_warning = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_RECONFIGURATION_WARNING))
    cg.add(var.set_error_reconfiguration_warning(error_reconfiguration_warning))

    error_no_consumption = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_NO_CONSUMPTION))
    cg.add(var.set_error_no_consumption(error_no_consumption))

    error_damage_meter_housing = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_DAMAGE_METER_HOUSING))
    cg.add(var.set_error_damage_meter_housing(error_damage_meter_housing))

    error_calculator_hardware_failure = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_CALCULATOR_HARDWARE_FAILURE))
    cg.add(var.set_error_calculator_hardware_failure(error_calculator_hardware_failure))

    error_leakage = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_LEAKAGE))
    cg.add(var.set_error_leakage(error_leakage))

    error_burst = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_BURST))
    cg.add(var.set_error_burst(error_burst))

    error_optical_communication = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_OPTICAL_COMMUNICATION))
    cg.add(var.set_error_optical_communication(error_optical_communication))

    error_low_battery = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_LOW_BATTERY))
    cg.add(var.set_error_low_battery(error_low_battery))

    error_software_failure = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_SOFTWARE_FAILURE))
    cg.add(var.set_error_software_failure(error_software_failure))

    error_hardware_failure = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_HARDWARE_FAILURE))
    cg.add(var.set_error_hardware_failure(error_hardware_failure))

    error_no_signal = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_NO_SIGNAL))
    cg.add(var.set_error_no_signal(error_no_signal))

    error_reverse_flow = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_REVERSE_FLOW))
    cg.add(var.set_error_reverse_flow(error_reverse_flow))

    error_flow_rate = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_FLOW_RATE))
    cg.add(var.set_error_flow_rate(error_flow_rate))

    error_freeze_alert = await binary_sensor.new_binary_sensor(config.get(CONF_ERROR_FREEZE_ALERT))
    cg.add(var.set_error_freeze_alert(error_freeze_alert))

    consecutive_errors_sensor = await sensor.new_sensor(config.get(CONF_CONSECUTIVE_ERRORS_SENSOR))
    cg.add(var.set_consecutive_errors_sensor(consecutive_errors_sensor))

    cg.add(var.set_consecutive_errors_limit(config[CONF_CONSECUTIVE_ERRORS_LIMIT]))