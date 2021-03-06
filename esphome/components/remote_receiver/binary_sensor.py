import esphome.codegen as cg
from esphome.components import binary_sensor, remote_base
from esphome.const import CONF_NAME

DEPENDENCIES = ["remote_receiver"]

CONFIG_SCHEMA = remote_base.validate_binary_sensor


def to_code(config):
    var = yield remote_base.build_binary_sensor(config)
    cg.add(var.set_name(config[CONF_NAME]))
    yield binary_sensor.register_binary_sensor(var, config)
