import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID
from esphome.core import coroutine_with_priority, CORE

CODEOWNERS = ["@greays"]
DEPENDENCIES = ["network"]


def AUTO_LOAD():
    if CORE.using_arduino:
        return ["async_tcp", "json"]
    if CORE.using_esp_idf:
        return ["web_server_idf", "json"]
    return []


st_edge_connector = cg.esphome_ns.namespace("st_edge_connector")
StEdgeConnector = st_edge_connector.class_("StEdgeConnector", cg.Component)

CONF_ST_EDGE_CONNECTOR_ID = "st_edge_connector_id"
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(StEdgeConnector),
    }
)


@coroutine_with_priority(-200.0)
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    if CORE.using_arduino:
        if CORE.is_esp32:
            cg.add_library("WiFi", None)
        # https://github.com/esphome/ESPAsyncWebServer/blob/master/library.json
        cg.add_library("esphome/ESPAsyncWebServer-esphome", "3.1.0")

    if CORE.is_esp32:
        # cg.add_library("WiFiClientSecure", None)
        cg.add_library("HTTPClient", None)
    if CORE.is_esp8266:
        cg.add_library("ESP8266HTTPClient", None)

    await cg.register_component(var, config)
