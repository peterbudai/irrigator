import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_ID
from esphome.core import CoroPriority, coroutine_with_priority

DEPENDENCIES = ["select"]
MULTI_CONF = True

status_select_ns = cg.esphome_ns.namespace("status_select")
StatusSelect = status_select_ns.class_("StatusSelect", cg.Component)

CONF_OUTPUT_ID = "output"
CONF_IDLE_OPTION = "idle_option"
CONF_WARNING_OPTION = "warning_option"
CONF_ERROR_OPTION = "error_option"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(StatusSelect),
        cv.Required(CONF_OUTPUT_ID): cv.use_id(select.Select),
        cv.Optional(CONF_IDLE_OPTION, default="idle"): cv.string,
        cv.Optional(CONF_WARNING_OPTION, default="warning"): cv.string,
        cv.Optional(CONF_ERROR_OPTION, default="error"): cv.string,
    }
).extend(cv.COMPONENT_SCHEMA)


@coroutine_with_priority(CoroPriority.STATUS)
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    sel = await cg.get_variable(config[CONF_OUTPUT_ID])
    cg.add(var.set_output(sel))
    cg.add(var.set_idle_option(config[CONF_IDLE_OPTION]))
    cg.add(var.set_warning_option(config[CONF_WARNING_OPTION]))
    cg.add(var.set_error_option(config[CONF_ERROR_OPTION]))
