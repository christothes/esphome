import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import pn5180
from esphome.const import CONF_ID

CODEOWNERS = ["@christothes"]
AUTO_LOAD = ["pn5180"]
MULTI_CONF = True

CONF_pn5180_ISO15693_ID = "pn5180_ISO15693_id"
CONF_ON_FINISHED_WRITE = "on_finished_write"

pn5180_ISO15693_ns = cg.esphome_ns.namespace("pn5180_ISO15693")
PN5180_ISO15693 = pn5180_ISO15693_ns.class_("PN5180_ISO15693", cg.PollingComponent)

CONFIG_SCHEMA = cv.All(
    pn5180.PN5180_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(PN5180_ISO15693),
        }
    )
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await pn5180.setup_pn5180(var, config)
