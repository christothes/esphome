import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.components import nfc, spi
from esphome.const import (
    CONF_ID,
    CONF_ON_TAG_REMOVED,
    CONF_ON_TAG,
    CONF_TRIGGER_ID,
    CONF_RESET_PIN,
    CONF_BUSY_PIN,
)

CODEOWNERS = ["@christothes"]
AUTO_LOAD = ["binary_sensor", "nfc"]
DEPENDENCIES = ["spi"]
MULTI_CONF = True

CONF_PN5180_ID = "pn5180_id"
CONF_ON_FINISHED_WRITE = "on_finished_write"

pn5180_ns = cg.esphome_ns.namespace("pn5180")
PN5180 = pn5180_ns.class_("PN5180", cg.PollingComponent)

PN5180OnFinishedWriteTrigger = pn5180_ns.class_(
    "PN5180OnFinishedWriteTrigger", automation.Trigger.template()
)

PN5180IsWritingCondition = pn5180_ns.class_(
    "PN5180IsWritingCondition", automation.Condition
)

PN5180_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PN5180),
            cv.Optional(CONF_ON_TAG): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(nfc.NfcOnTagTrigger),
                }
            ),
            cv.Optional(CONF_ON_FINISHED_WRITE): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        PN5180OnFinishedWriteTrigger
                    ),
                }
            ),
            cv.Optional(CONF_ON_TAG_REMOVED): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(nfc.NfcOnTagTrigger),
                }
            ),
            cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_BUSY_PIN): pins.gpio_output_pin_schema,
        }
    ).extend(cv.polling_component_schema("1s"))
    # The cs_pin is another name for the nss pin
    .extend(
        spi.spi_device_schema(
            cs_pin_required=True,
            default_data_rate=spi.SPIDataRate.DATA_RATE_7MHZ,
            default_mode=spi.SPIMode.MODE_0,
        )
    )
)


async def setup_pn5180(var, config):
    await cg.register_component(var, config)

    for conf in config.get(CONF_ON_TAG, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_ontag_trigger(trigger))
        await automation.build_automation(
            trigger, [(cg.std_string, "x"), (nfc.NfcTag, "tag")], conf
        )

    for conf in config.get(CONF_ON_TAG_REMOVED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_ontagremoved_trigger(trigger))
        await automation.build_automation(
            trigger, [(cg.std_string, "x"), (nfc.NfcTag, "tag")], conf
        )

    for conf in config.get(CONF_ON_FINISHED_WRITE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)


@automation.register_condition(
    "pn5180.is_writing",
    PN5180IsWritingCondition,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(PN5180),
        }
    ),
)
async def pn5180_is_writing_to_code(config, condition_id, template_arg, args):
    var = cg.new_Pvariable(condition_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await setup_pn5180(var, config)
    await spi.register_spi_device(var, config)
