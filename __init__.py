# =============================================================================
# ESPHome codegen for dmo_label_emulator.
#
# Only job: read the YAML options below and turn them into calls that build
# and configure a DmoLabelEmulator C++ object. No select-entity logic lives
# here anymore - the dropdown is a plain ESPHome `template` select defined
# directly in the main YAML file, which calls set_sku() via a lambda.
# =============================================================================
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID

dmo_label_emulator_ns = cg.esphome_ns.namespace("dmo_label_emulator")
DmoLabelEmulator = dmo_label_emulator_ns.class_("DmoLabelEmulator", cg.Component)

CONF_SLAVE_SDA_PIN = "slave_sda_pin"
CONF_SLAVE_SCL_PIN = "slave_scl_pin"
CONF_SLAVE_ADDRESS = "slave_address"
CONF_IRQ_PIN = "irq_pin"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DmoLabelEmulator),
        cv.Required(CONF_SLAVE_SDA_PIN): pins.internal_gpio_pin_number({"output": True, "input": True}),
        cv.Required(CONF_SLAVE_SCL_PIN): pins.internal_gpio_pin_number({"output": True, "input": True}),
        cv.Optional(CONF_SLAVE_ADDRESS, default=0x28): cv.hex_uint8_t,
        cv.Required(CONF_IRQ_PIN): pins.internal_gpio_pin_number({"output": True}),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_slave_pins(config[CONF_SLAVE_SDA_PIN], config[CONF_SLAVE_SCL_PIN]))
    cg.add(var.set_slave_address(config[CONF_SLAVE_ADDRESS]))
    cg.add(var.set_irq_pin(config[CONF_IRQ_PIN]))
