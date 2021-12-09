/**
 ****************************************************************************************
 *
 * @file wlan_coex.c
 *
 * @brief WLAN coexistence API source file.
 *
 * Copyright (C) 2019-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#if (WLAN_COEX_ENABLED)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "da1458x_scatter_config.h" // for BLE_CONNECTION_MAX_USER
#include "gpio.h"
#include "wlan_coex.h"
#include "datasheet.h"
#include "lld.h"
#include "reg_ble_em_tx_desc.h"
#include "reg_ble_em_cs.h"

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

static void check_signals(void);
static void modify_ble_prio(void);
static void ext_24g_eip_handler(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

static uint32_t wlan_coex_criteria[BLE_CONNECTION_MAX_USER + 1]   __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
static uint32_t rx_pkt_cnt_bad_previous                           __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern uint32_t rx_pkt_cnt_bad;
extern const wlan_coex_cfg_t wlan_coex_cfg;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void wlan_coex_BLE_set(void)
{
    uint8_t shift;

    if (wlan_coex_cfg.ble_eip_pin < GPIO_PIN_4)
    {
        shift = wlan_coex_cfg.ble_eip_pin * 8;
        // Enable BLE event in progress
        SetBits32(BLE_DIAGCNTL_REG, DIAG0 << shift, 0x1F);
        SetBits32(BLE_DIAGCNTL_REG, DIAG0_EN << shift, 1);
    }
    else
    {
        shift = (wlan_coex_cfg.ble_eip_pin - 4) * 8;
        // Enable BLE event in progress
        SetBits32(BLE_DIAGCNTL2_REG, DIAG4 << shift, 0x1F);
        SetBits32(BLE_DIAGCNTL2_REG, DIAG4_EN << shift, 1);
    }

    // Configure selected GPIO as BLE diagnostic
    SetWord16(GPIO_BASE + 0x6 + (wlan_coex_cfg.ble_eip_pin << 1), 0x312);

#if defined (CFG_WLAN_COEX_BLE_EVENT_INV)
    SetBits32(BLE_DIAGCNTL3_REG, (DIAG0_INV << (wlan_coex_cfg.ble_eip_pin * 4)), 1);
#endif
    SetBits32(BLE_DIAGCNTL3_REG, (DIAG0_BIT << (wlan_coex_cfg.ble_eip_pin * 4)), 7);
}

void wlan_coex_init(void)
{
    memset(wlan_coex_criteria, 0, sizeof(wlan_coex_criteria));

    // Enable BLE internal signals
    wlan_coex_BLE_set();

    check_signals();
}

void wlan_coex_prio_criteria_add(wlan_coex_ble_prio_t priority, uint16_t conhdl, uint16_t missed)
{
    // If conhdl is higher than supported connections
    if (conhdl > BLE_CONNECTION_MAX_USER)
    {
        // Assume it is general criteria
        conhdl = BLE_CONNECTION_MAX_USER;
    }

    wlan_coex_criteria[conhdl] |= priority;

    if (priority == WLAN_COEX_BLE_PRIO_MISSED)
    {
        wlan_coex_criteria[conhdl] |= missed << 16;
    }
}

void wlan_coex_prio_criteria_del(wlan_coex_ble_prio_t priority, uint16_t conhdl, uint16_t missed)
{
    // If conhdl is higher than supported connections
    if (conhdl > BLE_CONNECTION_MAX_USER)
    {
         // Assume it is general criteria
        conhdl = BLE_CONNECTION_MAX_USER;
    }

    wlan_coex_criteria[conhdl] &= ~priority;

    if (priority == WLAN_COEX_BLE_PRIO_MISSED)
    {
        wlan_coex_criteria[conhdl] &= 0x00FF;
    }
}

void wlan_coex_gpio_cfg(void)
{
    // WLAN_COEX_BLE_EVENT must be between 0 and 7 (port P0)
    ASSERT_WARNING((wlan_coex_cfg.ble_eip_pin <= GPIO_PIN_7) && (wlan_coex_cfg.ble_eip_port == GPIO_PORT_0));

    // Drive to inactive state the pin used for the BLE event in progress signal
    wlan_coex_set_ble_eip_pin_inactive();

    // Configure selected GPIO as output - BLE priority
    GPIO_ConfigurePin(wlan_coex_cfg.ble_prio_port, wlan_coex_cfg.ble_prio_pin, OUTPUT, PID_GPIO, false);

    // Configure selected GPIO as input with pull-down - 2.4GHz external device event in progress
    GPIO_ConfigurePin(wlan_coex_cfg.ext_24g_eip_port, wlan_coex_cfg.ext_24g_eip_pin, INPUT_PULLDOWN, PID_GPIO, false);

    // Configure 2.4GHz external device event in progress signal to act as a GPIO interrupt
    GPIO_RegisterCallback((IRQn_Type)(GPIO0_IRQn + wlan_coex_cfg.irq), ext_24g_eip_handler);
    GPIO_EnableIRQ(wlan_coex_cfg.ext_24g_eip_port, wlan_coex_cfg.ext_24g_eip_pin, (IRQn_Type)(GPIO0_IRQn + wlan_coex_cfg.irq),
                   /*low_input*/ false, /*release_wait*/ false, /*debounce_ms*/ 0);

#if defined (CFG_WLAN_COEX_DEBUG)
    GPIO_ConfigurePin(wlan_coex_cfg.debug_a_port, wlan_coex_cfg.debug_a_pin, OUTPUT, PID_GPIO, false);
    GPIO_ConfigurePin(wlan_coex_cfg.debug_b_port, wlan_coex_cfg.debug_b_pin, OUTPUT, PID_GPIO, false);
#endif
}

void wlan_coex_set_ble_eip_pin_inactive(void)
{
#if defined (CFG_WLAN_COEX_BLE_EVENT_INV)
    GPIO_ConfigurePin(wlan_coex_cfg.ble_eip_port, wlan_coex_cfg.ble_eip_pin, OUTPUT, PID_GPIO, true);
#else
    GPIO_ConfigurePin(wlan_coex_cfg.ble_eip_port, wlan_coex_cfg.ble_eip_pin, OUTPUT, PID_GPIO, false);
#endif
}

void wlan_coex_going_to_sleep(void)
{
#if defined (CFG_WLAN_COEX_DEBUG)
    GPIO_SetInactive(wlan_coex_cfg.debug_a_port, wlan_coex_cfg.debug_a_pin);
#endif
}

/**
 ****************************************************************************************
 * @brief GPIO IRQ handler for WLAN event in progress
 *
 * @note In case it fires it indicates that an external WLAN device wants to send data
 ****************************************************************************************
 */
static void ext_24g_eip_handler(void)
{
    IRQn_Type irq = (IRQn_Type) (GPIO0_IRQn + wlan_coex_cfg.irq);

#if defined (CFG_WLAN_COEX_DEBUG)
    GPIO_SetActive(wlan_coex_cfg.debug_b_port, wlan_coex_cfg.debug_b_pin);
#endif

    // Disable this interrupt
    NVIC_DisableIRQ(irq);

    // Check if we can overrule BLE activity
    check_signals();

    // Check current polarity and change accordingly
    if (GPIO_IRQ_INPUT_LEVEL_HIGH == GPIO_GetIRQInputLevel(irq))
    {
        // Change polarity to get an interrupt when WLAN event_in_process falls
        GPIO_SetIRQInputLevel(irq, GPIO_IRQ_INPUT_LEVEL_LOW);
    }
    else
    {
        // Change polarity to get an interrupt when WLAN event_in_process rises
        GPIO_SetIRQInputLevel(irq, GPIO_IRQ_INPUT_LEVEL_HIGH);
    }

    // Clear interrupt requests while disabled
    SetBits16(GPIO_RESET_IRQ_REG, RESET_GPIO0_IRQ << wlan_coex_cfg.irq, 1);

    // Clear interrupt requests while disabled
    NVIC_ClearPendingIRQ(irq);

    // Enable this interrupt
    NVIC_EnableIRQ(irq);

#if defined (CFG_WLAN_COEX_DEBUG)
    GPIO_SetInactive(wlan_coex_cfg.debug_b_port, wlan_coex_cfg.debug_b_pin);
#endif
}

void wlan_coex_finetimer_isr(void)
{
    // Handle WLAN coex priority
    modify_ble_prio();
}

void wlan_coex_prio_level(int level)
{
    if (level)
    {
        GPIO_SetActive(wlan_coex_cfg.ble_prio_port, wlan_coex_cfg.ble_prio_pin);
    }
    else
    {
        GPIO_SetInactive(wlan_coex_cfg.ble_prio_port, wlan_coex_cfg.ble_prio_pin);
    }

    check_signals();
}


/**
 ****************************************************************************************
 * @brief Check BLE priority over WLAN and suppress Rx/Tx if needed
 ****************************************************************************************
 */
static void check_signals(void)
{
    uint32_t radio_on_off = 0;

    GLOBAL_INT_STOP();

    // Check BLE_PRIORITY current level
    if (GetBits16((P0_DATA_REG + (wlan_coex_cfg.ble_prio_port * 0x20)), 1 << wlan_coex_cfg.ble_prio_pin) == 0)
    {
        // If not BLE_PRIORITY then check 2.4GHz external device event in progress pin level
        if (GetBits16((P0_DATA_REG + (wlan_coex_cfg.ext_24g_eip_port * 0x20)), 1 << wlan_coex_cfg.ext_24g_eip_pin))
        {
            // Turn off radio
            radio_on_off = 1;
        }
        else
        {
            // Turn on radio
            radio_on_off = 0;
        }
    }

#if defined (CFG_WLAN_COEX_DEBUG)
    if (radio_on_off == 0)
    {
        // Indicate that we are ok
        GPIO_SetInactive(wlan_coex_cfg.debug_a_port, wlan_coex_cfg.debug_a_pin);
    }
    else
    {
        // Indicate that we are overruled
        GPIO_SetActive(wlan_coex_cfg.debug_a_port, wlan_coex_cfg.debug_a_pin);
    }
#endif

    // Turn the radio off (or do not overrule it) depending on the above checks

    SetBits16(RF_OVERRULE_REG, (0x0004), radio_on_off);

    SetBits16(RF_OVERRULE_REG, (0x0001), radio_on_off);

    GLOBAL_INT_START();
}

/**
 ****************************************************************************************
 * @brief Check BLE priority over WLAN based on the first BLE packet of the next event
 ****************************************************************************************
 */
static int check_ble_prio(struct ea_elt_tag *elt)
{
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    uint16_t conhdl = evt->conhdl;

    // If conhdl is higher than supported connections
    if (conhdl > BLE_CONNECTION_MAX_USER)
    {
        // Assume it is general criteria
        conhdl = BLE_CONNECTION_MAX_USER;
    }

    uint32_t prio = wlan_coex_criteria[conhdl];

    if (evt->conhdl == LLD_ADV_HDL)
    {
        uint32_t chk = WLAN_COEX_BLE_PRIO_SCAN;
        while (chk <= WLAN_COEX_BLE_PRIO_CONREQ)
        {
            if (prio & chk)
            {
                switch (chk)
                {
                    // Scan priority
                    case WLAN_COEX_BLE_PRIO_SCAN:
                    {
                        if (evt->mode == LLD_EVT_SCAN_MODE) //event is scan
                        {
                            return 1;
                        }
                        break;
                    }
                    // Advertise priority
                    case WLAN_COEX_BLE_PRIO_ADV:
                    {
                        if (evt->mode == LLD_EVT_ADV_MODE) //event is advertise
                        {
                            return 1;
                        }
                        break;
                    }
                    // Connection request priority
                    case WLAN_COEX_BLE_PRIO_CONREQ:
                    {
                        if (evt->mode == LLD_EVT_SCAN_MODE) //event is scan
                        {
                            if (ble_cntl_get(LLD_ADV_HDL) == LLD_INITIATING) //connection request
                                return 1;
                        }
                        break;
                    }
                }
            }
                chk = chk << 1;
        }
    }
    else    //per connection criteria
    {
        uint32_t chk = WLAN_COEX_BLE_PRIO_LLCP;
        while (chk <= WLAN_COEX_BLE_PRIO_MISSED)
        {
            if (prio & chk)
            {
                switch (chk)
                {
                    case WLAN_COEX_BLE_PRIO_LLCP:
                    {
                        struct co_list *rdy = &evt->tx_rdy;
                        struct co_list *prog = &evt->tx_prog;

                        //check criteria
                        if (!co_list_is_empty(rdy))
                        {
                            if (ble_txllid_getf(((struct co_buf_tx_node *)rdy->first)->idx) == BLE_TXLLID_MASK)
                            {
                                return 1;
                            }
                        }

                        if (!co_list_is_empty(prog))
                        {
                            if (ble_txllid_getf(((struct co_buf_tx_node *)prog->first)->idx) == BLE_TXLLID_MASK)
                            {
                                return 1;
                            }
                        }
                        break;
                    }
                    case WLAN_COEX_BLE_PRIO_DATA:
                    {
                        return 1;
                    }
                    case WLAN_COEX_BLE_PRIO_MISSED:
                    {
                        if (rx_pkt_cnt_bad > rx_pkt_cnt_bad_previous)
                        {
                            if (rx_pkt_cnt_bad - rx_pkt_cnt_bad_previous >= (prio >> 16))
                            {
                                rx_pkt_cnt_bad_previous = rx_pkt_cnt_bad;
                                return 1;
                            }
                        }
                        else
                        {
                            if (0xFFFFFFFF - rx_pkt_cnt_bad_previous + 1 + rx_pkt_cnt_bad >= (prio >> 16))
                            {
                                rx_pkt_cnt_bad_previous = rx_pkt_cnt_bad;
                                return 1;
                            }
                        }
                        break;
                    }
                }
            }
            chk = chk << 1;
        }
    }
    return 0;
}

/**
 ****************************************************************************************
 * @brief Modify BLE priority over WLAN depending on the type of the next BLE packet
 ****************************************************************************************
 */
static void modify_ble_prio(void)
{
    struct ea_elt_tag *elt = (struct ea_elt_tag *)co_list_pick(&lld_evt_env.elt_prog);

    // Check if element was programmed
    if (elt == NULL)
    {
        wlan_coex_prio_level(0);
        return;
    }

    // Compare element to predefined element types
    if (check_ble_prio(elt))
    {
        wlan_coex_prio_level(1);
    }
    else
    {
        wlan_coex_prio_level(0);
    }
}

#endif
