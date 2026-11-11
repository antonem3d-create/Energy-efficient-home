#include <stm8s.h>

// ======================= ПИНЫ =======================
#define SEG_C_MASK   ((1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7))
#define SEG_F_PIN    (1<<2)
#define SEG_G_PIN    (1<<1)
#define SEG_D_MASK   (SEG_F_PIN | SEG_G_PIN)

#define DIGIT_LEFT   (1<<4)
#define DIGIT_RIGHT  (1<<5)
#define DIGIT_MASK   (DIGIT_LEFT | DIGIT_RIGHT)

#define BUZZER_PIN   (1<<4)   // PD4 (TIM2_CH1)
#define MOC_PIN      (1<<3)   // PA3
#define BTN_MINUS    (1<<1)   // PA1
#define BTN_PLUS     (1<<2)   // PA2
#define TR_SENSE_PIN (1<<3)   // PD3

// ======================= ПАРАМЕТРЫ =======================
#define MUX_PERIOD_MS             15
#define BLINK_PERIOD_MS           500
#define BEEP_BLINK_PERIOD_MS      80
#define INIT_DISPLAY_MS           5000UL
#define RECOVERY_MS               5000UL
#define PACKET_TIMEOUT_MS         10000UL
#define UART_BAUD                 9600
#define UBRR_VAL                  (16000000UL / UART_BAUD)
#define SERVICE_HOLD_MS           2000
#define BUTTONS_BLOCK_MS          300
#define WINDOW_MS                 100000UL
#define SLAVE_RESPONSE_DELAY_MS   40
#define ADC_STEPS                 16
#define ADC_STEP_MS               2
#define DEBOUNCE_MS               2000
#define EE_BASE                   0x4000
#define EE_MODE_OFF               450
#define EE_ID_OFF                 451
#define EE_DESIRED_OFF            452
#define NH_IGNORE_MS              3000UL   // карантин НН после входа в WORK

// ======================= ADC / VREFINT =======================
#define ADC_VDD_LOW_ADC           418
#define ADC_VDD_OK_ADC            398

// ======================= ЗУММЕР =======================
#define BUZZER_FREQ_HZ   2450
#define BUZZER_ARR       ((16000000UL / BUZZER_FREQ_HZ) - 1)
#define BUZZER_CCR       (BUZZER_ARR / 2)

// ======================= ДЕТЕКТОР СИМИСТОРА (ПР) =======================
#define TR_CHECK_DELAY_MS     100
#define TR_CHECK_PERIOD_MS    100
#define TR_CONFIRM_COUNT      3

// ======================= FLASH / RST =======================
#define F_DUKR    (*(volatile uint8_t*)0x5064)
#define F_IAPSR   (*(volatile uint8_t*)0x505F)
#define RST_SR    (*(volatile uint8_t*)0x50B3)

// ======================= СИМВОЛЫ =======================
#define CHAR_BLANK  0x00
#define CHAR_N      0x76
#define CHAR_S      0x39
#define CHAR_B      0x7D
#define CHAR_E      0x79
#define CHAR_O      0x3F
#define CHAR_P      0x37
#define CHAR_R      0x73
#define CHAR_V      0x7F

const uint8_t seg_code[10] = {
    0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110,
    0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111
};

// ======================= СОСТОЯНИЕ =======================
typedef enum {
    ST_SERVICE_SE,
    ST_SERVICE_MENU,
    ST_SERVICE_DE_CONFIRMED,
    ST_SERVICE_ID_SELECT,
    ST_INIT_DE,
    ST_INIT_NE,
    ST_RECOVERY_NE,
    ST_SOFT_CN,
    ST_SOFT_NE,
    ST_SOFT_UL,
    ST_SOFT_TR,
    ST_WORK_DE,
    ST_WORK_NE,
    ST_ERROR_HARD
} state_t;

state_t state = ST_SERVICE_SE;
state_t prev_state_ul = ST_WORK_DE;

volatile uint8_t display_buffer[2] = {0, 0};
uint8_t blink_hidden = 0;
uint16_t blink_last = 0;
uint32_t init_display_start = 0;
uint16_t recovery_start = 0;

uint8_t selected_mode = 0xDE;
uint8_t selected_id = 0;

uint8_t setpoint = 0;
uint32_t window_start = 0;
uint8_t relay_on = 0;
uint16_t adc_last = 0;

uint8_t my_id = 0;
uint8_t desired = 0;
uint8_t actual = 0;
uint8_t last_sent_desired = 0;
uint16_t desired_last_change_ms = 0;
uint32_t pos_current = 0;
uint32_t ne_window_start = 0;
uint32_t last_packet_ms = 0;
uint32_t show_only_desired_until = 0;
uint8_t ne_display_state = 0;
uint16_t ne_display_last = 0;

uint32_t slot_start = 0;
uint32_t slot_duration = 0;
uint8_t sync_received = 0;

uint32_t beep_end_ms = 0;

uint32_t adc_sum = 0;
uint8_t adc_count = 0;
uint8_t adc_active = 0;
uint16_t adc_step_start = 0;

char parse_buf[24];
uint8_t parse_idx = 0;
uint8_t parse_active = 0;

static uint16_t g_last_mux = 0;
static uint8_t g_current_digit = 0;
static uint16_t last_btn_poll = 0;
static uint16_t last_adc_poll = 0;
static uint32_t buttons_block_until = 0;
static uint32_t nh_ignore_until = 0;

static uint8_t btn_minus_stable = 1, btn_minus_cnt = 0;
static uint8_t btn_plus_stable  = 1, btn_plus_cnt  = 0;

static uint8_t both_stable = 0;
static uint16_t both_start_ms = 0;
static uint8_t both_triggered = 0;

static uint16_t beep_blink_last = 0;
static uint8_t  beep_blink_phase = 0;

// Детектор ПР
static uint8_t  tr_mismatch_cnt = 0;
static uint8_t  tr_match_cnt = 0;
static uint32_t tr_last_check_ms = 0;
static uint32_t tr_relay_changed_ms = 0;
static uint8_t  tr_prev_relay = 0;
static uint8_t  tr_fault = 0;

// ======================= ПРОТОТИПЫ =======================
void service_enter(void);
void enter_service_menu(void);
void enter_service_de_confirmed(void);
void enter_service_id_select(void);
void service_exit(void);
void enter_init_de(void);
void enter_init_ne(void);
void enter_recovery_ne(void);
void enter_soft(state_t s, uint8_t l, uint8_t r);
void enter_soft_tr(void);
void enter_work_de(void);
void enter_work_ne(void);
void enter_error_hard(uint8_t l, uint8_t r);
void show_string(uint8_t l, uint8_t r);
void show_id(uint8_t id);
void show_setpoint(uint8_t sp);
void block_buttons(uint32_t ms);
uint8_t buttons_blocked(void);
uint8_t is_service_state(state_t s);
void uart_send_packet(uint8_t id, uint8_t value);
void parse_packet(void);
void adc_start(void);
void adc_poll_step(void);
void init_tim2_beep(void);
void play_beep(uint32_t ms);
void tick_beep(void);
void blink_beep_tick(void);
void reset_buttons(void);
uint8_t debounce_minus(uint8_t rm);
uint8_t debounce_plus(uint8_t rp);
uint8_t dec_val(uint8_t v);
uint8_t inc_val(uint8_t v);
void handle_buttons_work(void);
void check_triac(void);
void init_tr_state(void);

// ======================= IWDG =======================
static inline void iwdg_feed(void) { IWDG->KR = 0xAA; }

void iwdg_init(void) {
    IWDG->KR  = 0xCC;
    IWDG->KR  = 0x55;
    IWDG->PR  = 0x06;
    IWDG->RLR = 0xFF;
    IWDG->KR  = 0xAA;
}

// ======================= ПИНЫ =======================
void init_pins(void) {
    GPIOC->DDR |= SEG_C_MASK;
    GPIOC->CR1 |= SEG_C_MASK;
    GPIOC->CR2 &= ~SEG_C_MASK;
    GPIOC->ODR &= ~SEG_C_MASK;

    GPIOD->DDR |= SEG_D_MASK;
    GPIOD->CR1 |= SEG_D_MASK;
    GPIOD->CR2 &= ~SEG_D_MASK;
    GPIOD->ODR &= ~SEG_D_MASK;

    GPIOB->DDR |= DIGIT_MASK;
    GPIOB->ODR |= DIGIT_MASK;

    GPIOD->DDR |= BUZZER_PIN;
    GPIOD->CR1 |= BUZZER_PIN;
    GPIOD->CR2 |= BUZZER_PIN;
    GPIOD->ODR &= ~BUZZER_PIN;

    GPIOA->DDR |= MOC_PIN;
    GPIOA->CR1 |= MOC_PIN;
    GPIOA->CR2 &= ~MOC_PIN;
    GPIOA->ODR &= ~MOC_PIN;

    GPIOA->DDR &= ~(BTN_MINUS | BTN_PLUS);
    GPIOA->CR1 |= (BTN_MINUS | BTN_PLUS);
    GPIOA->CR2 &= ~(BTN_MINUS | BTN_PLUS);

    // PD3: делаем "плавающим" (floating) как в КОД1, чтобы избежать конфликта с TIM2_CH2
    GPIOD->DDR &= ~TR_SENSE_PIN;
    GPIOD->CR1 &= ~TR_SENSE_PIN;
    GPIOD->CR2 &= ~TR_SENSE_PIN;
}

// ======================= ДИСПЛЕЙ =======================
void show_string(uint8_t l, uint8_t r) {
    display_buffer[0] = l;
    display_buffer[1] = r;
}

void show_id(uint8_t id) {
    if (id > 9) id = 0;
    display_buffer[0] = seg_code[0];
    display_buffer[1] = seg_code[id];
}

void show_setpoint(uint8_t sp) {
    if (sp == 0) {
        display_buffer[0] = CHAR_BLANK;
        display_buffer[1] = seg_code[0];
    } else if (sp < 10) {
        display_buffer[0] = CHAR_BLANK;
        display_buffer[1] = seg_code[sp];
    } else {
        display_buffer[0] = seg_code[sp / 10];
        display_buffer[1] = seg_code[sp % 10];
    }
}

void draw_digit(uint8_t digit) {
    GPIOB->ODR |= DIGIT_MASK;
    GPIOC->ODR &= ~SEG_C_MASK;
    GPIOD->ODR &= ~SEG_D_MASK;

    uint8_t code = display_buffer[digit];
    if (code & (1<<0)) GPIOC->ODR |= (1<<3);
    if (code & (1<<1)) GPIOC->ODR |= (1<<4);
    if (code & (1<<2)) GPIOC->ODR |= (1<<5);
    if (code & (1<<3)) GPIOC->ODR |= (1<<6);
    if (code & (1<<4)) GPIOC->ODR |= (1<<7);
    if (code & (1<<5)) GPIOD->ODR |= SEG_F_PIN;
    if (code & (1<<6)) GPIOD->ODR |= SEG_G_PIN;

    if (digit == 0) GPIOB->ODR &= ~DIGIT_LEFT;
    else            GPIOB->ODR &= ~DIGIT_RIGHT;
}

void mux_tick(void) {
    iwdg_feed();
    tick_beep();
    uint16_t now = (uint16_t)millis();
    if ((uint16_t)(now - g_last_mux) < MUX_PERIOD_MS) return;
    g_last_mux = now;

    if (blink_hidden) {
        GPIOB->ODR |= DIGIT_MASK;
        GPIOC->ODR &= ~SEG_C_MASK;
        GPIOD->ODR &= ~SEG_D_MASK;
    } else {
        draw_digit(g_current_digit);
    }
    g_current_digit ^= 1;
}

void pause_ms(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) mux_tick();
}

// ======================= ЗУММЕР =======================
void init_tim2_beep(void) {
    CLK->PCKENR1 |= (1 << 5);
    TIM2->CR1 &= ~(1 << 0);
    TIM2->IER = 0;
    TIM2->PSCR = 0x00;
    TIM2->ARRH = (uint8_t)(BUZZER_ARR >> 8);
    TIM2->ARRL = (uint8_t)(BUZZER_ARR & 0xFF);
    TIM2->CCR1H = (uint8_t)(BUZZER_CCR >> 8);
    TIM2->CCR1L = (uint8_t)(BUZZER_CCR & 0xFF);
    TIM2->CCMR1 = 0x68;
    TIM2->CCER1 = 0x00;
    TIM2->SR1 = 0;
    TIM2->CR1 = 0x81;
    beep_end_ms = 0;
}

void play_beep(uint32_t ms) {
    if (ms == 0) return;
    TIM2->CNTRH = 0;
    TIM2->CNTRL = 0;
    TIM2->CCER1 |= 0x01;
    beep_end_ms = millis() + ms;
}

void tick_beep(void) {
    if (beep_end_ms == 0) return;
    if ((int32_t)(millis() - beep_end_ms) >= 0) {
        TIM2->CCER1 &= ~0x01;
        GPIOD->ODR &= ~BUZZER_PIN;
        beep_end_ms = 0;
    }
}

void blink_beep_tick(void) {
    uint16_t now = (uint16_t)millis();
    if ((uint16_t)(now - beep_blink_last) >= BEEP_BLINK_PERIOD_MS) {
        beep_blink_last = now;
        beep_blink_phase ^= 1;
        if (beep_blink_phase) play_beep(BEEP_BLINK_PERIOD_MS);
    }
}

// ======================= БЛОКИРОВКА / КНОПКИ =======================
void block_buttons(uint32_t ms) { buttons_block_until = millis() + ms; }
uint8_t buttons_blocked(void) { return millis() < buttons_block_until; }

uint8_t is_service_state(state_t s) {
    return (s == ST_SERVICE_SE || s == ST_SERVICE_MENU ||
            s == ST_SERVICE_DE_CONFIRMED || s == ST_SERVICE_ID_SELECT);
}

void reset_buttons(void) {
    btn_minus_stable = 1; btn_minus_cnt = 0;
    btn_plus_stable  = 1; btn_plus_cnt  = 0;
    both_stable    = 0;
    both_triggered = 0;
    both_start_ms  = 0;
}

uint8_t debounce_minus(uint8_t rm) {
    if (rm != btn_minus_stable) {
        if (++btn_minus_cnt >= 2) {
            btn_minus_stable = rm; btn_minus_cnt = 0;
            return (rm == 0) ? 1 : 0;
        }
    } else btn_minus_cnt = 0;
    return 0;
}

uint8_t debounce_plus(uint8_t rp) {
    if (rp != btn_plus_stable) {
        if (++btn_plus_cnt >= 2) {
            btn_plus_stable = rp; btn_plus_cnt = 0;
            return (rp == 0) ? 1 : 0;
        }
    } else btn_plus_cnt = 0;
    return 0;
}

uint8_t dec_val(uint8_t v) {
    if (v == 99) return 95;
    if (v >= 5)  return v - 5;
    return 0;
}

uint8_t inc_val(uint8_t v) {
    if (v < 95) return v + 5;
    if (v < 99) return 99;
    return v;
}

// ======================= BLINK =======================
void handle_blink(void) {
    if (state == ST_SERVICE_MENU || state == ST_SERVICE_ID_SELECT ||
        state == ST_RECOVERY_NE) {
        uint16_t now = (uint16_t)millis();
        if ((uint16_t)(now - blink_last) >= BLINK_PERIOD_MS) {
            blink_last = now;
            blink_hidden = !blink_hidden;
        }
    } else {
        blink_hidden = 0;
    }
}

// ======================= АЦП (Как в КОД1, с защитным таймаутом) =======================
void init_adc(void) {
    CLK->PCKENR2 |= (1 << 3);

    ADC1->CR1  = 0x00;
    ADC1->CR2  = 0x00;
    ADC1->CSR  = 0x00;
    ADC1->TDRH = 0x00;
    ADC1->TDRL = 0x00;

    ADC1->CR1  = (0x04 << 4);
    ADC1->CR2  = (1 << 3);
    ADC1->TDRH = (1 << 1);

    ADC1->CR1 |= 0x01;
    for (volatile uint16_t i = 0; i < 1000; i++);

    ADC1->CSR = (ADC1->CSR & 0xF0) | 0x09;

    ADC1->CSR &= ~0x80;
    ADC1->CR1 |= 0x01;
    
    // Защита от зависания
    uint32_t timeout = 10000;
    while (!(ADC1->CSR & 0x80)) {
        if (--timeout == 0) break;
        iwdg_feed();
    }
    (void)ADC1->DRL;
    (void)ADC1->DRH;
    ADC1->CSR &= ~0x80;
}

void adc_start(void) {
    if (adc_active) return;
    adc_sum = 0;
    adc_count = 0;
    adc_active = 1;
    adc_step_start = (uint16_t)millis();
}

void adc_poll_step(void) {
    if (!adc_active) return;
    uint16_t now = (uint16_t)millis();
    if ((uint16_t)(now - adc_step_start) < ADC_STEP_MS) return;
    adc_step_start = now;

    ADC1->CSR &= ~0x80;
    ADC1->CR1 |= 0x01;
    
    // Защита от зависания
    uint32_t timeout = 10000;
    while (!(ADC1->CSR & 0x80)) {
        if (--timeout == 0) break;
        iwdg_feed();
    }

    uint8_t low  = ADC1->DRL;
    uint8_t high = ADC1->DRH;
    uint16_t result = ((uint16_t)high << 8) | low;
    ADC1->CSR &= ~0x80;

    adc_sum += result;
    adc_count++;

    if (adc_count >= ADC_STEPS) {
        adc_last = (uint16_t)(adc_sum / ADC_STEPS);
        adc_active = 0;

        if ((int32_t)(millis() - nh_ignore_until) < 0) return;

        if (adc_last > ADC_VDD_LOW_ADC) {
            if (state != ST_SOFT_UL) {
                prev_state_ul = state;
                enter_soft(ST_SOFT_UL, CHAR_N, CHAR_N);
            }
        } else if (adc_last < ADC_VDD_OK_ADC) {
            if (state == ST_SOFT_UL) {
                state = prev_state_ul;
                if (state == ST_WORK_DE) {
                    show_setpoint(setpoint);
                } else if (state == ST_WORK_NE) {
                    ne_display_state = 0;
                    ne_display_last = (uint16_t)millis();
                } else if (state == ST_RECOVERY_NE || state == ST_SOFT_NE) {
                    show_string(CHAR_S, CHAR_E);
                } else if (state == ST_SOFT_CN) {
                    show_string(CHAR_N, CHAR_O);
                } else if (state == ST_SOFT_TR) {
                    show_string(CHAR_P, CHAR_R);
                }
            }
        }
    }
}

void poll_adc(void) {
    uint16_t now = (uint16_t)millis();
    if ((uint16_t)(now - last_adc_poll) < 1000) return;
    last_adc_poll = now;
    adc_start();
}

// ======================= EEPROM =======================
uint8_t ee_read8(uint16_t off) { return *(volatile uint8_t*)(EE_BASE + off); }

void ee_write8(uint16_t off, uint8_t val) {
    F_DUKR = 0xAE;
    F_DUKR = 0x56;
    uint16_t timeout = 10000;
    while (!(F_IAPSR & 0x08)) {
        if (--timeout == 0) return;
        iwdg_feed();
    }
    *(volatile uint8_t*)(EE_BASE + off) = val;
    F_IAPSR &= ~0x08;
}

// ======================= UART =======================
void init_uart(void) {
    GPIOD->DDR |= (1<<5);
    GPIOD->CR1 |= (1<<5);
    GPIOD->CR2 &= ~(1<<5);
    GPIOD->ODR |= (1<<5);

    GPIOD->DDR &= ~(1<<6);
    GPIOD->CR1 |= (1<<6);
    GPIOD->CR2 &= ~(1<<6);

    UART1->CR1 = 0x00;
    UART1->CR2 = (1<<3) | (1<<2);
    UART1->BRR2 = (uint8_t)(((UBRR_VAL >> 12) & 0x0F) << 4) | (uint8_t)(UBRR_VAL & 0x0F);
    UART1->BRR1 = (uint8_t)((UBRR_VAL >> 4) & 0xFF);
}

static inline void uart_send_char(char c) {
    while (!(UART1->SR & (1<<7)));
    UART1->DR = c;
}

static inline uint8_t uart_rx_ready(void) {
    return (UART1->SR & (1<<5)) ? 1 : 0;
}

static inline uint8_t uart_recv(void) { return UART1->DR; }

void uart_send_packet(uint8_t id, uint8_t value) {
    uart_send_char('@');
    uart_send_char('0' + id);
    uart_send_char(',');
    if (value >= 100) {
        uart_send_char('0' + value / 100);
        uart_send_char('0' + (value / 10) % 10);
        uart_send_char('0' + value % 10);
    } else if (value >= 10) {
        uart_send_char('0' + value / 10);
        uart_send_char('0' + value % 10);
    } else {
        uart_send_char('0' + value);
    }
    uart_send_char('\r');
    uart_send_char('\n');
}

// ======================= ПАРСЕР =======================
void parse_packet(void) {
    uint8_t i = 0;
    uint8_t addr = 0;

    while (i < parse_idx && parse_buf[i] != ',') {
        if (parse_buf[i] >= '0' && parse_buf[i] <= '9') {
            addr = addr * 10 + (parse_buf[i] - '0');
        }
        i++;
    }
    if (i >= parse_idx) return;
    i++;

    if (i + 1 < parse_idx &&
        (parse_buf[i] == 'c' || parse_buf[i] == 'C') &&
        (parse_buf[i+1] == 'n' || parse_buf[i+1] == 'N')) {
        if (addr >= 1 && addr <= 9 && addr == my_id) {
            if (state == ST_WORK_NE || state == ST_INIT_NE ||
                state == ST_RECOVERY_NE || state == ST_SOFT_NE) {
                enter_soft(ST_SOFT_CN, CHAR_N, CHAR_O);
            }
            pause_ms(SLAVE_RESPONSE_DELAY_MS);
            uart_send_packet(my_id, desired);
        }
        return;
    }

    uint32_t values[3] = {0, 0, 0};
    for (uint8_t f = 0; f < 3 && i < parse_idx; f++) {
        while (i < parse_idx && parse_buf[i] != ',') {
            if (parse_buf[i] >= '0' && parse_buf[i] <= '9') {
                values[f] = values[f] * 10 + (parse_buf[i] - '0');
            }
            i++;
        }
        if (i < parse_idx && parse_buf[i] == ',') i++;
    }

    if (addr == 0) {
        if (values[0] <= 99999) {
            pos_current = values[0];
            ne_window_start = millis() - values[0];
            last_packet_ms = millis();
            sync_received = 1;
        }
    } else if (addr >= 1 && addr <= 9 && addr == my_id) {
        uint8_t power = (values[0] > 99) ? 99 : (uint8_t)values[0];
        uint32_t start = values[1];
        uint32_t dur = values[2];

        if (start == 0 && dur == 0) {
            // PING
        } else {
            actual = power;
            slot_start = (start > 99999) ? 99999 : start;
            slot_duration = (dur > WINDOW_MS) ? WINDOW_MS : dur;

            if (state == ST_SOFT_CN || state == ST_SOFT_NE) {
                state = ST_WORK_NE;
                ne_display_state = 0;
                ne_display_last = 0;
            }
        }

        pause_ms(SLAVE_RESPONSE_DELAY_MS);
        uart_send_packet(my_id, last_sent_desired);

        if (desired != last_sent_desired &&
            (uint16_t)((uint16_t)millis() - desired_last_change_ms) >= DEBOUNCE_MS) {
            last_sent_desired = desired;
            play_beep(10);
            ee_write8(EE_DESIRED_OFF, desired);
        }
    }
}

void handle_uart(void) {
    if (!uart_rx_ready()) return;
    char c = (char)uart_recv();

    if (c == '@') {
        parse_active = 1;
        parse_idx = 0;
        return;
    }
    if (parse_active) {
        if (c == '\r') return;
        if (c == '\n') {
            parse_active = 0;
            parse_packet();
            return;
        }
        if (parse_idx < sizeof(parse_buf) - 1) {
            parse_buf[parse_idx++] = c;
        }
    }
}

// ======================= PWM =======================
void update_pwm_de(void) {
    uint32_t now = millis();
    uint32_t elapsed = now - window_start;

    if (elapsed >= WINDOW_MS) {
        window_start = now;
        elapsed = 0;
    }
    uint32_t on_time;
    if (setpoint == 99) on_time = WINDOW_MS;
    else                on_time = (uint32_t)setpoint * (WINDOW_MS / 100);

    if (elapsed < on_time) {
        if (!relay_on) { GPIOA->ODR |= MOC_PIN; relay_on = 1; }
    } else {
        if (relay_on) { GPIOA->ODR &= ~MOC_PIN; relay_on = 0; }
    }
}

void update_pwm_ne(void) {
    if ((uint32_t)(millis() - last_packet_ms) >= PACKET_TIMEOUT_MS) {
        if (relay_on) { GPIOA->ODR &= ~MOC_PIN; relay_on = 0; }
        return;
    }
    if (!sync_received || slot_duration == 0) {
        if (relay_on) { GPIOA->ODR &= ~MOC_PIN; relay_on = 0; }
        return;
    }

    uint32_t pos_now = millis() - ne_window_start;
    while (pos_now >= WINDOW_MS) pos_now -= WINDOW_MS;
    uint32_t slot_end = slot_start + slot_duration;
    if (slot_end > WINDOW_MS) slot_end = WINDOW_MS;

    if (pos_now >= slot_start && pos_now < slot_end) {
        if (!relay_on) { GPIOA->ODR |= MOC_PIN; relay_on = 1; }
    } else {
        if (relay_on) { GPIOA->ODR &= ~MOC_PIN; relay_on = 0; }
    }
}

// ======================= ДЕТЕКТОР ПР =======================
void init_tr_state(void) {
    tr_fault = 0;
    tr_mismatch_cnt = 0;
    tr_match_cnt = 0;
    tr_prev_relay = relay_on;
    tr_relay_changed_ms = millis();
    tr_last_check_ms = millis();
}

void check_triac(void) {
    if (state != ST_WORK_NE && state != ST_WORK_DE) return;
    if (state == ST_WORK_NE && (uint32_t)(millis() - last_packet_ms) >= PACKET_TIMEOUT_MS) return;
    if (tr_fault) return;

    if (relay_on != tr_prev_relay) {
        tr_prev_relay = relay_on;
        tr_relay_changed_ms = millis();
        return;
    }
    if ((uint32_t)(millis() - tr_relay_changed_ms) < TR_CHECK_DELAY_MS) return;
    if ((uint32_t)(millis() - tr_last_check_ms) < TR_CHECK_PERIOD_MS) return;
    tr_last_check_ms = millis();

    uint8_t pd3 = (GPIOD->IDR & TR_SENSE_PIN) ? 1 : 0;
    uint8_t expected = relay_on;

    if (pd3 == expected) {
        tr_mismatch_cnt = 0;
        if (tr_match_cnt < 255) tr_match_cnt++;
    } else {
        tr_match_cnt = 0;
        if (tr_mismatch_cnt < 255) tr_mismatch_cnt++;
        if (tr_mismatch_cnt >= TR_CONFIRM_COUNT) {
            enter_soft_tr();
        }
    }
}

// ======================= ДИСПЛЕЙ nE =======================
void update_ne_display(void) {
    uint32_t now32 = millis();
    uint16_t now16 = (uint16_t)now32;
    uint8_t show_actual = 0;

    if (now32 < show_only_desired_until) {
        show_actual = 0;
    } else {
        if ((uint16_t)(now16 - ne_display_last) >= 1000) {
            ne_display_last = now16;
            ne_display_state ^= 1;
        }
        show_actual = ne_display_state;
    }

    if (show_actual) show_setpoint(actual);
    else             show_setpoint(desired);
}

// ======================= ПЕРЕХОДЫ =======================
void service_enter(void) {
    GPIOA->ODR &= ~MOC_PIN;
    desired = 0;
    play_beep(80);
    state = ST_SERVICE_SE;
    show_string(CHAR_S, CHAR_V);
    blink_hidden = 0;
    both_stable = 1;
    both_start_ms = (uint16_t)millis();
    both_triggered = 1;
    block_buttons(BUTTONS_BLOCK_MS);
}

void enter_service_menu(void) {
    uint8_t mode = ee_read8(EE_MODE_OFF);
    if (mode == 0xAE) {
        selected_mode = 0xAE;
        show_string(CHAR_S, CHAR_E);
    } else {
        selected_mode = 0xDE;
        show_string(CHAR_O, CHAR_S);
    }
    state = ST_SERVICE_MENU;
    blink_hidden = 0;
    blink_last = (uint16_t)millis();
    block_buttons(BUTTONS_BLOCK_MS);
    reset_buttons();
}

void enter_service_de_confirmed(void) {
    state = ST_SERVICE_DE_CONFIRMED;
    show_string(CHAR_O, CHAR_S);
    blink_hidden = 0;
    ee_write8(EE_MODE_OFF, 0xDE);
    play_beep(80);
    block_buttons(BUTTONS_BLOCK_MS);
    reset_buttons();
}

void enter_service_id_select(void) {
    state = ST_SERVICE_ID_SELECT;
    uint8_t id = ee_read8(EE_ID_OFF);
    if (id >= 1 && id <= 9) selected_id = id;
    else                    selected_id = 0;
    show_id(selected_id);
    blink_hidden = 0;
    blink_last = (uint16_t)millis();
    block_buttons(BUTTONS_BLOCK_MS);
    reset_buttons();
}

void service_exit(void) {
    play_beep(80);
    if (state == ST_SERVICE_DE_CONFIRMED) {
        ee_write8(EE_MODE_OFF, 0xDE);
        enter_work_de();
    } else if (state == ST_SERVICE_ID_SELECT) {
        if (selected_id < 1 || selected_id > 9) selected_id = 1;
        ee_write8(EE_MODE_OFF, 0xAE);
        ee_write8(EE_ID_OFF, selected_id);
        enter_init_ne();
    } else if (state == ST_SERVICE_MENU) {
        if (selected_mode == 0xDE) {
            ee_write8(EE_MODE_OFF, 0xDE);
            enter_work_de();
        } else {
            enter_service_id_select();
            return;
        }
    }
    block_buttons(BUTTONS_BLOCK_MS);
    both_stable = 0;
    both_triggered = 0;
    both_start_ms = 0;
}

void enter_init_de(void) {
    state = ST_INIT_DE;
    show_string(CHAR_O, CHAR_S);
    blink_hidden = 0;
    init_display_start = millis();
}

void enter_init_ne(void) {
    my_id = ee_read8(EE_ID_OFF);
    if (my_id < 1 || my_id > 9) {
        enter_error_hard(CHAR_S, CHAR_E);
        return;
    }
    state = ST_INIT_NE;
    show_string(CHAR_S, CHAR_E);
    blink_hidden = 0;
    init_display_start = millis();
    last_packet_ms = 0;
    parse_active = 0;
    parse_idx = 0;
    actual = 0;

    desired = ee_read8(EE_DESIRED_OFF);
    if (desired > 99) desired = 0;
    last_sent_desired = desired;
    desired_last_change_ms = 0;

    pos_current = 0;
    ne_window_start = millis();
    show_only_desired_until = 0;
    ne_display_state = 0;
    ne_display_last = 0;
    adc_active = 0;

    slot_start = 0;
    slot_duration = 0;
    sync_received = 0;
}

void enter_recovery_ne(void) {
    state = ST_RECOVERY_NE;
    show_string(CHAR_S, CHAR_E);
    blink_hidden = 0;
    blink_last = (uint16_t)millis();
    recovery_start = (uint16_t)millis();
    GPIOA->ODR &= ~MOC_PIN;
    relay_on = 0;
}

void enter_soft(state_t s, uint8_t l, uint8_t r) {
    state = s;
    show_string(l, r);
    blink_hidden = 0;
    GPIOA->ODR &= ~MOC_PIN;
    relay_on = 0;
    beep_end_ms = 0;
    TIM2->CCER1 &= ~0x01;
    GPIOD->ODR &= ~BUZZER_PIN;
}

void enter_soft_tr(void) {
    enter_soft(ST_SOFT_TR, CHAR_P, CHAR_R);
    tr_mismatch_cnt = 0;
    tr_fault = 1;
}

void enter_work_de(void) {
    state = ST_WORK_DE;
    setpoint = 0;
    show_setpoint(0);
    window_start = millis();
    relay_on = 0;
    GPIOA->ODR &= ~MOC_PIN;
    last_adc_poll = (uint16_t)millis();
    reset_buttons();
    adc_active = 0;
    nh_ignore_until = millis() + NH_IGNORE_MS;
    init_tr_state();
}

void enter_work_ne(void) {
    state = ST_WORK_NE;
    show_setpoint(0);
    blink_hidden = 0;
    GPIOA->ODR &= ~MOC_PIN;
    relay_on = 0;
    play_beep(80);
    last_adc_poll = (uint16_t)millis();
    reset_buttons();
    ne_display_state = 0;
    ne_display_last = (uint16_t)millis();
    adc_active = 0;
    nh_ignore_until = millis() + NH_IGNORE_MS;
    init_tr_state();
}

void enter_error_hard(uint8_t l, uint8_t r) {
    state = ST_ERROR_HARD;
    show_string(l, r);
    blink_hidden = 0;
    GPIOA->ODR &= ~MOC_PIN;
    beep_end_ms = 0;
    TIM2->CCER1 &= ~0x01;
    GPIOD->ODR &= ~BUZZER_PIN;
    adc_active = 0;
}

// ======================= ОБРАБОТКА КНОПОК =======================
void handle_service_hold(void) {
    uint16_t now = (uint16_t)millis();
    uint8_t rm = (GPIOA->IDR & BTN_MINUS) ? 1 : 0;
    uint8_t rp = (GPIOA->IDR & BTN_PLUS)  ? 1 : 0;

    if (state == ST_SERVICE_SE) {
        if (rm && rp) enter_service_menu();
        return;
    }

    uint8_t both_pressed = (!rm) && (!rp);

    if (both_pressed) {
        if (!both_stable) {
            both_stable = 1;
            both_start_ms = now;
            both_triggered = 0;
        } else if (!both_triggered && (uint16_t)(now - both_start_ms) >= SERVICE_HOLD_MS) {
            both_triggered = 1;
            if (is_service_state(state)) {
                service_exit();
            } else if (state != ST_ERROR_HARD) {
                service_enter();
            }
        }
    } else {
        both_stable = 0;
        both_triggered = 0;
    }
}

void handle_buttons_service(void) {
    if (buttons_blocked()) return;
    if ((GPIOA->IDR & BTN_MINUS) == 0 && (GPIOA->IDR & BTN_PLUS) == 0) return;

    uint16_t now = (uint16_t)millis();
    if ((uint16_t)(now - last_btn_poll) < 20) return;
    last_btn_poll = now;

    uint8_t rm = (GPIOA->IDR & BTN_MINUS) ? 1 : 0;
    uint8_t rp = (GPIOA->IDR & BTN_PLUS)  ? 1 : 0;
    uint8_t minus_pressed = debounce_minus(rm);
    uint8_t plus_pressed  = debounce_plus(rp);

    if (state == ST_SERVICE_MENU) {
        if (plus_pressed) {
            if (selected_mode == 0xDE) { enter_service_de_confirmed(); return; }
            else                       { enter_service_id_select();    return; }
        }
        if (minus_pressed) {
            if (selected_mode == 0xDE) {
                selected_mode = 0xAE;
                show_string(CHAR_S, CHAR_E);
            } else {
                selected_mode = 0xDE;
                show_string(CHAR_O, CHAR_S);
            }
            play_beep(40);
        }
        return;
    }

    if (state == ST_SERVICE_DE_CONFIRMED) {
        if (minus_pressed) {
            state = ST_SERVICE_MENU;
            selected_mode = 0xDE;
            show_string(CHAR_O, CHAR_S);
            blink_hidden = 0;
            blink_last = (uint16_t)millis();
            block_buttons(BUTTONS_BLOCK_MS);
            btn_plus_stable = 1; btn_plus_cnt = 0;
            both_stable = 0; both_triggered = 0;
        }
        return;
    }

    if (state == ST_SERVICE_ID_SELECT) {
        if (plus_pressed) {
            if (selected_id >= 9) selected_id = 1;
            else                  selected_id++;
            show_id(selected_id);
            play_beep(40);
        }
        if (minus_pressed) {
            if (selected_id <= 1) selected_id = 9;
            else                  selected_id--;
            show_id(selected_id);
            play_beep(40);
        }
        return;
    }
}

void handle_buttons_work(void) {
    if (buttons_blocked()) return;
    uint16_t now = (uint16_t)millis();
    if ((uint16_t)(now - last_btn_poll) < 20) return;
    last_btn_poll = now;

    uint8_t rm = (GPIOA->IDR & BTN_MINUS) ? 1 : 0;
    uint8_t rp = (GPIOA->IDR & BTN_PLUS)  ? 1 : 0;
    uint8_t minus_pressed = debounce_minus(rm);
    uint8_t plus_pressed  = debounce_plus(rp);

    if (state == ST_WORK_DE) {
        if (minus_pressed) {
            setpoint = dec_val(setpoint);
            show_setpoint(setpoint);
            play_beep(40);
        }
        if (plus_pressed) {
            setpoint = inc_val(setpoint);
            show_setpoint(setpoint);
            play_beep(40);
        }
    } else {
        if (minus_pressed) {
            desired = dec_val(desired);
            show_setpoint(desired);
            show_only_desired_until = millis() + 5000;
            desired_last_change_ms = (uint16_t)millis();
        }
        if (plus_pressed) {
            desired = inc_val(desired);
            show_setpoint(desired);
            show_only_desired_until = millis() + 5000;
            desired_last_change_ms = (uint16_t)millis();
        }
    }
}

// ======================= SETUP / LOOP =======================
void setup() {
    init_pins();
    init_adc();
    init_uart();
    init_tim2_beep();
    show_string(CHAR_BLANK, CHAR_BLANK);

    uint8_t rst_sr = RST_SR;
    RST_SR = 0x00;

    iwdg_init();

    uint32_t t0 = millis();
    while (millis() - t0 < 100) iwdg_feed();

    if (rst_sr & RST_SR_IWDGF) {
        enter_error_hard(CHAR_S, CHAR_B);
        return;
    }

    if ((GPIOA->IDR & BTN_MINUS) == 0 && (GPIOA->IDR & BTN_PLUS) == 0) {
        service_enter();
    } else {
        uint8_t mode = ee_read8(EE_MODE_OFF);
        if (mode == 0xAE) enter_init_ne();
        else              enter_init_de();
    }
}

void loop() {
    handle_blink();
    mux_tick();
    handle_service_hold();
    handle_uart();

    if (state == ST_ERROR_HARD) {
        blink_beep_tick();
        return;
    }

    if (state == ST_SOFT_UL) {
        blink_beep_tick();
        poll_adc();
        adc_poll_step();
        return;
    }

    if (state == ST_SOFT_TR) {
        blink_beep_tick();
        poll_adc();
        adc_poll_step();
        return;
    }

    if (is_service_state(state)) {
        handle_buttons_service();
        return;
    }

    if (state == ST_INIT_DE) {
        if (millis() - init_display_start >= INIT_DISPLAY_MS) {
            enter_work_de();
        }
        return;
    }

    if (state == ST_INIT_NE) {
        if (millis() - init_display_start >= INIT_DISPLAY_MS) {
            if (last_packet_ms >= init_display_start) {
                enter_work_ne();
            } else {
                enter_recovery_ne();
            }
        }
        return;
    }

    if (state == ST_RECOVERY_NE) {
        blink_beep_tick();
        if ((uint32_t)(millis() - last_packet_ms) < 500) {
            enter_work_ne();
            return;
        }
        if ((uint16_t)((uint16_t)millis() - recovery_start) >= RECOVERY_MS) {
            enter_soft(ST_SOFT_NE, CHAR_S, CHAR_E);
        }
        return;
    }

    if (state == ST_SOFT_CN) {
        blink_beep_tick();
        if ((uint32_t)(millis() - last_packet_ms) >= PACKET_TIMEOUT_MS) {
            enter_recovery_ne();
            return;
        }
        poll_adc();
        adc_poll_step();
        return;
    }

    if (state == ST_SOFT_NE) {
        blink_beep_tick();
        if ((uint32_t)(millis() - last_packet_ms) < 500) {
            enter_work_ne();
            return;
        }
        poll_adc();
        adc_poll_step();
        return;
    }

    if (state == ST_WORK_DE) {
        handle_buttons_work();
        update_pwm_de();
        poll_adc();
        adc_poll_step();
        check_triac();
        return;
    }

    if (state == ST_WORK_NE) {
        if ((uint32_t)(millis() - last_packet_ms) >= PACKET_TIMEOUT_MS) {
            enter_recovery_ne();
            return;
        }
        handle_buttons_work();
        update_pwm_ne();
        update_ne_display();
        poll_adc();
        adc_poll_step();
        check_triac();
        return;
    }
}
