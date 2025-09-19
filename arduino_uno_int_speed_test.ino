
const uint8_t pin_scope = LED_BUILTIN;
#define PIN_SCOPE_ON (PORTB |= _BV(PB5))
#define PIN_SCOPE_OFF (PORTB &= ~_BV(PB5))

const uint8_t pin_interrupt = 2;
#define PIN_INT_READ (PORTD & _BV(PD2))

const uint8_t PIN_A = 5, PIN_B = 6, PIN_Z = 7;

#define PIN_A_READ (PORTD & _BV(PD5))
#define PIN_B_READ (PORTD & _BV(PD6))
#define PIN_Z_READ (PORTD & _BV(PD7))

#define ENCODER_USE_PORT

#define USE_INLINE

#if defined(USE_INLINE)
#define OPTIONAL_INLINE inline __attribute__((always_inline))
#else
#define OPTIONAL_INLINE
#endif

void isr_basic_digitalwrite_4(void) {
  digitalWrite(pin_scope, HIGH);
  digitalWrite(pin_scope, LOW);
  digitalWrite(pin_scope, HIGH);
  digitalWrite(pin_scope, LOW);
  digitalWrite(pin_scope, HIGH);
  digitalWrite(pin_scope, LOW);
  digitalWrite(pin_scope, HIGH);
  digitalWrite(pin_scope, LOW);
}

void isr_basic_digitalread_8(void) {
  digitalWrite(pin_scope, HIGH);
  digitalRead(pin_interrupt);
  digitalRead(pin_interrupt);
  digitalRead(pin_interrupt);
  digitalRead(pin_interrupt);
  digitalRead(pin_interrupt);
  digitalRead(pin_interrupt);
  digitalRead(pin_interrupt);
  digitalRead(pin_interrupt);
  digitalWrite(pin_scope, LOW);
}

void isr_basic_portread_8(void) {
  PIN_SCOPE_ON;
  PIN_INT_READ;
  PIN_INT_READ;
  PIN_INT_READ;
  PIN_INT_READ;
  PIN_INT_READ;
  PIN_INT_READ;
  PIN_INT_READ;
  PIN_INT_READ;
  PIN_SCOPE_OFF;
}

void isr_basic_portwrite_4(void) {
  PIN_SCOPE_ON;
  PIN_SCOPE_OFF;
  PIN_SCOPE_ON;
  PIN_SCOPE_OFF;
  PIN_SCOPE_ON;
  PIN_SCOPE_OFF;
  PIN_SCOPE_ON;
  PIN_SCOPE_OFF;
}

void isr_basic_portwrite_max_isr(void) {
  PIN_SCOPE_ON;
  PIN_SCOPE_OFF;
}

volatile uint8_t uint8 = 0;
volatile uint16_t uint16 = 0;
volatile uint32_t uint32 = 0;

void isr_speed_8() {
  PIN_SCOPE_ON;
  uint8++;
  PIN_SCOPE_OFF;
}

void isr_speed_16() {
  PIN_SCOPE_ON;
  uint16++;
  PIN_SCOPE_OFF;
}

void isr_speed_32() {
  PIN_SCOPE_ON;
  uint32++;
  PIN_SCOPE_OFF;
}

class Encoder {
public:
  Encoder(
    const uint8_t pin_a,
    const uint8_t pin_b,
    const uint8_t pin_z,
    const uint8_t pin_z_active_state)
    : pin_a(pin_a),
      pin_b(pin_b),
      pin_z(pin_z),
      pin_z_active_state(pin_z_active_state) {
  }

  void setup() const {
    pinMode(pin_a, INPUT);
    pinMode(pin_b, INPUT);
    pinMode(pin_z, INPUT);
  }

  OPTIONAL_INLINE bool get_pin_a() const {
#if defined(ENCODER_USE_PORT)
    return PIN_A_READ;
#else
    return digitalRead(pin_a);
#endif
  }

  OPTIONAL_INLINE bool get_pin_b() const {
#if defined(ENCODER_USE_PORT)
    return PIN_B_READ;
#else
    return digitalRead(pin_b);
#endif
  }

  OPTIONAL_INLINE bool reset_detected() const {
#if defined(ENCODER_USE_PORT)
    return PIN_Z_READ;
#else
    return digitalRead(pin_z) == pin_z_active_state;
#endif
  }

private:
  const uint8_t pin_a;
  const uint8_t pin_b;
  const uint8_t pin_z;
  const uint8_t pin_z_active_state;
};

class Quadrature {
public:
  Quadrature(const Encoder& encoder)
    : encoder(encoder), state(0), counter(0), relative_zero(0), error_flag(0) {}

  OPTIONAL_INLINE void update_state_from_inputs() {
    state = ((state & 0b0011) << 2) | ((encoder.get_pin_b() & 1) << 1) | (encoder.get_pin_a() & 1);
  }

  OPTIONAL_INLINE void update_counter_from_quadrature() {
    // branchless counter update (increment, decrement, or keep)
    const uint16_t result = LOOKUP[state];
    const int8_t delta = (int8_t)result & 0xFF;
    counter += delta;
    // branchless error tracking for later handling
    const uint8_t error = (uint8_t)result >> 8;
    error_flag |= error;
    // branchless counter reset:
    // if reset is true (1) --> counter is and'ed with 0L (erased)
    // if reset is false (0) --> counter is and'ed with 0xFFFFFFFF (kept intact)
    const bool reset = encoder.reset_detected();
    const uint32_t mask = ((uint32_t)reset) - 1;
    counter &= mask;
    // branchless homing flag
    homing_canary = homing_canary || reset;
  }

  void setup() {
    // build a valid state from two reads
    update_state_from_inputs();
    update_state_from_inputs();
  }

  OPTIONAL_INLINE void increment_counter() {
    counter++;
  }

  static const uint8_t LOOKUP_SIZE = (1 << 4);

  typedef enum {
    CCW = 0x00FF,        // -1 without error
    NOOP = 0x0000,       // 0 without error
    CW = 0x0001,         // +1 without error
    INVALID_3 = 0x8300,  // 0 with error and state was 0b0011
    INVALID_6 = 0x8600,  // 0 with error and state was 0b0110
    INVALID_9 = 0x8900,  // 0 with error and state was 0b1001
    INVALID_C = 0x8C00,  // 0 with error and state was 0b1100
  } LookupResult;

  static const uint16_t LOOKUP[];

private:
  const Encoder& encoder;
  volatile uint8_t state;
  volatile uint32_t counter;
  uint32_t relative_zero;
  volatile uint8_t error_flag;
  volatile bool homing_canary;
};

const uint16_t Quadrature::LOOKUP[LOOKUP_SIZE] = {
  /* i old new        */
  /*    ba BA         */
  /* 0  00 00 same    */ NOOP,
  /* 1  00 01 cw 1    */ CW,
  /* 2  00 10 ccw 1   */ CCW,
  /* 3  00 11 invalid */ INVALID_3,
  /* 4  01 00 ccw 4   */ CCW,
  /* 5  01 01 same    */ NOOP,
  /* 6  01 10 invalid */ INVALID_6,
  /* 7  01 11 cw 2 Z  */ CW,
  /* 8  10 00 cw 4    */ CW,
  /* 9  10 01 invalid */ INVALID_9,
  /* A  10 10 same    */ NOOP,
  /* B  10 11 ccw 2   */ CCW,
  /* C  11 00 invalid */ INVALID_C,
  /* D  11 01 ccw 3 Z */ CCW,
  /* E  11 10 cw 3 Z  */ CW,
  /* F  11 11 same    */ NOOP
};

Encoder global_encoder(PIN_A, PIN_B, PIN_Z, LOW);

Quadrature global_quadrature(global_encoder);

void isr_speed_call_increment() {
  PIN_SCOPE_ON;
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  global_quadrature.increment_counter();
  PIN_SCOPE_OFF;
}

void isr_position_call_increment() {
  PIN_SCOPE_ON;
  global_quadrature.update_state_from_inputs();
  global_quadrature.update_counter_from_quadrature();
  PIN_SCOPE_OFF;
}

void setup() {
  pinMode(pin_scope, OUTPUT);
  digitalWrite(pin_scope, LOW);
  pinMode(pin_interrupt, INPUT_PULLUP);
  global_encoder.setup();
  // attachInterrupt(digitalPinToInterrupt(pin_interrupt), isr_basic_digitalwrite_4, LOW);
  // attachInterrupt(digitalPinToInterrupt(pin_interrupt), isr_basic_portwrite_4, LOW);
  // attachInterrupt(digitalPinToInterrupt(pin_interrupt), isr_speed_8, LOW);
  // attachInterrupt(digitalPinToInterrupt(pin_interrupt), isr_speed_16, LOW);
  // attachInterrupt(digitalPinToInterrupt(pin_interrupt), isr_speed_32, LOW);
  // attachInterrupt(digitalPinToInterrupt(pin_interrupt), isr_speed_call_increment, LOW);
  attachInterrupt(digitalPinToInterrupt(pin_interrupt), isr_position_call_increment, LOW);
  // attachInterrupt(digitalPinToInterrupt(pin_interrupt), isr_basic_digitalread_8, LOW);
  // attachInterrupt(digitalPinToInterrupt(pin_interrupt), isr_basic_portread_8, LOW);
}

void loop() {
  delay(1000);
}
