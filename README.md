# interrupt and pin speed

timing experiments for

- isr
- in/out
- inline
- increment
- quadrature encoder updates

on an uno rev3 16 mHz

## Arduino Uno Rev3 16Mhz

t = 62,5 ns

### digital writing

between two digitalWrite edges = 3.2us

between two isr execution effects = 8.4us

### digital reading

for 8 digitalreads --> 31.5us --> 4 us par lecture

between ISR --> 9.2 us 

### port reading

for 8 port read --> 625ns --> 78ns par lecture

### port writing

between two portwrite effects = 120-130ns
--> max output frequency ~770 kHz

time between two isr execution effects = 5.44us
--> max ISR frequency ~180 kHz

### simple volatile increment

uint8++ --> 436ns between events --> 2300 KHz
between 5.44us
total 5.8us 170kHz

uint16++ --> 750ns between events --> 1330 kHz
between isr 5.44
total 6.2us 161khz

uint32++ --> 1.37us between events --> 730 KHz
between isr --> 5.44us
total 6.8us 147 kHz

### function calls

call/uint32 increment
between calls 5.43us

10x call/inc without inline --> 18.8 us + 5.5us = 41 kHz
10x call/inc with inline --> 12.76 us + 5.5us = 55 kHz

100x call/inc without inline --> 187.7 us + 5.5us = 5.2 kHz
100x call/inc with inline --> 125.4 us + 5.5us = 7.6 kHz

with inline or without is the same for below : not enough calls to unwrap

  digitalread

    call/update(state+counter)
      isr duration --> 21.2
      between isr --> 6.4
      total isr --> 36.2 kHz
      sketch 1700 bytes / 73 ram

    timer1 no prescaler :
      quadrature.increment_counter --> 22
      quadrature.update_state_from_inputs --> 193
      quadrature.update_counter_from_quadrature --> 150

  port read

    call/update(state+counter) 
      isr duration --> 6.4 us
      between isr --> 5.4 us
      total isr --> 84.7 kHz
      sketch 1544 bytes / 73 ram

    timer1 no prescaler :
      quadrature.increment_counter --> 22
      quadrature.update_state_from_inputs --> 35
      quadrature.update_counter_from_quadrature --> 87
      (same without always_inline !)



# disassembly

after compiling in arduino ide, show detailed compilation commands, find the one for your ino file (compilation only, with -c flag)

then in user folder c:\users\xxx using cmd.exe copy command and add -S to it

"C:\\Users\\xxx\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\avr-gcc\\7.3.0-atmel3.6.1-arduino7/bin/avr-g++" -c -g -Os -Wall -Wextra -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10607 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR "-IC:\\Users\\xxx\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\avr\\1.8.6\\cores\\arduino" "-IC:\\Users\\xxx\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\avr\\1.8.6\\variants\\standard" "C:\\Users\\xxx\\AppData\\Local\\arduino\\sketches\\D35AD4692F44DBD300F0C68E747ED1B6\\sketch\\arduino_uno_int_speed_test.ino.cpp" -S

the -S generates assembly file but as strings ... :(

# timing with timers

https://www.locoduino.org/spip.php?article84


  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  TCNT1H = 0;
  TCNT1L = 0;
  bitClear(TIFR1, TOV1);
  TCCR1B = 1;

  work();

  TCCR1B = 0;
  count = (TCNT1H << 8) | TCNT1L;
  Serial.print("tov1=");
  Serial.print(bitRead(TIFR1, TOV1));
  bitClear(TIFR1, TOV1);
  Serial.print(" ctr=");
  Serial.println(count);
