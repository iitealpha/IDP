// Using code from forum: https://forum.arduino.cc/t/wifi-rev-2-timers/589912/5


int led = 10;
int led2 = 9;


//And the interrupt handler:

ISR(TCB0_INT_vect){
  // Do something
  flash_led();
  // Clear interrupt flag
  TCB0.INTFLAGS = TCB_CAPT_bm;
}

ISR(TCB1_INT_vect){
  // Do something
  flash_led2();
  // Clear interrupt flag
  TCB1.INTFLAGS = TCB_CAPT_bm;
}


void flash_led(){
  if (digitalRead(led)){
    digitalWrite(led, 0);
    //digitalWrite(led2, 0);
  }
  else{
    digitalWrite(led, 1);
    //digitalWrite(led2, 1);
  }
}

void flash_led2(){
  if (digitalRead(led2)){
    //digitalWrite(led, 0);
    digitalWrite(led2, 0);
  }
  else{
    //digitalWrite(led, 1);
    digitalWrite(led2, 1);
  }
}

// Note: Interrupt speed:
//    timer_compare_value = 125000 / interrupt_freq
//    seems to only work with round numbers (e.g. 1Hz, 2Hz, 5Hz, ..., 1kHz, ...)
//    Frequency less precise at high frequencies (e.g. around 1KHz)
//    Glitches at higher than 10KHz or so.
//    There are 3 timers, TCB0, TCB1, TCB2. However, these could be broken up further within the ISR

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  digitalWrite(led, 0);
  pinMode(led2, OUTPUT);
  digitalWrite(led2, 0);
  
  // Timer A (used as clock for B) clocked at 250kHz.
  TCB0.CTRLB = TCB_CNTMODE_INT_gc; // Use timer compare mode
  TCB0.CCMP = 62500; // Value to compare with. 62500 gives 2Hz.
  TCB0.INTCTRL = TCB_CAPT_bm; // Enable the interrupt
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use Timer A as clock, enable timer

  TCB1.CTRLB = TCB_CNTMODE_INT_gc; // Use timer compare mode
  TCB1.CCMP = 125; // Value to compare with. 62500 gives 2Hz.
  TCB1.INTCTRL = TCB_CAPT_bm; // Enable the interrupt
  TCB1.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use Timer A as clock, enable timer
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
}
