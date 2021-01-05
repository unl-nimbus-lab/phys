////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/overo.cpp                                                               //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <fcntl.h>
#include "overo.h"
#include <poll.h>
#include <sys/time.h>
using overo;

////////////////////////////////////////////////////////////////////////////////////////////////////

const char* overo::sc_ch[] = {
  "10",
  "64",
  "65",
  "114",
  "144",
  "145",
  "146",
  "147",
  "168",
  "186"
};
const char* overo::sc_irq_ch[] = {
  "10",
  "64",
  "65",
  "114",
  "146",
  "147",
  "186"
};
const char* overo::sc_dir[] = {
  "in",
  "out"
};
const char* overo::sc_trig[] = {
  "rising",
  "falling",
  "both"
};
const char* overo::sc_val[] = {
  "0",
  "1"
};

void gpio_config_for_pw_monitoring(GPIO_IRQ_CH irq_ch) {
  switch (irq_ch) {
    case GPIO_IRQ_10:
      gpio_set_direction(GPIO_10,IN);
      gpio_set_value(GPIO_10,LOW);
      gpio_set_trigger(GPIO_10,BOTH);
      break;
    case GPIO_IRQ_64:
      gpio_set_direction(GPIO_64,IN);
      gpio_set_value(GPIO_64,LOW);
      gpio_set_trigger(GPIO_64,BOTH);
      break;
    case GPIO_IRQ_65:
      gpio_set_value(GPIO_65,LOW);
      gpio_set_trigger(GPIO_65,BOTH);
      break;
    case GPIO_IRQ_114:
      gpio_set_value(GPIO_144,LOW);
      gpio_set_trigger(GPIO_114,BOTH);
    case GPIO_IRQ_146:
      gpio_set_direction(GPIO_146,IN);
      gpio_set_value(GPIO_146,LOW);
      gpio_set_trigger(GPIO_146,BOTH);
      break;
    case GPIO_IRQ_147:
      gpio_set_direction(GPIO_147,IN);
      gpio_set_value(GPIO_147,LOW);
      gpio_set_trigger(GPIO_147,BOTH);
      break;
    case GPIO_IRQ_186:
      gpio_set_direction(GPIO_186,IN);
      gpio_set_value(GPIO_186,LOW);
      gpio_set_trigger(GPIO_186,BOTH);
  }
}
void gpio_set_direction(GPIO_CH ch, GPIO_DIRECTION dir) {
  if ((ch == GPIO_10)||(ch == GPIO_64)||(ch == GPIO_146)||(ch == GPIO_147)) {
    // Valid for 10,64,146,147,186
    string cmd = "echo "+
                 string(sc_dir[dir])+
                 " > /sys/class/gpio/gpio"+
                 string(sc_ch[ch])+
                 "/direction";
    system(cmd.c_str());
  }
  else {
    ostringstream ss;
    ss << "invalid channel input: " << (uint32)ch;
    utilities::warning("fsw::hardware::overo::gpio_set_direction",ss.str());
  }
}
void gpio_set_trigger(GPIO_CH ch, GPIO_TRIGGER trig) {
  if ((ch == GPIO_10)||(ch == GPIO_64)||(ch == GPIO_65)||(ch == GPIO_114)||
      (ch == GPIO_146)||(ch == GPIO_147)||(ch == GPIO_186)) {
    // Valid for 10,64,65,114,146,147,186
    string cmd = "echo "+
                 string(sc_trig[trig])+
                 " > /sys/class/gpio/gpio"+
                 string(sc_ch[ch])+
                 "/edge";
    system(cmd.c_str());
  }
  else {
    ostringstream ss;
    ss << "invalid channel input: " << (uint32)ch;
    utilities::warning("fsw::hardware::overo::gpio_set_trigger",ss.str());
  }
}
void gpio_set_value(GPIO_CH ch, GPIO_VALUE val) {
  string cmd = "echo "+
               string(sc_val[val])+
               " > /sys/class/gpio/gpio"+
               string(sc_ch[ch])+
               "/value";
  system(cmd.c_str());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
