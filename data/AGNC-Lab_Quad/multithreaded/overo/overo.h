////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/overo.h                                                                 //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef overo
#define overo

////////////////////////////////////////////////////////////////////////////////////////////////////


  namespace thread {
    class overo {
      public:
        enum GPIO_CH {
          GPIO_10,
          GPIO_64,
          GPIO_65,
          GPIO_114,
          GPIO_144,
          GPIO_145,
          GPIO_146,
          GPIO_147,
          GPIO_168,
          GPIO_186
        };
        enum GPIO_IRQ_CH {
          GPIO_IRQ_10,
          GPIO_IRQ_64,
          GPIO_IRQ_65,
          GPIO_IRQ_114,
          GPIO_IRQ_146,
          GPIO_IRQ_147,
          GPIO_IRQ_186
        };
        enum GPIO_DIRECTION {
          IN,
          OUT
        };
        enum GPIO_TRIGGER {
          RISING,
          FALLING,
          BOTH
        };
        enum GPIO_VALUE {
          LOW,
          HIGH
        };
        
      private:
        static const char* sc_ch[];
        static const char* sc_irq_ch[];
        static const char* sc_dir[];
        static const char* sc_trig[];
        static const char* sc_val[];
        
      public:
        static void gpio_set_direction(GPIO_CH ch, GPIO_DIRECTION dir);
        static void gpio_set_trigger(GPIO_CH ch, GPIO_TRIGGER trig);
        static void gpio_set_value(GPIO_CH ch, GPIO_VALUE val);
        
    };
  }


////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
