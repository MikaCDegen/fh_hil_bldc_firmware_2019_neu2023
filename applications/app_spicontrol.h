#ifndef SPICONTROL_H_
#define SPICONTROL_H_

#include "conf_general.h"

// Functions
void spicontrol_tim_isr(void);
void spicontrol_begin(void);
void spicontrol_end(void);
void spicontrol_delay(void);
void spicontrol_transfer(uint32_t *in_buf, const uint32_t *out_buf, int length);
uint32_t spicontrol_get_val(void);


#endif /* SPICONTROL_H_ */
