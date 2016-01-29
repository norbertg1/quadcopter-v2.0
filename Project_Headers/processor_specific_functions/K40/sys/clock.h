
#ifndef MCG_H_
#define MCG_H_

#define CORE_CLOCK			 (96000000)
#define PERIPHERAL_BUS_CLOCK (CORE_CLOCK/2)

#pragma define_section relocate_code ".data" ".data" ".data" far_abs RX
#define __relocate_code__   __declspec(relocate_code)

__relocate_code__ void InitClock();

#endif /* MCG_H_ */
