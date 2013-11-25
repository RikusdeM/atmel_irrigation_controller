#include <asf.h>
#include "epoch.h"

volatile bool epoch_midnight = false;
static volatile uint32_t epoch = 0;


void epoch_set_ts(uint32_t ts)
{
    irqflags_t f = cpu_irq_save();
    if (ts % (24UL * 60UL * 60UL) < epoch % (24UL * 60UL * 60UL)) {
        epoch_midnight = true;
    }
    epoch = ts;
    cpu_irq_restore(f);
}

uint32_t epoch_get_ts(void)
{
    uint32_t ts;
    irqflags_t f = cpu_irq_save();
    ts = epoch;
    cpu_irq_restore(f);
    return ts;
}

void epoch_isr_inc_ts(void)
{
    if (epoch) {
        epoch++;
        epoch_midnight = epoch_midnight || ((epoch % (24UL * 60UL * 60UL)) == 0);
    }
}

uint32_t epoch_isr_get_ts(void)
{
    return epoch;
}