#include <stdio.h>
#include <registryFunction.h>
#include <epicsExport.h>
#include <aSubRecord.h>
#include <alarm.h>
#include <recGbl.h>

/*
 *  Subroutine to generate time entry for each element in the waveform.
 *  INPA - acquisition frequency in MHz
 *  INPB - number of elements in the array
 *  INPC - offset in us
 *  VALA = (C + 0/A, C + 1/A, C + 2/A, ..., C + (B-1)/A)
 */
static long timeAxisGenerator(aSubRecord *prec)
{
    double freq_mhz   = *((double *) prec->a);
    long   n_elements = *((unsigned long *) prec->b);
    double offset_us  = *((double *) prec->c);
    double *result    = (double *) prec->vala;

    long stat=0;

    if (n_elements > prec->nova)
    {
        stat = -1;
        printf("Error: TS Generator: Requested number of elements (%ld) exceeds limit (%d).\n",
                n_elements, prec->nova);
        n_elements = prec->nova;
    }

    int i;
    for (i=0; i<n_elements; i++)
    {
        result[i] = offset_us + i/freq_mhz;
    }
    prec->neva = n_elements;

    if (stat != 0)
    {
        recGblSetSevr(prec, CALC_ALARM, MINOR_ALARM);
    }
    return stat;
}

epicsRegisterFunction(timeAxisGenerator);
