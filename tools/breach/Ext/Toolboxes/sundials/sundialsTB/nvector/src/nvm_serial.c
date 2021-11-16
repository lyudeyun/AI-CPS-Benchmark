/*
 * -----------------------------------------------------------------
 * $Revision: 1.1 $
 * $Date: 2009-06-05 16:26:21 $
 * -----------------------------------------------------------------
 * Programmer: Radu Serban @ LLNL
 * -----------------------------------------------------------------
 * Copyright (c) 2005, The Regents of the University of California.
 * Produced at the Lawrence Livermore National Laboratory.
 * All rights reserved.
 * For details, see the LICENSE file.
 * -----------------------------------------------------------------
 * Vector constructors for the SUNDIALS Matlab interfaces.
 * -----------------------------------------------------------------
 */

#include <stdlib.h>
#include "nvm.h"
#include <nvector/nvector_serial.h>

void InitVectors()
{}

N_Vector NewVector(int n)
{
  N_Vector v;
  v = N_VNew_Serial((long int)n);
  return(v);
}
