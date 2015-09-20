/*
 * Copyright (C) 2014 
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file test1.h
 *  \brief testing testing testing!
 */

#include "std.h"
#include "math/pprz_geodetic_int.h"

#include "subsystems/navigation/waypoints.h"
#include "subsystems/navigation/common_flight_plan.h"

#include "messages.h"
#include "dl_protocol.h"

#ifdef GPSSENDING
#define GPSSENDING
      if (DL_REMOTE_GPS_ac_id(dl_buffer) == LEADER_AC_ID) {
	printf("unknown data detected");
	cherrytest(dl_buffer);
}

#endif

int funfun();
