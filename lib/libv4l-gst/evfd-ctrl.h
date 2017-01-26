/*
 * Copyright (C) 2015 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Suite 500, Boston, MA  02110-1335  USA
 */

#ifndef __EVFD_CTRL_H__
#define __EVFD_CTRL_H__

#include <sys/eventfd.h>
#include <stdint.h> /* uint32_t */
#include <glib.h>

struct event_state;

struct event_state * new_event_state();
void delete_event_state(struct event_state *state);
void set_event(struct event_state *state, int event);
void clear_event(struct event_state *state, int event);
int event_state_fd(struct event_state *state);

#endif /* __EVFD_CTRL_H__ */
