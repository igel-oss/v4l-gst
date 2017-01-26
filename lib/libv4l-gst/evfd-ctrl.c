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

#include <stdlib.h>
#include <poll.h>
#include <unistd.h>
#include <sys/syscall.h>

#include "evfd-ctrl.h"
#include "debug.h"

#if HAVE_VISIBILITY
#define PLUGIN_PUBLIC __attribute__ ((visibility("default")))
#else
#define PLUGIN_PUBLIC
#endif

#define SYS_READ(fd, buf, len) \
	syscall(SYS_read, (int)(fd), (void *)(buf), (size_t)(len));
#define SYS_WRITE(fd, buf, len) \
	syscall(SYS_write, (int)(fd), (const void *)(buf), (size_t)(len));

struct event_state {
	guint state;
	GMutex lock;
	int fd;
};

struct event_state * new_event_state() {
	struct event_state *state;
	int efd;
	efd = eventfd(0, EFD_NONBLOCK);
	if (efd < 0)
	{
		fprintf(stderr, "eventfd failed\n");
		return NULL;
	}
	state = calloc (1, sizeof (struct event_state));

	g_mutex_init(&state->lock);
	state->fd = efd;
	return state;
}

int event_state_fd(struct event_state *state) {
	return state->fd;
}

void delete_event_state(struct event_state *state) {
	close(state->fd);
	g_mutex_clear(&state->lock);
	free(state);
}

void set_event(struct event_state *state, int event) {
	g_mutex_lock(&state->lock);
	if ((state->state & event) == event) {
		g_mutex_unlock(&state->lock);
		return;
	}

	/* only accept POLLIN/POLLOUT requests,
	   but both are output as POLLIN */
	if ((event & ~(POLLIN | POLLOUT))) {
		g_mutex_unlock(&state->lock);
		return;
	}

	if (!state->state) {
		uint64_t buf = 1;
		/* Emit POLLIN */
		DBG_LOG("Emit POLLIN\n");
		SYS_WRITE(state->fd, &buf, sizeof(buf));
	}
	state->state |= event;
	g_mutex_unlock(&state->lock);
}

void clear_event(struct event_state *state, int event) {
	g_mutex_lock(&state->lock);
	/* No change */
	if ((~state->state & event) == event) {
		g_mutex_unlock(&state->lock);
		return;
	}

	/* only accept POLLIN/POLLOUT requests,
	   but both are output as POLLIN */
	if ((event & ~(POLLIN | POLLOUT))) {
		g_mutex_unlock(&state->lock);
		return;
	}

	if (!(state->state & ~event)) {
		uint64_t buf;
		/* Remove POLLIN */
		DBG_LOG("Remove POLLIN\n");
		SYS_READ(state->fd, &buf, sizeof(buf));
	}
	state->state &= ~event;
	g_mutex_unlock(&state->lock);
}
