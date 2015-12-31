/*
 * Copyright 2015 Glen Overby <gpoverby@gmail.com>
 *
 * rtl-jack, route data from DVB dongle to the JACK audio connection kit for
 * use by dttsp.
 *
 * This program is based on two other programs:
 * (1)
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * (2)
 * JACK Audio Connection Kit example client capture_client.c
 *  Copyright (C) 2001 Paul Davis
 *  Copyright (C) 2003 Jack O'Quin
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Notes about the program:
 *
 * This version practices decimation at a 6:1 ratio.
 * librtlsdr can only set certain sample rates.
 * x8 oversampling to 48khz did not work
 *
 * See-Also:
 *      https://github.com/glenoverby/dttsp
 *      dttsp-linux@yahoogroups.com
 *
 * Examples:
 *   x6 oversampling works:
 *
 *   ./rtl_jack -s 288000 -f 144385000  -b 16384 rtl
 *
 * To Do:
 *	- add a conversion thread inbetween the rtlsdr input and the JACK
 *	  output, and use libsamplerate to do the rate conversion instead of
 *	  the simple (bad) decimation.
 *	- emulate the usbsoftrock program for tuning.  This will allow easy
 *	  inegration with dttsp and the sdr-shell.
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <jack/jack.h>
#include <jack/ringbuffer.h>

#ifndef _WIN32
#include <unistd.h>
#else
#error "this has not been tried on windows"
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include "getopt/getopt.h"
#endif

#include "rtl-sdr.h"
#include "convenience/convenience.h"

#define RB_SIZE                 524288
#define DEFAULT_SAMPLE_RATE		2048000
#define DEFAULT_BUF_LENGTH		(16 * 16384)
#define MINIMAL_BUF_LENGTH		512
#define MAXIMAL_BUF_LENGTH		(256 * 16384)

static int do_exit = 0;
static uint32_t bytes_to_read = 0;
static rtlsdr_dev_t *dev = NULL;
struct rtlparams {
	uint32_t out_block_size;
};


/* JACK data */
typedef jack_default_audio_sample_t sample_t;
#define DEFAULT_RB_SIZE 16384
int nports = 2;
jack_port_t *ports[2];
//jack_default_audio_sample_t **in;
//jack_nframes_t nframes;
const size_t sample_size = sizeof(jack_default_audio_sample_t);

/*
 * Establish a ring buffer between JACK and librtlsdr.  This make a huge
 * assumption that the sample rates match.  If the rates don't match, then 
 * the sample rate matching stuff that alsa_in does is required (and it
 * isn't here).
 */
/* Synchronization between process thread and disk thread. */
#define DEFAULT_RB_SIZE 16384		/* ringbuffer size in frames */
unsigned long jack_sample_rate;
jack_nframes_t rb_size = DEFAULT_RB_SIZE;
jack_ringbuffer_t *rtldata;		/* ringbuffer for librtl data */
long overruns = 0;
jack_client_t *client;
struct jackparams {
	void	*buffer;		/* for reading from ring buffer */
	int	bufsize;		/* size of it */
};
int filling = 1;            /* filling buffer */

void usage(void)
{
	fprintf(stderr,
		"rtl_jack, an I/Q recorder for RTL2832 based DVB-T receivers\n\n"
		"Usage:\t -f frequency_to_tune_to [Hz]\n"
		"\t[-s samplerate (default: 2048000 Hz)]\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-g gain (default: 0 for auto)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-S force sync output (default: async)]\n"
		"\tjack-name\n\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dev);
	jack_client_close(client);
}
#endif

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	int chn;
	int32_t ba;			/* bytes available */
	int size;
	struct rtlparams *p;

    int dlen;                       /* length after decimation */
    int di;                         /* decimation counter */
    unsigned char *ddest, *dsrc;    /* decimation source and destination */

	if (do_exit)
		return;

	if ((bytes_to_read > 0) && (bytes_to_read < len)) {
		len = bytes_to_read;
		do_exit = 1;
		rtlsdr_cancel_async(dev);
	}

	p = (struct rtlparams*)ctx;
    dlen = len / 6;
	ba = jack_ringbuffer_write_space(rtldata);
	if (ba >= dlen) {
        size = (ba > dlen)? dlen : ba;	/* greater of data & space available */

        /* decimate buffer on top of itself.  Buffers are 256kb so decimation
           fits within the buffer.  */
        ddest = buf;
        ddest += 2;            /* skip first I & Q */
        dsrc = buf;
        *ddest = *dsrc;
        dsrc += 6;
        for (di=dlen; di > 0; di--) {
            *ddest = *dsrc;     /* copy I */
            ddest++;
            *ddest = *dsrc;     /* copy Q */
            ddest++;
            dsrc+=6;
        }

		//printf("rtlsdr: %d %d %d %d\n", len, dlen, size, ba);
        //printf("r");
		jack_ringbuffer_write(rtldata, (const char*)buf, (size_t)size);
	} else {
		//printf("rtlsdr: %d %d %d\n", len, dlen, ba);
		//printf("rtlsdr: %d %d %d\n", len, dlen, ba);
	}
}

static void start_rtl(int dev_index, uint32_t samp_rate, uint32_t frequency, int gain, int ppm_error)
{
	int r;

	if (dev_index == -1) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}

	/* Set the sample rate */
	verbose_set_sample_rate(dev, samp_rate);

	/* Set the frequency */
	verbose_set_frequency(dev, frequency);

	if (0 == gain) {
		 /* Enable automatic gain */
		verbose_auto_gain(dev);
	} else {
		/* Enable manual gain */
		gain = nearest_gain(dev, gain);
		verbose_gain_set(dev, gain);
	}

	verbose_ppm_set(dev, ppm_error);

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dev);
}

static void *
rtl_thread_main(void *arg)
{
	int r;
	struct rtlparams *params;
	params = (struct rtlparams *)arg;
	r = rtlsdr_read_async(dev, rtlsdr_callback, (void *)NULL,
			      0, params->out_block_size);
	return 0;
}

static int
jack_process (jack_nframes_t nframes, void *arg)
{
	struct jackparams *p;
	int want;			/* count of samples wanted */
	int get;			/* samples to get */
	int available;			/* bytes available */
	size_t bytes_per_frame = 2 * sample_size;

	jack_nframes_t count;
	void *bufi, *bufq;		/* output buffers */
	jack_default_audio_sample_t	*outi, *outq;	/* output buffer ptrs */
	uint8_t *input;

	p = (struct jackparams *)arg;
	if (do_exit)
		return 1;

	/* I could do away with the intermediate buffer and the copy by using
	 * jack_ringbuffer_get_read_vector */
	want = nframes * 2;
	available = jack_ringbuffer_read_space(rtldata);
	//get = (want > available) ? available : want;
	//get = (want > available) ? 0 : want;
    /* Want to keep the buffer filled to about 2 jack calls worth */
    if (filling) {
        if (available >= 8*want)
            filling = 0;
	    get = 0;
        printf("f");
    } else if (available == 0) {
        get = 0;
        filling = 1;
        printf("F");
    } else {
	    get = (want > available) ? available : want;
    }
	jack_ringbuffer_read(rtldata, p->buffer, get);
	input = (uint8_t *)p->buffer;

#if 0
    if (get == 0)
        //printf("J");
	    printf("jack: want %d avail %d get %d\n", want, available, get);
	    //printf("jack: want %d avail %d %d %lu aval %d = %d\n", want, avail);
    //        nframes, sizeof (jack_default_audio_sample_t) * nframes, available, get);
    else
        printf("j");
#endif

	bufi = jack_port_get_buffer (ports[0], nframes);
	bufq = jack_port_get_buffer (ports[1], nframes);
	outi = (jack_default_audio_sample_t *)bufi;
	outq = (jack_default_audio_sample_t *)bufq;

	/* Copy data  */
	for(count = 0; count < get; count++) {
		*outi++ = (jack_default_audio_sample_t)*input++;
		*outq++ = (jack_default_audio_sample_t)*input++;
	}
	for(; count < nframes; count++) {
		*outi++ = (jack_default_audio_sample_t)0;
		*outq++ = (jack_default_audio_sample_t)0;
	}

#if 0
	for (chn = 0; chn < nports; chn++) {
		in[chn] = jack_port_get_buffer (ports[chn], nframes);
		//printf("jack: %d : 0x%x\n", chn, in[chn]);


		/* if the ringbuffer doesn't have enough space to fill the
		 * buffer, fill it with silence then fill the buffer */
		if (ba < sizeof (jack_default_audio_sample_t) * nframes) {
			memset(in[chn], 127, sizeof (jack_default_audio_sample_t) * nframes);	// silence
		}
		if (ba) {
			if (ba > sizeof(jack_default_audio_sample_t) * nframes) {
				ba = sizeof (jack_default_audio_sample_t) * nframes;
			}
			jack_ringbuffer_read(rb[chn], (char *)in[chn], ba);
		}
	}
#endif

	/* Sndfile requires interleaved data.  It is simpler here to
	 * just queue interleaved samples to a single ringbuffer. */
//	for (i = 0; i < nframes; i++) {
//		for (chn = 0; chn < nports; chn++) {
//			if (jack_ringbuffer_write (rb, (void *) (in[chn]+i),
//					      sample_size)
//			    < sample_size)
//				overruns++;
//		}
//	}

	return 0;
}

static void
jack_shutdown (void *arg)
{
	fprintf(stderr, "JACK shut down, exiting ...\n");
	do_exit = 1;
}

static int
start_jack(char *portname, uint32_t samp_rate)
{
	//unsigned long jack_sample_rate;
	void *buffer;
	static struct jackparams params;

	/* JACK Setup */
	if ((client = jack_client_open (portname, JackNullOption, NULL)) == 0) {
		fprintf (stderr, "JACK server not running?\n");
		return 1;
	}

	jack_sample_rate = jack_get_sample_rate(client);
	if(samp_rate != (uint32_t)jack_sample_rate) {
		fprintf(stderr, "sample rate %d does not match jack sample rate %ld\n", 
			samp_rate, jack_sample_rate);
	}

	nports = 2;
	buffer = (void *) malloc(DEFAULT_RB_SIZE);
	memset(buffer, 0, DEFAULT_RB_SIZE);
	params.buffer = buffer;
	params.bufsize = DEFAULT_RB_SIZE;

	rtldata = jack_ringbuffer_create (RB_SIZE);
	memset(rtldata->buf, 0, rtldata->size);

	if ((ports[0] = jack_port_register (client, "i", JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0)) == 0) {
		fprintf (stderr, "cannot register output port \"i\"!\n");
			//jack_client_close (client);
	}
	//printf("i port is 0x%llx\n", ports[0]);
	if ((ports[1] = jack_port_register (client, "q", JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0)) == 0) {
		fprintf (stderr, "cannot register output port \"q\"!\n");
			//jack_client_close (client);
	}
	//printf("q port is 0x%llx\n", ports[1]);

	jack_set_process_callback (client, jack_process, &params);
	jack_on_shutdown (client, jack_shutdown, NULL);

	/* When JACK is running realtime, jack_activate() will have
	 * called mlockall() to lock our pages into memory.  But, we
	 * still need to touch any newly allocated pages before
	 * process() starts using them.  Otherwise, a page fault could
	 * create a delay that would force JACK to shut us down. */

	if (jack_activate (client)) {
		fprintf (stderr, "cannot activate client");
		return 1;
	}

	return 0;
}


int
stdinfreq()
{
	char input[255];
	uint32_t frequency = 100000000;
	
    while (!do_exit) {
		gets(input);
		if (!strncmp(input, "quit", 4)) {
            do_exit = 1;
			return 0;
		}
		frequency = (uint32_t)atofs(input);
		if (frequency != 0) {
			verbose_set_frequency(dev, frequency);
		}
	}
}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	char *portname = NULL;
	int n_read;
	int r, opt;
	int gain = 0;
	int ppm_error = 0;
	int sync_mode = 0;
	int dev_index = -1;
	int dev_given = 0;
	uint32_t frequency = 100000000;
	uint32_t samp_rate = DEFAULT_SAMPLE_RATE;
	uint32_t out_block_size = DEFAULT_BUF_LENGTH;
	pthread_t rtl_thread;
	struct rtlparams params;

	while ((opt = getopt(argc, argv, "d:f:g:s:b:n:p:S")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			frequency = (uint32_t)atofs(optarg);
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10); /* tenths of a dB */
			break;
		case 's':
			samp_rate = (uint32_t)atofs(optarg);
			break;
		case 'p':
			ppm_error = atoi(optarg);
			break;
		case 'b':
			out_block_size = (uint32_t)atof(optarg);
			break;
		case 'n':
			bytes_to_read = (uint32_t)atof(optarg) * 2;
			break;
		case 'S':
			sync_mode = 1;
			break;
		default:
			usage();
			break;
		}
	}

	if (argc <= optind) {
		usage();
	} else {
		portname = argv[optind];
	}

	if(out_block_size < MINIMAL_BUF_LENGTH ||
	   out_block_size > MAXIMAL_BUF_LENGTH ){
		fprintf(stderr,
			"Output block size wrong value, falling back to default\n");
		fprintf(stderr,
			"Minimal length: %u\n", MINIMAL_BUF_LENGTH);
		fprintf(stderr,
			"Maximal length: %u\n", MAXIMAL_BUF_LENGTH);
		out_block_size = DEFAULT_BUF_LENGTH;
	}

#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	start_jack(portname, samp_rate);
	start_rtl(dev_index, samp_rate, frequency, gain, ppm_error);

	fprintf(stderr, "Reading samples in async mode...\n");
	params.out_block_size = out_block_size;
//	r = rtlsdr_read_async(dev, rtlsdr_callback, (void *)NULL,
//			      0, out_block_size);
	pthread_create(&rtl_thread, NULL, rtl_thread_main, (void*)&params);

	stdinfreq();

	//pthread_join(rtl_thread, NULL);
    while (!do_exit) {
        sleep(1);
    }

	if (do_exit)
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error, exiting...\n");

	//jack_client_close (client);
	rtlsdr_cancel_async(dev);
	rtlsdr_close(dev);
	//jack_ringbuffer_free (rb);
out:
	return 0;
}
