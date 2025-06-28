/*
 * ngtcp2
 *
 * Copyright (c) 2025 ngtcp2 contributors
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include "ngtcp2_wave.h"

#include <string.h>

#include "ngtcp2_log.h"
#include "ngtcp2_macro.h"
#include "ngtcp2_conn_stat.h"

#define NGTCP2_WAVE_MAX_QUIC_PACKET_SIZE 1444

static void wave_reset(ngtcp2_cc_wave *wave, ngtcp2_conn_stat *cstat);

static void wave_reset(ngtcp2_cc_wave *wave, ngtcp2_conn_stat *cstat) {
    // Restore Wave state variables to default
    wave->default_burst_size = 10 * NGTCP2_WAVE_MAX_QUIC_PACKET_SIZE;
    wave->tx_time = 200 * NGTCP2_MILLISECONDS;
    wave->beta = 150 * NGTCP2_MILLISECONDS;
    wave->min_rtt = UINT64_MAX;
    wave->avg_rtt = cstat->smoothed_rtt;
    wave->alpha = 0;
    wave->ack_count = 0;
    wave->ack_data = 0;
    wave->bursts = (burst_stats *) NULL;

    // Update CC pacing data
    cstat->pacing_interval_m = (wave->tx_time / wave->default_burst_size) << 10;
    if (wave->default_burst_size > (uint64_t)-1) {
      cstat->send_quantum = (size_t)-1;
    } else {
      cstat->send_quantum = (size_t)wave->default_burst_size;
    }
    cstat->cwnd = UINT64_MAX;
    cstat->ssthresh = UINT64_MAX;

    ngtcp2_log_info(wave->cc.log, NGTCP2_LOG_EVENT_CCA,
                    "Wave state set to default values. Tx time=",
                    wave->tx_time);
}

void wave_cc_on_pkt_acked(ngtcp2_cc *cc,
                                    ngtcp2_conn_stat *cstat,
                                    const ngtcp2_cc_pkt *pkt,
                                    ngtcp2_tstamp ts) {
    // When a packet is acknowledged update the related burst_stat structure
    ngtcp2_cc_wave *wave = ngtcp2_struct_of(cc, ngtcp2_cc_wave, cc);
    burst_stats *current_burst = wave->bursts;
    burst_stats *next_burst_stat;
    burst_stats *prev_burst_stat;
    double ack_train_disp;
    double alpha;
    double avg_rtt;
    double delta_rtt;
    uint64_t tx_timer_calc;

    if (current_burst == NULL) {
        // Error: How can be null if I received an ack for that burst?
        // In practice this happens only for Handshake packets. Ignoring
        // Edit: this can happen if Wave returned from Adjustment mode,
        // deleting all data about burst sent previously
        return;
    }

    while (current_burst != NULL) {
        if (current_burst->send_time == pkt->sent_ts) {
            // Find the corresponding burst for that packet
            break;
        }
        if (current_burst->next != NULL) {
            // Iterate over the burst list
            current_burst = current_burst->next;
        } else {
            // Error: Burst not found in the list. Why?
            // Because of the same fact of previous: Handshake packets or
            // Adjustment mode. Ignoring both case
            return;
        }
    }

    // Once the corresponding burst is found, update the stats
    if (ts < current_burst->first_ack_time) {
        // Update the pilot RTT and ACK timestamp
        current_burst->first_ack_time = ts;
        current_burst->pilot_rtt = ts - pkt->sent_ts;
    }
    if (ts > current_burst->last_ack_time) {
        current_burst->last_ack_time = ts;
        current_burst->ack_num++;
        current_burst->cumulative_ack_rtt += ts - pkt->sent_ts;
    }
    current_burst->acked_bytes += pkt->pktlen;

    // If all ACKs for that burst were received, update the Wave CC state
    if (current_burst->acked_bytes != current_burst->send_burst_size) {
        return;
    }

    // Wave ack-dispersion works well with at least 3 acks per burst
    if (current_burst->ack_num < 3) {
        goto FREE_MEMORY;
    }

    // Compute ack train dispersion
    ack_train_disp = 1.0 * ((double)current_burst->last_ack_time -
                                   (double)current_burst->first_ack_time) *
                            (1.0 * (double)current_burst->ack_num /
                            (1.0 * (double)current_burst->ack_num - 1));

    // Compute the EWMA filter parameter
    alpha = 1.0 * ((double)current_burst->pilot_rtt - (double)wave->min_rtt) /
                   (1.0 * (double)current_burst->pilot_rtt);
    wave->alpha = (uint64_t)alpha;

    // Computing the average RTT
    avg_rtt = alpha * (double)wave->avg_rtt +
                     (1.0 - alpha) * (double)current_burst->pilot_rtt;
    wave->avg_rtt = (uint64_t)avg_rtt;

    // Compute delta RTT to check if Adjustment mode should be triggered
    delta_rtt = (double)wave->avg_rtt - (double)wave->min_rtt;

    if (delta_rtt > (double)wave->beta) {
        // Reset Wave State
        wave->tx_time = 200 * NGTCP2_MILLISECONDS;
        wave->min_rtt = UINT64_MAX;
        wave->avg_rtt = cstat->smoothed_rtt;
        wave->alpha = 0;
        wave->ack_count = 0;
        wave->ack_data = 0;

        // Update CC pacing data
        cstat->pacing_interval_m = (wave->tx_time / wave->default_burst_size) << 10;;

        // Discard all previous burst info and free memory
        current_burst = wave->bursts;
        while (current_burst != NULL) {
            // Iterate over the burst list and delete elements
            if (current_burst->next == NULL) {
                free(current_burst);
                break;
            } else {
                current_burst = current_burst->next;
                free(current_burst->previous);
            }
        }
        wave->bursts = (burst_stats *) NULL;
        return;
    }

    // Compute new transmission timer
    tx_timer_calc = (uint64_t) ((ack_train_disp + 0.5 * delta_rtt));
    if (tx_timer_calc < 1 * NGTCP2_MILLISECONDS) {
        wave->tx_time = 1 * NGTCP2_MILLISECONDS;
    } else {
        wave->tx_time = (tx_timer_calc / 1000000) * 1000000;
    }
    cstat->pacing_interval_m = (wave->tx_time / wave->default_burst_size) << 10;
    ngtcp2_log_info(wave->cc.log, NGTCP2_LOG_EVENT_CCA,
                    "New Wave Tx time=", wave->tx_time);

FREE_MEMORY: ;
    // Update the burst list and free memory
    next_burst_stat = current_burst->next;
    prev_burst_stat = current_burst->previous;
    if (prev_burst_stat == NULL && next_burst_stat == NULL) {
        // Current is the only node
        wave->bursts = NULL;
    } else if (prev_burst_stat == NULL && next_burst_stat != NULL) {
        // Current is the head
        wave->bursts = next_burst_stat;
        next_burst_stat->previous = NULL;
    } else if (prev_burst_stat != NULL && next_burst_stat != NULL) {
        // Current is in the middle
        prev_burst_stat->next = current_burst->next;
        next_burst_stat->previous = current_burst->previous;
    } else if (prev_burst_stat != NULL && next_burst_stat == NULL) {
        // Current is at the end
        prev_burst_stat->next = NULL;
    }
    free(current_burst);
}

void wave_cc_on_pkt_sent(ngtcp2_cc *cc,
                                   ngtcp2_conn_stat *cstat,
                                   const ngtcp2_cc_pkt *pkt) {
    // When a packet is sent, record the send time of the burst which belongs to
    ngtcp2_cc_wave *wave = ngtcp2_struct_of(cc, ngtcp2_cc_wave, cc);
    burst_stats * current_burst = wave->bursts;
    (void)cstat;

    if (current_burst == NULL) {
        // Init the stats if this is the first bust that was sent
        current_burst = malloc(sizeof(burst_stats));
        current_burst->send_time = pkt->sent_ts;
        current_burst->send_burst_size = pkt->pktlen;
        current_burst->first_ack_time = UINT64_MAX;
        current_burst->last_ack_time = 0;
        current_burst->ack_num = 0;
        current_burst->acked_bytes = 0;
        current_burst->cumulative_ack_rtt = 0;
        current_burst->pilot_rtt = UINT64_MAX;
        current_burst->previous = (burst_stats *) NULL; // Head of the List
        current_burst->next = (burst_stats *) NULL;

        wave->bursts = current_burst;
        return;
    }

    while (current_burst != NULL) {
        if (current_burst->send_time == pkt->sent_ts) {
            // If the packet sent is not the first, update related burst stats
            current_burst->send_burst_size += pkt->pktlen;
            break;
        }

        if (current_burst->next != NULL) {
          // Iterate over the burst list
            current_burst = current_burst->next;
        } else {
            // Append a new burst to the list
            current_burst->next = malloc(sizeof(burst_stats));
            current_burst->next->send_time = pkt->sent_ts;
            current_burst->next->send_burst_size = pkt->pktlen;
            current_burst->next->first_ack_time = UINT64_MAX;
            current_burst->next->last_ack_time = 0;
            current_burst->next->ack_num = 0;
            current_burst->next->acked_bytes = 0;
            current_burst->next->cumulative_ack_rtt = 0;
            current_burst->next->pilot_rtt = UINT64_MAX;
            current_burst->next->previous = current_burst;
            current_burst->next->next = (burst_stats *) NULL;

            break;
        }
    }
}


void wave_cc_on_ack_recv(ngtcp2_cc *cc,
                                   ngtcp2_conn_stat *cstat,
                                   const ngtcp2_cc_ack *ack,
                                   ngtcp2_tstamp ts) {
    // When an ack or a burst of ack is received, update the minimum RTT
    // and trigger the recovery mode
    ngtcp2_cc_wave *wave = ngtcp2_struct_of(cc, ngtcp2_cc_wave, cc);
    uint64_t tx_timer_calc;
    double alpha;
    double avg_rtt;
    double d_rtt;
    double delta_rtt;
    (void)ts;

    if (ack->bytes_delivered <= 0 || ack->rtt > 10 * wave->min_rtt) {
        // Why an ACK should deliver 0 (or lower) bytes? Ignoring.
        // Discard also unreliable (too old) samples
        return;
    }

    // Update min rtt
    if (ack->rtt < wave->min_rtt) {
        wave->min_rtt = ack->rtt;
    }

    // Trigger the Recovery Mode when acks come in burst
    if (ack->bytes_delivered >= wave->default_burst_size) {
        // Update average rtt
        //wave->avg_rtt = (uint64_t) ((1.0 - 0.2) * wave->avg_rtt + 0.2 * ack->rtt);

        // Compute the EWMA filter parameter
        alpha = 1.0 * ((double)ack->rtt - (double)wave->min_rtt) /
                       (1.0 * (double)ack->rtt);
        wave->alpha = (uint64_t)alpha;

        // Computing the average RTT
        avg_rtt = alpha * (double)wave->avg_rtt +
                         (1.0 - alpha) * (double)ack->rtt;
        wave->avg_rtt = (uint64_t)avg_rtt;

        d_rtt = (double)(ack->rtt - wave->min_rtt);
        delta_rtt = (double)(wave->avg_rtt - wave->min_rtt);

        if (delta_rtt > (double)wave->beta) {
            // Reset Wave State
            wave->tx_time = 200 * NGTCP2_MILLISECONDS;
            wave->min_rtt = UINT64_MAX;
            wave->avg_rtt = cstat->smoothed_rtt;
            wave->alpha = 0;
            wave->ack_count = 0;
            wave->ack_data = 0;

            // Update CC pacing data
            cstat->pacing_interval_m = (wave->tx_time / wave->default_burst_size) << 10;;
            return;
        }
        // Recovery Mode Transmission Timer
        tx_timer_calc = (uint64_t) (0.4 * (double)wave->tx_time + 0.2 * d_rtt);
        if (tx_timer_calc < 1 * NGTCP2_MILLISECONDS) {
            wave->tx_time = 1 * NGTCP2_MILLISECONDS;
        } else {
            wave->tx_time = (tx_timer_calc / 1000000) * 1000000;
        }
        cstat->pacing_interval_m = (wave->tx_time / wave->default_burst_size) << 10;

        ngtcp2_log_info(wave->cc.log, NGTCP2_LOG_EVENT_CCA,
                        "New Wave Tx time=", wave->tx_time);
    }
}

void wave_cc_reset(ngtcp2_cc *cc, ngtcp2_conn_stat *cstat,
                             ngtcp2_tstamp ts) {
    ngtcp2_cc_wave *wave = ngtcp2_struct_of(cc, ngtcp2_cc_wave, cc);
    (void)ts;
    wave_reset(wave, cstat);
}

void ngtcp2_cc_wave_init(ngtcp2_cc_wave *wave, ngtcp2_log *log,
                         ngtcp2_conn_stat *cstat) {
    memset(wave, 0, sizeof(*wave));

    wave->cc.log = log;
    wave->cc.on_pkt_acked = wave_cc_on_pkt_acked;
    wave->cc.on_pkt_sent = wave_cc_on_pkt_sent;
    wave->cc.on_ack_recv = wave_cc_on_ack_recv;
    wave->cc.reset = wave_cc_reset;

    wave_reset(wave, cstat);
}
