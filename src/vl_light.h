#pragma once

#include <vector>
#include <string>

#include "vl_messages.h"

typedef std::vector<vive_headset_lighthouse_pulse2> lighthouse_reports;

typedef std::function<bool(vive_headset_lighthouse_pulse2)> sample_filter;


struct vl_light_sample_group {
    char channel;
    char sweep;
    int epoch;
    int skip;
    int rotor;
    int seq;
    lighthouse_reports samples;
};


// Decode Vive Lighthouse sync pulses
//
//   [station, sweep, databit] = decode_pulse(S)
//
//   S        Struct of samples for one pulse.
//
// S must contain only pulse samples for the same pulse, which means
// their timestamps must be close. The format of S is the same as
// load_dump() returns.
//
//   skip     Nx1 vector: 0 or 1; -1 for error in decoding.
//   sweep    Nx1 vector: 0 for horizontal sweep, 1 for vertical sweep
//   databit  Nx1 vector: one bit of over-the-light data stream
//
// Reference: https://github.com/nairol/LighthouseRedox/blob/master/docs/Light//20Emissions.md

/*
void decode_pulse(int S) {
//function [skip, sweep, databit] = decode_pulse(S);
    ndups = length(S.sensor_id) - length(unique(S.sensor_id));

    // not fatal
    if (ndups != 0)
        printf('//d duplicate sensors\n', ndups);

    // robust against outlier samples
    pulselen = median(S.length);

    key = [
        2500, -1, -1, -1;
        3000, 0, 0, 0;
        3500, 0, 1, 0;
        4000, 0, 0, 1;
        4500, 0, 1, 1;
        5000, 1, 0, 0;
        5500, 1, 1, 0;
        6000, 1, 0, 1;
        6500, 1, 1, 1;
        7000, -1, -1, -1;
    ];

    // classify to the closest key value
    [mindiff,  ind] = min(abs(key(:,1) - pulselen));

    skip = key(ind, 2);
    sweep = key(ind, 3);
    databit = key(ind, 4);
}
*/

// Recognize the Vive Lighthouse channel from a pulse
//
// ch = channel_detect(last_pulse_time, new_pulse_time);
//
//	last_pulse_time	timestamp of the previous pulse
//	new_pulse_time	timestamp of the pulse to be identified
//
//	ch		Either 'A', 'B', 'C', or 'error' for unrecognized.
//
// This function requires the globals 'tick_rate' and 'rotor_rps'
// to be set.

/*
char channel_detect(last_pulse_time, new_pulse_time) {
//function ch = channel_detect(last_pulse_time, new_pulse_time);

    global tick_rate;
    global rotor_rps;

    // Two sweeps per rotation
    period = tick_rate / rotor_rps / 2;
    // ??
    space = 20e3;

    dt = new_pulse_time - last_pulse_time;
    if (abs(dt - period) < 4000)
        ch = 'A';
    else if (abs(dt - (period - space)) < 4000)
        ch = 'B';
    else if (abs(dt - space) < 4000)
        ch = 'C';
    else
        ch = 'error';
    return ch;
}
*/

// Get a sub-set of a struct of arrays
//
// R = subset(S, elms)
//
//	S	A struct with each field being an array of the same size.
//	elms	A logical mask or a vector of indices to choose elements.
//
//	R	A struct identical to S, but with only the chosen
//		elements included in the member vectors.

/*
void subset(S, elms) {
    //R = struct();
    struct R;
    for ([val, key] = S)
        R.(key) = val(elms);
}
*/




// Convert absolute sample ticks to relative angle ticks
//
// angle_ticks = ticks_sample_to_angle(samples, epoch)
//
//	samples		A struct with fields 'timestamp', 'length',
//			as defined by load_dump(), but for a single
//			sweep of a single station.
//	epoch		The sweep starting timestamp.
//
//	angle_ticks	Each sample converted to angle-ticks.
//
// The timestamp is adjusted to point to the middle of the lit up
// period. Assuming the laser line cross section power profile is
// symmetric, this will remove the differences from variations in laser
// line width at the sensor. Then epoch is subtracted to produce
// the time delta directly proportional to the angle.
/*
void ticks_sample_to_angle(samples, epoch) {
    angle_ticks = samples.timestamp + samples.length ./ 2 - epoch;
    return angle_ticks;
}
*/

// Convert tick-delta to millimeters
//
// mm = ticks_to_mm(ticks, dist)
//
//	ticks	delta in Lighthouse tick units
//	dist	distance between lighthouse station and sensors,
//		in meters
//
//	mm	The position delta in millimeters.
/*
void ticks_to_mm(ticks, dist) {
    global tick_rate;
    global rotor_rps;

    angle = ticks / tick_rate * rotor_rps * 2 * pi;
    mm = tan(angle) * dist * 1000;
    return mm;
}
*/

// Update pulse detection state machine
//
// [last_pulse, current_sweep, seq, out_pulse] = ...
//	update_pulse_state(pulse_samples, last_pulse, current_sweep, seq)
//
// Inputs:
//	pulse_samples	A struct as defined in load_dump().
//
// Input-outputs (state to be updated):
//	last_pulse	epoch of the previous pulse
//	current_sweep	information of the current sweep, a struct
//			with fields:
//			- epoch: the pulse begin timestamp
//			- channel: 'A', 'B', or 'C'
//			- skip: the skip bit
//			- sweep: 'H' or 'V'
//			Can be empty due to detection errors.
//	seq		scanning cycle sequence number
//
// Outputs:
//	out_pulse	Only set for valid non-skipped pulses. A struct
//			with fields:
//			- channel: 'A', 'B', or 'C'
//			- sweep: 'H' or 'V' (old name for 'rotor')
//			- seq: scanning cycle sequence number
//			- epoch: the base timestamp indicating the zero raw angle
//			- samples: the subset of D with the samples indicating this pulse



std::tuple<int, vl_light_sample_group, int, vl_light_sample_group> update_pulse_state(
        lighthouse_reports pulse_samples, int last_pulse, vl_light_sample_group current_sweep, int seq) {

    //function [last_pulse, current_sweep, seq, out_pulse] = ...
    //		update_pulse_state(pulse_samples, last_pulse, current_sweep, seq);

    vl_light_sample_group out_pulse;

    //vl_pulse pulse = process_pulse_set(pulse_samples, last_pulse);
    vl_light_sample_group pulse;
    last_pulse = pulse.epoch;

    // unsigned n_sensors = length(pulse_samples.timestamp);
    unsigned n_sensors = pulse_samples.size();

    if (pulse.channel == 'e' || n_sensors < 5) {
        // Invalid pulse, reset state since cannot know which
        // sweep following sweep samples would belong in.
        //current_sweep = [];
        //out_pulse = [];

        return {last_pulse, current_sweep, seq, out_pulse};
    }

    if (!pulse.skip) {
        // Valid pulse, but skip flag set.
        // Leave current_sweep as is.
        //out_pulse = [];
        //else {
        // Valid pulse starting a new sweep.

        // Count complete sweep sequences. For mode A, that is horz+vert.
        // For mode B+C, that is horz+vert for both stations.
        if ((pulse.channel == 'A' || pulse.channel == 'B') && pulse.sweep == 'H')
            seq += 1;

        printf("Start sweep seq %d: ch %c, sweep %c, pulse detected by %d sensors\n", seq, pulse.channel, pulse.sweep, n_sensors);

        current_sweep = pulse;

        out_pulse = {
            /*channel*/ pulse.channel,
            /*sweep*/ pulse.sweep,
            /*epoch*/ pulse.epoch,
            /*skip*/ 0,
            /*rotor*/ 0,
            /*seq*/ seq,
            /*samples*/ pulse_samples
        };

    }

    return {last_pulse, current_sweep, seq, out_pulse};
}



/*
void process_pulse_set(S, last_pulse) {
    [skip, sweepi, databit] = decode_pulse(S);

    // Pick median as the pulse timestamp.

    // Pulse lit duration varies according to the data bit sent
    // over-the-light, and the only correct point is the beginning
    // of the lit period.

    // It seems the starting times for lit periods do not all align
    // exactly, there can be few sensors that activate slightly late.
    // Their stopping time looks to me much better in sync, but
    // let's try simply the starting time concensus.
    t = median(S.timestamp);

    ch = channel_detect(last_pulse, t);

    key = [ 'e', 'H', 'V' ];
    sweep = key(sweepi + 2);

    if (length(S.timestamp) < 5)
        printf('Warning: channel //s pulse at //d (len //d, samples //d): skip //d, sweep //c, data //d\n', ...
            ch, t, median(S.length), length(S.timestamp), skip, sweep, databit);

    // no use for databit here
    P = struct('epoch', t, ...
        'channel', ch, ...
        'skip', skip, ...
        'sweep', sweep);
    return P;
}
*/


/*
void collect_readings(station, sweeps) {
    // Collect all readings into a nice data structure
    // x and y angles, and a timestamp (x sweep epoch)
    // array R(sensor_id + 1).x, .y, .t
    R = struct('x', [], 'y', [], 't', []);

    maxseq = max([sweeps.seq]);

    // loop over sequences
    for (i = 1:maxseq;) {
        // choose station and sequence
        mask = [sweeps.channel] == station & [sweeps.seq] == i;

        x_ind = find(mask & [sweeps.rotor] == 'H');
        y_ind = find(mask & [sweeps.rotor] == 'V');

        if (length(x_ind) < 1 || length(y_ind) < 1)
            // Either or both sweeps are empty, ignore.
            break;


        if (length(x_ind) != 1 || length(y_ind) != 1)
            // x_ind, y_ind
            error("Unexpected number of indices, should be just one each");

        x_samples = sweeps(x_ind).samples;
        y_samples = sweeps(y_ind).samples;

        x_ang = ticks_sample_to_angle(x_samples, sweeps(x_ind).epoch);
        y_ang = ticks_sample_to_angle(y_samples, sweeps(y_ind).epoch);

        // loop over sensors ids, only interested in both x and y
        for (s = 0:max(x_samples.sensor_id)) {
            xi = find(x_samples.sensor_id == s);
            yi = find(y_samples.sensor_id == s);

            if (length(xi) > 1 || length(yi) > 1)
                error("Same sensor sampled multiple times??");

            if (length(xi) < 1 || length(yi) < 1)
                continue;

            R(s+1).x(end + 1) = x_ang(xi);
            R(s+1).y(end + 1) = y_ang(yi);

            // Assumes all measurements happened at the same time,
            // which is wrong.
            R(s+1).t(end + 1) = sweeps(x_ind).epoch;
        }
    }
}
*/




lighthouse_reports filter_reports (lighthouse_reports reports, sample_filter filter_fun) {
    lighthouse_reports results;
    for (auto sample : reports)
        if (filter_fun(sample))
            results.push_back(sample);
    return results;
}

// Sanitize Vive light samples
//
// D = sanitize(S)
//
//	S	Struct of samples, see load_dump().
//
//	D	The same struct with bad entries dropped.
//
// This function drops all entries { 0xffffffff, 0xff, 0xffff } as there is
// no known purpose for them.

/*
lighthouse_reports sanitize(const lighthouse_reports& S) {
    return subset(S, !(S.timestamp == 0xffffffff & S.sensor_id == 0xff & S.length == 0xffff));
}
*/

bool is_sample_valid(vive_headset_lighthouse_pulse2 s) {
    return !(s.timestamp == 0xffffffff && s.sensor_id == 0xff && s.length == 0xffff);
}

// Process and classify Lighthouse samples
//
// [sweeps, pulses] = process_lighthouse_samples(D)
//
//	D	A struct as returned from load_dump().
//
//	sweeps	A struct array with the fields:
//		- channel: 'A', 'B', or 'C'
//		- rotor: 'H' or 'V'
//		- seq: scanning cycle sequence number
//		- epoch: the base timestamp indicating the zero raw angle
//		- samples: the subset of D with the samples in this sweep
//
//	pulses	A struct array with the fields:
//		- channel: 'A', 'B', or 'C'
//		- sweep: 'H' or 'V' (old name for 'rotor')
//		- seq: scanning cycle sequence number
//		- epoch: the base timestamp indicating the zero raw angle
//		- samples: the subset of D with the samples indicating this pulse
//
// The struct D should have been sanitized first, see sanitize().
//
// Only the meaningful pulses are returned, i.e. those with the bit skip=false.


lighthouse_reports subset(lighthouse_reports D, std::vector<int> indices) {
    lighthouse_reports results;
    for (auto i : indices)
        results.push_back(D[i]);
    return results;
}

bool isempty(vl_light_sample_group samples) {
    return samples.samples.empty();
}

std::tuple<std::vector<vl_light_sample_group>, std::vector<vl_light_sample_group>> process_lighthouse_samples(lighthouse_reports D) {

    // state for the processing loop
    std::vector<int> pulse_inds;
    std::vector<int> sweep_inds;

    int last_pulse = -1e6;
    int seq = 0;

    vl_light_sample_group current_sweep;
    //pulse_range = [Inf -Inf]; // begin, end timestamp
    std::pair<uint32_t, uint32_t> pulse_range = {UINT_MAX, 0};

    //pulses = struct([]);
    //sweeps = struct([]);

    std::vector<vl_light_sample_group> pulses;
    std::vector<vl_light_sample_group> sweeps;

    // Process all samples
    //for (vive_headset_lighthouse_pulse2 sample : D) {
    for (unsigned i = 0; i < D.size(); i++) {
        vive_headset_lighthouse_pulse2 sample = D[i];
        if (sample.length < 2000) {
            // sweep sample

            if (!pulse_inds.empty()) {
                // process the pulse set
                vl_light_sample_group pulse;
                std::tie(last_pulse, current_sweep, seq, pulse) = update_pulse_state(subset(D, pulse_inds), last_pulse, current_sweep, seq);

                //[last_pulse, current_sweep, seq, pulse] = update_pulse_state(subset(D, pulse_inds), last_pulse, current_sweep, seq);

                pulse_inds.clear();
                pulse_range = {UINT_MAX, 0};
                if (!isempty(pulse))
                    pulses.push_back(pulse);
            }

            if (isempty(current_sweep))
                // do not know which sweep, so skip
                continue;

            // accumulate sweep samples for a single sweep
            sweep_inds.push_back(i);
        } else {
            // pulse sample

            if (!sweep_inds.empty() && !isempty(current_sweep)) {
                // store one sweep

                vl_light_sample_group sweep = {
                    /*channel*/ current_sweep.channel,
                    /*sweep*/ 0,
                    /*epoch*/ current_sweep.epoch,
                    /*skip*/ 0,
                    /*rotor*/ current_sweep.sweep,
                    /*seq*/ seq,
                    /*samples*/ subset(D, sweep_inds)
                };

                sweeps.push_back(sweep);
                sweep_inds.clear();
            }

            // A pulse belongs to the existing set if it overlaps
            // the whole set seen so far.
            if (pulse_inds.empty() || (sample.timestamp <= pulse_range.second && sample.timestamp + sample.length >= pulse_range.first)) {
                // compute the time span of pulses seen so far
                pulse_range = {
                    std::min(pulse_range.first, sample.timestamp),
                    std::max(pulse_range.second, sample.timestamp + sample.length)
                };

                // accumulate a single pulse set
                pulse_inds.push_back(i);
            } else {
                // Otherwise, a new pulse set start immediately after
                // the previous one without any sweep samples in between.

                if (sample.timestamp + sample.length < pulse_range.first)
                    printf("Out of order pulse at index %d\n", i);

                // process the pulse set
                vl_light_sample_group pulse;
                std::tie(last_pulse, current_sweep, seq, pulse) = update_pulse_state(subset(D, pulse_inds), last_pulse, current_sweep, seq);

                pulse_inds.clear();
                pulse_range = {UINT_MAX, 0};
                if (!isempty(pulse))
                    pulses.push_back(pulse);

            }
        }
    }

    return {sweeps, pulses};

} // process_lighthouse_samples()

void vl_light_classify_samples(lighthouse_reports *raw_light_samples) {

    // Take just a little bit for analysis
    // deliberately start middle of a sweep


    lighthouse_reports sanitized_light_samples = filter_reports(*raw_light_samples, &is_sample_valid);

      // std::vector<vive_headset_lighthouse_pulse_report2> D = sanitize(raw_data);
 /*
      // remove timestamp offset
      min_t = min(D.timestamp);
      D.timestamp = D.timestamp - min_t;
*/

    std::vector<vl_light_sample_group> sweeps;
    std::vector<vl_light_sample_group> pulses;
    std::tie(sweeps, pulses) = process_lighthouse_samples(sanitized_light_samples);

            /*
      R_B = collect_readings('B', sweeps);
      R_C = collect_readings('C', sweeps);
      */

    printf("raw: %ld\n", raw_light_samples->size());
    printf("valid: %ld\n", sanitized_light_samples.size());
}
