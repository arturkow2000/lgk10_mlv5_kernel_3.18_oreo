/*
* Copyright(c) 2015 by LG Electronics. Confidential and Proprietary All Rights Reserved. 
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#include <linux/cpu.h>
#include <linux/cpufreq.h>

/* definition of cpu information */
#define LITTLE_CORE_NUMBER	4
#define BIG_CORE_NUMBER	4
#define CORE_NUMBER	(LITTLE_CORE_NUMBER + BIG_CORE_NUMBER)

#define FREQUENCY_DENOMINATOR	1000

#define LITTLE_CORE_0_IDX_FREQUENCY	(1001000 / FREQUENCY_DENOMINATOR)
#define LITTLE_CORE_1_IDX_FREQUENCY	(910000 / FREQUENCY_DENOMINATOR)
#define LITTLE_CORE_2_IDX_FREQUENCY	(819000 / FREQUENCY_DENOMINATOR)
#define LITTLE_CORE_3_IDX_FREQUENCY	(689000 / FREQUENCY_DENOMINATOR)
#define LITTLE_CORE_4_IDX_FREQUENCY	(598000 / FREQUENCY_DENOMINATOR)
#define LITTLE_CORE_5_IDX_FREQUENCY	(494000 / FREQUENCY_DENOMINATOR)
#define LITTLE_CORE_6_IDX_FREQUENCY	(338000 / FREQUENCY_DENOMINATOR)
#define LITTLE_CORE_7_IDX_FREQUENCY	(156000 / FREQUENCY_DENOMINATOR)

#define BIG_CORE_0_IDX_FREQUENCY	(1508000 / FREQUENCY_DENOMINATOR)
#define BIG_CORE_1_IDX_FREQUENCY	(1430000 / FREQUENCY_DENOMINATOR)
#define BIG_CORE_2_IDX_FREQUENCY	(1352000 / FREQUENCY_DENOMINATOR)
#define BIG_CORE_3_IDX_FREQUENCY	(1196000 / FREQUENCY_DENOMINATOR)
#define BIG_CORE_4_IDX_FREQUENCY	(1027000 / FREQUENCY_DENOMINATOR)
#define BIG_CORE_5_IDX_FREQUENCY	(871000 / FREQUENCY_DENOMINATOR)
#define BIG_CORE_6_IDX_FREQUENCY	(663000 / FREQUENCY_DENOMINATOR)
#define BIG_CORE_7_IDX_FREQUENCY	(286000 / FREQUENCY_DENOMINATOR)

#define CORE_LOAD_MAX	100

#define LOAD_NORMALIZATION_FACTOR	10
#define CORE_NORMALIZATION_FACTOR	1000

#define LITTLE_CORE_FREQUENCY_RESERVE_CRITERIA	17
#define BIG_CORE_FREQUENCY_RESERVE_CRITERIA	19

#define LITTLE_CORE_PERFORMANCE_RESERVE_CRITERIA	\
	(LITTLE_CORE_FREQUENCY_RESERVE_CRITERIA * LOAD_NORMALIZATION_FACTOR)
#define BIG_CORE_PERFORMANCE_RESERVE_CRITERIA	\
	(BIG_CORE_FREQUENCY_RESERVE_CRITERIA * LOAD_NORMALIZATION_FACTOR)

/* variable for history buffer */
#define CPU_DATA_BUFFER_SIZE	32

struct cpu_data {
	unsigned int little_core_load;
	unsigned int big_core_load;
	unsigned int online_little_core_num;
	unsigned int online_big_core_num;
	unsigned int cpu_freq[CORE_NUMBER];

	unsigned int little_core_load_average;
	unsigned int big_core_load_average;
	unsigned int online_little_core_num_average;
	unsigned int online_big_core_num_average;
	unsigned int little_cpu_freq_average;
	unsigned int big_cpu_freq_average;
};

static struct cpu_data cpu_data_buffer[CPU_DATA_BUFFER_SIZE];
static unsigned int current_cpu_data_index;
static struct cpu_data* nn_cpu_data;

#define NN_OUTPUT_LITTLE_CPU_DATA	0
#define NN_OUTPUT_BIG_CPU_DATA	0

void get_cpu_data(unsigned int load)
{
	struct cpu_data* current_cpu_data;
	int i;
	int index;
	int count;
	int little_calc_finish_flag;
	int big_calc_finish_flag;

	nn_cpu_data = current_cpu_data = &cpu_data_buffer[current_cpu_data_index];

	current_cpu_data->online_little_core_num = 0;
	current_cpu_data->online_big_core_num = 0;

	for (i = 0; i < CORE_NUMBER; ++i) {
		if (cpu_online(i)) {
			if (i < LITTLE_CORE_NUMBER)
				current_cpu_data->online_little_core_num += CORE_NORMALIZATION_FACTOR;
			else
				current_cpu_data->online_big_core_num += CORE_NORMALIZATION_FACTOR;

			current_cpu_data->cpu_freq[i] = cpufreq_quick_get(i) / FREQUENCY_DENOMINATOR;
		} else {
			current_cpu_data->cpu_freq[i] = 0;
		}
	}

	current_cpu_data->little_core_load = load * LOAD_NORMALIZATION_FACTOR *
		current_cpu_data->online_little_core_num /
		(current_cpu_data->online_little_core_num + current_cpu_data->online_big_core_num);

	current_cpu_data->big_core_load = load * LOAD_NORMALIZATION_FACTOR *
		current_cpu_data->online_big_core_num /
		(current_cpu_data->online_little_core_num + current_cpu_data->online_big_core_num);

	/* calculate average */
	nn_cpu_data->little_core_load_average = 0;
	nn_cpu_data->big_core_load_average = 0;
	nn_cpu_data->online_little_core_num_average = 0;
	nn_cpu_data->online_big_core_num_average = 0;
	nn_cpu_data->little_cpu_freq_average = 0;
	nn_cpu_data->big_cpu_freq_average = 0;

	little_calc_finish_flag = 0;
	big_calc_finish_flag = 0;

	count = 0;

	index = current_cpu_data_index;

	do {
		current_cpu_data = &cpu_data_buffer[index];

		if (little_calc_finish_flag == 0) {
			if (current_cpu_data->online_little_core_num == 0) {
				if (count == 0) {
					nn_cpu_data->little_core_load_average = 0;
					nn_cpu_data->online_little_core_num_average = 0;
					nn_cpu_data->little_cpu_freq_average = 0;
				} else {
					nn_cpu_data->little_core_load_average /= count;
					nn_cpu_data->online_little_core_num_average /= count;
					nn_cpu_data->little_cpu_freq_average /= count;
				}

				little_calc_finish_flag = 1;
				if (big_calc_finish_flag == 1)
					break;
			} else {
				nn_cpu_data->little_core_load_average += current_cpu_data->little_core_load;
				nn_cpu_data->online_little_core_num_average += current_cpu_data->online_little_core_num;
				nn_cpu_data->little_cpu_freq_average += current_cpu_data->cpu_freq[0];
			}
		}

		if (big_calc_finish_flag == 0) {
			if (current_cpu_data->online_big_core_num == 0) {
				if (count == 0) {
					nn_cpu_data->big_core_load_average = 0;
					nn_cpu_data->online_big_core_num_average = 0;
					nn_cpu_data->big_cpu_freq_average = 0;
				} else {
					nn_cpu_data->big_core_load_average /= count;
					nn_cpu_data->online_big_core_num_average /= count;
					nn_cpu_data->big_cpu_freq_average /= count;
				}

				big_calc_finish_flag = 1;
				if (little_calc_finish_flag == 1)
					break;
			} else {
				nn_cpu_data->big_core_load_average += current_cpu_data->big_core_load;
				nn_cpu_data->online_big_core_num_average += current_cpu_data->online_big_core_num;
				nn_cpu_data->big_cpu_freq_average += current_cpu_data->cpu_freq[LITTLE_CORE_NUMBER];
			}
		}

		++count;

		if (index == 0)
			index = CPU_DATA_BUFFER_SIZE - 1;
		else
			--index;
	} while (index != current_cpu_data_index);

	if (little_calc_finish_flag == 0) {
		nn_cpu_data->little_core_load_average /= count;
		nn_cpu_data->online_little_core_num_average /= count;
		nn_cpu_data->little_cpu_freq_average /= count;
	}

	if (big_calc_finish_flag == 0) {
		nn_cpu_data->big_core_load_average /= count;
		nn_cpu_data->online_big_core_num_average /= count;
		nn_cpu_data->big_cpu_freq_average /= count;
	}

#if NN_OUTPUT_LITTLE_CPU_DATA
	pr_err("[hanjunyeong][lit] input: %d,%d,%d,%d,%d,%d,%d,%d,%d, count: %d\n",
		nn_cpu_data->little_core_load_average, nn_cpu_data->online_little_core_num_average, nn_cpu_data->little_cpu_freq_average,
		nn_cpu_data->little_core_load, nn_cpu_data->online_little_core_num,
		nn_cpu_data->cpu_freq[0], nn_cpu_data->cpu_freq[1], nn_cpu_data->cpu_freq[2], nn_cpu_data->cpu_freq[3],
		count);
#endif
#if NN_OUTPUT_BIG_CPU_DATA
	pr_err("[hanjunyeong][big] input: %d,%d,%d,%d,%d,%d,%d,%d,%d, count: %d\n",
		nn_cpu_data->big_core_load_average, nn_cpu_data->online_big_core_num_average, nn_cpu_data->big_cpu_freq_average,
		nn_cpu_data->big_core_load, nn_cpu_data->online_big_core_num,
		nn_cpu_data->cpu_freq[4], nn_cpu_data->cpu_freq[5], nn_cpu_data->cpu_freq[6], nn_cpu_data->cpu_freq[7],
		count);
#endif

	if (current_cpu_data_index < CPU_DATA_BUFFER_SIZE - 1)
		++current_cpu_data_index;
	else
		current_cpu_data_index = 0;
}

/* neural networks */
#define NN_INPUT_NEURON_NUMBER	18
#define NN_HIDDEN1_NEURON_NUMBER	4
#define NN_OUTPUT_NEURON_NUMBER	2

int nn_input_neuron[NN_INPUT_NEURON_NUMBER];
int nn_hidden0_neuron[NN_HIDDEN1_NEURON_NUMBER];
int nn_output_neuron[NN_OUTPUT_NEURON_NUMBER];

int nn_input_hidden0_synapse[NN_INPUT_NEURON_NUMBER][NN_HIDDEN1_NEURON_NUMBER];
int nn_hidden0_output_synapse[NN_HIDDEN1_NEURON_NUMBER][NN_OUTPUT_NEURON_NUMBER];

#define SYNAPSE_DENOMINATOR	1000

#define NN_INPUT_HIDDEN0_SYNAPSE_INIT	(400)
#define NN_INPUT_HIDDEN0_SYNAPSE_MIN	(200)
#define NN_INPUT_HIDDEN0_SYNAPSE_MAX	(1300)

#define NN_HIDDEN0_OUTPUT_0_0_SYNAPSE_INIT	(800)
#define NN_HIDDEN0_OUTPUT_1_0_SYNAPSE_INIT	(200)
#define NN_HIDDEN0_OUTPUT_2_1_SYNAPSE_INIT	(800)
#define NN_HIDDEN0_OUTPUT_3_1_SYNAPSE_INIT	(200)
#define NN_HIDDEN0_OUTPUT_X_X_SYNAPSE_MIN	(100)
#define NN_HIDDEN0_OUTPUT_X_X_SYNAPSE_MAX	(900)

static int neural_networks_init_flag;

#define CPU_DATA_POPULATION	2
#define CPU_DATA_SAMPLE	1
static int cpu_data_number;

#define NN_OUTPUT_INFOMATION	0

static void initialize_neural_networks(void)
{
	int i;
	int j;

	nn_cpu_data = &cpu_data_buffer[current_cpu_data_index];

	/* initialize input to hidden0 synapse */
	/* little cpu loads */
	for (i = 0; i < NN_INPUT_NEURON_NUMBER; ++i) {
		for (j = 0; j < NN_HIDDEN1_NEURON_NUMBER; ++j)
			nn_input_hidden0_synapse[i][j] = NN_INPUT_HIDDEN0_SYNAPSE_INIT;
	}

	/* initialize hidden0 to output synapse */
	nn_hidden0_output_synapse[0][0] = NN_HIDDEN0_OUTPUT_0_0_SYNAPSE_INIT;
	nn_hidden0_output_synapse[1][0] = NN_HIDDEN0_OUTPUT_1_0_SYNAPSE_INIT;
	nn_hidden0_output_synapse[2][1] = NN_HIDDEN0_OUTPUT_2_1_SYNAPSE_INIT;
	nn_hidden0_output_synapse[3][1] = NN_HIDDEN0_OUTPUT_3_1_SYNAPSE_INIT;

	cpu_data_number = 0;
}

static void set_synapse_weight(int* weight, int differential, int min, int max)
{
	*weight += differential;

	if (max < *weight)
		*weight = max;
	else if (*weight < min)
		*weight = min;
}

void training_neural_networks(int little_online_cores, int big_online_cores)
{
	int real_power_average;
	int load_average;
	int real_power;
	int load;
	int differential_value;

	int i;

	if (neural_networks_init_flag == 0) {
		initialize_neural_networks();
		neural_networks_init_flag = 1;
	}
	
	++cpu_data_number;
	if (cpu_data_number != CPU_DATA_POPULATION)
		return;

	cpu_data_number = 0;

	if (little_online_cores < 2 && big_online_cores < 2)
		return;

	if (0 < little_online_cores) {
		if (nn_cpu_data->little_core_load_average == 0)
			return;

		real_power_average = nn_cpu_data->online_little_core_num_average *
			nn_cpu_data->little_cpu_freq_average / LITTLE_CORE_0_IDX_FREQUENCY;
		load_average = nn_cpu_data->little_core_load_average;

		if (LITTLE_CORE_PERFORMANCE_RESERVE_CRITERIA < real_power_average * 100 / load_average) {
			differential_value = -(nn_cpu_data->little_core_load_average / 50);
			set_synapse_weight(&nn_input_hidden0_synapse[0][0], differential_value,
				NN_INPUT_HIDDEN0_SYNAPSE_MIN, NN_INPUT_HIDDEN0_SYNAPSE_MAX);

			differential_value = -(nn_cpu_data->online_little_core_num_average / 100);
			set_synapse_weight(&nn_input_hidden0_synapse[1][0], differential_value,
				NN_INPUT_HIDDEN0_SYNAPSE_MIN, NN_INPUT_HIDDEN0_SYNAPSE_MAX);

			differential_value = -(nn_cpu_data->little_cpu_freq_average / 50);
			set_synapse_weight(&nn_input_hidden0_synapse[2][0], differential_value,
				NN_INPUT_HIDDEN0_SYNAPSE_MIN, NN_INPUT_HIDDEN0_SYNAPSE_MAX);
		}

		if (nn_cpu_data->little_core_load == 0)
			return;

		real_power = nn_cpu_data->online_little_core_num *
			nn_cpu_data->cpu_freq[0] / LITTLE_CORE_0_IDX_FREQUENCY;
		load = nn_cpu_data->little_core_load;

		if (LITTLE_CORE_PERFORMANCE_RESERVE_CRITERIA < real_power * 100 / load) {
			differential_value = -1;
			for (i = 3; i < 5 + little_online_cores; ++i)
				set_synapse_weight(&nn_input_hidden0_synapse[i][1], differential_value,
					NN_INPUT_HIDDEN0_SYNAPSE_MIN, NN_INPUT_HIDDEN0_SYNAPSE_MAX);
		}

		if (LITTLE_CORE_3_IDX_FREQUENCY <= nn_cpu_data->little_cpu_freq_average) {
			differential_value = nn_cpu_data->little_cpu_freq_average / 100;
			for (i = 0; i < 3; ++i)
				set_synapse_weight(&nn_input_hidden0_synapse[i][0], differential_value,
					NN_INPUT_HIDDEN0_SYNAPSE_MIN, NN_INPUT_HIDDEN0_SYNAPSE_MAX);
		}

		if (LITTLE_CORE_2_IDX_FREQUENCY <= nn_cpu_data->cpu_freq[0]) {
			differential_value = 1;
			for (i = 3; i < 5 + little_online_cores; ++i)
				set_synapse_weight(&nn_input_hidden0_synapse[i][1], differential_value,
					NN_INPUT_HIDDEN0_SYNAPSE_MIN, NN_INPUT_HIDDEN0_SYNAPSE_MAX);
		}
	}

	if (0 < big_online_cores) {
		if (nn_cpu_data->big_core_load_average == 0)
			return;

		real_power_average = nn_cpu_data->online_big_core_num_average *
			nn_cpu_data->big_cpu_freq_average / BIG_CORE_0_IDX_FREQUENCY;
		load_average = nn_cpu_data->big_core_load_average;

		if (BIG_CORE_PERFORMANCE_RESERVE_CRITERIA < real_power_average * 100 / load_average) {
			differential_value = -(nn_cpu_data->big_core_load_average / 50);
			set_synapse_weight(&nn_input_hidden0_synapse[9][2], differential_value,
				NN_INPUT_HIDDEN0_SYNAPSE_MIN, NN_INPUT_HIDDEN0_SYNAPSE_MAX);

			differential_value = -(nn_cpu_data->online_big_core_num_average / 100);
			set_synapse_weight(&nn_input_hidden0_synapse[10][2], differential_value,
				NN_INPUT_HIDDEN0_SYNAPSE_MIN, NN_INPUT_HIDDEN0_SYNAPSE_MAX);

			differential_value = -(nn_cpu_data->big_cpu_freq_average / 50);
			set_synapse_weight(&nn_input_hidden0_synapse[11][2], differential_value,
				NN_INPUT_HIDDEN0_SYNAPSE_MIN, NN_INPUT_HIDDEN0_SYNAPSE_MAX);
		}

		if (nn_cpu_data->big_core_load == 0)
			return;

		real_power = nn_cpu_data->online_big_core_num *
			nn_cpu_data->cpu_freq[LITTLE_CORE_NUMBER] / LITTLE_CORE_0_IDX_FREQUENCY;
		load = nn_cpu_data->big_core_load;

		if (BIG_CORE_PERFORMANCE_RESERVE_CRITERIA < real_power * 100 / load) {
			differential_value = -1;
			for (i = 12; i < 14 + big_online_cores; ++i)
				set_synapse_weight(&nn_input_hidden0_synapse[i][3], differential_value,
					NN_INPUT_HIDDEN0_SYNAPSE_MIN, NN_INPUT_HIDDEN0_SYNAPSE_MAX);
		}

		if (BIG_CORE_3_IDX_FREQUENCY <= nn_cpu_data->big_cpu_freq_average) {
			differential_value = nn_cpu_data->big_cpu_freq_average / 100;
			for (i = 9; i < 12; ++i)
				set_synapse_weight(&nn_input_hidden0_synapse[i][2], differential_value,
					NN_INPUT_HIDDEN0_SYNAPSE_MIN, NN_INPUT_HIDDEN0_SYNAPSE_MAX);
		}

		if (BIG_CORE_2_IDX_FREQUENCY <= nn_cpu_data->cpu_freq[LITTLE_CORE_NUMBER]) {
			differential_value = 1;
			for (i = 12; i < 14 + big_online_cores; ++i)
				set_synapse_weight(&nn_input_hidden0_synapse[i][3], differential_value,
					NN_INPUT_HIDDEN0_SYNAPSE_MIN, NN_INPUT_HIDDEN0_SYNAPSE_MAX);
		}
	}

#if NN_OUTPUT_INFOMATION
	pr_err("[sched_nn][lit] nn_input_hidden0_synapse: (%d,%d,%d) (%d,%d,%d,%d,%d,%d) (%d,%d)\n",
		nn_input_hidden0_synapse[0][0], nn_input_hidden0_synapse[1][0], nn_input_hidden0_synapse[2][0],
		nn_input_hidden0_synapse[3][1], nn_input_hidden0_synapse[4][1], nn_input_hidden0_synapse[5][1],
		nn_input_hidden0_synapse[6][1], nn_input_hidden0_synapse[7][1], nn_input_hidden0_synapse[8][1],
		nn_hidden0_output_synapse[0][0], nn_hidden0_output_synapse[1][0]);

	pr_err("[sched_nn][big] nn_input_hidden0_synapse: (%d,%d,%d) (%d,%d,%d,%d,%d,%d) (%d %d)\n",
		nn_input_hidden0_synapse[9][2], nn_input_hidden0_synapse[10][2], nn_input_hidden0_synapse[11][2],
		nn_input_hidden0_synapse[12][3], nn_input_hidden0_synapse[13][3], nn_input_hidden0_synapse[14][3],
		nn_input_hidden0_synapse[15][3], nn_input_hidden0_synapse[16][3], nn_input_hidden0_synapse[17][3],
		nn_hidden0_output_synapse[2][1], nn_hidden0_output_synapse[3][1]);
#endif
}

int run_neural_networks(int little_online_cores, int big_online_cores)
{
	int i;

	if (neural_networks_init_flag == 0) {
		initialize_neural_networks();
		neural_networks_init_flag = 1;
	}

	/* set input neuron */
	/* little core info average */
	nn_input_neuron[0] = nn_cpu_data->little_core_load_average;
	nn_input_neuron[1] = nn_cpu_data->online_little_core_num_average;
	nn_input_neuron[2] = nn_cpu_data->little_cpu_freq_average;

	/* little core info */
	nn_input_neuron[3] = nn_cpu_data->little_core_load;
	nn_input_neuron[4] = nn_cpu_data->online_little_core_num;
	nn_input_neuron[5] = nn_cpu_data->cpu_freq[0];
	nn_input_neuron[6] = nn_cpu_data->cpu_freq[1];
	nn_input_neuron[7] = nn_cpu_data->cpu_freq[2];
	nn_input_neuron[8] = nn_cpu_data->cpu_freq[3];

	/* big core info average */
	nn_input_neuron[9] = nn_cpu_data->big_core_load_average;
	nn_input_neuron[10] = nn_cpu_data->online_big_core_num_average;
	nn_input_neuron[11] = nn_cpu_data->big_cpu_freq_average;

	/* big core info */
	nn_input_neuron[12] = nn_cpu_data->big_core_load;
	nn_input_neuron[13] = nn_cpu_data->online_big_core_num;
	nn_input_neuron[14] = nn_cpu_data->cpu_freq[4];
	nn_input_neuron[15] = nn_cpu_data->cpu_freq[5];
	nn_input_neuron[16] = nn_cpu_data->cpu_freq[6];
	nn_input_neuron[17] = nn_cpu_data->cpu_freq[7];

	/* set hidden0 neuron */
	for (i = 0; i < NN_HIDDEN1_NEURON_NUMBER; ++i)
		nn_hidden0_neuron[i] = 0;

	/* set output neuron */
	for (i = 0; i < NN_OUTPUT_NEURON_NUMBER; ++i)
		nn_output_neuron[i] = 0;

	/* run neural networks */
	for (i = 0; i < 3; ++i)
		nn_hidden0_neuron[0] += nn_input_neuron[i] * nn_input_hidden0_synapse[i][0];

	nn_hidden0_neuron[0] /= SYNAPSE_DENOMINATOR;

	for (i = 3; i < 9; ++i)
		nn_hidden0_neuron[1] += nn_input_neuron[i] * nn_input_hidden0_synapse[i][1];

	nn_hidden0_neuron[1] /= SYNAPSE_DENOMINATOR;

	for (i = 9; i < 12; ++i)
		nn_hidden0_neuron[2] += nn_input_neuron[i] * nn_input_hidden0_synapse[i][2];

	nn_hidden0_neuron[2] /= SYNAPSE_DENOMINATOR;

	for (i = 12; i < 18; ++i)
		nn_hidden0_neuron[3] += nn_input_neuron[i] * nn_input_hidden0_synapse[i][3];

	nn_hidden0_neuron[3] /= SYNAPSE_DENOMINATOR;

	for (i = 0; i < 2; ++i)
		nn_output_neuron[0] += nn_hidden0_neuron[i] * nn_hidden0_output_synapse[i][0];

	nn_output_neuron[0] /= SYNAPSE_DENOMINATOR;

	for (i = 2; i < NN_HIDDEN1_NEURON_NUMBER; ++i)
		nn_output_neuron[1] += nn_hidden0_neuron[i] * nn_hidden0_output_synapse[i][1];

	nn_output_neuron[1] /= SYNAPSE_DENOMINATOR;

#if NN_OUTPUT_INFOMATION
	pr_err("[sched_nn][lit] input: %d,%d,%d,%d,%d,%d,%d,%d,%d, hidden: %d,%d, output: %d\n",
		nn_input_neuron[0], nn_input_neuron[1], nn_input_neuron[2], nn_input_neuron[3],
		nn_input_neuron[4], nn_input_neuron[5], nn_input_neuron[6], nn_input_neuron[7],
		nn_input_neuron[8], nn_hidden0_neuron[0], nn_hidden0_neuron[1], nn_output_neuron[0]);

	pr_err("[sched_nn][big] input: %d,%d,%d,%d,%d,%d,%d,%d,%d, hidden: %d,%d, output: %d\n",
		nn_input_neuron[9], nn_input_neuron[10], nn_input_neuron[11], nn_input_neuron[12],
		nn_input_neuron[13], nn_input_neuron[14], nn_input_neuron[15], nn_input_neuron[16],
		nn_input_neuron[17], nn_hidden0_neuron[2], nn_hidden0_neuron[3], nn_output_neuron[1]);
#endif

	return (nn_output_neuron[0] + nn_output_neuron[1]) / LOAD_NORMALIZATION_FACTOR;
}

