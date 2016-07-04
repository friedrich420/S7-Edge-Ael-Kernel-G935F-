/*		drivers/cpufreq/cpufreq_psychoactive.c
        The "Psychoactive" Governor is primarily based on scary and conservative sources. The initial goal of this driver is to provide a power efficient performance for the S7 and S7 Edge. The full credits belong to the authors of scary governor primarily, but also ondemand, conservative and interactive governors. Psychoactive is merely the result of minor coding/tweaking to scary governor for better performance. The present amateur attempt belongs to Kyri E. Unfortunately I could not find the name of the original author of scary governor.
        
*/

/*
	The following is from the scary governor original description: 

        scary governor based off of conservatives source with some of scary features
        
        For devs - If you're going to port this driver to other devices, make sure to edit the default sleep frequencies & prev frequencies or else you might be going outside your devices hardware limits.
*/

#include <linux/kernel.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/ipa.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/moduleparam.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/ktime.h>
#include <linux/earlysuspend.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/pm_qos.h>
#include <asm/cputime.h>
#include <linux/input.h>

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
#include <soc/samsung/cpufreq.h>
#endif

#ifdef CONFIG_CPU_THERMAL_IPA
#include "cpu_load_metric.h"
#endif

#ifdef CONFIG_PMU_COREMEM_RATIO
#include "pmu_func.h"
#endif

struct cpufreq_psychoactive_cpuinfo {
	struct timer_list cpu_timer;
	struct timer_list cpu_slack_timer;
	spinlock_t load_lock; /* protects the next 4 fields */
	u64 time_in_idle;
	u64 time_in_idle_timestamp;
	u64 cputime_speedadj;
	u64 cputime_speedadj_timestamp;
	u64 last_evaluated_jiffy;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;
	spinlock_t target_freq_lock; /*protects target freq */
	unsigned int target_freq;
	unsigned int floor_freq;
	unsigned int max_freq;
	unsigned int min_freq;
	u64 floor_validate_time;
	u64 local_fvtime; /* per-cpu floor_validate_time */
	u64 hispeed_validate_time; /* cluster hispeed_validate_time */
	u64 local_hvtime; /* per-cpu hispeed_validate_time */
	u64 max_freq_hyst_start_time;
	struct rw_semaphore enable_sem;
	bool reject_notification;
	int governor_enabled;
	struct cpufreq_psychoactive_tunables *cached_tunables;
	int first_cpu;
#ifdef CONFIG_PMU_COREMEM_RATIO
	int region;
	int prev_region;
#endif
};

#define DEF_FREQUENCY_UP_THRESHOLD		(80)
#define DEF_FREQUENCY_DOWN_THRESHOLD		(40)
#ifdef CONFIG_ZEN_INTERACTIVE
#define DEF_SAMPLING_DOWN_FACTOR		(10)
#else
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#endif
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define MICRO_FREQUENCY_UP_THRESHOLD		(95)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)
#define DEF_SMOOTH_UI				(1)
#define DEFAULT_SLEEP_MIN_FREQ                  728000
#define DEFAULT_SLEEP_MAX_FREQ			2600000
#define DEFAULT_FREQ_STEP			(10)

#define TRANSITION_LATENCY_LIMIT	(10 * 1000 * 1000)
#define OPTIMAL_POSITION		(3)
#define TABLE_SIZE			(15)
#define HYSTERESIS			(7)
#define UP_THRESH			(120)

#define MIN_SAMPLING_RATE_RATIO			(2)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)

#define MULTI_MODE  2
#define SINGLE_MODE 1
#define NO_MODE     0
#define DEFAULT_MULTI_ENTER_TIME (4 * DEFAULT_TIMER_RATE)
#define DEFAULT_MULTI_EXIT_TIME (16 * DEFAULT_TIMER_RATE)
#define DEFAULT_SINGLE_ENTER_TIME (8 * DEFAULT_TIMER_RATE)
#define DEFAULT_SINGLE_EXIT_TIME (4 * DEFAULT_TIMER_RATE)
#define DEFAULT_SINGLE_CLUSTER0_MIN_FREQ 0
#define DEFAULT_MULTI_CLUSTER0_MIN_FREQ 0

#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

#define DEFAULT_SLEEP_PREV_FREQ			728000
#define DEFAULT_PREV_MAX			1560000
static unsigned int suspended;
static unsigned int sleep_max_freq=DEFAULT_SLEEP_MAX_FREQ;
static unsigned int sleep_min_freq=DEFAULT_SLEEP_MIN_FREQ;
static unsigned int sleep_prev_freq=DEFAULT_SLEEP_PREV_FREQ;
static unsigned int sleep_prev_max=DEFAULT_PREV_MAX;
static unsigned int min_sampling_rate;

static void do_dbs_timer(struct work_struct *work);

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	unsigned int down_skip;
	unsigned int requested_freq;
	int cpu;
	unsigned int enable:1;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, cs_cpu_dbs_info);

/* Target load.  Lower values result in higher CPU speeds. */
#define DEFAULT_TARGET_LOAD 80
static unsigned int default_target_loads[] = {DEFAULT_TARGET_LOAD};

#define DEFAULT_TIMER_RATE 7000
#define DEFAULT_ABOVE_HISPEED_DELAY DEFAULT_TIMER_RATE
static unsigned int default_above_hispeed_delay[] = {
	DEFAULT_ABOVE_HISPEED_DELAY };

static unsigned int dbs_enable;	/* number of CPUs using this policy */

/*
 * dbs_mutex protects data in dbs_tuners_ins from concurrent changes on
 * different CPUs. It protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(dbs_mutex);
static struct task_struct *up_task;
static struct workqueue_struct *down_wq;
static struct work_struct freq_scale_down_work;
static cpumask_t up_cpumask;
static spinlock_t up_cpumask_lock;
static cpumask_t down_cpumask;
static spinlock_t down_cpumask_lock;

static struct workqueue_struct	*kconservative_wq;

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int sampling_down_factor;
	unsigned int up_threshold;
	unsigned int down_threshold;
	unsigned int ignore_nice;
	unsigned int freq_step;
	unsigned int smooth_ui;
} dbs_tuners_ins = {
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.down_threshold = DEF_FREQUENCY_DOWN_THRESHOLD,
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.ignore_nice = 0,
	.freq_step = DEFAULT_FREQ_STEP,
	.smooth_ui = DEF_SMOOTH_UI,
};

static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
							cputime64_t *wall)
{
	cputime64_t idle_time;
	cputime64_t cur_wall_time;
	cputime64_t busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());
	busy_time = (kcpustat_cpu(cpu).cpustat[CPUTIME_USER] +
			kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM]);

	busy_time = (busy_time + kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ]);
	busy_time = (busy_time + kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ]);
	busy_time = (busy_time + kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL]);
	busy_time = (busy_time + kcpustat_cpu(cpu).cpustat[CPUTIME_NICE]);

	idle_time = (cur_wall_time - busy_time);
	if (wall)
		*wall = (cputime64_t)jiffies_to_usecs(cur_wall_time);

	return (cputime64_t)jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu, cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

/* keep track of frequency transitions */
static int
dbs_cpufreq_notifier(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpu_dbs_info_s *this_dbs_info = &per_cpu(cs_cpu_dbs_info,
							freq->cpu);

	struct cpufreq_policy *policy;

	if (!this_dbs_info->enable)
		return 0;

	policy = this_dbs_info->cur_policy;

	/*
	 * we only care if our internally tracked freq moves outside
	 * the 'valid' ranges of freqency available to us otherwise
	 * we do not change it
	*/
	if (this_dbs_info->requested_freq > policy->max
			|| this_dbs_info->requested_freq < policy->min)
		this_dbs_info->requested_freq = freq->new;

	return 0;
}

static struct notifier_block dbs_cpufreq_notifier_block = {
	.notifier_call = dbs_cpufreq_notifier
};

/************************** sysfs interface ************************/
static ssize_t show_sampling_rate_max(struct cpufreq_policy *policy, char *buf)
{
	printk_once(KERN_INFO "CPUFREQ: conservative sampling_rate_max "
		    "sysfs file is deprecated - used by: %s\n", current->comm);
	return sprintf(buf, "%u\n", -1U);
}

static ssize_t show_sampling_rate_min(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", min_sampling_rate);
}

#define define_one_ro(_name)		\
static struct freq_attr _name =		\
__ATTR(_name, 0444, show_##_name, NULL)

define_one_ro(sampling_rate_max);
define_one_ro(sampling_rate_min);

/* cpufreq_conservative Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct cpufreq_policy *unused, char *buf)				\
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(sampling_down_factor, sampling_down_factor);
show_one(up_threshold, up_threshold);
show_one(down_threshold, down_threshold);
show_one(ignore_nice_load, ignore_nice);
show_one(freq_step, freq_step);

static ssize_t store_sampling_down_factor(struct cpufreq_policy *unused,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;

	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.sampling_down_factor = input;
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_sampling_rate(struct cpufreq_policy *unused,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.sampling_rate = max(input, min_sampling_rate);
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_up_threshold(struct cpufreq_policy *unused,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);

	mutex_lock(&dbs_mutex);
	if (ret != 1 || input > 100 ||
			input <= dbs_tuners_ins.down_threshold) {
		mutex_unlock(&dbs_mutex);
		return -EINVAL;
	}

	dbs_tuners_ins.up_threshold = input;
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_down_threshold(struct cpufreq_policy *unused,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	mutex_lock(&dbs_mutex);
	/* cannot be lower than 11 otherwise freq will not fall */
	if (ret != 1 || input < 11 || input > 100 ||
			input >= dbs_tuners_ins.up_threshold) {
		mutex_unlock(&dbs_mutex);
		return -EINVAL;
	}

	dbs_tuners_ins.down_threshold = input;
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_ignore_nice_load(struct cpufreq_policy *policy,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	mutex_lock(&dbs_mutex);
	if (input == dbs_tuners_ins.ignore_nice) { /* nothing to do */
		mutex_unlock(&dbs_mutex);
		return count;
	}
	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(cs_cpu_dbs_info, j);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_freq_step(struct cpufreq_policy *policy,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 100)
		input = 100;

	/* no need to test here if freq_step is zero as the user might actually
	 * want this, they would be crazy though :) */
	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.freq_step = input;
	mutex_unlock(&dbs_mutex);

	return count;
}

#define define_one_rw(_name) \
static struct freq_attr _name = \
__ATTR(_name, 0644, show_##_name, store_##_name)

define_one_rw(sampling_rate);
define_one_rw(sampling_down_factor);
define_one_rw(up_threshold);
define_one_rw(down_threshold);
define_one_rw(ignore_nice_load);
define_one_rw(freq_step);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_max.attr,
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&sampling_down_factor.attr,
	&up_threshold.attr,
	&down_threshold.attr,
	&ignore_nice_load.attr,
	&freq_step.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "psychoactive",
};

/************************** sysfs end ************************/

static void psychoactive_suspend(int cpu, int suspend)
{
    struct cpu_dbs_info_s *this_psychoactive = &per_cpu(cs_cpu_dbs_info, smp_processor_id());
    struct cpufreq_policy *policy = this_psychoactive->cur_policy;
    unsigned int new_freq;

    if (!this_psychoactive->enable || sleep_max_freq==0)
        return;

    if (suspend) 
    {
            if (policy->min >= sleep_max_freq)
            {
                sleep_prev_freq=policy->min;
                policy->min= sleep_min_freq;
            }
            if (policy->max > sleep_max_freq)
            {
                sleep_prev_max=policy->max;
                policy->max=sleep_max_freq;
            }
        if (policy->cur > sleep_max_freq) 
        {
            new_freq = sleep_max_freq;
            if (new_freq > policy->max)
                new_freq = policy->max;
            if (new_freq < policy->min)
                new_freq = policy->min;
            __cpufreq_driver_target(policy, new_freq,CPUFREQ_RELATION_H);
       }
       
    }
    else
    {
        if (policy->min < sleep_prev_freq)
            policy->min=sleep_prev_freq;
        if (policy->max < sleep_prev_max)
            policy->max=sleep_prev_max;
    }
    
}

static void psychoactive_early_suspend(struct early_suspend *handler) 
{
    int i;
    suspended = 1;
    for_each_online_cpu(i)
    psychoactive_suspend(i,1);
}

static void psychoactive_late_resume(struct early_suspend *handler) 
{
    int i;
    suspended = 0;
    for_each_online_cpu(i)
    psychoactive_suspend(i,0);
}

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int load = 0;
	unsigned int freq_target;

	struct cpufreq_policy *policy;
	unsigned int j;

	policy = this_dbs_info->cur_policy;

	/*
	 * Every sampling_rate, we check, if current idle time is less
	 * than 20% (default), then we try to increase frequency
	 * Every sampling_rate*sampling_down_factor, we check, if current
	 * idle time is more than 80%, then we try to decrease frequency
	 *
	 * Any frequency increase takes it to the maximum frequency.
	 * Frequency reduction happens at minimum steps of
	 * 5% (default) of maximum frequency
	 */

	/* Get Absolute Load */
	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_info_s *j_dbs_info;
		cputime64_t cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;

		j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);

		wall_time = (unsigned int) (cur_wall_time -
				j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int) (cur_idle_time -
				j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		if (dbs_tuners_ins.ignore_nice) {
			cputime64_t cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = (kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
					 j_dbs_info->prev_cpu_nice);
			/*
			 * Assumption: nice time between sampling periods will
			 * be less than 2^32 jiffies for 32 bit sys
			 */
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

			j_dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;
	}

	/*
	 * break out if we 'cannot' reduce the speed as the user might
	 * want freq_step to be zero
	 */
	if (dbs_tuners_ins.freq_step == 0)
		return;

	/* Check for frequency increase */
	if (load > dbs_tuners_ins.up_threshold) 
    {
		this_dbs_info->down_skip = 0;

		/* if we are already at full speed then break out early */
   		if (this_dbs_info->requested_freq == policy->max)
   			return;
   		freq_target = (dbs_tuners_ins.freq_step * policy->max) / 100;
   		/* max freq cannot be less than 100. but who knows.... */
   		if (unlikely(freq_target == 0))
   			freq_target = 5;
    
   		this_dbs_info->requested_freq += freq_target;
   		if (this_dbs_info->requested_freq > policy->max)
   			this_dbs_info->requested_freq = policy->max;

        __cpufreq_driver_target(policy, this_dbs_info->requested_freq,CPUFREQ_RELATION_H);

   		return;
    }

	/*
	 * The optimal frequency is the frequency that is the lowest that
	 * can support the current CPU usage without triggering the up
	 * policy. To be safe, we focus 10 points under the threshold.
	 */
	if (load < (dbs_tuners_ins.down_threshold - 10)) {
		freq_target = (dbs_tuners_ins.freq_step * policy->max) / 100;

		this_dbs_info->requested_freq -= freq_target;
		if (this_dbs_info->requested_freq < policy->min)
			this_dbs_info->requested_freq = policy->min;

		/*
		 * if we cannot reduce the frequency anymore, break out early
		 */
		if (policy->cur == policy->min)
			return;

		__cpufreq_driver_target(policy, this_dbs_info->requested_freq,
				CPUFREQ_RELATION_H);
		return;
	}
}

static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;

	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	delay -= jiffies % delay;

	mutex_lock(&dbs_info->timer_mutex);

	dbs_check_cpu(dbs_info);

	queue_delayed_work_on(cpu, kconservative_wq, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	delay -= jiffies % delay;

	dbs_info->enable = 1;
	INIT_DEFERRABLE_WORK(&dbs_info->work, do_dbs_timer);
	queue_delayed_work_on(dbs_info->cpu, kconservative_wq, &dbs_info->work,
				delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	dbs_info->enable = 0;
	cancel_delayed_work_sync(&dbs_info->work);
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;
    suspended=0;

	this_dbs_info = &per_cpu(cs_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

		rc = sysfs_create_group(&policy->kobj, &dbs_attr_group);
		if (rc) {
			mutex_unlock(&dbs_mutex);
			return rc;
		}

		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			if (dbs_tuners_ins.ignore_nice) {
				j_dbs_info->prev_cpu_nice =
						kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			}
		}
		this_dbs_info->down_skip = 0;
		this_dbs_info->requested_freq = policy->cur;

		mutex_init(&this_dbs_info->timer_mutex);
		dbs_enable++;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 1) {
			unsigned int latency;
			/* policy latency is in nS. Convert it to uS first */
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;

			/*
			 * conservative does not implement micro like ondemand
			 * governor, thus we are bound to jiffes/HZ
			 */
			min_sampling_rate =
				MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(10);
			/* Bring kernel and HW constraints together */
			min_sampling_rate = max(min_sampling_rate,
					MIN_LATENCY_MULTIPLIER * latency);
			dbs_tuners_ins.sampling_rate =
				max(min_sampling_rate,
				    latency * LATENCY_MULTIPLIER);

			cpufreq_register_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
		}
		mutex_unlock(&dbs_mutex);

		dbs_timer_init(this_dbs_info);

		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		sysfs_remove_group(&policy->kobj, &dbs_attr_group);
		dbs_enable--;
		mutex_destroy(&this_dbs_info->timer_mutex);

		/*
		 * Stop the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 0)
			cpufreq_unregister_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);

		mutex_unlock(&dbs_mutex);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->min, CPUFREQ_RELATION_C);
		mutex_unlock(&this_dbs_info->timer_mutex);

		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_psychoactive
static
#endif
struct cpufreq_governor cpufreq_gov_psychoactive = {
	.name			= "psychoactive",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	int err;

	kconservative_wq = create_workqueue("kconservative");
	if (!kconservative_wq) {
		printk(KERN_ERR "Creation of kconservative failed\n");
		return -EFAULT;
	}
    register_early_suspend(&psychoactive_power_suspend);
	err = cpufreq_register_governor(&cpufreq_gov_psychoactive);
	if (err)
		destroy_workqueue(kconservative_wq);

	return err;
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_psychoactive);
	destroy_workqueue(kconservative_wq);
}


#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_psychoactive
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
