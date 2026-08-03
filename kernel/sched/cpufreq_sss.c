// SPDX-License-Identifier: GPL-2.0
/*
 * CPUFreq governor based on scheduler-provided CPU utilization data.
 * SSS: schedutil clone with split up/down rate limit and hispeed floor.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 */

#include <trace/hooks/sched.h>

#define IOWAIT_BOOST_MIN        (SCHED_CAPACITY_SCALE / 8)

struct sss_tunables {
        struct gov_attr_set     attr_set;
        unsigned int            up_rate_limit_us;
        unsigned int            down_rate_limit_us;
        unsigned int            hispeed_load;
        unsigned int            hispeed_freq;
};

struct sss_policy {
        struct cpufreq_policy   *policy;

        struct sss_tunables     *tunables;
        struct list_head        tunables_hook;

        raw_spinlock_t          update_lock;
        u64                     last_freq_update_time;
        s64                     up_rate_delay_ns;
        s64                     down_rate_delay_ns;
        s64                     min_rate_limit_ns;
        unsigned int            next_freq;
        unsigned int            cached_raw_freq;

        /* The next fields are only needed if fast switch cannot be used: */
        struct                  irq_work irq_work;
        struct                  kthread_work work;
        struct                  mutex work_lock;
        struct                  kthread_worker worker;
        struct task_struct      *thread;
        bool                    work_in_progress;

        bool                    limits_changed;
        bool                    need_freq_update;
};

struct sss_cpu {
        struct update_util_data update_util;
        struct sss_policy       *sg_policy;
        unsigned int            cpu;

        bool                    iowait_boost_pending;
        unsigned int            iowait_boost;
        u64                     last_update;

        unsigned long           util;
        unsigned long           bw_dl;

        /* The field below is for single-CPU policies only: */
#ifdef CONFIG_NO_HZ_COMMON
        unsigned long           saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct sss_cpu, sss_cpu);

/************************ Governor internals ***********************/

static bool sss_should_update_freq(struct sss_policy *sg_policy, u64 time)
{
        s64 delta_ns;

        if (!cpufreq_this_cpu_can_update(sg_policy->policy))
                return false;

        if (unlikely(sg_policy->limits_changed)) {
                sg_policy->limits_changed = false;
                sg_policy->need_freq_update = true;
                return true;
        }

        delta_ns = time - sg_policy->last_freq_update_time;

        return delta_ns >= sg_policy->min_rate_limit_ns;
}

static bool sss_up_down_rate_limit(struct sss_policy *sg_policy, u64 time,
                                   unsigned int next_freq)
{
        s64 delta_ns = time - sg_policy->last_freq_update_time;

        if (next_freq > sg_policy->next_freq)
                return delta_ns < sg_policy->up_rate_delay_ns;

        if (next_freq < sg_policy->next_freq)
                return delta_ns < sg_policy->down_rate_delay_ns;

        return false;
}

static bool sss_update_next_freq(struct sss_policy *sg_policy, u64 time,
                                  unsigned int next_freq)
{
        if (sg_policy->need_freq_update) {
                sg_policy->need_freq_update = false;
                if (sg_policy->next_freq == next_freq &&
                    !cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS))
                        return false;
        } else if (sg_policy->next_freq == next_freq) {
                return false;
        } else if (sss_up_down_rate_limit(sg_policy, time, next_freq)) {
                return false;
        }

        sg_policy->next_freq = next_freq;
        sg_policy->last_freq_update_time = time;

        return true;
}

static void sss_deferred_update(struct sss_policy *sg_policy)
{
        if (!sg_policy->work_in_progress) {
                sg_policy->work_in_progress = true;
                irq_work_queue(&sg_policy->irq_work);
        }
}

static __always_inline
unsigned long sss_get_capacity_ref_freq(struct cpufreq_policy *policy)
{
        unsigned int freq = arch_scale_freq_ref(policy->cpu);

        if (freq)
                return freq;

        if (arch_scale_freq_invariant())
                return policy->cpuinfo.max_freq;

        return policy->cur + (policy->cur >> 2);
}

/**
 * sss_get_next_freq - Compute a new frequency for a given cpufreq policy.
 * @sg_policy: sss policy object to compute the new frequency for.
 * @util: Current CPU utilization.
 * @max: CPU capacity.
 *
 * Same math as schedutil's get_next_freq(), plus a hispeed floor: if the
 * raw load (util/max, before uclamp/perf mapping) is >= tunables->hispeed_load
 * and hispeed_freq is configured (non-zero), the result is floored to
 * hispeed_freq before being resolved against the driver's frequency table.
 */
static unsigned int sss_get_next_freq(struct sss_policy *sg_policy,
                                      unsigned long util, unsigned long max)
{
        struct cpufreq_policy *policy = sg_policy->policy;
        struct sss_tunables *tunables = sg_policy->tunables;
        unsigned int freq;
        unsigned long next_freq = 0;
        unsigned int load_pct = 0;

        if (max)
                load_pct = (unsigned int)((util * 100) / max);

        freq = sss_get_capacity_ref_freq(policy);
        util = map_util_perf(util);
        trace_android_vh_map_util_freq(util, freq, max, &next_freq, policy,
                        &sg_policy->need_freq_update);
        if (next_freq)
                freq = next_freq;
        else
                freq = map_util_freq(util, freq, max);

        if (tunables->hispeed_freq && load_pct >= tunables->hispeed_load &&
            freq < tunables->hispeed_freq)
                freq = tunables->hispeed_freq;

        if (freq == sg_policy->cached_raw_freq && !sg_policy->need_freq_update)
                return sg_policy->next_freq;

        sg_policy->cached_raw_freq = freq;
        return cpufreq_driver_resolve_freq(policy, freq);
}

static void sss_get_util(struct sss_cpu *sg_cpu)
{
        unsigned long util = cpu_util_cfs_boost(sg_cpu->cpu);
        struct rq *rq = cpu_rq(sg_cpu->cpu);

        sg_cpu->bw_dl = cpu_bw_dl(rq);
        sg_cpu->util = effective_cpu_util(sg_cpu->cpu, util,
                                          FREQUENCY_UTIL, NULL);
}

static bool sss_iowait_reset(struct sss_cpu *sg_cpu, u64 time,
                             bool set_iowait_boost)
{
        s64 delta_ns = time - sg_cpu->last_update;

        if (delta_ns <= TICK_NSEC)
                return false;

        sg_cpu->iowait_boost = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
        sg_cpu->iowait_boost_pending = set_iowait_boost;

        return true;
}

static void sss_iowait_boost(struct sss_cpu *sg_cpu, u64 time,
                             unsigned int flags)
{
        bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

        if (sg_cpu->iowait_boost &&
            sss_iowait_reset(sg_cpu, time, set_iowait_boost))
                return;

        if (!set_iowait_boost)
                return;

        if (sg_cpu->iowait_boost_pending)
                return;
        sg_cpu->iowait_boost_pending = true;

        if (sg_cpu->iowait_boost) {
                sg_cpu->iowait_boost =
                        min_t(unsigned int, sg_cpu->iowait_boost << 1, SCHED_CAPACITY_SCALE);
                return;
        }

        sg_cpu->iowait_boost = IOWAIT_BOOST_MIN;
}

static void sss_iowait_apply(struct sss_cpu *sg_cpu, u64 time,
                             unsigned long max_cap)
{
        unsigned long boost;

        if (!sg_cpu->iowait_boost)
                return;

        if (sss_iowait_reset(sg_cpu, time, false))
                return;

        if (!sg_cpu->iowait_boost_pending) {
                sg_cpu->iowait_boost >>= 1;
                if (sg_cpu->iowait_boost < IOWAIT_BOOST_MIN) {
                        sg_cpu->iowait_boost = 0;
                        return;
                }
        }

        sg_cpu->iowait_boost_pending = false;

        boost = (sg_cpu->iowait_boost * max_cap) >> SCHED_CAPACITY_SHIFT;
        boost = uclamp_rq_util_with(cpu_rq(sg_cpu->cpu), boost, NULL);
        if (sg_cpu->util < boost)
                sg_cpu->util = boost;
}

#ifdef CONFIG_NO_HZ_COMMON
static bool sss_cpu_is_busy(struct sss_cpu *sg_cpu)
{
        unsigned long idle_calls = tick_nohz_get_idle_calls_cpu(sg_cpu->cpu);
        bool ret = idle_calls == sg_cpu->saved_idle_calls;

        sg_cpu->saved_idle_calls = idle_calls;
        return ret;
}
#else
static inline bool sss_cpu_is_busy(struct sss_cpu *sg_cpu) { return false; }
#endif /* CONFIG_NO_HZ_COMMON */

static inline void sss_ignore_dl_rate_limit(struct sss_cpu *sg_cpu)
{
        if (cpu_bw_dl(cpu_rq(sg_cpu->cpu)) > sg_cpu->bw_dl)
                sg_cpu->sg_policy->limits_changed = true;
}

static inline bool sss_update_single_common(struct sss_cpu *sg_cpu,
                                            u64 time, unsigned long max_cap,
                                            unsigned int flags)
{
        sss_iowait_boost(sg_cpu, time, flags);
        sg_cpu->last_update = time;

        sss_ignore_dl_rate_limit(sg_cpu);

        if (!sss_should_update_freq(sg_cpu->sg_policy, time))
                return false;

        sss_get_util(sg_cpu);
        sss_iowait_apply(sg_cpu, time, max_cap);

        return true;
}

static void sss_update_single_freq(struct update_util_data *hook, u64 time,
                                   unsigned int flags)
{
        struct sss_cpu *sg_cpu = container_of(hook, struct sss_cpu, update_util);
        struct sss_policy *sg_policy = sg_cpu->sg_policy;
        unsigned int cached_freq = sg_policy->cached_raw_freq;
        unsigned long max_cap;
        unsigned int next_f;

        max_cap = arch_scale_cpu_capacity(sg_cpu->cpu);

        if (!sss_update_single_common(sg_cpu, time, max_cap, flags))
                return;

        next_f = sss_get_next_freq(sg_policy, sg_cpu->util, max_cap);
        if (!uclamp_rq_is_capped(cpu_rq(sg_cpu->cpu)) &&
            sss_cpu_is_busy(sg_cpu) && next_f < sg_policy->next_freq &&
            !sg_policy->need_freq_update) {
                next_f = sg_policy->next_freq;

                sg_policy->cached_raw_freq = cached_freq;
        }

        if (!sss_update_next_freq(sg_policy, time, next_f))
                return;

        if (sg_policy->policy->fast_switch_enabled) {
                cpufreq_driver_fast_switch(sg_policy->policy, next_f);
        } else {
                raw_spin_lock(&sg_policy->update_lock);
                sss_deferred_update(sg_policy);
                raw_spin_unlock(&sg_policy->update_lock);
        }
}

static void sss_update_single_perf(struct update_util_data *hook, u64 time,
                                   unsigned int flags)
{
        struct sss_cpu *sg_cpu = container_of(hook, struct sss_cpu, update_util);
        unsigned long prev_util = sg_cpu->util;
        unsigned long max_cap;

        if (!arch_scale_freq_invariant()) {
                sss_update_single_freq(hook, time, flags);
                return;
        }

        max_cap = arch_scale_cpu_capacity(sg_cpu->cpu);

        if (!sss_update_single_common(sg_cpu, time, max_cap, flags))
                return;

        if (!uclamp_rq_is_capped(cpu_rq(sg_cpu->cpu)) &&
            sss_cpu_is_busy(sg_cpu) && sg_cpu->util < prev_util)
                sg_cpu->util = prev_util;

        cpufreq_driver_adjust_perf(sg_cpu->cpu, map_util_perf(sg_cpu->bw_dl),
                                   map_util_perf(sg_cpu->util), max_cap);

        sg_cpu->sg_policy->last_freq_update_time = time;
}

static unsigned int sss_next_freq_shared(struct sss_cpu *sg_cpu, u64 time)
{
        struct sss_policy *sg_policy = sg_cpu->sg_policy;
        struct cpufreq_policy *policy = sg_policy->policy;
        unsigned long util = 0, max_cap;
        unsigned int j;

        max_cap = arch_scale_cpu_capacity(sg_cpu->cpu);

        for_each_cpu(j, policy->cpus) {
                struct sss_cpu *j_sg_cpu = &per_cpu(sss_cpu, j);

                sss_get_util(j_sg_cpu);
                sss_iowait_apply(j_sg_cpu, time, max_cap);

                util = max(j_sg_cpu->util, util);
        }

        return sss_get_next_freq(sg_policy, util, max_cap);
}

static void
sss_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
        struct sss_cpu *sg_cpu = container_of(hook, struct sss_cpu, update_util);
        struct sss_policy *sg_policy = sg_cpu->sg_policy;
        unsigned int next_f;

        raw_spin_lock(&sg_policy->update_lock);

        sss_iowait_boost(sg_cpu, time, flags);
        sg_cpu->last_update = time;

        sss_ignore_dl_rate_limit(sg_cpu);

        if (sss_should_update_freq(sg_policy, time)) {
                next_f = sss_next_freq_shared(sg_cpu, time);

                if (!sss_update_next_freq(sg_policy, time, next_f))
                        goto unlock;

                if (sg_policy->policy->fast_switch_enabled)
                        cpufreq_driver_fast_switch(sg_policy->policy, next_f);
                else
                        sss_deferred_update(sg_policy);
        }
unlock:
        raw_spin_unlock(&sg_policy->update_lock);
}

static void sss_work(struct kthread_work *work)
{
        struct sss_policy *sg_policy = container_of(work, struct sss_policy, work);
        unsigned int freq;
        unsigned long flags;

        raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
        freq = sg_policy->next_freq;
        sg_policy->work_in_progress = false;
        raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);

        mutex_lock(&sg_policy->work_lock);
        __cpufreq_driver_target(sg_policy->policy, freq, CPUFREQ_RELATION_L);
        mutex_unlock(&sg_policy->work_lock);
}

static void sss_irq_work(struct irq_work *irq_work)
{
        struct sss_policy *sg_policy;

        sg_policy = container_of(irq_work, struct sss_policy, irq_work);

        kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}

/************************** sysfs interface ************************/

static struct sss_tunables *sss_global_tunables;
static DEFINE_MUTEX(sss_global_tunables_lock);

static inline struct sss_tunables *to_sss_tunables(struct gov_attr_set *attr_set)
{
        return container_of(attr_set, struct sss_tunables, attr_set);
}

static void sss_update_rate_limit_ns(struct sss_policy *sg_policy,
                                     struct sss_tunables *tunables)
{
        sg_policy->up_rate_delay_ns =
                (s64)tunables->up_rate_limit_us * NSEC_PER_USEC;
        sg_policy->down_rate_delay_ns =
                (s64)tunables->down_rate_limit_us * NSEC_PER_USEC;
        sg_policy->min_rate_limit_ns =
                min(sg_policy->up_rate_delay_ns, sg_policy->down_rate_delay_ns);
}

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
        struct sss_tunables *tunables = to_sss_tunables(attr_set);

        return sprintf(buf, "%u\n", tunables->up_rate_limit_us);
}

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
        struct sss_tunables *tunables = to_sss_tunables(attr_set);

        return sprintf(buf, "%u\n", tunables->down_rate_limit_us);
}

static ssize_t hispeed_load_show(struct gov_attr_set *attr_set, char *buf)
{
        struct sss_tunables *tunables = to_sss_tunables(attr_set);

        return sprintf(buf, "%u\n", tunables->hispeed_load);
}

static ssize_t hispeed_freq_show(struct gov_attr_set *attr_set, char *buf)
{
        struct sss_tunables *tunables = to_sss_tunables(attr_set);

        return sprintf(buf, "%u\n", tunables->hispeed_freq);
}

static ssize_t
up_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
        struct sss_tunables *tunables = to_sss_tunables(attr_set);
        struct sss_policy *sg_policy;
        unsigned int rate_limit_us;

        if (kstrtouint(buf, 10, &rate_limit_us))
                return -EINVAL;

        tunables->up_rate_limit_us = rate_limit_us;

        list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
                sss_update_rate_limit_ns(sg_policy, tunables);

        return count;
}

static ssize_t
down_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
        struct sss_tunables *tunables = to_sss_tunables(attr_set);
        struct sss_policy *sg_policy;
        unsigned int rate_limit_us;

        if (kstrtouint(buf, 10, &rate_limit_us))
                return -EINVAL;

        tunables->down_rate_limit_us = rate_limit_us;

        list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
                sss_update_rate_limit_ns(sg_policy, tunables);

        return count;
}

static ssize_t
hispeed_load_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
        struct sss_tunables *tunables = to_sss_tunables(attr_set);
        unsigned int val;

        if (kstrtouint(buf, 10, &val))
                return -EINVAL;

        tunables->hispeed_load = min(val, 100U);

        return count;
}

static ssize_t
hispeed_freq_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
        struct sss_tunables *tunables = to_sss_tunables(attr_set);
        unsigned int val;

        if (kstrtouint(buf, 10, &val))
                return -EINVAL;

        tunables->hispeed_freq = val;

        return count;
}

static struct governor_attr up_rate_limit_us   = __ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);
static struct governor_attr hispeed_load       = __ATTR_RW(hispeed_load);
static struct governor_attr hispeed_freq       = __ATTR_RW(hispeed_freq);

static struct attribute *sss_attrs[] = {
        &up_rate_limit_us.attr,
        &down_rate_limit_us.attr,
        &hispeed_load.attr,
        &hispeed_freq.attr,
        NULL
};
ATTRIBUTE_GROUPS(sss);

static void sss_tunables_free(struct kobject *kobj)
{
        struct gov_attr_set *attr_set = to_gov_attr_set(kobj);

        kfree(to_sss_tunables(attr_set));
}

static const struct kobj_type sss_tunables_ktype = {
        .default_groups = sss_groups,
        .sysfs_ops = &governor_sysfs_ops,
        .release = &sss_tunables_free,
};

/********************** cpufreq governor interface *********************/

struct cpufreq_governor sss_gov;

static struct sss_policy *sss_policy_alloc(struct cpufreq_policy *policy)
{
        struct sss_policy *sg_policy;

        sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
        if (!sg_policy)
                return NULL;

        sg_policy->policy = policy;
        raw_spin_lock_init(&sg_policy->update_lock);
        return sg_policy;
}

static void sss_policy_free(struct sss_policy *sg_policy)
{
        kfree(sg_policy);
}

static int sss_kthread_create(struct sss_policy *sg_policy)
{
        struct task_struct *thread;
        struct sched_attr attr = {
                .size           = sizeof(struct sched_attr),
                .sched_policy   = SCHED_DEADLINE,
                .sched_flags    = SCHED_FLAG_SUGOV,
                .sched_nice     = 0,
                .sched_priority = 0,
                .sched_runtime  =  1000000,
                .sched_deadline = 10000000,
                .sched_period   = 10000000,
        };
        struct cpufreq_policy *policy = sg_policy->policy;
        int ret;

        if (policy->fast_switch_enabled)
                return 0;

        kthread_init_work(&sg_policy->work, sss_work);
        kthread_init_worker(&sg_policy->worker);
        thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
                                "sss:%d",
                                cpumask_first(policy->related_cpus));
        if (IS_ERR(thread)) {
                pr_err("failed to create sss thread: %ld\n", PTR_ERR(thread));
                return PTR_ERR(thread);
        }

        ret = sched_setattr_nocheck(thread, &attr);
        if (ret) {
                kthread_stop(thread);
                pr_warn("%s: failed to set SCHED_DEADLINE\n", __func__);
                return ret;
        }

        sg_policy->thread = thread;
        kthread_bind_mask(thread, policy->related_cpus);
        init_irq_work(&sg_policy->irq_work, sss_irq_work);
        mutex_init(&sg_policy->work_lock);

        wake_up_process(thread);

        return 0;
}

static void sss_kthread_stop(struct sss_policy *sg_policy)
{
        if (sg_policy->policy->fast_switch_enabled)
                return;

        kthread_flush_worker(&sg_policy->worker);
        kthread_stop(sg_policy->thread);
        mutex_destroy(&sg_policy->work_lock);
}

static struct sss_tunables *sss_tunables_alloc(struct sss_policy *sg_policy)
{
        struct sss_tunables *tunables;

        tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
        if (tunables) {
                gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
                if (!have_governor_per_policy())
                        sss_global_tunables = tunables;
        }
        return tunables;
}

static void sss_clear_global_tunables(void)
{
        if (!have_governor_per_policy())
                sss_global_tunables = NULL;
}

static int sss_init(struct cpufreq_policy *policy)
{
        struct sss_policy *sg_policy;
        struct sss_tunables *tunables;
        int ret = 0;

        if (policy->governor_data)
                return -EBUSY;

        cpufreq_enable_fast_switch(policy);

        sg_policy = sss_policy_alloc(policy);
        if (!sg_policy) {
                ret = -ENOMEM;
                goto disable_fast_switch;
        }

        ret = sss_kthread_create(sg_policy);
        if (ret)
                goto free_sg_policy;

        mutex_lock(&sss_global_tunables_lock);

        if (sss_global_tunables) {
                if (WARN_ON(have_governor_per_policy())) {
                        ret = -EINVAL;
                        goto stop_kthread;
                }
                policy->governor_data = sg_policy;
                sg_policy->tunables = sss_global_tunables;

                gov_attr_set_get(&sss_global_tunables->attr_set, &sg_policy->tunables_hook);
                goto out;
        }

        tunables = sss_tunables_alloc(sg_policy);
        if (!tunables) {
                ret = -ENOMEM;
                goto stop_kthread;
        }

        tunables->up_rate_limit_us = 0;
        tunables->down_rate_limit_us = 0;
        tunables->hispeed_load = 90;
        tunables->hispeed_freq = 0;

        policy->governor_data = sg_policy;
        sg_policy->tunables = tunables;

        ret = kobject_init_and_add(&tunables->attr_set.kobj, &sss_tunables_ktype,
                                   get_governor_parent_kobj(policy), "%s",
                                   sss_gov.name);
        if (ret)
                goto fail;

out:
        mutex_unlock(&sss_global_tunables_lock);
        return 0;

fail:
        kobject_put(&tunables->attr_set.kobj);
        policy->governor_data = NULL;
        sss_clear_global_tunables();

stop_kthread:
        sss_kthread_stop(sg_policy);
        mutex_unlock(&sss_global_tunables_lock);

free_sg_policy:
        sss_policy_free(sg_policy);

disable_fast_switch:
        cpufreq_disable_fast_switch(policy);

        pr_err("initialization failed (error %d)\n", ret);
        return ret;
}

static void sss_exit(struct cpufreq_policy *policy)
{
        struct sss_policy *sg_policy = policy->governor_data;
        struct sss_tunables *tunables = sg_policy->tunables;
        unsigned int count;

        mutex_lock(&sss_global_tunables_lock);

        count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
        policy->governor_data = NULL;
        if (!count)
                sss_clear_global_tunables();

        mutex_unlock(&sss_global_tunables_lock);

        sss_kthread_stop(sg_policy);
        sss_policy_free(sg_policy);
        cpufreq_disable_fast_switch(policy);
}

static int sss_start(struct cpufreq_policy *policy)
{
        struct sss_policy *sg_policy = policy->governor_data;
        void (*uu)(struct update_util_data *data, u64 time, unsigned int flags);
        unsigned int cpu;

        sss_update_rate_limit_ns(sg_policy, sg_policy->tunables);
        sg_policy->last_freq_update_time        = 0;
        sg_policy->next_freq                    = 0;
        sg_policy->work_in_progress             = false;
        sg_policy->limits_changed               = false;
        sg_policy->cached_raw_freq              = 0;

        sg_policy->need_freq_update = cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS);

        for_each_cpu(cpu, policy->cpus) {
                struct sss_cpu *sg_cpu = &per_cpu(sss_cpu, cpu);

                memset(sg_cpu, 0, sizeof(*sg_cpu));
                sg_cpu->cpu                     = cpu;
                sg_cpu->sg_policy               = sg_policy;
        }

        if (policy_is_shared(policy))
                uu = sss_update_shared;
        else if (policy->fast_switch_enabled && cpufreq_driver_has_adjust_perf())
                uu = sss_update_single_perf;
        else
                uu = sss_update_single_freq;

        for_each_cpu(cpu, policy->cpus) {
                struct sss_cpu *sg_cpu = &per_cpu(sss_cpu, cpu);

                cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util, uu);
        }
        return 0;
}

static void sss_stop(struct cpufreq_policy *policy)
{
        struct sss_policy *sg_policy = policy->governor_data;
        unsigned int cpu;

        for_each_cpu(cpu, policy->cpus)
                cpufreq_remove_update_util_hook(cpu);

        synchronize_rcu();

        if (!policy->fast_switch_enabled) {
                irq_work_sync(&sg_policy->irq_work);
                kthread_cancel_work_sync(&sg_policy->work);
        }
}

static void sss_limits(struct cpufreq_policy *policy)
{
        struct sss_policy *sg_policy = policy->governor_data;

        if (!policy->fast_switch_enabled) {
                mutex_lock(&sg_policy->work_lock);
                cpufreq_policy_apply_limits(policy);
                mutex_unlock(&sg_policy->work_lock);
        }

        sg_policy->limits_changed = true;
}

struct cpufreq_governor sss_gov = {
        .name                   = "sss",
        .owner                  = THIS_MODULE,
        .flags                  = CPUFREQ_GOV_DYNAMIC_SWITCHING,
        .init                   = sss_init,
        .exit                   = sss_exit,
        .start                  = sss_start,
        .stop                   = sss_stop,
        .limits                 = sss_limits,
};

cpufreq_governor_init(sss_gov);

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SSS
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &sss_gov;
}
#endif
