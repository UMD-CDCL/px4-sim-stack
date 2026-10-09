// Run production POSIX timer routines against controlled callback ordering.
#include <queue.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <set>

using hrt_abstime = uint64_t;
using hrt_callout = void (*)(void *);
struct hrt_call {
    sq_entry_t link{};
    hrt_abstime deadline{};
    hrt_abstime period{};
    hrt_callout callout{};
    void *arg{};
};
static sq_queue_t callout_queue{};
static hrt_abstime now = 100;
static void hrt_lock() {}
static void hrt_unlock() {}
static hrt_abstime hrt_absolute_time() { return now; }
static void hrt_call_reschedule() {}
#define PX4_DEBUG(...) ((void)0)
#include "timer.inc"

static void rearm_periodic(void *arg)
{
    auto *timer = static_cast<hrt_call *>(arg);
    hrt_call_internal(timer, now + 10000, 10000, rearm_periodic, timer);
}

static void rearm_one_shot(void *arg)
{
    auto *timer = static_cast<hrt_call *>(arg);
    hrt_call_internal(timer, now + 7, 0, nullptr, nullptr);
}

static void cancel_timer(void *arg)
{
    auto *timer = static_cast<hrt_call *>(arg);
    sq_rem(&timer->link, &callout_queue);
    timer->deadline = 0;
    timer->period = 0;
}

static void delay_timer(void *arg)
{
    // hrt_call_delay sets a new deadline without inserting the entry.
    static_cast<hrt_call *>(arg)->deadline = hrt_absolute_time() + 777;
}

int main(int argc, char **argv)
{
    if (argc != 2) { return 2; }
    hrt_callout callback = nullptr;
    uint64_t expected_deadline = 120, expected_period = 20;
    unsigned expected_count = 2;
    if (!strcmp(argv[1], "periodic_rearm")) {
        callback = rearm_periodic; expected_deadline = 10100; expected_period = 10000;
    } else if (!strcmp(argv[1], "one_shot_rearm")) {
        callback = rearm_one_shot; expected_deadline = 107; expected_period = 0;
    } else if (!strcmp(argv[1], "cancel")) {
        callback = cancel_timer; expected_deadline = 0; expected_period = 0; expected_count = 1;
    } else if (!strcmp(argv[1], "delay")) {
        callback = delay_timer; expected_deadline = 877;
    } else if (strcmp(argv[1], "normal")) { return 2; }

    hrt_call victim{}, bystander{};
    hrt_call_internal(&bystander, 120, 20, nullptr, nullptr);
    hrt_call_internal(&victim, 100, 20, callback, &victim);
    hrt_call_invoke();
    std::set<sq_entry_t *> seen;
    bool unique = true;
    for (auto *node = callout_queue.head; node; node = node->flink) {
        if (!seen.insert(node).second) { unique = false; break; }
    }
    bool pass = unique && seen.size() == expected_count && seen.count(&bystander.link)
        && (expected_count == 1 || seen.count(&victim.link))
        && victim.deadline == expected_deadline && victim.period == expected_period;
    printf("%s: %s (unique=%d, entries=%zu, deadline=%lu, period=%lu)\n",
           argv[1], pass ? "PASS" : "FAIL", unique, seen.size(), victim.deadline, victim.period);
    return pass ? 0 : 1;
}
