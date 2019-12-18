#include "drake/common/profiler.h"

#include <memory>
#include <ratio>
#include <sstream>
#include <vector>

#include <fmt/format.h>

namespace drake {
namespace common {

using std::vector;

void Timer::start() { start_ = std::chrono::high_resolution_clock::now(); }

}  // namespace common
}  // namespace drake

#ifdef ENABLE_TIMERS

namespace {

using drake::common::TimerIndex;
using drake::common::LapTimer;

/**  Class for storing sets of timers for profiling. This is a singleton class
 and _not_ threadsafe.  */
class Profiler {
 public:
  /**  Returns a pointer to the singleton instance, creating it as necessary.
   */
  static Profiler& getMutableInstance() {
    if (PROFILER == 0x0) {
      PROFILER = std::unique_ptr<Profiler>(new Profiler());
    }
    return *PROFILER;
  }

  /**  Creates a lap timer which uses the given label for display.
   @param  label  The string to display when reporting the profiling results.
   @returns  The identifier for the created timer.  */
  TimerIndex addTimer(std::string label) {
    DRAKE_DEMAND(timers_.size() == display_string_.size());
    size_t id = timers_.size();
    timers_.emplace_back();
    display_string_.emplace_back(std::move(label));
    return TimerIndex(id);
  }

  /**  Reports the number of timers.  */
  int timerCount() const { return static_cast<int>(timers_.size()); }

  /**  Starts the ith timer.
   @param  i  The identifier for the timer. Only checked in debug mode.  */
  void start(TimerIndex i) {
    DRAKE_ASSERT(i < timers_.size());
    timers_[i].start();
  }

  /**  Reports the time elapsed between this call and the last start for the ith
       timer.
   @param  i  The identifier for the timer. Only checked in debug mode.
   @tparam Units  the units in which the elapsed time will be reported.  */
  template<typename Units>
  double elapsed(TimerIndex i) const {
    DRAKE_ASSERT(i < timers_.size());
    return timers_[i].elapsed<Units>();
  }

  /**  Reports the time elapsed from the previous call to lap() or start() to
   this call for the ith timer. The clock is still "running" and the next lap
   starts.
   @param  i  The identifier for the timer. Only checked in debug mode.
   @tparam Units  the units in which the elapsed time will be reported.  */
  template<typename Units>
  double lap(TimerIndex i) {
    DRAKE_ASSERT(i < timers_.size());
    return timers_[i].lap<Units>();
  }

  /**  Reports the average lap time across all recorded laps for the ith timer.
   @param  i  The identifier for the timer. Only checked in debug mode.
   @tparam Units  the units in which the elapsed time will be reported.  */
  template<typename Units>
  double average(TimerIndex i) const {
    DRAKE_ASSERT(i < timers_.size());
    return timers_[i].average<Units>();
  }

  /**  Reports the total time across all recorded laps for the ith timer.
   @param  i  The identifier for the timer. Only checked in debug mode.
   @tparam Units  the units in which the elapsed time will be reported.  */
  template<typename Units>
  double total(TimerIndex i) const {
    DRAKE_ASSERT(i < timers_.size());
    return timers_[i].total<Units>();
  }

  /**  Reports the number of laps the ith counter has had.
   @param  i  The identifier for the timer. Only checked in debug mode.  */
  int laps(TimerIndex i) const {
    DRAKE_ASSERT(i < timers_.size());
    return timers_[i].laps();
  }

#if 0
  /**  Reports the average lap time across all recorded laps for
   *the first "count" timers.
   *
   *	@param		count		The first count timers' average times will be
   *reported. Only checked in debug mode.
   *	@param		scale		The scale of the units to report the elapsed
   *time in. e.g., 1.0 --> seconds, 0.001 -->, 1e-6 --> microseconds.
   *	@param		averages	A pointer to an array of floats sufficiently
   *large to hold count values.
   */
  void averages(size_t count, float scale, float* averages) {
    assert(count <= timers_.size() &&
           "Timer index outside of valid range in Profiler::averages()");
    for (size_t i = 0; i < count; ++i) {
      averages[i] = timers_[i].average(scale);
    }
  }
#endif

  /**  Returns the display string for the given LapTimer.
   @param  i  The index of the desired timer. Only validated in debug mode.  */
  const std::string& displayString(TimerIndex i) const {
    DRAKE_ASSERT(i < display_string_.size());
    return display_string_[i];
  }

 private:
  // Singleton pointer.
  static std::unique_ptr<Profiler> PROFILER;

  Profiler() = default;

  // The timers.
  std::vector<LapTimer> timers_;

  // The timer names; there should be one name for each timer.
  std::vector<::std::string> display_string_;
};

std::unique_ptr<Profiler> Profiler::PROFILER{nullptr};

}  // namespace

TimerIndex addTimer(std::string displayString) {
  return Profiler::getMutableInstance().addTimer(std::move(displayString));
}

void startTimer(TimerIndex index) {
  Profiler::getMutableInstance().start(index);
}

void stopTimer(TimerIndex index) {
  Profiler::getMutableInstance().lap<LapTimer::Units>(index);
}

void lapTimer(TimerIndex index) {
  Profiler::getMutableInstance().lap<LapTimer::Units>(index);
}

double averageTimeInSec(TimerIndex index) {
  return Profiler::getMutableInstance().average<std::ratio<1, 1>>(index);
}

std::string TableOfAverages() {
  using MilliSeconds = std::ratio<1, 1000>;
  const Profiler& profiler = Profiler::getMutableInstance();
  const int count = profiler.timerCount();
  std::stringstream ss;
  ss << "All registered profiles average times:\n";
  ss << fmt::format("{:>22}{:>10}{:>17}  {}", "Time (ms)", "Samples",
                    "Total Time (ms)", "Label") << "\n";
  for (TimerIndex i(0); i < count; ++i) {
    double time = profiler.average<MilliSeconds>(i);

    ss << fmt::format("{:>22}{:>10}{:>17}  {}", time, profiler.laps(i),
                      profiler.total<MilliSeconds>(i),
                      profiler.displayString(i));
    if (i < count - 1) ss << "\n";
  }
  return ss.str();
}

#endif  // ENABLE_TIMERS
