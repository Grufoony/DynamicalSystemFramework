#pragma once

#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>

#include <array>
#include <atomic>
#include <chrono>
#include <format>
#include <string>
#include <string_view>
#include <sys/ioctl.h>
#include <unistd.h>

namespace dsf::utility {

  using namespace std::chrono_literals;

  // ---------------------------------------------------------------------------
  // Unicode fractional block characters  ▏▎▍▌▋▊▉█  (plus space = 9 symbols)
  // ---------------------------------------------------------------------------
  inline constexpr std::array<std::string_view, 9> BLOCKS = {" ",
                                                             "\xE2\x96\x8F",
                                                             "\xE2\x96\x8E",
                                                             "\xE2\x96\x8D",
                                                             "\xE2\x96\x8C",
                                                             "\xE2\x96\x8B",
                                                             "\xE2\x96\x8A",
                                                             "\xE2\x96\x89",
                                                             "\xE2\x96\x88"};

  class progress_bar;  // forward declaration

  // ---------------------------------------------------------------------------
  // progress_sink
  //   A custom spdlog sink that renders a progress_bar at the bottom of the
  //   terminal, erasing and redrawing it around every log message.
  // ---------------------------------------------------------------------------
  class progress_sink : public spdlog::sinks::base_sink<std::mutex> {
  public:
    explicit progress_sink(std::FILE* file = stderr)
        : m_file{file}, m_is_tty{static_cast<bool>(isatty(fileno(file)))} {
      m_refresh_width();
    }

    // Called by progress_bar on construction / destruction.
    void attach(progress_bar* bar) {
      std::lock_guard lock{mutex_};
      m_bar = bar;
      m_bar_drawn = false;
    }

    void detach(bool erase = true) {
      std::lock_guard lock{mutex_};
      if (erase)
        m_erase_bar();
      m_bar = nullptr;
    }

    // Called by progress_bar::update() to trigger a redraw.
    void repaint() {
      std::lock_guard lock{mutex_};
      m_repaint(nullptr);
    }

    [[nodiscard]] int width() const noexcept { return m_width; }

  protected:
    void sink_it_(const spdlog::details::log_msg& msg) override { m_repaint(&msg); }
    void flush_() override { std::fflush(m_file); }

  private:
    std::FILE* m_file;
    bool m_is_tty;
    int m_width{80};
    bool m_bar_drawn{false};
    progress_bar* m_bar{nullptr};  // non-owning; lifetime managed by progress_bar RAII

    void m_refresh_width() noexcept {
      winsize ws{};
      if (ioctl(fileno(m_file), TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0)
        m_width = static_cast<int>(ws.ws_col);
    }

    void m_erase_bar() noexcept {
      if (m_bar_drawn && m_is_tty) {
        std::fputs("\x1B[A\x1B[2K\r", m_file);
        m_bar_drawn = false;
      }
    }

    // Core repaint: erase bar → emit log line (if any) → redraw bar.
    // Must be called while holding mutex_.
    void m_repaint(const spdlog::details::log_msg* msg);
  };

  // ---------------------------------------------------------------------------
  // progress_bar
  //   Attach to a logger that owns a progress_sink.  The bar renders at the
  //   bottom of the terminal, below all log output.
  // ---------------------------------------------------------------------------
  class progress_bar {
  public:
    progress_bar(std::shared_ptr<spdlog::logger> logger,
                 std::string_view desc,
                 std::size_t total,
                 bool ascii = false)
        : m_logger{std::move(logger)},
          m_desc{desc},
          m_total{total},
          m_ascii{ascii},
          m_start{std::chrono::steady_clock::now()} {
      // Find the first progress_sink in the logger's sink list.
      for (auto& s : m_logger->sinks()) {
        if (auto ps = std::dynamic_pointer_cast<progress_sink>(s)) {
          m_sink = ps.get();
          break;
        }
      }
      if (m_sink) {
        m_sink->attach(this);
        m_sink->repaint();
      }
    }

    ~progress_bar() {
      if (!m_sink)
        return;
      // Keep the bar visible on completion; erase it on early exit.
      const bool complete = m_n.load(std::memory_order_relaxed) >= m_total;
      m_sink->detach(/*erase=*/!complete);
    }

    progress_bar(const progress_bar&) = delete;
    progress_bar& operator=(const progress_bar&) = delete;

    progress_bar& operator++() {
      update(1);
      return *this;
    }
    void operator+=(std::size_t delta) { update(delta); }

    /// Increment the counter by `delta` and repaint if the minimum interval has elapsed.
    void update(std::size_t delta = 1) {
      auto const cur = (m_n += delta);
      auto const now = std::chrono::steady_clock::now();
      auto const elapsed =
          std::chrono::duration_cast<std::chrono::nanoseconds>(now - m_start);
      auto const last =
          std::chrono::nanoseconds{m_last_repaint_ns.load(std::memory_order_relaxed)};

      if (cur >= m_total || (elapsed - last) >= m_min_interval) {
        m_last_repaint_ns.store(elapsed.count(), std::memory_order_relaxed);
        if (m_sink)
          m_sink->repaint();
      }
    }

    /// Render the bar into a fixed-width string.
    /// Format: "desc:  75% |███████-------| n/total [HH:MM:SS / HH:MM:SS]\n"
    [[nodiscard]] std::string render(int width) const {
      using namespace std::chrono;

      auto const n = m_n.load(std::memory_order_relaxed);
      auto const now = steady_clock::now();
      auto const elapsed = duration_cast<seconds>(now - m_start);
      float const frac =
          (m_total > 0) ? static_cast<float>(n) / static_cast<float>(m_total) : 0.f;
      auto const remain = (frac > 1e-6f)
                              ? duration_cast<seconds>(elapsed * (1.f / frac - 1.f))
                              : seconds{0};

      // Use hh_mm_ss for clean decomposition of elapsed / remaining time.
      const hh_mm_ss el{elapsed};
      const hh_mm_ss rm{remain};

      auto const left = std::format("{}: {:3.0f}% |", m_desc, frac * 100.f);
      auto const right =
          std::format("| {}/{} [{:02d}:{:02d}:{:02d} / {:02d}:{:02d}:{:02d}]\n",
                      n,
                      m_total,
                      el.hours().count(),
                      el.minutes().count(),
                      el.seconds().count(),
                      rm.hours().count(),
                      rm.minutes().count(),
                      rm.seconds().count());

      int const bar_chars = width - static_cast<int>(left.size()) -
                            static_cast<int>(right.size()) +
                            1;  // +1: right already includes '\n'

      std::string result;
      result.reserve(static_cast<std::size_t>(width) * 3);
      result += left;
      if (bar_chars > 0)
        result += m_render_bar(bar_chars, frac);
      result += right;
      return result;
    }

  private:
    std::shared_ptr<spdlog::logger> m_logger;
    std::string m_desc;
    std::size_t m_total;
    bool m_ascii;
    std::atomic<std::size_t> m_n{0};
    std::chrono::steady_clock::time_point m_start;
    std::atomic<long long> m_last_repaint_ns{0};
    std::chrono::nanoseconds m_min_interval{80ms};
    progress_sink* m_sink{nullptr};  // non-owning

    [[nodiscard]] std::string m_render_bar(int width, float frac) const {
      std::string bar;
      bar.reserve(static_cast<std::size_t>(width) * 3);

      if (m_ascii) {
        const int filled = static_cast<int>(frac * static_cast<float>(width));
        bar.append(static_cast<std::size_t>(filled), '#');
        bar.append(static_cast<std::size_t>(width - filled), '-');
      } else {
        constexpr int steps = static_cast<int>(BLOCKS.size()) - 1;  // 8
        const int eighths = static_cast<int>(frac * static_cast<float>(width) * steps);
        const int full = eighths / steps;
        const int part = eighths % steps;
        const int empty = width - full - (part ? 1 : 0);

        for (int i = 0; i < full; ++i)
          bar += BLOCKS.back();
        if (part)
          bar += BLOCKS[static_cast<std::size_t>(part)];
        for (int i = 0; i < empty; ++i)
          bar += BLOCKS.front();
      }
      return bar;
    }
  };

  // ---------------------------------------------------------------------------
  // progress_sink::m_repaint — defined after progress_bar is complete.
  // ---------------------------------------------------------------------------
  inline void progress_sink::m_repaint(const spdlog::details::log_msg* msg) {
    if (!m_is_tty) {
      if (msg) {
        spdlog::memory_buf_t buf;
        formatter_->format(*msg, buf);
        std::fwrite(buf.data(), 1, buf.size(), m_file);
        std::fflush(m_file);
      }
      return;
    }

    m_refresh_width();
    m_erase_bar();

    if (msg) {
      spdlog::memory_buf_t buf;
      formatter_->format(*msg, buf);
      std::fwrite(buf.data(), 1, buf.size(), m_file);
    }

    if (m_bar) {
      auto const line = m_bar->render(m_width);
      std::fwrite(line.data(), 1, line.size(), m_file);
      m_bar_drawn = true;
    }

    std::fflush(m_file);
  }

  // ---------------------------------------------------------------------------
  // Factory helpers
  // ---------------------------------------------------------------------------

  /// Create (or retrieve) a logger backed by a progress_sink writing to `file`.
  [[nodiscard]] inline std::shared_ptr<spdlog::logger> make_progress_logger(
      std::string_view name, std::FILE* file = stderr) {
    if (auto existing = spdlog::get(std::string{name}))
      return existing;

    auto sink = std::make_shared<progress_sink>(file);
    auto logger = std::make_shared<spdlog::logger>(std::string{name}, std::move(sink));
    spdlog::register_logger(logger);
    return logger;
  }

  /// Convenience: create a progress_bar on stderr without manually managing a logger.
  [[nodiscard]] inline std::unique_ptr<progress_bar> default_progress_bar(
      std::string_view desc,
      std::size_t total,
      bool ascii = false,
      std::string_view logger_name = "progress_bar_stderr") {
    auto logger = make_progress_logger(logger_name);
    return std::make_unique<progress_bar>(std::move(logger), desc, total, ascii);
  }

}  // namespace dsf::utility