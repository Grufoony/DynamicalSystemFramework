#include "dsf/utility/progress_bar.hpp"

#include <cstdio>
#include <memory>
#include <string>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using namespace dsf::utility;

TEST_CASE("progress_bar") {
  // A logger without a progress_sink: the bar only counts and renders on demand
  auto logger = std::make_shared<spdlog::logger>("progress_bar_test_null");

  SUBCASE("ASCII rendering") {
    progress_bar bar{logger, "Test", 4, true};
    bar += 2;
    // 60 columns leave 21 characters to the bar: "Test:  50% |" + bar + "| 2/4 [...]\r"
    auto const line = bar.render(60);
    CHECK(line.starts_with("Test:  50% |" + std::string(10, '#') + std::string(11, '-') +
                           "| 2/4 ["));
    CHECK_EQ(line.back(), '\r');
  }

  SUBCASE("Unicode rendering") {
    progress_bar bar{logger, "Test", 8};
    ++bar;
    ++bar;
    ++bar;
    // 3/8 of 21 characters is 7 full blocks plus 7/8 of a block, then 13 blanks
    std::string expectedBar;
    for (auto i{0}; i < 7; ++i) {
      expectedBar += BLOCKS.back();
    }
    expectedBar += BLOCKS[7];
    expectedBar += std::string(13, ' ');
    CHECK(bar.render(60).starts_with("Test:  38% |" + expectedBar + "| 3/8 ["));
  }

  SUBCASE("Logging to a file which is not a terminal") {
    std::FILE* file = std::tmpfile();
    REQUIRE(file != nullptr);
    auto fileLogger = std::make_shared<spdlog::logger>(
        "progress_bar_test_file", std::make_shared<progress_sink>(file));
    {
      progress_bar bar{fileLogger, "Test", 2};
      fileLogger->info("hello from the progress bar test");
      bar.update(2);
    }
    std::rewind(file);
    std::string content;
    for (int c; (c = std::fgetc(file)) != EOF;) {
      content += static_cast<char>(c);
    }
    std::fclose(file);
    // Messages pass through, while the bar is only drawn on terminals
    CHECK(content.find("hello from the progress bar test") != std::string::npos);
    CHECK(content.find("Test:") == std::string::npos);
  }
}
