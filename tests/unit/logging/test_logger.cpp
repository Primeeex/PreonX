#include <gtest/gtest.h>

#include "foundation/logging/logger.hpp"
#include "foundation/logging/console_sink.hpp"
#include "foundation/logging/log_level.hpp"

namespace {

TEST(LogLevelTest, LevelNames) {
    EXPECT_STREQ(foundation::log_level_name(foundation::LogLevel::Trace), "TRACE");
    EXPECT_STREQ(foundation::log_level_name(foundation::LogLevel::Debug), "DEBUG");
    EXPECT_STREQ(foundation::log_level_name(foundation::LogLevel::Info), "INFO");
    EXPECT_STREQ(foundation::log_level_name(foundation::LogLevel::Warning), "WARN");
    EXPECT_STREQ(foundation::log_level_name(foundation::LogLevel::Error), "ERROR");
    EXPECT_STREQ(foundation::log_level_name(foundation::LogLevel::Fatal), "FATAL");
}

TEST(LogLevelTest, LevelColors) {
    EXPECT_NE(foundation::log_level_color(foundation::LogLevel::Trace), nullptr);
    EXPECT_NE(foundation::log_level_color(foundation::LogLevel::Info), nullptr);
    EXPECT_NE(foundation::log_level_color(foundation::LogLevel::Error), nullptr);
}

TEST(ConsoleSinkTest, Construction) {
    foundation::ConsoleSink sink;
    EXPECT_TRUE(sink.color_enabled());
}

TEST(ConsoleSinkTest, ColorToggle) {
    foundation::ConsoleSink sink;
    sink.set_color_enabled(false);
    EXPECT_FALSE(sink.color_enabled());
    sink.set_color_enabled(true);
    EXPECT_TRUE(sink.color_enabled());
}

TEST(LoggerTest, DefaultLevel) {
    foundation::Logger logger;
    EXPECT_EQ(logger.level(), foundation::LogLevel::Info);
}

TEST(LoggerTest, SetLevel) {
    foundation::Logger logger;
    logger.set_level(foundation::LogLevel::Trace);
    EXPECT_EQ(logger.level(), foundation::LogLevel::Trace);
    logger.set_level(foundation::LogLevel::Fatal);
    EXPECT_EQ(logger.level(), foundation::LogLevel::Fatal);
}

TEST(LoggerTest, LogDoesNotCrash) {
    foundation::Logger logger;
    logger.trace("test trace");
    logger.debug("test debug");
    logger.info("test info");
    logger.warning("test warning");
    logger.error("test error");
    logger.fatal("test fatal");
}

TEST(LoggerTest, FormattedLog) {
    foundation::Logger logger;
    logger.set_level(foundation::LogLevel::Trace);
    logger.info("Hello {} {} {}", "world", 42, 3.14);
    logger.debug("Debug message: {}", "test");
    logger.warning("Warning: {}", 123);
}

TEST(LoggerTest, Flush) {
    foundation::Logger logger;
    logger.flush();
}

TEST(LoggerTest, AddCustomSink) {
    foundation::Logger logger;
    logger.add_sink(std::make_unique<foundation::ConsoleSink>());
    logger.info("test");
}

} // namespace
