#include "BasicService.h"
#include "tiny_framework/logging.h"
#include "version.h"
#include "yaml-cpp/yaml.h"

BasicService::BasicService()
  : tiny::application()
{
}

BasicService::~BasicService()
{
    // todo
}

bool BasicService::initialize(const std::string& busi_config)
{
    if (!init_config(busi_config))
    {
        return false;
    }

    // 【注意】需要app.yaml中daemon设置为true时，添加定时器才生效
    // 这里count需要声明为static，否则post_timer_event结束后，函数退出，count因退出作用域而会被销毁
    static uint32_t count = 0;
    event_loop_.post_timer_event(
    "timer_test",
    [&]() { LOG_I(BasicService) << "timer_test, count: " << ++count; },
    std::chrono::seconds(1),
    5);

    LOG_I(BasicService) << app_name() << "_" << app_version() << "(" << app_buildtime()
                   << ") already initialized.";
    return true;
}

bool BasicService::init_config(const std::string& busi_config)
{
    if (busi_config.empty())
    {
        LOG_COUT(BasicService) << "busi_config is empty";
    }

    try
    {
        // 加载配置文件
        YAML::Node config = YAML::LoadFile(busi_config);

        // 解析单进程配置
        auto test_property = config["test_property"].as<std::string>();
        LOG_I(BasicService) << LOG_KV(test_property);

        return true;
    }
    catch (const YAML::Exception& e)
    {
        TINYLOG_STDERR(BasicService) << "parsing " << busi_config << " YAML: " << e.what();
        return false;
    }
    catch (const std::exception& e)
    {
        TINYLOG_STDERR(BasicService) << "Error: " << e.what();
        return false;
    }
}

void BasicService::on_terminate()
{
    LOG_COUT(BasicService) << "on_terminate";
}

std::string BasicService::app_version() const
{
    return APP_VERSION;
}

std::string BasicService::app_buildtime() const
{
    return APP_BUILD_TIME;
}

std::string BasicService::app_name() const
{
    return APP_NAME;
}
