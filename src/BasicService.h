#pragma once

#include "common_util/singleton.h"
#include "tiny_framework/application.h"
#include <string>

class BasicService : public tiny::application
{
    CUTL_SINGLETON_REF(BasicService)

private:
    virtual bool initialize(const std::string& busi_config) override;
    virtual void on_terminate() override;
    std::string app_name() const override;
    virtual std::string app_version() const override;
    virtual std::string app_buildtime() const override;

private:
    bool init_config(const std::string& busi_config);
};

#define BasicServiceInst BasicService::get_instance()